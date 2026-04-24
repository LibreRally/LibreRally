using System;
using System.Net.Sockets;
using LibreRally.Vehicle;
using Stride.BepuPhysics;
using Stride.Core.Diagnostics;
using Stride.Core.Mathematics;
using Stride.Engine;

namespace LibreRally.Telemetry
{
	/// <summary>
	/// Manages OutGauge and OutSim socket lifecycle and packet transmission for the active vehicle.
	/// </summary>
	internal sealed class VehicleTelemetrySession
	{
		private static readonly Logger Log = GlobalLogger.GetLogger("VehicleTelemetrySession");
		private const double FailureLogIntervalSeconds = 5d;

		private UdpClient? _outGaugeClient;
		private string? _outGaugeTargetHost;
		private int _outGaugeTargetPort;
		private bool _outGaugeSendFailed;
		private double _outGaugeNextFailureLogTimeSeconds;
		private float _outGaugeElapsed;

		private UdpClient? _outSimClient;
		private string? _outSimTargetHost;
		private int _outSimTargetPort;
		private bool _outSimSendFailed;
		private double _outSimNextFailureLogTimeSeconds;
		private float _outSimElapsed;
		private bool _outSimHasPreviousLinearVelocity;
		private Vector3 _outSimPreviousLinearVelocity;

		/// <summary>
		/// Gets or sets a value indicating whether OutGauge telemetry is enabled.
		/// </summary>
		public bool OutGaugeEnabled { get; set; } = true;

		/// <summary>
		/// Gets or sets the delay between OutGauge packets in centiseconds.
		/// </summary>
		public int OutGaugeDelayCentiseconds { get; set; } = 1;

		/// <summary>
		/// Gets or sets the target OutGauge host name or IP address.
		/// </summary>
		public string OutGaugeIp { get; set; } = "127.0.0.1";

		/// <summary>
		/// Gets or sets the target OutGauge port.
		/// </summary>
		public int OutGaugePort { get; set; } = 4444;

		/// <summary>
		/// Gets or sets the OutGauge packet identifier.
		/// </summary>
		public int OutGaugeId { get; set; }

		/// <summary>
		/// Gets or sets a value indicating whether OutSim telemetry is enabled.
		/// </summary>
		public bool OutSimEnabled { get; set; }

		/// <summary>
		/// Gets or sets the delay between OutSim packets in centiseconds.
		/// </summary>
		public int OutSimDelayCentiseconds { get; set; } = 1;

		/// <summary>
		/// Gets or sets the target OutSim host name or IP address.
		/// </summary>
		public string OutSimIp { get; set; } = "127.0.0.1";

		/// <summary>
		/// Gets or sets the target OutSim port.
		/// </summary>
		public int OutSimPort { get; set; } = 4123;

		/// <summary>
		/// Gets or sets the OutSim packet identifier.
		/// </summary>
		public int OutSimId { get; set; }

		/// <summary>
		/// Gets or sets a value indicating whether an OutGauge socket should be kept connected.
		/// </summary>
		public bool OutGaugeConnectionRequested { get; private set; } = true;

		/// <summary>
		/// Gets or sets a value indicating whether an OutSim socket should be kept connected.
		/// </summary>
		public bool OutSimConnectionRequested { get; private set; } = true;

		/// <summary>
		/// Advances both telemetry transports for the current frame.
		/// </summary>
		/// <param name="game">The active Stride game instance.</param>
		/// <param name="deltaTime">The current frame delta time.</param>
		/// <param name="car">The active rally car.</param>
		public void Update(Game game, float deltaTime, RallyCarComponent? car)
		{
			SendOutGaugeTelemetry(game, deltaTime, car);
			SendOutSimTelemetry(game, deltaTime, car);
		}

		/// <summary>
		/// Sets the OutGauge enabled state and updates the socket lifecycle.
		/// </summary>
		/// <param name="enabled"><see langword="true" /> to enable OutGauge; otherwise, <see langword="false" />.</param>
		public void SetOutGaugeEnabled(bool enabled)
		{
			OutGaugeEnabled = enabled;
			_outGaugeElapsed = 0f;
			if (!enabled)
			{
				DisposeOutGaugeClient();
				return;
			}

			if (OutGaugeConnectionRequested)
			{
				EnsureOutGaugeClient();
			}
		}

		/// <summary>
		/// Sets the OutSim enabled state and updates the socket lifecycle.
		/// </summary>
		/// <param name="enabled"><see langword="true" /> to enable OutSim; otherwise, <see langword="false" />.</param>
		public void SetOutSimEnabled(bool enabled)
		{
			OutSimEnabled = enabled;
			_outSimElapsed = 0f;
			_outSimHasPreviousLinearVelocity = false;
			if (!enabled)
			{
				DisposeOutSimClient();
				return;
			}

			if (OutSimConnectionRequested)
			{
				EnsureOutSimClient();
			}
		}

		/// <summary>
		/// Requests an OutGauge socket connection.
		/// </summary>
		public void ConnectOutGauge()
		{
			OutGaugeConnectionRequested = true;
			EnsureOutGaugeClient();
		}

		/// <summary>
		/// Closes the OutGauge socket and suppresses reconnects.
		/// </summary>
		public void DisconnectOutGauge()
		{
			OutGaugeConnectionRequested = false;
			DisposeOutGaugeClient();
		}

		/// <summary>
		/// Recreates the OutGauge socket for the current endpoint.
		/// </summary>
		public void ReconnectOutGauge()
		{
			OutGaugeConnectionRequested = true;
			DisposeOutGaugeClient();
			EnsureOutGaugeClient();
		}

		/// <summary>
		/// Requests an OutSim socket connection.
		/// </summary>
		public void ConnectOutSim()
		{
			OutSimConnectionRequested = true;
			EnsureOutSimClient();
		}

		/// <summary>
		/// Closes the OutSim socket and suppresses reconnects.
		/// </summary>
		public void DisconnectOutSim()
		{
			OutSimConnectionRequested = false;
			DisposeOutSimClient();
		}

		/// <summary>
		/// Recreates the OutSim socket for the current endpoint.
		/// </summary>
		public void ReconnectOutSim()
		{
			OutSimConnectionRequested = true;
			DisposeOutSimClient();
			EnsureOutSimClient();
		}

		/// <summary>
		/// Builds the OutGauge overlay summary string for the current configuration and socket state.
		/// </summary>
		/// <returns>The OutGauge summary string.</returns>
		public string BuildOutGaugeSummary()
		{
			var endpoint = DescribeTelemetryEndpoint(OutGaugeIp, OutGaugePort);
			var enabledState = OutGaugeEnabled ? "enabled" : "disabled";
			return $"OutGauge // {enabledState} // {endpoint} // {DescribeOutGaugeSocketState()} // {(OutGaugeId == 0 ? "id=none" : $"id={OutGaugeId}")}";
		}

		/// <summary>
		/// Builds the OutSim overlay summary string for the current configuration and socket state.
		/// </summary>
		/// <returns>The OutSim summary string.</returns>
		public string BuildOutSimSummary()
		{
			var endpoint = DescribeTelemetryEndpoint(OutSimIp, OutSimPort);
			var enabledState = OutSimEnabled ? "enabled" : "disabled";
			return $"OutSim // {enabledState} // {endpoint} // {DescribeOutSimSocketState()} // {(OutSimId == 0 ? "id=none" : $"id={OutSimId}")}";
		}

		/// <summary>
		/// Describes the current OutGauge socket state for status messaging.
		/// </summary>
		/// <returns>The socket-state description.</returns>
		public string DescribeOutGaugeSocketState() =>
			DescribeSocketState(OutGaugeConnectionRequested, _outGaugeClient != null, _outGaugeSendFailed);

		/// <summary>
		/// Describes the current OutSim socket state for status messaging.
		/// </summary>
		/// <returns>The socket-state description.</returns>
		public string DescribeOutSimSocketState() =>
			DescribeSocketState(OutSimConnectionRequested, _outSimClient != null, _outSimSendFailed);

		private void SendOutGaugeTelemetry(Game game, float deltaTime, RallyCarComponent? car)
		{
			if (!OutGaugeEnabled || !OutGaugeConnectionRequested || car == null)
			{
				_outGaugeElapsed = 0f;
				if (!OutGaugeEnabled || !OutGaugeConnectionRequested)
				{
					DisposeOutGaugeClient();
				}

				return;
			}

			var sendIntervalSeconds = Math.Max(0, OutGaugeDelayCentiseconds) * 0.01f;
			_outGaugeElapsed += Math.Max(0f, deltaTime);
			if (sendIntervalSeconds > 0f && _outGaugeElapsed < sendIntervalSeconds)
			{
				return;
			}

			_outGaugeElapsed = 0f;
			EnsureOutGaugeClient();
			if (_outGaugeClient == null || string.IsNullOrWhiteSpace(_outGaugeTargetHost))
			{
				return;
			}

			try
			{
				var sessionMilliseconds = Math.Max(0d, game.UpdateTime.Total.TotalMilliseconds);
				var snapshot = OutGaugeProtocol.FromCar(car, unchecked((uint)sessionMilliseconds));
				var payload = OutGaugeProtocol.Encode(snapshot, OutGaugeId);
				_outGaugeClient.Send(payload, payload.Length, _outGaugeTargetHost, _outGaugeTargetPort);
				_outGaugeSendFailed = false;
			}
			catch (SocketException ex)
			{
				HandleOutGaugeSendFailure(game, ex);
			}
			catch (ObjectDisposedException ex)
			{
				HandleOutGaugeSendFailure(game, ex);
			}
			catch (InvalidOperationException ex)
			{
				HandleOutGaugeSendFailure(game, ex);
			}
		}

		private void SendOutSimTelemetry(Game game, float deltaTime, RallyCarComponent? car)
		{
			if (!OutSimEnabled || !OutSimConnectionRequested || car == null)
			{
				_outSimElapsed = 0f;
				_outSimHasPreviousLinearVelocity = false;
				if (!OutSimEnabled || !OutSimConnectionRequested)
				{
					DisposeOutSimClient();
				}

				return;
			}

			var sendIntervalSeconds = Math.Max(0, OutSimDelayCentiseconds) * 0.01f;
			_outSimElapsed += Math.Max(0f, deltaTime);
			if (sendIntervalSeconds > 0f && _outSimElapsed < sendIntervalSeconds)
			{
				return;
			}

			var sampleDelta = Math.Max(_outSimElapsed, 1e-4f);
			_outSimElapsed = 0f;
			EnsureOutSimClient();
			if (_outSimClient == null || string.IsNullOrWhiteSpace(_outSimTargetHost))
			{
				return;
			}

			var chassisBody = car.CarBody.Get<BodyComponent>();
			if (chassisBody == null)
			{
				_outSimHasPreviousLinearVelocity = false;
				return;
			}

			var linearVelocity = chassisBody.LinearVelocity;
			var acceleration = _outSimHasPreviousLinearVelocity
				? (linearVelocity - _outSimPreviousLinearVelocity) / sampleDelta
				: Vector3.Zero;
			_outSimPreviousLinearVelocity = linearVelocity;
			_outSimHasPreviousLinearVelocity = true;

			try
			{
				var sessionMilliseconds = Math.Max(0d, game.UpdateTime.Total.TotalMilliseconds);
				var snapshot = OutSimProtocol.FromCar(car, unchecked((uint)sessionMilliseconds), acceleration);
				var payload = OutSimProtocol.Encode(snapshot, OutSimId);
				_outSimClient.Send(payload, payload.Length);
				_outSimSendFailed = false;
			}
			catch (SocketException ex)
			{
				HandleOutSimTelemetryFailure(game, ex);
			}
			catch (ObjectDisposedException ex)
			{
				HandleOutSimTelemetryFailure(game, ex);
			}
			catch (InvalidOperationException ex)
			{
				HandleOutSimTelemetryFailure(game, ex);
			}
		}

		private void EnsureOutGaugeClient()
		{
			var targetHost = OutGaugeIp?.Trim();
			var targetPort = OutGaugePort;
			if (string.IsNullOrWhiteSpace(targetHost) || targetPort is < 1 or > 65535)
			{
				DisposeOutGaugeClient();
				return;
			}

			if (_outGaugeClient != null &&
			    string.Equals(_outGaugeTargetHost, targetHost, StringComparison.OrdinalIgnoreCase) &&
			    _outGaugeTargetPort == targetPort)
			{
				return;
			}

			DisposeOutGaugeClient();
			_outGaugeClient = new UdpClient();
			_outGaugeTargetHost = targetHost;
			_outGaugeTargetPort = targetPort;
			_outGaugeSendFailed = false;
			_outGaugeNextFailureLogTimeSeconds = 0d;
		}

		private void EnsureOutSimClient()
		{
			var targetHost = OutSimIp?.Trim();
			var targetPort = OutSimPort;
			if (string.IsNullOrWhiteSpace(targetHost) || targetPort is < 1 or > 65535)
			{
				DisposeOutSimClient();
				return;
			}

			if (_outSimClient != null &&
			    string.Equals(_outSimTargetHost, targetHost, StringComparison.OrdinalIgnoreCase) &&
			    _outSimTargetPort == targetPort)
			{
				return;
			}

			DisposeOutSimClient();
			_outSimClient = new UdpClient();
			_outSimTargetHost = targetHost;
			_outSimTargetPort = targetPort;
			try
			{
				_outSimClient.Connect(targetHost, targetPort);
			}
			catch (SocketException ex)
			{
				HandleOutSimTelemetryFailure(null, ex);
				DisposeOutSimClient();
				return;
			}

			_outSimSendFailed = false;
			_outSimNextFailureLogTimeSeconds = 0d;
		}

		private void DisposeOutGaugeClient()
		{
			_outGaugeClient?.Dispose();
			_outGaugeClient = null;
			_outGaugeTargetHost = null;
			_outGaugeTargetPort = 0;
			_outGaugeSendFailed = false;
			_outGaugeNextFailureLogTimeSeconds = 0d;
		}

		private void DisposeOutSimClient()
		{
			_outSimClient?.Dispose();
			_outSimClient = null;
			_outSimTargetHost = null;
			_outSimTargetPort = 0;
			_outSimSendFailed = false;
			_outSimNextFailureLogTimeSeconds = 0d;
			_outSimHasPreviousLinearVelocity = false;
		}

		private void HandleOutGaugeSendFailure(Game? game, Exception ex)
		{
			var sessionSeconds = Math.Max(0d, game?.UpdateTime.Total.TotalSeconds ?? 0d);
			if (!_outGaugeSendFailed || sessionSeconds >= _outGaugeNextFailureLogTimeSeconds)
			{
				Log.Warning($"OutGauge send failed ({_outGaugeTargetHost}:{_outGaugeTargetPort}): {ex.Message}");
				_outGaugeNextFailureLogTimeSeconds = sessionSeconds + FailureLogIntervalSeconds;
			}

			_outGaugeSendFailed = true;
		}

		private void HandleOutSimTelemetryFailure(Game? game, Exception ex)
		{
			var sessionSeconds = Math.Max(0d, game?.UpdateTime.Total.TotalSeconds ?? 0d);
			if (!_outSimSendFailed || sessionSeconds >= _outSimNextFailureLogTimeSeconds)
			{
				Log.Warning($"OutSim telemetry failed ({_outSimTargetHost}:{_outSimTargetPort}): {ex.Message}");
				_outSimNextFailureLogTimeSeconds = sessionSeconds + FailureLogIntervalSeconds;
			}

			_outSimSendFailed = true;
		}

		private static string DescribeTelemetryEndpoint(string? host, int port)
		{
			var trimmedHost = host?.Trim();
			return string.IsNullOrWhiteSpace(trimmedHost) || port is < 1 or > 65535
				? "endpoint not configured"
				: $"{trimmedHost}:{port}";
		}

		private static string DescribeSocketState(bool connectionRequested, bool socketOpen, bool sendFailed)
		{
			if (!connectionRequested)
			{
				return "socket disconnected";
			}

			if (!socketOpen)
			{
				return "socket closed";
			}

			return sendFailed ? "socket open, last send failed" : "socket open";
		}
	}
}
