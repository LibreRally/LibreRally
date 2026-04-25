using System;
using System.Collections.Concurrent;
using System.IO;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Text.Json;
using System.Threading;
using System.Threading.Tasks;

namespace LibreRally.Telemetry
{
	internal sealed class VehicleLiveTuningBridgeServer : IDisposable
	{
		private static readonly JsonSerializerOptions SerializerOptions = new(JsonSerializerDefaults.Web);
		private readonly TcpListener _listener;
		private readonly ConcurrentQueue<PendingRequest> _pendingRequests = new();
		private readonly CancellationTokenSource _shutdown = new();
		private Task? _acceptLoop;

		public VehicleLiveTuningBridgeServer(int port)
		{
			_listener = new TcpListener(IPAddress.Loopback, port);
		}

		public int Port => ((IPEndPoint)_listener.LocalEndpoint).Port;

		public void Start()
		{
			_listener.Start();
			_acceptLoop = Task.Run(() => AcceptLoopAsync(_shutdown.Token), _shutdown.Token);
		}

		public void Pump(
			Func<LiveTuningSnapshot> getSnapshot,
			Func<LiveTuningPatch, LiveTuningBridgeResponse> applyPatch,
			Func<LiveTuningBridgeResponse> reloadVehicle)
		{
			while (_pendingRequests.TryDequeue(out var pendingRequest))
			{
				var response = HandleRequest(pendingRequest.Request, getSnapshot, applyPatch, reloadVehicle);
				pendingRequest.Complete(response);
			}
		}

		private static LiveTuningBridgeResponse HandleRequest(
			LiveTuningBridgeRequest request,
			Func<LiveTuningSnapshot> getSnapshot,
			Func<LiveTuningPatch, LiveTuningBridgeResponse> applyPatch,
			Func<LiveTuningBridgeResponse> reloadVehicle)
		{
			return request.Command.Trim().ToLowerInvariant() switch
			{
				"ping" => new LiveTuningBridgeResponse
				{
					Succeeded = true,
					Command = "ping",
					Summary = "LibreRally live tuning bridge is reachable.",
					Snapshot = getSnapshot(),
				},
				"get_snapshot" => new LiveTuningBridgeResponse
				{
					Succeeded = true,
					Command = "get_snapshot",
					Summary = "Captured live vehicle snapshot.",
					Snapshot = getSnapshot(),
				},
				"apply_patch" when request.Patch != null => applyPatch(request.Patch),
				"apply_patch" => new LiveTuningBridgeResponse
				{
					Succeeded = false,
					Command = "apply_patch",
					Summary = "Missing live tuning patch payload.",
				},
				"reload_vehicle" => reloadVehicle(),
				_ => new LiveTuningBridgeResponse
				{
					Succeeded = false,
					Command = request.Command,
					Summary = $"Unsupported live tuning bridge command '{request.Command}'.",
				},
			};
		}

		private async Task AcceptLoopAsync(CancellationToken cancellationToken)
		{
			while (!cancellationToken.IsCancellationRequested)
			{
				TcpClient client;
				try
				{
					client = await _listener.AcceptTcpClientAsync(cancellationToken);
				}
				catch (OperationCanceledException)
				{
					break;
				}
				catch (ObjectDisposedException)
				{
					break;
				}

				_ = Task.Run(() => HandleClientAsync(client, cancellationToken), cancellationToken);
			}
		}

		private async Task HandleClientAsync(TcpClient client, CancellationToken cancellationToken)
		{
			using (client)
			using (var stream = client.GetStream())
			using (var reader = new StreamReader(stream, Encoding.UTF8, detectEncodingFromByteOrderMarks: false, leaveOpen: true))
			using (var writer = new StreamWriter(stream, new UTF8Encoding(false), leaveOpen: true) { AutoFlush = true })
			{
				LiveTuningBridgeResponse response;
				try
				{
					var line = await reader.ReadLineAsync(cancellationToken);
					if (string.IsNullOrWhiteSpace(line))
					{
						return;
					}

					var request = JsonSerializer.Deserialize<LiveTuningBridgeRequest>(line, SerializerOptions);
					if (request == null || string.IsNullOrWhiteSpace(request.Command))
					{
						response = new LiveTuningBridgeResponse
						{
							Succeeded = false,
							Command = string.Empty,
							Summary = "Invalid live tuning bridge request payload.",
						};
					}
					else
					{
						var pendingRequest = new PendingRequest(request);
						_pendingRequests.Enqueue(pendingRequest);
						response = await pendingRequest.Task.WaitAsync(TimeSpan.FromSeconds(5), cancellationToken);
					}
				}
				catch (JsonException ex)
				{
					response = new LiveTuningBridgeResponse
					{
						Succeeded = false,
						Command = string.Empty,
						Summary = $"Invalid live tuning JSON: {ex.Message}",
					};
				}
				catch (TimeoutException)
				{
					response = new LiveTuningBridgeResponse
					{
						Succeeded = false,
						Command = string.Empty,
						Summary = "Timed out waiting for the game thread to process the live tuning request.",
					};
				}
				catch (IOException ex)
				{
					response = new LiveTuningBridgeResponse
					{
						Succeeded = false,
						Command = string.Empty,
						Summary = $"Live tuning bridge I/O error: {ex.Message}",
					};
				}

				await writer.WriteLineAsync(JsonSerializer.Serialize(response, SerializerOptions));
			}
		}

		public void Dispose()
		{
			_shutdown.Cancel();
			_listener.Stop();
			_shutdown.Dispose();
		}

		private sealed class PendingRequest
		{
			private readonly TaskCompletionSource<LiveTuningBridgeResponse> _completionSource =
				new(TaskCreationOptions.RunContinuationsAsynchronously);

			public PendingRequest(LiveTuningBridgeRequest request)
			{
				Request = request;
			}

			public LiveTuningBridgeRequest Request { get; }

			public Task<LiveTuningBridgeResponse> Task => _completionSource.Task;

			public void Complete(LiveTuningBridgeResponse response)
			{
				_completionSource.TrySetResult(response);
			}
		}
	}
}
