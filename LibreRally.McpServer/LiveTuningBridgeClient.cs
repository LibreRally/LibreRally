using System.Net.Sockets;
using System.Text;
using System.Text.Json;
using LibreRally.Telemetry;

namespace LibreRally.McpServer;

internal interface ILiveTuningBridgeClient
{
	LiveTuningBridgeResponse Send(string host, int port, LiveTuningBridgeRequest request);
}

internal sealed class LiveTuningBridgeClient : ILiveTuningBridgeClient
{
	private static readonly JsonSerializerOptions SerializerOptions = new(JsonSerializerDefaults.Web);

	public LiveTuningBridgeResponse Send(string host, int port, LiveTuningBridgeRequest request)
	{
		using var client = new TcpClient();
		client.Connect(host, port);
		using var stream = client.GetStream();
		using var writer = new StreamWriter(stream, new UTF8Encoding(false), leaveOpen: true) { AutoFlush = true };
		using var reader = new StreamReader(stream, Encoding.UTF8, detectEncodingFromByteOrderMarks: false, leaveOpen: true);

		writer.WriteLine(JsonSerializer.Serialize(request, SerializerOptions));
		var responseLine = reader.ReadLine();
		if (string.IsNullOrWhiteSpace(responseLine))
		{
			throw new InvalidOperationException("Live tuning bridge returned an empty response.");
		}

		return JsonSerializer.Deserialize<LiveTuningBridgeResponse>(responseLine, SerializerOptions)
		       ?? throw new InvalidOperationException("Live tuning bridge response was not valid JSON.");
	}
}
