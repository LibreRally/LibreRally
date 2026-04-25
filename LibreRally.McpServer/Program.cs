using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Hosting;
using Microsoft.Extensions.Logging;
using ModelContextProtocol.Server;

namespace LibreRally.McpServer;

internal static class Program
{
	private static async Task<int> Main(string[] args)
	{
		var builder = Host.CreateApplicationBuilder(args);
		builder.Logging.AddConsole(options =>
		{
			options.LogToStandardErrorThreshold = LogLevel.Trace;
		});
		builder.Services.AddSingleton<ILiveTuningBridgeClient, LiveTuningBridgeClient>();
		builder.Services.AddSingleton<LibreRallyToolService>();
		builder.Services
			.AddMcpServer()
			.WithStdioServerTransport()
			.WithToolsFromAssembly();

		await builder.Build().RunAsync();
		return 0;
	}
}
