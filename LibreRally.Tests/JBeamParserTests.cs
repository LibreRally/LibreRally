using System;
using System.IO;
using LibreRally.Vehicle.JBeam;
using Xunit;

namespace LibreRally.Tests;

public sealed class JBeamParserTests
{
    [Fact]
    public void ParseVariableDefaultsFile_AllowsTrailingStrayRootPunctuation()
    {
        using var workspace = new TempWorkspace();
        var filePath = Path.Combine(workspace.RootPath, "malformed_common_part.jbeam");
        File.WriteAllText(
            filePath,
            """
            {
            "test_part": {
                "slotType":"main",
                "variables": [
                    ["name", "type", "unit", "category", "default", "min", "max", "title", "description"],
                    ["$springheight_R", "range", "+m", "Suspension", 0.03, -0.05, 0.05, "Spring Height", "Raise or lower the suspension height"],
                ],
                "nodes": [
                    ["id", "posX", "posY", "posZ"],
                    ["n1", 0, 0, 0],
                ],
            },
            },
            """);

        var vars = JBeamParser.ParseVariableDefaultsFile(filePath);
        var part = Assert.Single(JBeamParser.ParseFile(filePath, vars));

        Assert.Equal(0.03f, vars["springheight_R"]);
        Assert.Equal("test_part", part.Name);
    }

    [Fact]
    public void ParseVariableDefaultsFile_AllowsLeadingZeroNumberLiteralsInUnrelatedSections()
    {
        using var workspace = new TempWorkspace();
        var filePath = Path.Combine(workspace.RootPath, "leading_zero_common_part.jbeam");
        File.WriteAllText(
            filePath,
            """
            {
              "test_part": {
                "slotType":"main",
                "variables": [
                  ["name", "type", "unit", "category", "default", "min", "max", "title", "description"],
                  ["$ride_height", "range", "+m", "Suspension", 0.03, -0.05, 0.05, "Ride Height", "Raise or lower the suspension"]
                ],
                "torsionbars": [
                  ["id1:", "id2:", "id3:", "id4:"],
                  {"spring":00, "damp":40, "deform":40000, "strength":1000},
                  ["n1", "n2", "n3", "n4"]
                ],
                "nodes": [
                  ["id", "posX", "posY", "posZ"],
                  ["n1", 0, 0, 0]
                ]
              }
            }
            """);

        var vars = JBeamParser.ParseVariableDefaultsFile(filePath);
        var part = Assert.Single(JBeamParser.ParseFile(filePath, vars));

        Assert.Equal(0.03f, vars["ride_height"]);
        Assert.Equal("test_part", part.Name);
    }

    [Fact]
    public void ParseFile_IgnoresUnparseableStringNodeBooleans()
    {
        using var workspace = new TempWorkspace();
        var filePath = Path.Combine(workspace.RootPath, "string_boolean_nodes.jbeam");
        File.WriteAllText(
            filePath,
            """
            {
              "test_part": {
                "slotType":"main",
                "nodes": [
                  ["id", "posX", "posY", "posZ"],
                  {"collision":"$= $components.noSections ~= 2"},
                  {"selfCollision":""},
                  ["n1", 0, 0, 0]
                ]
              }
            }
            """);

        var part = Assert.Single(JBeamParser.ParseFile(filePath));
        var node = Assert.Single(part.Nodes);

        Assert.True(node.Properties.Collision);
        Assert.True(node.Properties.SelfCollision);
    }

    [Fact]
    public void Assemble_IgnoresBrokenSupplementalFilesDuringVariableDefaultPass()
    {
        using var workspace = new TempWorkspace();
        var vehicleFolder = Path.Combine(workspace.RootPath, "vehicle");
        var commonFolder = Path.Combine(workspace.RootPath, "common");
        Directory.CreateDirectory(vehicleFolder);
        Directory.CreateDirectory(commonFolder);

        File.WriteAllText(
            Path.Combine(vehicleFolder, "main.jbeam"),
            """
            {
              "test_car": {
                "slotType":"main",
                "nodes":[
                  ["id","posX","posY","posZ"],
                  ["w1", 1, 2, 3]
                ]
              }
            }
            """);

        File.WriteAllText(Path.Combine(commonFolder, "broken.jbeam"), "this is not jbeam");

        var definition = JBeamAssembler.Assemble([vehicleFolder, commonFolder], vehicleFolder);

        Assert.Contains("w1", definition.Nodes.Keys);
        Assert.Equal("test_car", definition.VehicleName);
    }

    [Fact]
    public void Parse_HandlesNumbersStartingWithDot()
    {
        var jbeam = """{ "test_part": { "val": .5 } }""";
        var parts = JBeamParser.Parse(jbeam);
        Assert.Single(parts);
    }

    [Fact]
    public void Parse_HandlesMissingCommaBetweenStrings()
    {
        var jbeam = """{ "test_part": { "test": ["a", "b" "c"] } }""";
        var parts = JBeamParser.Parse(jbeam);
        Assert.Single(parts);
    }

    [Fact]
    public void Parse_HandlesMissingCommaAfterNumber()
    {
        var jbeam = """{ "test_part": { "a": 0.28 "b": "c" } }""";
        var parts = JBeamParser.Parse(jbeam);
        Assert.Single(parts);
    }

    private sealed class TempWorkspace : IDisposable
    {
        public TempWorkspace()
        {
            RootPath = Path.Combine(Path.GetTempPath(), "LibreRally.Tests", Guid.NewGuid().ToString("N"));
            Directory.CreateDirectory(RootPath);
        }

        public string RootPath { get; }

        public void Dispose()
        {
            if (Directory.Exists(RootPath))
            {
                Directory.Delete(RootPath, recursive: true);
            }
        }
    }
}
