using System;
using System.Collections;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Runtime.Loader;
using Xunit;

namespace LibreRally.Tests;

public class BasicCarPipelineTests
{
    private static readonly Lazy<Assembly> LibreRallyAssembly = new(LoadLibreRallyAssembly);

    [Fact]
    public void BasicCar_ShouldParseNodes()
    {
        object config = LoadConfig();
        object bodyPart = ParseSinglePart("body.jbeam", config);

        Assert.Equal("basic_car_body", GetProperty<string>(bodyPart, "Name"));

        var nodes = GetList(GetProperty(bodyPart, "Nodes"));
        Assert.True(nodes.Length >= 10);
        Assert.Contains(nodes, node => GetProperty<string>(node, "Id") == "bc2");

        IDictionary refNodes = GetDictionary(GetProperty(bodyPart, "RefNodes"));
        Assert.Equal("bc2", refNodes["ref"]);
    }

    [Fact]
    public void BasicCar_ShouldParseBeams()
    {
        object config = LoadConfig();
        object wheelsPart = ParseSinglePart("wheels.jbeam", config);

        var beams = GetList(GetProperty(wheelsPart, "Beams"));
        Assert.True(beams.Length >= 8);

        var nodes = GetList(GetProperty(wheelsPart, "Nodes"));
        Assert.Contains(nodes, node => ContainsString(GetProperty(GetProperty(node, "Properties"), "Groups"), "wheel_FL"));
        Assert.Contains(nodes, node => ContainsString(GetProperty(GetProperty(node, "Properties"), "Groups"), "wheel_RR"));
    }

    [Fact]
    public void BasicCar_ShouldAssembleVehicle()
    {
        object config = LoadConfig();
        object definition = AssembleVehicle(config);

        Assert.Equal("basic_car", GetProperty<string>(definition, "VehicleName"));

        IDictionary nodes = GetDictionary(GetProperty(definition, "Nodes"));
        Assert.True(nodes.Count >= 20);

        var beams = GetList(GetProperty(definition, "Beams"));
        Assert.True(beams.Length >= 40);

        IDictionary refNodes = GetDictionary(GetProperty(definition, "RefNodes"));
        Assert.Equal("bc2", refNodes["ref"]);

        IDictionary vars = GetDictionary(GetProperty(definition, "Vars"));
        Assert.Equal(52000f, Convert.ToSingle(vars["spring_F_asphalt"]));

        object wheelNode = nodes["wfl1"]!;
        Assert.True(ContainsString(GetProperty(wheelNode, "Groups"), "wheel_FL"));
    }

    [Fact]
    public void BasicCar_ShouldBuildPhysics()
    {
        object config = LoadConfig();
        object definition = AssembleVehicle(config);
        object result = BuildVehicle(definition);

        object rootEntity = GetProperty(result, "RootEntity");
        object chassisEntity = GetProperty(result, "ChassisEntity");
        object wheelFl = GetProperty(result, "WheelFL");
        object wheelFr = GetProperty(result, "WheelFR");
        object wheelRl = GetProperty(result, "WheelRL");
        object wheelRr = GetProperty(result, "WheelRR");

        Assert.Equal("basic_car", GetProperty<string>(rootEntity, "Name"));
        Assert.Equal("chassis", GetProperty<string>(chassisEntity, "Name"));
        Assert.NotNull(GetComponent(wheelFl, "LibreRally.Vehicle.Physics.WheelSettings"));
        Assert.NotNull(GetComponent(wheelFr, "LibreRally.Vehicle.Physics.WheelSettings"));
        Assert.NotNull(GetComponent(wheelRl, "LibreRally.Vehicle.Physics.WheelSettings"));
        Assert.NotNull(GetComponent(wheelRr, "LibreRally.Vehicle.Physics.WheelSettings"));
        Assert.Equal("wheel_FL", GetProperty<string>(wheelFl, "Name"));
        Assert.Equal("wheel_FR", GetProperty<string>(wheelFr, "Name"));
        Assert.Equal("wheel_RL", GetProperty<string>(wheelRl, "Name"));
        Assert.Equal("wheel_RR", GetProperty<string>(wheelRr, "Name"));
    }

    private static object LoadConfig()
    {
        Type loader = GetLibreRallyType("LibreRally.Vehicle.JBeam.PcConfigLoader");
        return loader.GetMethod("Load", BindingFlags.Public | BindingFlags.Static)!.Invoke(null, new object[] { Path.Combine(GetVehicleFolder(), "basic_car.pc") })!;
    }

    private static string GetVehicleFolder()
    {
        DirectoryInfo? directory = new(AppContext.BaseDirectory);
        while (directory != null)
        {
            string candidate = Path.Combine(directory.FullName, "LibreRally.sln");
            if (File.Exists(candidate))
            {
                return Path.Combine(directory.FullName, "LibreRally", "Resources", "BeamNG Vehicles", "basic_car");
            }

            directory = directory.Parent;
        }

        throw new DirectoryNotFoundException("Could not locate the repository root for the basic_car tests.");
    }

    private static object ParseSinglePart(string fileName, object config)
    {
        Type parser = GetLibreRallyType("LibreRally.Vehicle.JBeam.JBeamParser");
        IDictionary vars = GetDictionary(GetProperty(config, "Vars"));
        object parts = parser.GetMethod("ParseFile", BindingFlags.Public | BindingFlags.Static)!.Invoke(
            null,
            new object[] { Path.Combine(GetVehicleFolder(), fileName), vars })!;

        return Assert.Single(GetList(parts));
    }

    private static object AssembleVehicle(object config)
    {
        Type assembler = GetLibreRallyType("LibreRally.Vehicle.JBeam.JBeamAssembler");
        return assembler.GetMethod("Assemble", BindingFlags.Public | BindingFlags.Static)!.Invoke(
            null,
            new[] { GetVehicleFolder(), config })!;
    }

    private static object BuildVehicle(object definition)
    {
        Type builder = GetLibreRallyType("LibreRally.Vehicle.Physics.VehiclePhysicsBuilder");
        return builder.GetMethod("Build", BindingFlags.Public | BindingFlags.Static)!.Invoke(null, new[] { definition })!;
    }

    private static Type GetLibreRallyType(string typeName)
    {
        return GetLibreRallyAssembly().GetType(typeName, throwOnError: true)!;
    }

    private static Assembly GetLibreRallyAssembly()
    {
        return LibreRallyAssembly.Value;
    }

    private static Assembly LoadLibreRallyAssembly()
    {
        string assemblyPath = Path.Combine(GetRepositoryRoot(), "LibreRally", "bin", "Debug", "net10.0-windows", "LibreRally.dll");
        if (!File.Exists(assemblyPath))
            throw new FileNotFoundException("Build LibreRally/LibreRally.csproj before running the basic_car pipeline tests.", assemblyPath);

        return AssemblyLoadContext.Default.LoadFromAssemblyPath(assemblyPath);
    }

    private static string GetRepositoryRoot()
    {
        DirectoryInfo? directory = new(AppContext.BaseDirectory);
        while (directory != null)
        {
            string candidate = Path.Combine(directory.FullName, "LibreRally.sln");
            if (File.Exists(candidate))
                return directory.FullName;

            directory = directory.Parent;
        }

        throw new DirectoryNotFoundException("Could not locate the repository root.");
    }

    private static IDictionary GetDictionary(object value)
    {
        return Assert.IsAssignableFrom<IDictionary>(value);
    }

    private static object GetProperty(object target, string propertyName)
    {
        return target.GetType().GetProperty(propertyName, BindingFlags.Public | BindingFlags.Instance)!.GetValue(target)!;
    }

    private static T GetProperty<T>(object target, string propertyName)
    {
        return (T)GetProperty(target, propertyName);
    }

    private static object[] GetList(object value)
    {
        return Assert.IsAssignableFrom<IEnumerable>(value).Cast<object>().ToArray();
    }

    private static bool ContainsString(object value, string expected)
    {
        return Assert.IsAssignableFrom<IEnumerable>(value).Cast<object?>().Any(item => string.Equals(item?.ToString(), expected, StringComparison.Ordinal));
    }

    private static object? GetComponent(object entity, string componentTypeName)
    {
        Type componentType = GetLibreRallyAssembly().GetType(componentTypeName, throwOnError: true)!;
        MethodInfo getMethod = entity.GetType()
            .GetMethods(BindingFlags.Public | BindingFlags.Instance)
            .Single(method => method.Name == "Get" && method.IsGenericMethodDefinition && method.GetParameters().Length == 0);

        return getMethod.MakeGenericMethod(componentType).Invoke(entity, null);
    }

}
