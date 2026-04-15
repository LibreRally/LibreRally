using System;
using System.IO;
using System.Linq;
using System.Xml.Linq;
var daeOverride = Environment.GetEnvironmentVariable("LIBRERALLY_DAE_PATH");
var defaultDae = Path.Combine(
    Directory.GetCurrentDirectory(),
    "LibreRally",
    "Resources",
    "BeamNG Vehicles",
    "basic_car",
    "FormulaBeeModel.dae");
var dae = string.IsNullOrWhiteSpace(daeOverride) ? defaultDae : daeOverride;
XNamespace ns = "http://www.collada.org/2005/11/COLLADASchema";
var doc = XDocument.Load(dae);
var geoms = doc.Descendants(ns + "geometry").Select(x => (string?)x.Attribute("name") ?? (string?)x.Attribute("id") ?? "").Where(x => !string.IsNullOrWhiteSpace(x)).Distinct(StringComparer.OrdinalIgnoreCase).OrderBy(x => x).ToArray();
Console.WriteLine(string.Join(Environment.NewLine, geoms));
