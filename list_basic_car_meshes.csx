using System;
using System.IO;
using System.Linq;
using System.Xml.Linq;
var dae = @"C:\Users\pierc\source\repos\archanox\LibreRally\LibreRally\Resources\BeamNG Vehicles\basic_car\FormulaBeeModel.dae";
XNamespace ns = "http://www.collada.org/2005/11/COLLADASchema";
var doc = XDocument.Load(dae);
var geoms = doc.Descendants(ns + "geometry").Select(x => (string?)x.Attribute("name") ?? (string?)x.Attribute("id") ?? "").Where(x => !string.IsNullOrWhiteSpace(x)).Distinct(StringComparer.OrdinalIgnoreCase).OrderBy(x => x).ToArray();
Console.WriteLine(string.Join(Environment.NewLine, geoms));
