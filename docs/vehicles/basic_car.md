# Basic Car reference vehicle

The `basic_car` vehicle is LibreRally's reference BeamNG-style JBeam vehicle.

It exists to:

- serve as the default development vehicle
- validate LibreRally's JBeam loading pipeline against BeamNG tutorial-style content
- provide a stable fixture for future importer and parser changes

The vehicle lives in `LibreRally/Resources/BeamNG Vehicles/basic_car` and vendors the BeamNG basic car tutorial's Formula Bee asset set directly:

- `FormulaBeeModel.dae`
- `TutoFormulaBee.jbeam`
- `TutoFormulaBee_*.jbeam`
- `main.materials.json`
- `basic_car.pc`

`basic_car.pc` is LibreRally's compatibility layer over the tutorial assets: it selects the active `TutoFormulaBee` slot tree while preserving the tuning vars LibreRally's current BEPU drivetrain and suspension code reads at runtime.

LibreRally now resolves the tutorial's shared wheel, tyre, and running-gear meshes from BeamNG common content when they are not bundled locally. The tutorial's placeholder `TutoFBee_Spaceframe` and `TutoFBee_Gauges` flexbodies are intentionally not used because BeamNG does not ship those example meshes; the tutorial expects a mod author to replace them with real art.

LibreRally loads this vehicle through the existing pipeline:

`VehicleLoader -> PcConfigLoader -> JBeamParser -> JBeamAssembler -> VehicleDefinition -> VehiclePhysicsBuilder`

New reference or tutorial vehicles should follow the same folder structure so they can be validated through the same pipeline with minimal special handling. If a tutorial JBeam still references author-supplied placeholder meshes, LibreRally should prefer the real shared BeamNG content and treat the placeholder visuals as content gaps rather than inventing replacement art.
