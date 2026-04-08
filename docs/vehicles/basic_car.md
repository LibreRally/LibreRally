# Basic Car reference vehicle

The `basic_car` vehicle is LibreRally's reference BeamNG-style JBeam vehicle.

It exists to:

- serve as the default development vehicle
- validate LibreRally's JBeam loading pipeline against BeamNG tutorial-style content
- provide a stable fixture for future importer and parser changes

The vehicle lives in `/home/runner/work/LibreRally/LibreRally/LibreRally/Resources/BeamNG Vehicles/basic_car` and keeps the same multi-file layout used by the BeamNG basic car tutorial:

- `basic_car.jbeam`
- `body.jbeam`
- `suspension_F.jbeam`
- `suspension_R.jbeam`
- `wheels.jbeam`
- `powertrain.jbeam`
- `basic_car.pc`

LibreRally loads this vehicle through the existing pipeline:

`VehicleLoader -> PcConfigLoader -> JBeamParser -> JBeamAssembler -> VehicleDefinition -> VehiclePhysicsBuilder`

New reference or tutorial vehicles should follow the same folder structure so they can be validated through the same pipeline with minimal special handling.
