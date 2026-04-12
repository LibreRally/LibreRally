# Basic Car reference vehicle

The `basic_car` vehicle is LibreRally's reference BeamNG-style JBeam vehicle.

It exists to:

- serve as the default development vehicle
- validate LibreRally's JBeam loading pipeline against BeamNG tutorial-style content
- provide a stable fixture for future importer and parser changes

The vehicle lives in `LibreRally/Resources/BeamNG Vehicles/basic_car` and now vendors the BeamNG basic car tutorial's Formula Bee asset set directly:

- `FormulaBeeModel.dae`
- `TutoFormulaBee.jbeam`
- `TutoFormulaBee_*.jbeam`
- `main.materials.json`
- `basic_car.pc`

`basic_car.pc` is LibreRally's compatibility layer over the tutorial assets: it selects the active `TutoFormulaBee` slot tree while preserving the tuning vars LibreRally's current BEPU drivetrain and suspension code reads at runtime.

The tutorial references some shared BeamNG common wheel and texture assets that are not bundled in this repository. LibreRally therefore uses the vendored `FormulaBeeModel.dae` plus its existing fallback tyre and neutral material behavior where those shared assets are absent.

LibreRally loads this vehicle through the existing pipeline:

`VehicleLoader -> PcConfigLoader -> JBeamParser -> JBeamAssembler -> VehicleDefinition -> VehiclePhysicsBuilder`

New reference or tutorial vehicles should follow the same folder structure so they can be validated through the same pipeline with minimal special handling.
