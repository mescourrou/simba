# Configuration file
The structure of the configuration file is explained below. The config of the file is composed of the following attributes.

- [`log`](20_log.md): Defines the logging behavior.
- [`results`](30_results.md): Defines how results are computed, after the simulation.
- `max_time`: Simulation end time, in seconds. Must be positive.
- [`time_analysis`](40_time_analysis.md): Defines the generation of time performance data.
- `random_seed`: Optional seed to have a deterministic behavior. If no seed is given, a random one is chosen.
- [`robots`](10_robots.md): List of robots.
- [`computation_units`](11_computation_units.md): List of computation units.
- [`scenario`](12_scenario.md): Events configuration.

**Note**: The GUI version of SiMBA embed a UI tool to make the configuration. Click on the "Configurator" button at the top of the window.
You can also use the autocompletion in VSCode. Add `# yaml-language-server: $schema=path/to/your/config.schema.json` at the top of your
config file. The schema file can be found in the release or in the repository.