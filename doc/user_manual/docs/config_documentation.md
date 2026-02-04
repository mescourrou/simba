# Configuration file documentation
This documentation is auto-generated.

To get more information on the parameter, check the documentation (or follow the links).

List of parameters:

`version`: String
`log`: [LoggerConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/logger/struct.LoggerConfig.html)
	`included_nodes`: String, List
	`excluded_nodes`: String, List
	`log_level`: [LogLevel](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/logger/enum.LogLevel.html), Enum
		- `type`: Off  
		- `type`: Error  
		- `type`: Warn  
		- `type`: Info  
		- `type`: Debug  
		- `type`: Internal => [InternalLog](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/logger/enum.InternalLog.html), List, Enum
			- `type`: All  
			- `type`: NetworkMessages  
			- `type`: ServiceHandling  
			- `type`: SetupSteps  
			- `type`: SetupStepsDetailed  
			- `type`: SensorManager  
			- `type`: SensorManagerDetailed  
			- `type`: NodeRunning  
			- `type`: NodeRunningDetailed  
			- `type`: NodeSyncDetailed  
			- `type`: API  
			- `type`: NavigatorDetailed  
			- `type`: Scenario  
`results`: [ResultConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/simulator/results/struct.ResultConfig.html), Optional
	`result_path`: String, Optional
	`show_figures`: Boolean
	`analyse_script`: String, Optional
	`figures_path`: String, Optional
	`python_params`: User-specific struct
	`save_mode`: ResultSaveMode
`base_path`: String
`max_time`: Float
`time_analysis`: [TimeAnalysisConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/time_analysis/time_analysis_config/struct.TimeAnalysisConfig.html), Optional
	`exporter`: [ProfileExporterConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/time_analysis/exporter/enum.ProfileExporterConfig.html), Enum
		- `type`: TraceEventExporter  
	`keep_last`: Boolean
	`output_path`: String
	`analysis_unit`: String
`random_seed`: Float, Optional
`robots`: [RobotConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/node/node_factory/struct.RobotConfig.html), List
	`name`: String
	`navigator`: [NavigatorConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/navigators/mod/enum.NavigatorConfig.html), Enum
		- `type`: TrajectoryFollower => [TrajectoryFollowerConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/navigators/trajectory_follower/struct.TrajectoryFollowerConfig.html)
			`trajectory_path`: String
			`forward_distance`: Float
			`target_speed`: Float
			`stop_distance`: Float
			`stop_ramp_coefficient`: Float
		- `type`: External => [ExternalNavigatorConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/navigators/external_navigator/struct.ExternalNavigatorConfig.html)
			Insert User-specific struct
		- `type`: Python => [PythonNavigatorConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/navigators/python_navigator/struct.PythonNavigatorConfig.html)
			`file`: String
			`class_name`: String
			Insert User-specific struct
		- `type`: GoTo => [GoToConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/navigators/go_to/struct.GoToConfig.html)
			`target_point`: f32, Optional, Array[2]
			`target_speed`: Float
			`stop_distance`: Float
			`stop_ramp_coefficient`: Float
	`controller`: [ControllerConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/controllers/mod/enum.ControllerConfig.html), Enum
		- `type`: PID => [PIDConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/controllers/pid/struct.PIDConfig.html)
			`robot_model`: [RobotModelConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/physics/robot_models/mod/enum.RobotModelConfig.html), Optional, Enum
				- `type`: Unicycle => [UnicycleConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/physics/robot_models/unicycle/struct.UnicycleConfig.html)
					`wheel_distance`: Float
				- `type`: Holonomic => [HolonomicConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/physics/robot_models/holonomic/struct.HolonomicConfig.html)
					`max_longitudinal_velocity`: Float
					`max_lateral_velocity`: Float
					`max_angular_velocity`: Float
			`proportional_gains`: Float, List
			`derivative_gains`: Float, List
			`integral_gains`: Float, List
		- `type`: External => [ExternalControllerConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/controllers/external_controller/struct.ExternalControllerConfig.html)
			Insert User-specific struct
		- `type`: Python => [PythonControllerConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/controllers/python_controller/struct.PythonControllerConfig.html)
			`file`: String
			`class_name`: String
			Insert User-specific struct
	`physics`: [PhysicsConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/physics/mod/enum.PhysicsConfig.html), Enum
		- `type`: Internal => [InternalPhysicConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/physics/internal_physics/struct.InternalPhysicConfig.html)
			`model`: [RobotModelConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/physics/robot_models/mod/enum.RobotModelConfig.html), See above
			`initial_state`: [StateConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/state_estimators/mod/struct.StateConfig.html)
				`pose`: Float, List
				`velocity`: Float, List
				`random`: [RandomVariableTypeConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/utils/determinist_random_variable/enum.RandomVariableTypeConfig.html), List, Enum
					- `type`: None  
					- `type`: Fixed => [FixedRandomVariableConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/utils/distributions/fixed/struct.FixedRandomVariableConfig.html)
						`values`: Float, List
					- `type`: Uniform => [UniformRandomVariableConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/utils/distributions/uniform/struct.UniformRandomVariableConfig.html)
						`min`: Float, List
						`max`: Float, List
					- `type`: Normal => [NormalRandomVariableConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/utils/distributions/normal/struct.NormalRandomVariableConfig.html)
						`mean`: Float, List
						`covariance`: Float, List
					- `type`: Poisson => [PoissonRandomVariableConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/utils/distributions/poisson/struct.PoissonRandomVariableConfig.html)
						`lambda`: Float, List
					- `type`: Exponential => [ExponentialRandomVariableConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/utils/distributions/exponential/struct.ExponentialRandomVariableConfig.html)
						`lambda`: Float, List
				`variable_order`: String, List
			`faults`: [PhysicsFaultModelConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/physics/fault_models/fault_model/enum.PhysicsFaultModelConfig.html), List, Enum
				- `type`: AdditiveRobotCentered => [AdditiveRobotCenteredPhysicsFaultConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/physics/fault_models/additive_robot_centered/struct.AdditiveRobotCenteredPhysicsFaultConfig.html)
					`distributions`: [RandomVariableTypeConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/utils/determinist_random_variable/enum.RandomVariableTypeConfig.html), See above, List
					`variable_order`: String, List
					`proportionnal_to_velocity`: Float, Optional
		- `type`: External => [ExternalPhysicsConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/physics/external_physics/struct.ExternalPhysicsConfig.html)
			Insert User-specific struct
		- `type`: Python => [PythonPhysicsConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/physics/python_physics/struct.PythonPhysicsConfig.html)
			`file`: String
			`class_name`: String
			Insert User-specific struct
	`state_estimator`: [StateEstimatorConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/state_estimators/mod/enum.StateEstimatorConfig.html), Enum
		- `type`: Perfect => [PerfectEstimatorConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/state_estimators/perfect_estimator/struct.PerfectEstimatorConfig.html)
			`prediction_period`: Float
			`targets`: String, List
			`map_path`: String, Optional
		- `type`: External => [ExternalEstimatorConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/state_estimators/external_estimator/struct.ExternalEstimatorConfig.html)
			Insert User-specific struct
		- `type`: Python => [PythonEstimatorConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/state_estimators/python_estimator/struct.PythonEstimatorConfig.html)
			`file`: String
			`class_name`: String
			Insert User-specific struct
	`sensor_manager`: [SensorManagerConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/sensors/sensor_manager/struct.SensorManagerConfig.html)
		`sensors`: [ManagedSensorConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/sensors/sensor_manager/struct.ManagedSensorConfig.html), List
			`name`: String
			`send_to`: String, List
			`triggered`: Boolean
			`config`: [SensorConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/sensors/mod/enum.SensorConfig.html), Enum
				- `type`: OrientedLandmarkSensor => [OrientedLandmarkSensorConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/sensors/oriented_landmark_sensor/struct.OrientedLandmarkSensorConfig.html)
					`detection_distance`: Float
					`map_path`: String
					`period`: Float, Optional
					`faults`: [FaultModelConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/sensors/fault_models/fault_model/enum.FaultModelConfig.html), List, Enum
						- `type`: AdditiveRobotCentered => [AdditiveRobotCenteredFaultConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/sensors/fault_models/additive_robot_centered/struct.AdditiveRobotCenteredFaultConfig.html)
							`apparition`: [BernouilliRandomVariableConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/utils/distributions/bernouilli/struct.BernouilliRandomVariableConfig.html)
								`probability`: Float, List
							`distributions`: [RandomVariableTypeConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/utils/determinist_random_variable/enum.RandomVariableTypeConfig.html), See above, List
							`variable_order`: String, List
							`proportional_to`: String, Optional
						- `type`: AdditiveRobotCenteredPolar => [AdditiveRobotCenteredPolarFaultConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/sensors/fault_models/additive_robot_centered_polar/struct.AdditiveRobotCenteredPolarFaultConfig.html)
							`apparition`: [BernouilliRandomVariableConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/utils/distributions/bernouilli/struct.BernouilliRandomVariableConfig.html), See above
							`distributions`: [RandomVariableTypeConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/utils/determinist_random_variable/enum.RandomVariableTypeConfig.html), See above, List
							`variable_order`: String, List
						- `type`: AdditiveObservationCenteredPolar => [AdditiveObservationCenteredPolarFaultConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/sensors/fault_models/additive_observation_centered_polar/struct.AdditiveObservationCenteredPolarFaultConfig.html)
							`apparition`: [BernouilliRandomVariableConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/utils/distributions/bernouilli/struct.BernouilliRandomVariableConfig.html), See above
							`distributions`: [RandomVariableTypeConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/utils/determinist_random_variable/enum.RandomVariableTypeConfig.html), See above, List
							`variable_order`: String, List
							`proportional_to`: String, Optional
						- `type`: Clutter => [ClutterFaultConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/sensors/fault_models/clutter/struct.ClutterFaultConfig.html)
							`apparition`: [RandomVariableTypeConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/utils/determinist_random_variable/enum.RandomVariableTypeConfig.html), See above
							`distributions`: [RandomVariableTypeConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/utils/determinist_random_variable/enum.RandomVariableTypeConfig.html), See above, List
							`variable_order`: String, List
							`observation_id`: String
						- `type`: Misdetection => [MisdetectionFaultConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/sensors/fault_models/misdetection/struct.MisdetectionFaultConfig.html)
							`apparition`: [BernouilliRandomVariableConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/utils/distributions/bernouilli/struct.BernouilliRandomVariableConfig.html), See above
						- `type`: Misassociation => [MisassociationFaultConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/sensors/fault_models/misassociation/struct.MisassociationFaultConfig.html)
							`apparition`: [BernouilliRandomVariableConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/utils/distributions/bernouilli/struct.BernouilliRandomVariableConfig.html), See above
							`distribution`: [RandomVariableTypeConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/utils/determinist_random_variable/enum.RandomVariableTypeConfig.html), See above
							`sort`: [Sort](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/sensors/fault_models/misassociation/enum.Sort.html), Enum
								- `type`: None  
								- `type`: Random  
								- `type`: Distance  
							`source`: [Source](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/sensors/fault_models/misassociation/enum.Source.html), Enum
								- `type`: Map => String
								- `type`: Robots  
						- `type`: Python => [PythonFaultModelConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/sensors/fault_models/python_fault_model/struct.PythonFaultModelConfig.html)
							`file`: String
							`class_name`: String
							Insert User-specific struct
					`filters`: [SensorFilterConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/sensors/sensor_filters/mod/enum.SensorFilterConfig.html), List, Enum
						- `type`: RangeFilter => [RangeFilterConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/sensors/sensor_filters/range_filter/struct.RangeFilterConfig.html)
							`variables`: String, List
							`min_range`: Float, List
							`max_range`: Float, List
							`inside`: Boolean
						- `type`: IdFilter => [IdFilterConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/sensors/sensor_filters/id_filter/struct.IdFilterConfig.html)
							`accepted`: String, List
							`rejected`: String, List
							`priority_accept`: Boolean
						- `type`: PythonFilter => [PythonFilterConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/sensors/sensor_filters/python_filter/struct.PythonFilterConfig.html)
							`file`: String
							`class_name`: String
							Insert User-specific struct
					`xray`: Boolean
				- `type`: OdometrySensor => OdometrySensorConfig
				- `type`: SpeedSensor => [SpeedSensorConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/sensors/speed_sensor/struct.SpeedSensorConfig.html)
					`period`: Float, Optional
					`faults`: [FaultModelConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/sensors/fault_models/fault_model/enum.FaultModelConfig.html), See above, List
					`filters`: [SensorFilterConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/sensors/sensor_filters/mod/enum.SensorFilterConfig.html), See above, List
					`lie_integration`: Boolean
				- `type`: DisplacementSensor => [DisplacementSensorConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/sensors/displacement_sensor/struct.DisplacementSensorConfig.html)
					`period`: Float, Optional
					`faults`: [FaultModelConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/sensors/fault_models/fault_model/enum.FaultModelConfig.html), See above, List
					`filters`: [SensorFilterConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/sensors/sensor_filters/mod/enum.SensorFilterConfig.html), See above, List
					`lie_movement`: Boolean
				- `type`: GNSSSensor => [GNSSSensorConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/sensors/gnss_sensor/struct.GNSSSensorConfig.html)
					`period`: Float, Optional
					`faults`: [FaultModelConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/sensors/fault_models/fault_model/enum.FaultModelConfig.html), See above, List
					`filters`: [SensorFilterConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/sensors/sensor_filters/mod/enum.SensorFilterConfig.html), See above, List
				- `type`: RobotSensor => [RobotSensorConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/sensors/robot_sensor/struct.RobotSensorConfig.html)
					`detection_distance`: Float
					`period`: Float, Optional
					`faults`: [FaultModelConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/sensors/fault_models/fault_model/enum.FaultModelConfig.html), See above, List
					`filters`: [SensorFilterConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/sensors/sensor_filters/mod/enum.SensorFilterConfig.html), See above, List
				- `type`: External => [ExternalSensorConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/sensors/external_sensor/struct.ExternalSensorConfig.html)
					Insert User-specific struct
	`network`: [NetworkConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/networking/network/struct.NetworkConfig.html)
		`range`: Float
		`reception_delay`: Float
	`state_estimator_bench`: [BenchStateEstimatorConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/state_estimators/mod/struct.BenchStateEstimatorConfig.html), List
		`name`: String
		`config`: [StateEstimatorConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/state_estimators/mod/enum.StateEstimatorConfig.html), See above
	`autospawn`: Boolean
	`labels`: String, List
`computation_units`: [ComputationUnitConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/node/node_factory/struct.ComputationUnitConfig.html), List
	`name`: String
	`network`: [NetworkConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/networking/network/struct.NetworkConfig.html), See above
	`state_estimators`: [BenchStateEstimatorConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/state_estimators/mod/struct.BenchStateEstimatorConfig.html), See above, List
	`labels`: String, List
`scenario`: [ScenarioConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/scenario/config/struct.ScenarioConfig.html)
	`events`: [EventConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/scenario/config/struct.EventConfig.html), List
		`triggering_nodes`: String, List
		`trigger`: [EventTriggerConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/scenario/config/enum.EventTriggerConfig.html), Enum
			- `type`: Time => [TimeEventTriggerConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/scenario/config/struct.TimeEventTriggerConfig.html)
				`time`: [NumberConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/config/enum.NumberConfig.html), Enum
					- `type`: Num => Float
					- `type`: Rand => [RandomVariableTypeConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/utils/determinist_random_variable/enum.RandomVariableTypeConfig.html), See above
				`occurences`: [NumberConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/config/enum.NumberConfig.html), See above
			- `type`: Proximity => [ProximityEventTriggerConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/scenario/config/struct.ProximityEventTriggerConfig.html)
				`protected_target`: String, Optional
				`distance`: Float
				`inside`: Boolean
			- `type`: Area => [AreaEventTriggerConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/scenario/config/enum.AreaEventTriggerConfig.html), Enum
				- `type`: Rect => [RectAreaEventTriggerConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/scenario/config/struct.RectAreaEventTriggerConfig.html)
					`bottom_left`: )
					`top_right`: )
					`inside`: Boolean
				- `type`: Circle => [CircleAreaEventTriggerConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/scenario/config/struct.CircleAreaEventTriggerConfig.html)
					`center`: )
					`radius`: Float
					`inside`: Boolean
		`event_type`: [EventTypeConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/scenario/config/enum.EventTypeConfig.html), Enum
			- `type`: Spawn => [SpawnEventConfig](https://homepages.laas.fr/mescourrou/Recherche/Logiciels/multi-robot-simulator/rust/simba/scenario/config/struct.SpawnEventConfig.html)
				`model_name`: String
				`node_name`: String
			- `type`: Kill => String
