from typing import Dict, List
from enum import Enum

class Pose:
    def __init__(self):
        self.x: float
        self.y: float
        self.theta: float

class State:
    def __init__(self):
        self.pose: Pose
        self.velocity: float
        
class ControllerError:
    def __init__(self):
        self.lateral: float
        self.theta: float
        self.velocity: float
        

class OrientedLandmarkObservation:
    def __init__(self):
        self.id: int
        self.pose: Pose

class OdometryObservation:
    def __init__(self):
        self.linear_velocity: float
        self.angular_velocity: float

class GNSSObservation: 
    def __init__(self):
        self.position: List[float]
        self.velocity: List[float]

class OrientedRobotObservation:
    def __init__(self):
        self.name: str
        self.pose: Pose

class SensorObservation(Enum):
    OrientedLandmark: OrientedLandmarkObservation
    Odometry: OdometryObservation
    GNSS: GNSSObservation
    OrientedRobot: OrientedRobotObservation
    
class Observation:
    def __init__(self):
        self.sensor_name: str
        self.observer: str
        self.time: float
        self.sensor_observation: SensorObservation

class Command:
    def __init__(self):
        self.left_wheel_speed: float
        self.right_wheel_speed: float
        

class StateEstimator:
    def state(self) -> State:
        raise NotImplementedError()
    
    def record(self) -> str:
        raise NotImplementedError()

    def from_record(self, record: str):
        raise NotImplementedError()

    def prediction_step(self, time: float):
        raise NotImplementedError()

    def correction_step(self, observations: List[Observation], time: float):
        raise NotImplementedError()

    def next_time_step(self) -> float:
        raise NotImplementedError()
    
class Controller:
    def record(self) -> str:
        raise NotImplementedError()

    def from_record(self, record: str):
        raise NotImplementedError()

    def make_command(self, error: ControllerError, time: float) -> Command:
        raise NotImplementedError()
    
class Navigator:
    def record(self) -> str:
        raise NotImplementedError()

    def from_record(self, record: str):
        raise NotImplementedError()

    def compute_error(self, state: State) -> ControllerError:
        raise NotImplementedError()
        
class Physics:
    def record(self) -> str:
        raise NotImplementedError()

    def from_record(self, record: str):
        raise NotImplementedError()

    def update_state(self, time): 
        raise NotImplementedError()
        
    def apply_command(self, command: Command, time):
        raise NotImplementedError()
        
    def state(self, time) -> State: 
        raise NotImplementedError()

class PluginAPI:
    def get_state_estimator(self, config: Dict, global_config: Dict) -> StateEstimator:
        raise NotImplementedError()
    
    def get_controller(self, config: Dict, global_config: Dict) -> Controller:
        raise NotImplementedError()
    
    def get_navigator(self, config: Dict, global_config: Dict) -> Navigator:
        raise NotImplementedError()
    
    def get_physic(self, config: Dict, global_config: Dict) -> Physics:
        raise NotImplementedError()
    
class Simulator:
    def from_config(config_path: str, plugin_api: PluginAPI | None, loglevel: str = "off") -> Simulator:
        raise NotImplementedError()
    
    def run(self):
        raise NotImplementedError()