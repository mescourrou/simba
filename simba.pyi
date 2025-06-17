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
        
class WorldState:
    def __init__(self):
        self.ego: State | None
        self.objects: Dict[State]
        self.landmarks: Dict[State]
        self.occupancy_grid: OccupancyGrid | None
        
class OccupancyGrid:
    def __init__(self, center: List[float], cell_height: float, cell_width: float, nb_rows: int, nb_cols: int):
        """Creates an Occupancy grid

        Args:
            center (Vector of 3 floats): Pose of the center (x, y, orientation)
            cell_height (float): Height of the cell (on the y axis)
            cell_width (float): Width of the cell (on the x axis)
            nb_rows (int): Number of cells vertically (on the y axis)
            nb_cols (int): Number of cells horizontally (on the x axis)
        """
        raise NotImplementedError()

    def get_idx(self, row: int, col: int) -> float | None:
        """Get the value at the given index

        Args:
            row (int): row in the grid
            col (int): column in the grid

        Returns:
            float : Value of the cell
            None : Cell not found
        """
        raise NotImplementedError()

    def set_idx(self, row: int, col: int, value: float) -> bool:
        """Set the value at the given index

        Args:
            row (int): row in the grid
            col (int): column in the grid

        Returns:
            bool : True if the cell was found and modified
        """
        raise NotImplementedError()

    def get_pos(self, position: List[float]) -> float | None:
        """Get the value at the given position

        Args:
            position (Vector of 2 floats) : Position to get

        Returns:
            float : Value of the cell
            None : Cell not found
        """
        raise NotImplementedError()

    def set_pos(self, position: List[float], value: float) -> bool:
        """Set the value at the given position

        Args:
            position (Vector of 2 floats) : Position to set
            value (float) : Value to set

        Returns:
            bool : True if the cell was found and modified
        """
        raise NotImplementedError()
        
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
    def state(self) -> WorldState:
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

    def compute_error(self, world_state: WorldState) -> ControllerError:
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