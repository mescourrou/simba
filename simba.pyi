from typing import Dict, List, Tuple
from enum import Enum
# from warnings import deprecated # Available in python 3.13

class Pose:
    def __init__(self):
        self.x: float
        self.y: float
        self.theta: float

class Vec2:
    def __init__(self):
        self.x: float
        self.y: float
        
class Vec3:
    def __init__(self):
        self.x: float
        self.y: float
        self.z: float

class State:
    def __init__(self):
        self.pose: Pose
        self.velocity: Vec3  # (longitudinal_velocity, lateral_velocity, angular_velocity)
        
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
        self.labels: List[str]
        self.pose: Pose
        self.applied_faults: str """ Applied faults in JSON format """
        self.width: float
        self.height: float

class SpeedObservation:
    def __init__(self):
        self.linear_velocity: float
        self.angular_velocity: float
        self.applied_faults: str """ Applied faults in JSON format """

class DisplacementObservation:
    def __init__(self):
        self.translation: Vec2
        self.rotation: float
        self.applied_faults: str """ Applied faults in JSON format """

class GNSSObservation: 
    def __init__(self):
        self.pose: Vec3
        self.velocity: Vec2
        self.applied_faults: str """ Applied faults in JSON format """

class OrientedRobotObservation:
    def __init__(self):
        self.name: str
        self.labels: List[str]
        self.pose: Pose
        self.applied_faults: str """ Applied faults in JSON format """

class SensorObservation(Enum):
    OrientedLandmark: OrientedLandmarkObservation
    Speed: SpeedObservation
    GNSS: GNSSObservation
    OrientedRobot: OrientedRobotObservation
    
    def __init__(self):
        self.kind: str

    def as_oriented_landmark(self) -> OrientedLandmarkObservation | None:
        raise NotImplementedError()
    
    def as_speed(self) -> SpeedObservation | None:
        raise NotImplementedError()
    
    def as_gnss(self) -> GNSSObservation | None:
        raise NotImplementedError()
    
    def as_oriented_robot(self) -> OrientedRobotObservation | None:
        raise NotImplementedError()
    
class Observation:
    def __init__(self):
        self.sensor_name: str
        self.observer: str
        self.time: float
        self.sensor_observation: SensorObservation


class Command(Enum):
    Unicycle: UnicycleCommand
    Holonomic: HolonomicCommand

    def __init__(self):
        self.kind: str

    def as_unicycle_command(self) -> UnicycleCommand | None:
        raise NotImplementedError()

    def as_holonomic_command(self) -> HolonomicCommand | None: 
        raise NotImplementedError()

    def from_unicycle_command(cmd: UnicycleCommand) -> Command:
        raise NotImplementedError()

    def from_holonomic_command(cmd: HolonomicCommand) -> Command:
        raise NotImplementedError()

class UnicycleCommand:
    def __init__(self):
        self.left_wheel_speed: float
        self.right_wheel_speed: float


class HolonomicCommand:
    def __init__(self):
        self.longitudinal_velocity: float
        self.lateral_velocity: float
        self.angular_velocity: float


class GoToMessage:
    def __init__(self, target:Tuple[float, float]|None=None):
        self.target_point: Tuple[float, float]|None

class SensorTriggerMessage:
    def __init__(self):
        pass
        
class MessageFlag(Enum):
    # God mode, messages are instaneous.
    God = 1
    # Ask to unsubscribe
    Unsubscribe = 2
    # Ask to kill the receiving node
    Kill = 3

class MessageTypes(Enum):
    String: str
    GoTo: GoToMessage
    SensorTrigger: SensorTriggerMessage

    def __init__(self):
        self.kind: str

    def as_goto(self) -> GoToMessage | None:
        raise NotImplementedError()

    def as_sensor_trigger(self) -> SensorTriggerMessage | None: 
        raise NotImplementedError()

    def from_goto(cmd: GoToMessage) -> MessageTypes:
        raise NotImplementedError()

    def from_sensor_trigger(cmd: SensorTriggerMessage) -> MessageTypes:
        raise NotImplementedError()

class Envelope:
    def __init__(self):
        self.msg_from: str
        self.message: MessageTypes
        self.timestamp: float

class Node:
    def name(self):
        raise NotImplementedError()
    
    def send_message(self, to: str, message: MessageTypes, time: float, flags: List[MessageFlag]=[]):
        """Send a message to the given channel

        Args:
            to (str): Channel to send the message to. If relative (does not contain '/'), the root is /simba/node/{node_name}/
            message (MessageTypes): Message to send.
            time (float): Timestamp to send
            flags (List[MessageFlag], optional): Flags for the message (eg. Kill). Defaults to [].
        """
        raise NotImplementedError()
    
    def subscribe(self, topics: List[str]) -> Client:
        raise NotImplementedError()
    
    def make_channel(self, topic: str) -> None:
        raise NotImplementedError()
    
class Client:
    def subscribe(self, key: str) -> None:
        raise NotImplementedError()

    def subscribe_instantaneous(self, key: str) -> None:
        raise NotImplementedError()

    def send(self, to: str, message: MessageTypes, time: float, flags: List[MessageFlag] = []) -> None:
        raise NotImplementedError()

    def try_receive(self, time: float) -> Tuple[str, Envelope] | None:
        raise NotImplementedError()

    def next_message_time(self) -> float | None:
        raise NotImplementedError()

    def subscribed_keys(self) -> List[str]:
        raise NotImplementedError()

    def node_id(self) -> str:
        raise NotImplementedError()

class StateEstimator:
    def __init__(self, config: dict, initial_time: float):
        raise NotImplementedError()
    
    def post_init(self, node: Node) -> None:
        pass
    
    def state(self) -> WorldState:
        raise NotImplementedError()
    
    def record(self) -> str:
        raise NotImplementedError()

    def prediction_step(self, node: Node, command: Command, time: float):
        raise NotImplementedError()

    def correction_step(self, node: Node, observations: List[Observation], time: float):
        raise NotImplementedError()

    def next_time_step(self) -> float:
        raise NotImplementedError()
    
    def pre_loop_hook(self, node: Node, time: float):
        raise NotImplementedError()
    
class Controller:
    def post_init(self, node: Node) -> None:
        pass

    def record(self) -> str:
        raise NotImplementedError()

    def make_command(self, node: Node, error: ControllerError, time: float) -> Command:
        raise NotImplementedError()
    
    def pre_loop_hook(self, node: Node, time: float):
        raise NotImplementedError()
    
    def next_time_step(self) -> float|None:
        pass
    
class Navigator:
    def post_init(self, node: Node) -> None:
        pass

    def record(self) -> str:
        raise NotImplementedError()

    def compute_error(self, node: Node, world_state: WorldState) -> ControllerError:
        raise NotImplementedError()
    
    def pre_loop_hook(self, node: Node, time: float):
        raise NotImplementedError()

    def next_time_step(self) -> float|None:
        pass
        
class Physics:
    def post_init(self, node: Node) -> None:
        pass

    def record(self) -> str:
        raise NotImplementedError()

    def update_state(self, time: float): 
        raise NotImplementedError()
        
    def apply_command(self, command: Command, time: float):
        raise NotImplementedError()
        
    def state(self, time: float) -> State: 
        raise NotImplementedError()
    
    def next_time_step(self) -> float|None:
        pass

class PluginAPI:
    def get_state_estimator(self, config: Dict, global_config: Dict, initial_time: float) -> StateEstimator:
        raise NotImplementedError()
    
    def get_controller(self, config: Dict, global_config: Dict, initial_time: float) -> Controller:
        raise NotImplementedError()
    
    def get_navigator(self, config: Dict, global_config: Dict, initial_time: float) -> Navigator:
        raise NotImplementedError()
    
    def get_physics(self, config: Dict, global_config: Dict, initial_time: float) -> Physics:
        raise NotImplementedError()
    
class Simulator:
    def from_config(config_path: str, plugin_api: PluginAPI | None, loglevel: str = "off") -> Simulator:
        raise NotImplementedError()
    
    def run(self):
        raise NotImplementedError()
    
def run_gui(plugin_api: PluginAPI | None):
        raise NotImplementedError()


class FaultModel:
    def post_init(self, node: Node) -> None:
        pass

    def add_faults(self, time: float, seed: float, obs_list: List[SensorObservation]) -> List[SensorObservation]:
        raise NotImplementedError()

class PhysicsFaultModel:
    def post_init(self, node: Node) -> None:
        pass

    def add_faults(self, time: float, state: State) -> State:
        raise NotImplementedError()

class SensorFilter:
    def post_init(self, node: Node) -> None:
        pass
    
    def filter_observations(self, time: float, observation: SensorObservation, observer_state: State, observee_state: State | None) -> SensorObservation|None:
        raise NotImplementedError()