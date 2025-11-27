#!/bin/env python3

import my_rust_plugin
from my_rust_plugin import simba

from typing import Dict, List, Tuple
import json

class StateEstimator(simba.StateEstimator):
    def __init__(self, config: dict):
        self.last_time = 0
        self.period = config["period"]
        self.filter_name = "anonyme"
        if "filter_name" in config:
            self.filter_name = config["filter_name"]

        self._state = 0

    def state(self) -> simba.WorldState:
        world_state = simba.WorldState()
        world_state.ego = simba.State()
        world_state.ego.pose.x = 1
        world_state.ego.pose.y = 2
        world_state.ego.pose.theta = 0
        world_state.ego.velocity = 3
        return world_state

    def record(self) -> str:
        print("This is record from python!")
        return json.dumps({
            "last_time": self.last_time,
            "period": self.period,
            "state": self._state})

    def prediction_step(self, node: simba.Node, time: float):
        print(f"Doing prediction step in {self.filter_name}")
        self._state += 1
        self.last_time = time
        print(f"{self.filter_name}: Prediction {self._state}")

    def correction_step(self, node: simba.Node, observations: List[simba.Observation], t: float):
        print(f"Doing correction step with observations for robot {self.filter_name}:")
        for obs in observations:
            sensor_obs = obs.sensor_observation
            # Not the best interface, but it works!
            # You can also use the sensor name given in the config!
            match sensor_obs.kind:
                case "OrientedLandmark":
                    landmark=sensor_obs.as_oriented_landmark()
                    print(f"Observation of landmark {landmark.id}: {landmark.pose}")
                case "Odometry":
                    odom=sensor_obs.as_odometry()
                    print(f"Odometry: {odom}")
                case _:
                    print(f"Other: {sensor_obs.kind}")
        self._state += 100
        print(f"{self.filter_name}: Prediction {self._state}")


    def next_time_step(self):
        print("Returning next time step from python")
        return self.last_time + self.period
    
    def pre_loop_hook(self, node: simba.Node, time: float):
        pass


class SimulatorAPI(simba.PluginAPI):
    def get_state_estimator(self, config, global_config):
        config = json.loads(config)
        print(f"Config received by python: {type(config)} {config}")
        return StateEstimator(config)

def main():

    simulator_api = SimulatorAPI()

    my_rust_plugin.start(simulator_api)


if __name__ == "__main__":
    main()
