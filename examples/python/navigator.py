#!/bin/env python3

import simba
import json
import numpy as np
import math

class Navigator(simba.Navigator):
    def __init__(self, config: dict, initial_time: float):
        if "radius" in config:
            self.radius = config["radius"]
        else:
            self.radius = 10
        if "velocity" in config:
            self.velocity = config["velocity"]
        else:
            self.velocity = 10

    def record(self) -> str:
        print("This is record from python!")
        return json.dumps({})

    def compute_error(self, node: simba.Node, world_state: simba.WorldState) -> simba.ControllerError:
        error = simba.ControllerError()
        v = world_state.ego.velocity.x
        x = world_state.ego.pose.x
        y = world_state.ego.pose.y
        theta = world_state.ego.pose.theta
        
        # Very bad error computation, but that's an example
        pose = np.array([x, y, theta])
        r = np.linalg.norm(pose[:2])
        error.lateral = self.radius - r
        targetted_theta = math.atan2(y, x) + np.pi/2
        error.theta = ((targetted_theta - theta) + np.pi) % (2*np.pi)
        error.velocity = self.velocity - v
        return error

    def pre_loop_hook(self, node: simba.Node, time: float):
        pass
    

class SimulatorAPI(simba.PluginAPI):
    def get_navigator(self, config: dict, global_config: dict, initial_time: float):
        config = json.loads(config)
        print(f"Config received by python: {type(config)} {config}")
        return Navigator(config, initial_time)

def main():

    simulator_api = SimulatorAPI()

    simulator = simba.Simulator.from_config(
        "config/config_navigator.yaml", simulator_api
    )
    simulator.run()


if __name__ == "__main__":
    main()
