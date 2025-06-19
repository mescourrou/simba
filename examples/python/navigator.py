#!/bin/env python3

import simba
import json
import numpy as np
import math

class Navigator(simba.Navigator):
    def __init__(self, config):
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

    def from_record(self, record: str):
        record = json.loads(record)
        print(f"Receiving record: {record}")

    def compute_error(self, world_state: simba.WorldState) -> simba.ControllerError:
        error = simba.ControllerError()
        v = world_state.ego.velocity
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


class SimulatorAPI(simba.PluginAPI):
    def get_navigator(self, config, global_config):
        config = json.loads(config)
        print(f"Config received by python: {type(config)} {config}")
        return Navigator(config)

def main():

    simulator_api = SimulatorAPI()

    simulator = simba.Simulator.from_config(
        "config/config_navigator.yaml", simulator_api
    )
    simulator.run()


if __name__ == "__main__":
    main()
