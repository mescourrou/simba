#!/bin/env python3

import simba
import json
import numpy as np
import math

import simba.simba

class Physics(simba.Physics):
    def __init__(self, config):
        self.last_time = 0.
        self.wheel_distance = 0.25,
        self.curr_state = np.array([0.,0.,0.,0.,0.])
        self.current_command = np.array([0.,0.])
        
        if "wheel_distance" in config:
            self.wheel_distance = config["wheel_distance"]
        if "initial_state" in config:
            self.curr_state = np.array(config["initial_state"])

    def record(self) -> str:
        print("This is record from python!")
        return json.dumps({
            "last_time": self.last_time,
            "state": self.curr_state.tolist(),
            "current_command": self.current_command.tolist()
            })

    def from_record(self, record: str):
        record = json.loads(record)
        print(f"Receiving record: {record}")
        self.last_time = record["last_time"]
        self.curr_state = np.array(record["state"])
        self.current_command = np.array(record["current_command"])

    def update_state(self, time): 
        dt = time - self.last_time
        assert(dt > 0.)

        theta = self.curr_state[2]

        displacement_wheel_left = self.current_command[0] * dt
        displacement_wheel_right = self.current_command[1] * dt

        translation = (displacement_wheel_left + displacement_wheel_right) / 2.
        rotation = (displacement_wheel_right - displacement_wheel_left) / self.wheel_distance

        self.last_time = time

        self.curr_state[0] += translation * math.cos(theta + rotation / 2.)
        self.curr_state[1] += translation * math.sin(theta + rotation / 2.)
        self.curr_state[2] += rotation

        self.curr_state[3] = translation / dt
        while self.curr_state[2] > np.pi:
            self.curr_state[2] -= 2*np.pi
        while self.curr_state[2] <= -np.pi:
            self.curr_state[2] += 2*np.pi
        
        
    def apply_command(self, command, time):
        self.current_command[0] = command.left_wheel_speed
        self.current_command[1] = command.right_wheel_speed
        
    def state(self, time): 
        state = simba.State()
        state.pose.x = self.curr_state[0]
        state.pose.y = self.curr_state[1]
        state.pose.theta = self.curr_state[2]
        state.velocity = self.curr_state[3]
        return state


class SimulatorAPI(simba.PluginAPI):
    def get_physic(self, config, global_config):
        config = json.loads(config)
        return Physics(config)

def main():

    simulator_api = SimulatorAPI()

    simulator = simba.Simulator.from_config(
        "config/config_physics.yaml", simulator_api
    )
    simulator.run()


if __name__ == "__main__":
    main()
