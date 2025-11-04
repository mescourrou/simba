#!/bin/env python3

import simba
import json

class Controller(simba.Controller):
    def __init__(self, config: dict):
        self.last_time = 0
        if "speed" in config:
            self.speed = config["speed"]
        else:
            self.speed = 2

    def record(self) -> str:
        print("This is record from python!")
        return json.dumps({
            "last_time": self.last_time})

    def make_command(self, node: simba.Node, error: simba.ControllerError, time: float) -> simba.Command:
        self.last_time = time
        command = simba.Command()
        command.left_wheel_speed = self.speed
        command.right_wheel_speed = self.speed
        return command

    def pre_loop_hook(self, node: simba.Node, time: float):
        pass


class SimulatorAPI(simba.PluginAPI):
    def get_controller(self, config: dict, global_config: dict):
        config = json.loads(config)
        print(f"Config received by python: {type(config)} {config}")
        return Controller(config)

def main():

    simulator_api = SimulatorAPI()

    simba.run_gui(simulator_api)

if __name__ == "__main__":
    main()
