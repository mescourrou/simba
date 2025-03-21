#!/bin/env python3

import simba
import json

class Controller(simba.Controller):
    def __init__(self, config):
        self.last_time = 0
        if "speed" in config:
            self.speed = config["speed"]
        else:
            self.speed = 2

    def record(self) -> str:
        print("This is record from python!")
        return json.dumps({
            "last_time": self.last_time})

    def from_record(self, record: str):
        record = json.loads(record)
        print(f"Receiving record: {record}")
        self.last_time = record["last_time"]

    def make_command(self, error, time) -> simba.Command:
        self.last_time = time
        command = simba.Command()
        command.left_wheel_speed = self.speed
        command.right_wheel_speed = self.speed
        return command


class SimulatorAPI(simba.PluginAPI):
    def get_controller(self, config, global_config):
        config = json.loads(config)
        print(f"Config received by python: {type(config)} {config}")
        return Controller(config)

def main():

    simulator_api = SimulatorAPI()

    simulator = simba.Simulator.from_config(
        "config/config_controller.yaml", simulator_api, loglevel="info"
    )
    simulator.run()


if __name__ == "__main__":
    main()
