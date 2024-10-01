#!/bin/env python3

import turtlebot_simulator
import json


class StateEstimator:
    def __init__(self, config):
        self.last_time = 0
        self.period = config["period"]

    def state(self, time):
        return [1, 2, 3]

    def record(self) -> str:
        print("This is record from python!")
        return json.dumps({
            "last_time": self.last_time,
            "period": self.period})

    def from_record(self, record: str):
        record = json.loads(record)
        print(f"Receiving record: {record}")
        self.last_time = record["last_time"]
        self.period = record["period"]

    def prediction_step(self, time):
        print("Doing prediction step")
        self.last_time = time

    def correction_step(self, time):
        print("Doing correction step")

    def next_time_step(self):
        print("Returning next time step from python")
        return self.last_time + self.period


class SimulatorAPI:
    def get_state_estimator(self, config, config_path):
        config = json.loads(config)
        print(f"Config received by python: {type(config)} {config}")
        return StateEstimator(config)

def main():

    simulator_api = turtlebot_simulator.PythonAPI(SimulatorAPI())

    simulator = turtlebot_simulator.Simulator.from_config(
        "config/config.yaml", simulator_api, loglevel="debug"
    )
    simulator.show()
    simulator.run(60.0)


if __name__ == "__main__":
    main()
