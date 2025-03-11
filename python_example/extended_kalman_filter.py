#!/bin/env python3

import simba
import json
from simba import state_estimators

class StateEstimator:
    def __init__(self, config):
        self.last_time = 0
        self.period = config["period"]

    def state(self):
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

    def correction_step(self, observations, time):
        print("Doing correction step with observations for robot {robot.name}:")
        for obs in observations:
            # Not the best interface, but it works!
            match obs:
                case state_estimators.Observation.OrientedLandmark():
                    print(f"Observation of landmark {obs[0].id}: {obs[0].pose}")
                case state_estimators.Observation.Odometry():
                    print(f"Odometry: {obs[0]}")
                case _:
                    print("Other")


    def next_time_step(self):
        print("Returning next time step from python")
        return self.last_time + self.period


class SimulatorAPI:
    def get_state_estimator(self, config, config_path):
        config = json.loads(config)
        print(f"Config received by python: {type(config)} {config}")
        return StateEstimator(config)

def main():

    simulator_api = simba.PythonAPI(SimulatorAPI())

    simulator = simba.Simulator.from_config(
        "config/config.yaml", simulator_api, loglevel="info"
    )
    simulator.run(simulator_api)


if __name__ == "__main__":
    main()
