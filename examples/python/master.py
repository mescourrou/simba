#!/bin/env python3

import simba
import json
import time
from typing import List, Dict

class StateEstimator(simba.StateEstimator):
    def __init__(self, config: dict, initial_time: float):
        self.last_time = initial_time

    def post_init(self, node: simba.Node) -> None:
        node.make_channel("my_channel")
        self.client = node.subscribe(["/simba/command/robot1"])

    def state(self) -> simba.WorldState:
        return simba.WorldState()

    def record(self) -> str:
        return "{}"

    def prediction_step(self, node: simba.Node, command: simba.Command, time: float):
        pass

    def correction_step(self, node: simba.Node, observations: List[simba.Observation], time: float):
        pass


    def next_time_step(self):
        return 10000000000
    
    def pre_loop_hook(self, node: simba.Node, time: float):
        print("Master")
        if abs(5 - time) < 0.001:
            print("Try to send message")
            msg = simba.MessageTypes.GoTo(simba.GoToMessage([10,10]))
            self.client.send("/simba/command/robot1", msg, time)
            print("Message sent")

        if abs(20 - time) < 0.001:
            print("Try to send message")
            msg = simba.MessageTypes.GoTo(simba.GoToMessage([-10,10]))
            self.client.send("/simba/command/robot1", msg, time)
            print("Message sent")

        if abs(30 - time) < 0.001:
            print("Try to send message")
            msg = simba.MessageTypes.GoTo(simba.GoToMessage([-10,-10]))
            self.client.send("/simba/command/robot1", msg, time)
            print("Message sent")


class SimulatorAPI(simba.PluginAPI):
    def get_state_estimator(self, config, global_config, initial_time: float):
        config = json.loads(config)
        print(f"Config received by python: {type(config)} {config}")
        return StateEstimator(config, initial_time)

def main():

    simulator_api = SimulatorAPI()

    simulator = simba.Simulator.from_config(
            "config/config_master.yaml", simulator_api
        )
    simulator.run()


if __name__ == "__main__":
    main()
