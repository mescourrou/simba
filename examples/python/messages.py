#!/bin/env python3

import simba
import json
import time
from typing import List, Dict

class StateEstimator(simba.StateEstimator):
    def __init__(self, config: dict, initial_time: float):
        self.last_time = initial_time
        self.period = config["period"]
        print(f"Period = {self.period}")
        self.filter_name = "anonyme"
        if "filter_name" in config:
            self.filter_name = config["filter_name"]
        self.client = None

        self._state = 0

    def state(self) -> simba.WorldState:
        world_state = simba.WorldState()
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
        print(f"Doing prediction step in {node.name()} at time {time}")
        self._state += 1
        self.last_time = time
        if node.name() == "robot1":
            if abs(time - 30) < 0.001:
                node.send_message("/simba/node/robot2/test", simba.MessageTypes.String("Bye Bye"), time, [simba.MessageFlag.Kill])
            elif time < 30:
                node.send_message("/simba/node/robot2/test", simba.MessageTypes.from_goto(simba.GoToMessage((0, 0))), time)
        else:
            node.send_message("/simba/node/robot1/test", simba.MessageTypes.String("Hello from robot2"), time)
        print(f"{node.name()}: Prediction {self._state}")

    def correction_step(self, node: simba.Node, observations: List[simba.Observation], t: float):
        print(f"Doing correction step with observations for robot {node.name()}:")
        self._state += 100
        print(f"{node.name()}: Correction {self._state}")


    def next_time_step(self):
        next_time = self.last_time + self.period
        print(f"Returning next time step from python: {next_time}")
        return next_time
    
    def pre_loop_hook(self, node: simba.Node, time: float):
        if self.client is None:
            node.make_channel("test")
            self.client = node.subscribe(["test"])
        messages = []
        msg = self.client.try_receive(time)
        while msg is not None:
            match msg[1].message:
                case simba.MessageTypes.String(s):
                    v = f"String({s})"
                case simba.MessageTypes.GoTo(g):
                    v = f"GoTo({g})"

            messages.append((msg[0], msg[1].msg_from, v, msg[1].timestamp))
            msg = self.client.try_receive(time)
        print(f"Received messages: {messages}")


class SimulatorAPI(simba.PluginAPI):
    def get_state_estimator(self, config, global_config, initial_time: float):
        config = json.loads(config)
        print(f"Config received by python: {type(config)} {config}")
        return StateEstimator(config, initial_time)

def main():

    simulator_api = SimulatorAPI()

    simulator = simba.Simulator.from_config(
            "config/config_state_estimator.yaml", simulator_api
        )
    simulator.run()


if __name__ == "__main__":
    main()
