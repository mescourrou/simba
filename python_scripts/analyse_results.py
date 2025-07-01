#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
import IPython

def analyse(records: dict, config: dict, figure_path: str, figure_type: str, additionnal_param: dict|None):

    # config = data["config"]
    # record = data["record"]
    
    all_turtles_data = dict()
    
    class TurtleData():
        def __init__(self):
            self.times = list()
            self.positions = list()

    for record in records:
        t = record["time"]
        node_type = list(record["node"].keys())[0]
        turtle_name = record["node"][node_type]["name"]
        if not turtle_name in all_turtles_data:
            all_turtles_data[turtle_name] = TurtleData()
            
        turtle_data = all_turtles_data[turtle_name]
            
        if "physics" in record["node"][node_type]:
            turtle_data.times.append(t)
            try:
                perfect_physics = record["node"][node_type]["physics"]["Perfect"]
                real_pose = None
                if "states" in perfect_physics:
                    states = perfect_physics["states"]
                    for (target, state) in states:
                        if target == "self":
                            real_pose = state["pose"]
                elif "state" in perfect_physics:
                    real_pose = perfect_physics["state"]["pose"]
                if real_pose is None:
                    raise Exception()
            except: # For external Physics (in python example)
                print(record["node"][node_type]["physics"])
                real_pose = record["node"][node_type]["physics"]["External"]["state"][0:3]
            turtle_data.positions.append(real_pose)
        
    f, ax = plt.subplots()
    for turtle, data in all_turtles_data.items():
        if len(data.positions) == 0:
            continue
        real_pose_np = np.array(data.positions)
        ax.plot(real_pose_np[:,0], real_pose_np[:,1], label=f"{turtle}")

    ax.set_title("Trajectories")
    ax.legend()
    
    if figure_path != "":
        f.savefig(f"{figure_path}/Trajectories_all{figure_type}", bbox_inches='tight')
