#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt

def analyse(records: dict, config: dict, figure_path: str, figure_type: str):

    # config = data["config"]
    # record = data["record"]
    
    all_turtles_data = dict()
    
    class TurtleData():
        def __init__(self):
            self.times = list()
            self.positions = list()

    for record in records:
        t = record["time"]
        turtle_name = record["robot"]["name"]
        if not turtle_name in all_turtles_data:
            all_turtles_data[turtle_name] = TurtleData()
            
        turtle_data = all_turtles_data[turtle_name]
            
        turtle_data.times.append(t)
        real_pose = record["robot"]["physic"]["Perfect"]["state"]["pose"]
        turtle_data.positions.append(real_pose)
        
    f, ax = plt.subplots()
    for turtle, data in all_turtles_data.items():
        real_pose_np = np.array(data.positions)
        ax.plot(real_pose_np[:,0], real_pose_np[:,1], label=f"{turtle}")

    ax.set_title("Trajectories")
    ax.legend()
    
    if figure_path != "":
        f.savefig(f"{figure_path}/Trajectories_all{figure_type}", bbox_inches='tight')
