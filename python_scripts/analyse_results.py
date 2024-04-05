#!/usr/bin/python3


import json

import numpy as np
import matplotlib.pyplot as plt

def analyse(result_filename: str, show_figures: bool, figure_path: str, figure_type: str):

    with open(result_filename) as file:
        data = json.load(file)
        
    class TurtleData:
        def __init__(self) -> None:
            self.times = list()
            self.errors = list()
            self.estimated_poses = list()
            self.real_poses = list()

    all_turtles_data = dict()

    config = data["config"]
    record = data["record"]

    for row in record:
        turtle = row["turtle"]["name"]
        if not turtle in all_turtles_data:
            all_turtles_data[turtle] = TurtleData()
            
        turtle_data = all_turtles_data[turtle]
            
        turtle_data.times.append(row["time"])
        
        physics = row["turtle"]["physic"]
        if "Perfect" in physics:
            real_pose = np.array(physics["Perfect"]["state"]["pose"])
            turtle_data.real_poses.append(real_pose)
        else:
            raise Exception(f"Physics module not known: {physics}")
        
        state_estimator = row["turtle"]["state_estimator"]
        if "Perfect" in state_estimator:
            estimated_pose = np.array(state_estimator["Perfect"]["state"]["pose"])
            turtle_data.estimated_poses.append(estimated_pose)
        elif "External" in state_estimator:
            estimated_pose = np.array(state_estimator["External"]["state"]["pose"])
            turtle_data.estimated_poses.append(estimated_pose)
        else:
            raise Exception(f"State Estimator module not known: {state_estimator}")
        

        position_error = np.linalg.norm(real_pose[0:2] - estimated_pose[0:2])
        
        turtle_data.errors.append(position_error)
        
        
        
    f, ax = plt.subplots()

    for turtle, data in all_turtles_data.items():
        ax.plot(data.times, data.errors, label=turtle)

    ax.set_title("Localization errors")
    ax.legend()
    if figure_path != "":
        f.savefig(f"{figure_path}/LocalizationError_all{figure_type}", bbox_inches='tight')

    f, ax = plt.subplots()
    for turtle, data in all_turtles_data.items():
        estimated_pose_np = np.array(data.estimated_poses)
        ax.plot(estimated_pose_np[:,0], estimated_pose_np[:,1], label=f"{turtle} - Estimated")
        real_pose_np = np.array(data.real_poses)
        ax.plot(real_pose_np[:,0], real_pose_np[:,1], label=f"{turtle} - Real")

    ax.set_title("Trajectories")
    ax.legend()
    
    if figure_path != "":
        f.savefig(f"{figure_path}/Trajectories_all{figure_type}", bbox_inches='tight')

    if show_figures:
        plt.show()

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(
                        prog='Turtlebot Simulator Result Analyser',
                        description='Analyse Turtlebot Simulator output')

    parser.add_argument('filename') 

    args = parser.parse_args()
    
    analyse(args.filename, True, ".", ".pdf")