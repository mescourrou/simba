#!/usr/bin/python3

import argparse
import json

import numpy as np
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser(
                    prog='Turtlebot Simulator Result Analyser',
                    description='Analyse Turtlebot Simulator output')

parser.add_argument('filename') 

args = parser.parse_args()

with open(args.filename) as file:
    data = json.load(file)
    
class TurtleData:
    def __init__(self) -> None:
        self.times = list()
        self.errors = list()
        self.estimated_poses = list()
        self.real_poses = list()

all_turtles_data = dict()

for row in data:
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
    else:
        raise Exception(f"State Estimator module not known: {state_estimator}")
    

    position_error = np.linalg.norm(real_pose[0:2] - estimated_pose[0:2])
    
    turtle_data.errors.append(position_error)
    
    
    
plt.figure()

for turtle, data in all_turtles_data.items():
    plt.plot(data.times, data.errors, label=turtle)

plt.title("Localization errors")
plt.legend()

plt.figure()
for turtle, data in all_turtles_data.items():
    estimated_pose_np = np.array(data.estimated_poses)
    plt.plot(estimated_pose_np[:,0], estimated_pose_np[:,1], label=f"{turtle} - Estimated")
    real_pose_np = np.array(data.real_poses)
    plt.plot(real_pose_np[:,0], real_pose_np[:,1], label=f"{turtle} - Real")

plt.title("Trajectories")
plt.legend()

plt.show()