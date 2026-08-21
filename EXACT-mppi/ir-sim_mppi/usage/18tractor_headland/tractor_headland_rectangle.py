import irsim

from irsim.lib.path_planners.a_star import AStarPlanner
from irsim.lib.path_planners.probabilistic_road_map import PRMPlanner
from irsim.lib.path_planners.rrt import RRT
from irsim.lib.path_planners.rrt_star import RRTStar

import numpy as np

env = irsim.make(save_ani=False, full=False)

env_map = env.get_map()
# planner = RRTStar(
#     env_map,
#     robot_radius=0.5,
#     expand_dis=1.5,
#     path_resolution=0.25,
#     goal_sample_rate=5,
#     max_iter=1000,
#     connect_circle_dist=0.5,
#     search_until_max_iter=False,
# )


planner = AStarPlanner(
    env_map,
    resolution=0.2,
)

robot_info = env.get_robot_info()
robot_state = env.get_robot_state()
trajectory = planner.planning(robot_state, robot_info.goal)
trajectory = np.flip(trajectory, axis=1)
trajectory = np.concatenate([robot_state[:2], trajectory, robot_info.goal[:2]], axis=1)

if trajectory is not None:
    env.draw_trajectory(trajectory, traj_type="r-")
    print("Trajectory found")
else:
    print("No trajectory found")
    
    
# env.load_behavior("custom_behavior_pure_pursuit")
    
for _i in range(1000):
    env.step()
    env.render()

    if env.done():
        break

env.end(5)