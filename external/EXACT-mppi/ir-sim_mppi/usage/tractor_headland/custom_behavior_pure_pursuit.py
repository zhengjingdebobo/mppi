from irsim.lib import register_behavior
from irsim.util.util import WrapToPi

import numpy as np

@register_behavior("acker", "pure_pursuit")
def beh_acker_pure_pursuit(ego_object, external_objects=None, **kwargs):
    
    state = ego_object.state
    goal = ego_object.goal
    goal_threshold = ego_object.goal_threshold
    _, max_vel = ego_object.get_vel_range()
    
    wheelbase = ego_object.wheelbase
    lookahead_dist = kwargs.get("lookahead_dist", 1.0)
    
    behavior_vel = pure_pursuit(state, goal, max_vel, wheelbase, lookahead_dist)
    
    return behavior_vel
    
# def get_track_point(self):
#     possible_pt_idx = np.where(
#         self.cum_dist_at_waypoint[:]
#         - self.cum_dist_at_waypoint[self.nearest_pt_idx]
#         >= self.lookahead_dist
#     )[0]

#     if len(possible_pt_idx) == 0:
#         track_pt_idx = len(self.current_trajectory) - 1
#     else:
#         track_pt_idx = possible_pt_idx[0]
#     track_pt = self.current_trajectory[track_pt_idx, :]

#     return track_pt
    
def pure_pursuit(state, goal, max_vel, wheelbase, lookahead_dist=1.0):
    
    # self.update_nearest_point()
    # nearest_pt = get_nearest_point()
    # track_pt = get_track_point()
    
    # # actual lookahead distance
    # lookahead_dist = np.sqrt(
    #     (track_pt[0] - state[0, 0]) ** 2
    #     + (track_pt[1] - state[1, 0]) ** 2
    # )

    # # pure-pursuit angle
    # alpha = (
    #     np.arctan2(
    #         track_pt[1] - state[1, 0],
    #         track_pt[0] - state[0, 0],
    #     )
    #     - state[2, 0]
    # )
    # if self.speed_dir < 0:  # in case reverse
    #     alpha = np.pi - alpha
    # alpha = WrapToPi(alpha)

    # # steering angle
    # steer = np.arctan(2 * wheelbase / lookahead_dist * np.sin(alpha))
    # steer = np.clip(steer, -max_vel[1, 0], max_vel[1, 0])

    # # curvature
    # kappa = 2.0 * np.sin(alpha) / lookahead_dist

    # # linear speed
    # project_dist_to_end = self.get_project_dist_to_end()
    # speed = self.desired_speed

    # # angular velocity
    # omega = speed * kappa
    
    return np.array([[0.0], [0.0]])
    
    
    
    
    
    
    
    
