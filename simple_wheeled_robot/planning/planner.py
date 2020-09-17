from .rrt_star_reeds_shepp import RRTStarReedsShepp
from typing import Optional, Tuple, Dict, List
import numpy as np


class Planner(object):
    def __init__(self):
        self.__q_goal = None  # type: Optional[Tuple[float, float, float]]
        self.__q_start = None  # type: Optional[Tuple[float, float, float]]
        self.__planner = None  # type: Optional[RRTStarReedsShepp]
        self.__velocity_vector = {"left_wheel": [], "right_wheel": []}  # type: Optional[Dict[str, List[float]]]
        self.__trajectories = None  # type: Optional[List[List[float]]]

    def init_plan(self, q_start, q_goal):
        self.__q_start = q_start
        self.__q_goal = q_goal
        self.__trajectories = []
        
    def plan_path(self):
        D = 162  # Robot width in mm
        velocity = 400
        omega = 2*velocity/D
        alpha = self.__q_start[2] + np.deg2rad(90.0)
        threshold = 1e-3
        # dt = 0.1

        # Trajectory one
        if np.fabs(self.__q_goal[0] - self.__q_start[0]) > 0:
            enumerator = self.__q_goal[1] - self.__q_start[1]
            denominator = self.__q_goal[0] - self.__q_start[0]
            if enumerator > 0 and denominator > 0:
                alpha = np.arctan(enumerator / denominator)
            elif enumerator < 0 and denominator > 0:
                alpha = - np.arctan(np.fabs(enumerator) / denominator)
            elif enumerator > 0 and denominator < 0:
                alpha = np.deg2rad(180.0) - np.arctan(enumerator / np.fabs(denominator))
            elif enumerator < 0 and denominator < 0:
                alpha = np.deg2rad(180.0) + np.arctan(np.fabs(enumerator) / np.fabs(denominator))
            delta_theta = alpha - self.__q_start[2] - np.deg2rad(90.0)
            rotation_time = delta_theta/omega
            if np.fabs(rotation_time) > threshold:
                vl = -omega*D/2
                vr = omega*D/2
                if rotation_time < 0:
                    vl = -vl
                    vr = -vr
                    rotation_time = -rotation_time
                self.__trajectories.append([vl, vr, rotation_time])
        elif self.__q_goal[1] - self.__q_start[1] < 0:
            alpha = np.deg2rad(-90)
            delta_theta = alpha - self.__q_start[2] - np.deg2rad(90.0)
            print(delta_theta)
            rotation_time = delta_theta/omega
            vl = -omega * D / 2
            vr = omega * D / 2
            self.__trajectories.append([vl, vr, np.fabs(rotation_time)])

        # Trajectory two
        displacement = np.sqrt((self.__q_goal[1] - self.__q_start[1])**2 + (self.__q_goal[0] - self.__q_start[0])**2)
        translation_time = displacement/velocity
        if np.fabs(translation_time) > threshold:
            vl = vr = velocity
            if translation_time < 0:
                vl = -vl
                vr = -vr
                translation_time = -translation_time
            self.__trajectories.append([vl, vr, translation_time])
        
        # Trajectory three
        delta_theta = self.__q_goal[2] - alpha + np.deg2rad(90.0)
        rotation_time = delta_theta/omega
        if np.fabs(rotation_time) > threshold:
            vl = -omega*D/2
            vr = omega*D/2
            if rotation_time < 0:
                vl = -vl
                vr = -vr
                rotation_time = -rotation_time
            self.__trajectories.append([vl, vr, rotation_time])

        return self.__trajectories
        # threshold = 1e-3
        # path = self.__planner.planning(animation=False, search_until_max_iter=False)
        # if not path:
        #     raise Exception("Planning Error: No feasible path found.")
        # # print(path)
        # self.__path = path
        # index = len(path) - 1
        # print(path)
        # path.append(self.__q_start)
        # while index > 0:
        #     omega = (path[index-1][2] - path[index][2]) / dt
        #     if np.cos(path[index-1][2] + np.deg2rad(90.0)) > threshold:
        #         velocity_right = (path[index - 1][1] - path[index][1]) / (dt * np.cos(path[index - 1][2] +
        #                                                                               np.deg2rad(90.0)))
        #     else:
        #         velocity_right = (path[index - 1][1] - path[index][1]) / dt
        #     if np.sin(path[index-1][2] + np.deg2rad(90.0)) > threshold:
        #         velocity_left = -(path[index - 1][0] - path[index][0]) / (dt * np.sin(path[index - 1][2] +
        #                                                                               np.deg2rad(90.0)))
        #     else:
        #         velocity_left = - (path[index - 1][1] - path[index][1]) / dt*(path[index - 1][2] + np.deg2rad(90.0))
        #     print("velocity error: ", np.fabs(velocity_right-velocity_left))
        #     velocity = (velocity_left + velocity_right) / 2
        #     velocity_left_wheel = velocity - omega*D/2
        #     velocity_right_wheel = velocity + omega*D/2
        #     self.__velocity_vector["left_wheel"].append(velocity_left_wheel)
        #     self.__velocity_vector["right_wheel"].append(velocity_right_wheel)
        #     index -= 1

    # def execute_path(self):
        # D = 162
        # dt = 0.1
        # x, y, theta = self.__q_start
        # pv, thetav = [], []
        # for i in range(len(self.__velocity_vector["left_wheel"])):
        #     pv.append(np.array([x, y]))
        #     thetav.append(theta)
        #     v = (self.__velocity_vector["left_wheel"][i] + self.__velocity_vector["right_wheel"][i]) / 2
        #     omega = (-self.__velocity_vector["left_wheel"][i] + self.__velocity_vector["right_wheel"][i]) / D
        #     x -= v * np.sin(theta) * dt
        #     y += v * np.cos(theta) * dt
        #     theta += omega * dt
        # print("Final x position error: ", np.fabs(pv[len(pv) - 1][0] - self.__q_goal[0]))
        # print("Final y position error: ", np.fabs(pv[len(pv) - 1][1] - self.__q_goal[1]))
        # print("Final orientation error: ", thetav[len(pv) - 1] - self.__q_goal[2])
        # return pv, thetav