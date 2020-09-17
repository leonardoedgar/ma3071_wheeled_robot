#! /usr/bin/env python3
import numpy as np
from matplotlib import pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib import patches
from .robot import Robot
from simple_wheeled_robot.planning.planner import Planner


class RobotSimulator(object):
    def __init__(self, robot, planner):
        # type: (Robot, Planner) -> None
        self.__robot = robot
        self.__planner = planner
        # self.__simulation_time = 3
        self.__simulation_timestep = 0.01
        self.__q_start = (0, 0, np.deg2rad(0.0))

    def run(self, q_goal):
        # Setting up the animation
        self.__planner.init_plan(q_start=self.__q_start, q_goal=q_goal)
        trajectories = self.__planner.plan_path()
        print(trajectories)
        x, y, theta = self.__q_start
        pv, thetav = [], []
        D = 162
        pv.append(np.array([x, y]))
        thetav.append(theta)
        for velocity_left, velocity_right, total_time in trajectories:
            time_elapsed = 0
            velocity = (velocity_left + velocity_right) / 2
            omega = (velocity_right - velocity_left) / D
            while time_elapsed < total_time:
                x -= velocity * np.sin(theta) * self.__simulation_timestep
                y += velocity * np.cos(theta) * self.__simulation_timestep
                theta += omega * self.__simulation_timestep
                pv.append(np.array([x, y]))
                thetav.append(theta)
                time_elapsed += self.__simulation_timestep
        print("Final x position error: ", np.fabs(pv[len(pv) - 1][0] - q_goal[0]))
        print("Final y position error: ", np.fabs(pv[len(pv) - 1][1] - q_goal[1]))
        print("Final orientation error: ", np.rad2deg(thetav[len(pv) - 1]) - np.rad2deg(q_goal[2]))
        self.__visualise(pv, thetav)

    def __visualise(self, pv, thetav):
        num_frames = len(pv)
        plt.close()
        fig = plt.figure()
        ax = fig.add_subplot(111, aspect='equal')
        plt.axis([-1000, 1000, -1000, 1000])
        plt.grid('on')
        body_obj = ax.add_patch(patches.Polygon(self.__robot.body_points, color='gray'))
        wheel1_obj = ax.add_patch(patches.Polygon(self.__robot.rear_left_wheel_points, color='goldenrod'))
        wheel2_obj = ax.add_patch(patches.Polygon(self.__robot.rear_right_wheel_points, color='goldenrod'))
        wheel3_obj = ax.add_patch(patches.Polygon(self.__robot.front_left_wheel_points, color='goldenrod'))
        wheel4_obj = ax.add_patch(patches.Polygon(self.__robot.front_right_wheel_points, color='goldenrod'))
        bumper_obj = ax.add_patch(patches.Polygon(self.__robot.bumper_points, color='gray'))
        objs = [body_obj, wheel1_obj, wheel2_obj, wheel3_obj, wheel4_obj, bumper_obj]

        anim = FuncAnimation(fig, self.__animate, fargs=(objs, pv, thetav),
                             frames=num_frames, interval=self.__simulation_timestep*1000, repeat=False, blit=False)
        plt.show()

    def __animate(self, i, objs, pv, thetav):
        for index in range(len(self.__robot.points)):
            points = []
            for point in self.__robot.points[index]:
                points.append(np.dot(np.array([[np.cos(thetav[i]), -np.sin(thetav[i])],
                                               [np.sin(thetav[i]), np.cos(thetav[i])]]), point) + pv[i])
            objs[index].set_xy(points)
