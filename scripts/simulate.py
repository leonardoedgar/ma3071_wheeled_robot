from simple_wheeled_robot.simulator.robot_simulator import RobotSimulator
from simple_wheeled_robot.simulator.robot import Robot
from simple_wheeled_robot.planning.planner import Planner
import numpy as np


if __name__ == '__main__':
    simulator = RobotSimulator(robot=Robot(), planner=Planner())
    simulator.run(q_goal=(-500.0, -500.0, np.deg2rad(135.0)))
