#! /usr/bin/env python3
import numpy as np


class Robot(object):
    def __init__(self):
        # Graphic model (reference point is the center of the car)
        self.__body_points = np.array([[-52, -105.5],
                                       [52, -105.5],
                                       [52, 105.5],
                                       [-52, 105.5]])
        self.__rear_left_wheel_points = np.array([[-80, -91],
                                                  [-52, -91],
                                                  [-52, -24],
                                                  [-80, -24]])
        self.__rear_right_wheel_points = np.array([[52, -91],
                                                   [80, -91],
                                                   [80, -24],
                                                   [52, -24]])
        self.__front_left_wheel_points = np.array([[-80, 24],
                                                   [-52, 24],
                                                   [-52, 91],
                                                   [-80, 91]])
        self.__front_right_wheel_points = np.array([[52, 24],
                                                    [80, 24],
                                                    [80, 91],
                                                    [52, 91]])
        self.__bumper_points = np.array([[-52, 105.5],
                                         [52, 105.5],
                                         [0, 145.5]])

    @property
    def body_points(self):
        return self.__body_points

    @property
    def rear_left_wheel_points(self):
        return self.__rear_left_wheel_points

    @property
    def rear_right_wheel_points(self):
        return self.__rear_right_wheel_points

    @property
    def front_left_wheel_points(self):
        return self.__front_left_wheel_points

    @property
    def front_right_wheel_points(self):
        return self.__front_right_wheel_points

    @property
    def bumper_points(self):
        return self.__bumper_points

    @property
    def points(self):
        return [self.__body_points, self.__rear_left_wheel_points, self.rear_right_wheel_points,
                self.__front_left_wheel_points, self.__front_right_wheel_points, self.__bumper_points]

