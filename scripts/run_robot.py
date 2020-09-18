#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
from simple_wheeled_robot.planning.planner import Planner
import numpy as np

# ==============================================================
# Robot motion given left & right motor speed and traveling time
# ==============================================================

# Pin number indication
# pins for left motor
PWMA = 18  # PWM signal (setting the left motor speed)
AIN1 = 22  # left forward
AIN2 = 27  # left backward
# pins for right motor
PWMB = 23  # PWM signal (setting the right motor speed)
BIN1 = 25  # right forward
BIN2 = 24  # right backward
# pins for infrared sensors
# =============================================================================
IRLEFT = 12
IRRIGHT = 16

def car_stop(t_time):
    """ stop the car
    """
    L_Motor.ChangeDutyCycle(0)
    GPIO.output(AIN2, False)
    GPIO.output(AIN1, False)

    R_Motor.ChangeDutyCycle(0)
    GPIO.output(BIN2, False)
    GPIO.output(BIN1, False)
    time.sleep(t_time)


def car_motion(lspeed, rspeed, t):
    """ command the car to move given left & right motor speed and duration
    """
    if lspeed < 0:
        L_Motor.ChangeDutyCycle(-lspeed)
        GPIO.output(AIN2, True)
        GPIO.output(AIN1, False)
    else:
        L_Motor.ChangeDutyCycle(lspeed)
        GPIO.output(AIN2, False)
        GPIO.output(AIN1, True)
    if rspeed < 0:
        R_Motor.ChangeDutyCycle(-rspeed)
        GPIO.output(BIN2, True)
        GPIO.output(BIN1, False)
    else:
        R_Motor.ChangeDutyCycle(rspeed)
        GPIO.output(BIN2, False)
        GPIO.output(BIN1, True)

    time.sleep(t)


def initialise_gpio():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(AIN2, GPIO.OUT)
    GPIO.setup(AIN1, GPIO.OUT)
    GPIO.setup(PWMA, GPIO.OUT)

    GPIO.setup(BIN1, GPIO.OUT)
    GPIO.setup(BIN2, GPIO.OUT)
    GPIO.setup(PWMB, GPIO.OUT)
    GPIO.setup(IRLEFT, GPIO.IN)
    GPIO.setup(IRRIGHT, GPIO.IN)


if __name__ == '__main__':
    initialise_gpio()
    L_Motor = GPIO.PWM(PWMA, 100)
    L_Motor.start(0)

    R_Motor = GPIO.PWM(PWMB, 100)
    R_Motor.start(0)

    planner = Planner()
    q_start = 0.0, 0.0, np.deg2rad(0.0)

    # start motion
    try:
        while True:
            user_input = input("Enter final pose (x, y, theta): ").split()
            if len(user_input) != 3:
                raise Exception("Invalid input")
            try:
                x, y, theta = [float(data) for data in user_input]
            except ValueError:
                raise Exception("Invalid Input")
            else:
                car_stop(7)
                planner.init_plan(q_start=q_start, q_goal=(x, y, np.deg2rad(theta)))
                trajectories = planner.plan_path()
                for left_velocity, right_velocity, total_time in trajectories:
                    start_time = time.time()
                    while time.time() - start_time < total_time:
                        if not (GPIO.input(IRLEFT) and GPIO.input(IRRIGHT)):
                            car_stop(3)
                            raise Exception("Obstacles detected")
                        car_motion(left_velocity/10.0, right_velocity/10.0, 0.1)
                car_stop(10)
                break
    except KeyboardInterrupt or Exception:
        car_stop(3)
    finally:
        GPIO.cleanup()
