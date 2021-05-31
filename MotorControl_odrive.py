#!/usr/bin/env python3
"""
Example usage of the ODrive python library to monitor and control ODrive devices

information for future configuration

odrv0.axis0.motor.config.current_lim = 10
odrv0.axis0.controller.config.vel_limit = 58 
odrv0.axis0.motor.config.calibration_current = 5
odrv0.config.brake_resistance = 0
odrv0.axis0.motor.config.pole_pairs = 6
odrv0.axis0.motor.config.torque_constant = 0.045
odrv0.axis0.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
odrv0.axis0.encoder.config.cpr = 1024 * 8

odrv0.axis1.motor.config.current_lim = 10
odrv0.axis1.controller.config.vel_limit = 58 
odrv0.axis1.motor.config.calibration_current = 5
odrv0.config.brake_resistance = 0
odrv0.axis1.motor.config.pole_pairs = 6
odrv0.axis1.motor.config.torque_constant = 0.045
odrv0.axis1.motor.config.motor_type = MOTOR_TYPE_HIGH_CURRENT
odrv0.axis1.encoder.config.cpr = 1024 * 8 


"""

from __future__ import print_function
import odrive
from odrive.enums import *
import time



class MotorControl():
    def __init__(self):
        #init_odrive serial number in
        self.odrv0 = odrive.find_any(serial_number = "207735823056")
        self.odrv1 = odrive.find_any(serial_number = "208D35853056") # or 35790860398678
        
    def goal_velocity(self, motorId, rps):
        if motorId == 0: 
            self.odrv0.axis0.controller.input_vel = rps
        if motorId == 1: 
            self.odrv0.axis1.controller.input_vel = rps
        if motorId == 2: 
            self.odrv1.axis0.controller.input_vel = rps
        if motorId == 3: 
            self.odrv1.axis1.controller.input_vel = rps
		# print(rps)
    def set_speed(self, motorId):
        if motorId == 0: 
            return self.odrv0.axis0.encoder.vel_estimate
        if motorId == 1: 
            return self.odrv0.axis1.encoder.vel_estimate
        if motorId == 2: 
            return self.odrv0.axis0.encoder.vel_estimate
        if motorId == 3: 
            return self.odrv0.axis1.encoder.vel_estimate
if __name__ == '__main__':
    motor = MotorControl()
    time.sleep(1)	
    motor.goal_velocity(0, 5)
    time.sleep(10)
    print('speed = ', motor.set_speed(0))
    motor.goal_velocity(0, 0)


