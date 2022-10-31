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



odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
odrv0.axis0.motor.config.pre_calibrated = True
odrv0.axis0.config.startup_encoder_offset_calibration = True
odrv0.axis0.config.startup_closed_loop_control = True
odrv0.save_configuration()
odrv0.reboot()

"""

from __future__ import print_function
import odrive
from odrive.enums import *
from odrive.utils import *
import time



class MotorControl():
    def __init__(self):
        #init_odrive serial number in
        try:
            self.odrv0 = odrive.find_any(serial_number = "206E31B3324D")
        #self.odrv1 = odrive.find_any(serial_number = "306634523439") # or 35790860398678
        except: 
            print("false conect")
            return False

    def goal_velocity(self, motorId, rps):
        if motorId == 0: 
            self.odrv0.axis0.controller.input_vel = rps
        if motorId == 1: 
            self.odrv0.axis1.controller.input_vel = rps
    def set_speed(self, motorId):
        if motorId == 0: 
            return self.odrv0.axis0.encoder.vel_estimate
        if motorId == 1: 
            return self.odrv0.axis1.encoder.vel_estimate
    def get_voltage(self):
        return self.odrv0.vbus_voltage
    def check_status_encoder(self):
        encoder_0 = self.odrv0.axis0.encoder.error
        encoder_1 = self.odrv0.axis1.encoder.error
        if encoder_0 == 0:
            encoder_0 = True
        else:
            encoder_0 = False
        if encoder_1 == 0:
            encoder_1 = True
        else:
            encoder_1 = False
        return [encoder_0, encoder_1]

    def check_status_motor(self):
        motor_0 = self.odrv0.axis0.motor.error
        motor_1 = self.odrv0.axis1.motor.error
        if motor_0 == 0:
            motor_0 = True
        else:
            motor_0 = False
        if motor_1 == 0:
            motor_1 = True
        else:
            motor_1 = False
        return [motor_0, motor_1]

    def reboot(self):
        self.odrv0.reboot()

if __name__ == '__main__':
    motor = MotorControl() 
    print(motor.get_voltage())
    print(motor.check_status_encoder())
    print(motor.check_status_motor())
    #motor.reboot()
    time.sleep(2)	
    motor.get_voltage()
    motor.goal_velocity(0, -4)
    motor.goal_velocity(1, 4)

    #time.sleep(7)
    motor.goal_velocity(0, 4)
    motor.goal_velocity(1, -4)
    time.sleep(7)


