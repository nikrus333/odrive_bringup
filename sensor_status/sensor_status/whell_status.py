import rclpy
from rclpy.node import Node
import sys
from geometry_msgs.msg import Twist

from motor_control_msg.msg import Motorcontrol

from .test_odrive import MotorControl

# class for control move\status motors and encoders
class WhellStatus(Node):
    def __init__(self, *args):
        self.type_devise = "odrive" # input parametr (rosparam)
        self.type_mode = "dif"
        self.path = "/dev/ttyUSB0"
        self.devices = 0 #MotorControl()

        super().__init__('bring_up_node')
        self.sub_control_comand = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(
            Twist, 
            '/odom',
            10)
        self.publisher_status = self.create_publisher(
            Motorcontrol, 
            '/status_all',
            10)
        timer_period = 1 # 5 second
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.conect_status = self.conect()

    def conect(self):
        try:
            self.devices = MotorControl()
            return True
        except:
            return False

    def listener_callback(self, data):
        print(data)

    def timer_callback(self):
        data = Motorcontrol()
        data.conect = self.get_connect_status()
        if self.get_connect_status() == True:
            data.voltage = self.get_voltage()
            encoder_err = self.get_encoder_err()
            data.encoder_err_0 = encoder_err[0]
            data.encoder_err_1 = encoder_err[1]
            motor_err = self.get_motor_err()
            data.motor_err_0 = motor_err[0]
            data.motor_err_1 = motor_err[1]

        else:
            data.voltage = 0.0
            data.encoder_err_0 = False
            data.encoder_err_1 = False 
        self.publisher_status.publish(data)

#get status\error
    def get_voltage(self):
        voltage = self.devices.get_voltage()
        return voltage

    def get_connect_status(self):
        if self.conect_status == True:
            return True
        else:
            return False

    def get_encoder_err(self):
        encoder_err = self.devices.get_encoder_err()
        return encoder_err
    
    def get_motor_err(self):
        motor_err = self.devices.get_motor_err()
        return motor_err
        
    def data_status(self):
        pass
        return status, error

    def test(self):
        pass
        return status, error

def main(args=None):
    rclpy.init(args=args)
    whell = WhellStatus()
    rclpy.spin((whell))
    whell.destroy_node()
    rclpy.shutdown()
    #print(whell.power_status())
    print('Hi from sensor_status.')

if __name__ == '__main__':
    main()