import rclpy
from rclpy.node import Node
import sys
from time import sleep, time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
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
        self.timer = self.create_timer(timer_period, self.timer_callback_status)

        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_cb, 2)
        self.joints_states_pub = self.create_publisher(JointState, "/joint_states", 4)
        self.create_timer(0.05, self.timer_callback)

        self.control_timeout = time()
        self.R = 0.115
        self.l = 0.430
        self.v_X_targ = 0
        self.v_Y_targ = 0
        self.k = 0.39
        self.w_Z_targ = 0
        self.msg = JointState()
        self.msg.name.append("left")
        self.msg.name.append("right")
        self.msg.velocity.append(0)
        self.msg.velocity.append(0)
        self.conect_status = self.conect()
        

    def listener_callback(self, data):
        print(data)

    def timer_callback_status(self):
        data = Motorcontrol()
        data.conect = self.get_connect_status()
        if self.get_connect_status() == True:
            print('here')
            data.voltage = self.get_voltage()
            encoder_err = self.get_encoder_err()
            data.is_encoder_err_0 = encoder_err[0]
            data.is_encoder_err_1 = encoder_err[1]
            motor_err = self.get_motor_err()
            data.is_motor_err_0 = motor_err[0]
            data.is_motor_err_1 = motor_err[1]

        else:
            data.voltage = 0.0
            data.is_encoder_err_0 = True
            data.is_encoder_err_1 = True
            data.is_motor_err_0 = True
            data.is_motor_err_1 = True

        self.publisher_status.publish(data)

#get status\error
    def conect(self):
        try:
            self.devices = MotorControl()
            return True
        except:
            return False

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

#function for drive
    def cmd_cb(self, data):
        self.v_X_targ = data.linear.x
        self.v_Y_targ = data.linear.y
        self.w_Z_targ = data.angular.z
        self.control_timeout = time()
	
    def timer_callback(self):
        if self.get_connect_status() == True:
            v_l, v_r = self.get_motors_speed()
            if(float(time()) - self.control_timeout < 1.5): 
                self.set_speed(self.v_X_targ, self.v_Y_targ, self.w_Z_targ)
            else:
                self.set_speed(0, 0, 0)
            self.msg.velocity[0] = v_l
            self.msg.velocity[1] = v_r
            self.joints_states_pub.publish(self.msg)
	
    def set_speed(self, vX, vY, wZ): #meteres per second / radians per second
        wZ = -1*wZ
        v_r = ((2*vX - self.l*wZ)/(2*self.R))/(0.39)
        v_l = (-1*(2*vX + self.l*wZ)/(2*self.R))/(0.39)
        self.devices.goal_velocity(0, v_l)
        self.devices.goal_velocity(1, v_r)
    
    def get_motors_speed(self):  
        v_l = -1*self.devices.get_speed(0)*0.39
        v_r = self.devices.get_speed(1)*0.39
        return v_l, v_r
        
    def sign(self, value):
        if value >= 0:
            return 1
        else:
            return -1


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