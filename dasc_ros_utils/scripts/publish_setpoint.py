#!/usr/bin/env python3
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition
import numpy as np
from scipy.interpolate import CubicSpline
import time
# import matplotlib
# matplotlib.use('GTK')  # Or any other X11 back-end
import matplotlib.pyplot as plt
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import socket

received_x=[]
received_y=[]

class SetpointAssignerNode(Node):
    def __init__(self):
        super().__init__('setpoint_assigner')
        self.publisher = self.create_publisher(TrajectorySetpoint, '/px4_1/fmu/in/trajectory_setpoint', 10)
        # self.marker_pub=self.create_publisher(Marker,'path_marker',10)
        # qos_profile = QoSProfile(reliability=QoSProfile.ReliabilityPolicy.RELIABLE)

        # Now, adding the UDP code to decentralize communication between robots
        self.UDP_IP = "192.168.1.149"
        self.UDP_PORT_SEND_2 = 5006
        self.UDP_PORT_SEND_3 = 5007
        self.UDP_PORT_RECEIVE = 5005

        self.sock_receive = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
        self.sock_receive.bind((self.UDP_IP, self.UDP_PORT_RECEIVE))

        self.timer = self.create_timer(0.065, self.publish_text)
        # self.subscription = self.create_subscription(VehicleLocalPosition,'/px4_1/fmu/out/vehicle_local_position',self.listener_callback,10)
        self.subscription = self.create_subscription(VehicleLocalPosition,'/px4_1/fmu/out/vehicle_local_position',self.listener_callback,QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        # self.subscription
        
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        

        self.msg = TrajectorySetpoint()
        self.num_points=2200 #2000 to make it work
        self.angles=np.linspace(0,2*np.pi,self.num_points)
        # # Shape 8
        # Adding required offset
        self.x=1.5*np.cos(self.angles)
        self.x=self.x+1.0
        self.y=1.5*np.sin(2*self.angles)
        # Reverse the direction
        self.x=self.x[::-1]
        self.y=self.y[::-1]
        # changing the starting point

        # Split the array into four quarters
        q11 = self.x[:len(self.x)//4]
        q12 = self.x[len(self.x)//4:2*len(self.x)//4]
        q13 = self.x[2*len(self.x)//4:3*len(self.x)//4]
        q14 = self.x[3*len(self.x)//4:]
        # Concatenate the quarters in the desired order
        self.x = np.concatenate((q14, q11, q12, q13))

        # Split the array into four quarters
        q21 = self.y[:len(self.y)//4]
        q22 = self.y[len(self.y)//4:2*len(self.y)//4]
        q23 = self.y[2*len(self.y)//4:3*len(self.y)//4]
        q24 = self.y[3*len(self.y)//4:]
        # Concatenate the quarters in the desired order
        self.y = np.concatenate((q24, q21, q22, q23))


        print("Reference x: ",self.x)
        print("Reference y: ",self.y)
        # print(type(self.x))
        self.v=0.75
        self.vw=0.10
        # msg = TrajectorySetpoint()
        # num_points=200
        # angles=np.linspace(0,2*np.pi,num_points)
        # # Shape 8
        # x=np.cos(angles)
        # y=np.sin(2*angles)
        # v=0.5
        # x_waypoints=[0,1,2]
        # y_waypoints=[1,3,2]
        # f=CubicSpline(x_waypoints,y_waypoints,bc_type='natural')
        # x_new=np.linspace(0,3,10)
        # y_new=f(x_new)
        # n=len(x_waypoints)
        # for i in range(n-1):
        #     f=CubicSpline([x_waypoints[i],x_waypoints[i+1]])
        # print(x_new)
        # print(y_new)
        plt.plot(self.x,self.y,'b')
        # plt.show()
        # plt.savefig("dummy_name.png")
        # Circle diameter=1
        # x=np.sin(angles)
        # y=np.cos(angles)
        # v=0.05
        # Any spline
        # x=
        self.count=0
        

    def publish_text(self):
        if self.count < self.num_points:
            self.msg.position = [self.x[self.count], self.y[self.count], -0.16]
            self.msg.velocity = [self.v, self.v, self.vw]
            self.publisher.publish(self.msg)
            self.get_logger().info('Publishing setpoint %d' % self.count)
            self.count += 1

            # Adding code for broadcasting coordinates
            message_send = "(1,1) from robot 1"
            sock_send_2 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
            sock_send_2.sendto(message_send.encode(), ("192.168.1.132", self.UDP_PORT_SEND_2))
            sock_send_2.close()

            # sock_send_3 = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)  # UDP
            # sock_send_3.sendto(message_send.encode(), (UDP_IP, UDP_PORT_SEND_3))
            # sock_send_3.close()

            data, addr = self.sock_receive.recvfrom(1024)  # buffer size is 1024 bytes
            print("Received message:", data.decode())
        else:
            self.msg.velocity = [0.0, 0.0, 0.0]
            self.publisher.publish(self.msg)
            self.get_logger().info('Endpoint Reached')
            # print(type(received_x))
            # print(received_x)
            cvt_received_x=np.asarray(received_x)
            cvt_received_y=np.asarray(received_y)
            # print(cvt_received_x)
            plt.plot(cvt_received_x,cvt_received_y,'r')
            plt.grid()
            plt.show()
            plt.savefig("dummy_name_1.png")
            print("Image saved")
            print("Size received: ",len(cvt_received_x))
            print("Size published: ", len(self.x))
            # self.timer.cancel()  # Stop the timer when all points are published
            self.count=0


    def listener_callback(self, msg):
        global received_x,received_y
        # print("Entered subscriber")
        # self.get_logger().info('I heard: "%s"' % msg)
        # self.get_logger().info('%f, %f' % (msg.x, msg.y))
        received_x.append(msg.x)
        received_y.append(msg.y)
        # print("%f, %f" %(msg.x,msg.y))



def main(args=None):
    rclpy.init(args=args)
    setpoint_assigner_node = SetpointAssignerNode()
    rclpy.spin(setpoint_assigner_node)
    setpoint_assigner_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

