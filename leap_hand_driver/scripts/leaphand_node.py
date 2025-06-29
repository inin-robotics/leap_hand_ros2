#!/usr/bin/env python3

import threading

import numpy as np
import rclpy

# import leap_hand_utils.leap_hand_utils as lhu
# from leap_hand.src import LeapPosition
from leap_hand.srv import LeapEffort, LeapPosVelEff, LeapVelocity
from leap_hand_utils.dynamixel_client import DynamixelClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

#LEAP hand conventions:
#180 is flat out home pose for the index, middle, ring, finger MCPs.
#Applying a positive angle closes the joints more and more to curl closed.
#The MCP is centered at 180 and can move positive or negative to that.

#The joint numbering goes from Index (0-3), Middle(4-7), Ring(8-11) to Thumb(12-15) and from MCP Side, MCP Forward, PIP, DIP for each finger.
#For instance, the MCP Side of Index is ID 0, the MCP Forward of Ring is 9, the DIP of Ring is 11

#I recommend you only query when necessary and below 90 samples a second.  Used the combined commands if you can to save time.  Also don't forget about the USB latency settings in the readme.
#The services allow you to always have the latest data when you want it, and not spam the communication lines with unused data.

class LeapNode(Node):
    def __init__(self):
        super().__init__('leaphand_node')
        # Some parameters to control the hand
        kP = self.declare_parameter('kP', 800.0).get_parameter_value().double_value
        kI = self.declare_parameter('kI', 0.0).get_parameter_value().double_value
        kD = self.declare_parameter('kD', 200.0).get_parameter_value().double_value
        port = self.declare_parameter('port', '/dev/ttyUSB0').get_parameter_value().string_value
        input_format = self.declare_parameter('format', 'leap').get_parameter_value().string_value
        side = self.declare_parameter('side', 'left').get_parameter_value().string_value
        ids = self.declare_parameter('ids', '0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15').get_parameter_value().string_value
        curr_lim = self.declare_parameter('curr_lim', 350.0).get_parameter_value().double_value
        assert input_format in ['leap', 'allegro', 'ones'], "Input format must be one of 'leap', 'allegro', or 'ones'."
        assert side in ['left', 'right'], "Side must be either 'left' or 'right'."
        
        # Subscribes to a variety of sources that can command the hand
        self.input_format = input_format
        self.create_subscription(JointState, 'joint_states', self._receive_pose, 10)

        # Creates services that can give information about the hand out
        self.create_service(LeapVelocity, 'leap_velocity', self.vel_srv)
        self.create_service(LeapEffort, 'leap_effort', self.eff_srv)
        self.create_service(LeapPosVelEff, 'leap_pos_vel_eff', self.pos_vel_eff_srv)
        self.create_service(LeapPosVelEff, 'leap_pos_vel', self.pos_vel_srv)
        self._motors = list(map(int, ids.split(',')))
        self._motors.sort()
        self._dxl_client = DynamixelClient(self._motors, port, 4000000)
        self._dxl_client.connect()

        # Enables position-current control mode and the default parameters
        self._dxl_client.sync_write(self._motors, np.ones(len(self._motors)) * 5, 11, 1)
        self._dxl_client.set_torque_enabled(self._motors, True)
        self._dxl_client.sync_write(self._motors, np.ones(len(self._motors)) * kP, 84, 2)  # Pgain stiffness     
        self._dxl_client.sync_write([0,4,8], np.ones(3) * (kP * 0.75), 84, 2)  # Pgain stiffness for side to side should be a bit less
        self._dxl_client.sync_write(self._motors, np.ones(len(self._motors)) * kI, 82, 2)  # Igain
        self._dxl_client.sync_write(self._motors, np.ones(len(self._motors)) * kD, 80, 2)  # Dgain damping
        self._dxl_client.sync_write([0,4,8], np.ones(3) * (kD * 0.75), 80, 2)  # Dgain damping for side to side should be a bit less
        # Max at current (in unit 1ma) so don't overheat and grip too hard
        self._dxl_client.sync_write(self._motors, np.ones(len(self._motors)) * curr_lim, 102, 2)
        self._dxl_client.init_offsets(0 if side == 'left' else 1)
        self._dxl_client.write_desired_pos(self._motors, np.zeros(16))

        self._lock_client = threading.Lock()
        # self._pub_joints = self.create_publisher(JointState, 'joint_states', 10)
        # self.create_timer(0.1, self._pos_timer_callback)
        self.get_logger().info("Iitialized finished.")

    def _receive_pose(self, msg: JointState):
        assert len(msg.position) == 16, "Pose must have exactly 16 elements corresponding to the 16 joints of the LEAP hand."
        pos_names = []
        for pos, name in zip(msg.position, msg.name):
            pos_names.append((pos, name))
        pos_names = sorted(pos_names, key=lambda x: int(x[1].rsplit("_", 1)[1]))
        pos_sorted = []
        for pos, _ in pos_names:
            pos_sorted.append(pos)
        pos_sorted = np.array(pos_sorted, dtype=np.float32)
        # if self.input_format == 'allegro':
        #     # Allegro compatibility, first read the allegro publisher and then convert to leap
        #     # It adds 180 to the input to make the fully open position at 0 instead of 180.
        #     pose = lhu.LEAPhand_to_allegro(pose, zeros=False)
        # elif self.input_format == 'ones':
        #     # Sim compatibility, first read the sim publisher and then convert to leap
        #     # Sim compatibility for policies, it assumes the ranges are [-1,1] and then convert to leap hand ranges.
        #     pose = lhu.LEAPhand_to_sim_ones(pose)
        with self._lock_client:
            self._dxl_client.write_desired_pos(self._motors, pos_sorted)

    def _pos_timer_callback(self):
        with self._lock_client:
            joints = self._dxl_client.read_pos().tolist()
        # Publish the joint states
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        joint_state_msg.name = [f'joint_{i}' for i in range(len(joints))]
        joint_state_msg.position = joints
        self._pub_joints.publish(joint_state_msg)

    # Service that reads and returns the l of the robot in LEAP Embodiment
    def vel_srv(self, _, response):
        with self._lock_client:
            response.velocity = self._dxl_client.read_vel().tolist()
        return response

    # Service that reads and returns the effort/current of the robot in LEAP Embodiment
    def eff_srv(self, _, response):
        with self._lock_client:
            response.effort = self._dxl_client.read_cur().tolist()
        return response

    #Use these combined services to save a lot of latency if you need multiple datapoints
    def pos_vel_srv(self, _, response):
        with self._lock_client:
            output = self._dxl_client.read_pos_vel()
        response.position = output[0].tolist()
        response.velocity = output[1].tolist()
        response.effort = np.zeros_like(output[1]).tolist()
        return response

    #Use these combined services to save a lot of latency if you need multiple datapoints
    def pos_vel_eff_srv(self, _, response):
        with self._lock_client:
            output = self._dxl_client.read_pos_vel_cur()
        response.position = output[0].tolist()
        response.velocity = output[1].tolist()
        response.effort = output[2].tolist()
        return response

def main(args=None):
    rclpy.init(args=args)
    leaphand_node = LeapNode()
    rclpy.spin(leaphand_node)
    leaphand_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
