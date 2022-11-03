#!/usr/bin/env python3

import sys
import rospy
import math
from time import sleep
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from eric_a_bringup.msg import MotorPacket
from geometry_msgs.msg import Twist, Pose, Point, Vector3, Quaternion
from tf.broadcaster import TransformBroadcaster
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from eric_a_bringup.srv import ResetOdom, ResetOdomResponse
from std_srvs.srv import TriggerRequest, Trigger

class OdomPose(object):
   x = 0.0
   y = 0.0
   theta = 0.0
   timestamp = 0
   pre_timestamp = 0

class OdomVel(object):
   x = 0.0
   y = 0.0
   w = 0.0

class Joint(object):
   joint_name = ['wheel_left_joint', 'wheel_right_joint']
   joint_pos = [0.0, 0.0]
   joint_vel = [0.0, 0.0]

class RobotConfig(object):
   body_circumference = 0        # circumference length of robot for spin in place
   wheel_separation = 0.0        # Default Vehicle width in mm
   wheel_radius = 0.0            # Wheel radius
   wheel_circumference = 0       # Wheel circumference
   max_lin_vel_wheel = 0.0       # Maximum speed can be applied to each wheel (mm/s)
   max_lin_vel_x = 0             # Speed limit for vehicle (m/s)
   max_ang_vel_z = 0             # Rotational Speed limit for vehicle (rad/s)

   encoder_gear_ratio = 0
   encoder_step = 0
   encoder_pulse_per_wheel_rev = 0
   encoder_pulse_per_gear_rev = 0

class PacketHandler:
   def __init__(self):
      self._vel = [0.0, 0.0]   #linear, angular
      self._enc = [0.0, 0.0]   #left encorder , right encorder
      self._wodom = [0.0, 0.0] #left odom, right odom
      # self._rpm = [0.0, 0.0]   #left rpm, rigt rpm
      # self._wvel = [0.0, 0.0]  #wheel left vel, wheel right vel
   
   def read_packet(self, msg):
      self._vel = [msg.vw[0], msg.vw[1]]
      self._enc = [msg.encod[0], msg.encod[1]]
      self._wodom = [msg.odo[0], msg.odo[1]]
      # self._rpm = [msg.rpm[0], msg.rpm[1]]
      # self._wvel = [msg.diffv[0], msg.diffv[1]]

   def write_odometry_reset(self):
      result = self.odomReset_request()
      rospy.loginfo("Odom reset %s", result)
      sleep(0.05)

   def odomReset_request(self):
      # rospy.wait_for_service('motorOdom_reset')
      try:
         odomRest = rospy.ServiceProxy('motorOdom_reset', Trigger)
         return odomRest()
      except rospy.ServiceException as e:
         print("Service call failed: %s"%e)


class hongdorosMotorNode:
   def __init__(self):
      # Open serial port
      self.tf_prefix = rospy.get_param("~tf_prefix", "")
      self.ph = PacketHandler()
      self.ph.write_odometry_reset() ##모터로 가는 odom 0으로 초기화
      sleep(0.1)

      # Storaging
      self.odom_pose = OdomPose()
      self.odom_vel = OdomVel()
      self.joint = Joint() 

      self.enc_left_tot_prev, self.enc_right_tot_prev = 0.0, 0.0   
      self.enc_offset_left, self.enc_offset_right = 0.0, 0.0

      self.is_enc_offset_set = False
      self.is_imu_offset_set = False
      self.orientation = [0.0, 0.0, 0.0, 0.0]
      self.last_theta = 0.0

      # Set vehicle specific configurations
      self.config = RobotConfig()
      self.config.wheel_separation = 0.2052
      self.config.wheel_radius = 0.0575
      self.config.max_lin_vel_wheel = 1101.0
      self.config.max_lin_vel_x = 1.101
      self.config.max_ang_vel_z = 5.365
      self.config.encoder_pulse_per_gear_rev = 1000
      self.config.encoder_gear_ratio = 21
      self.config.body_circumference = self.config.wheel_separation * math.pi
      self.config.wheel_circumference = self.config.wheel_radius * 2 * math.pi
      self.config.encoder_pulse_per_wheel_rev = self.config.encoder_pulse_per_gear_rev * self.config.encoder_gear_ratio * 4
      self.config.encoder_step = self.config.wheel_circumference / self.config.encoder_pulse_per_wheel_rev

      rospy.loginfo('Wheel Track:{:.2f}m, Radius:{:.3f}m'.format(self.config.wheel_separation, self.config.wheel_radius))
      rospy.loginfo('Platform Rotation arc length: {:04f}m'.format(self.config.body_circumference))
      rospy.loginfo('Wheel circumference: {:04f}m'.format(self.config.wheel_circumference))
      rospy.loginfo('Encoder step: {:04f}m/pulse'.format(self.config.encoder_step))
      rospy.loginfo('Encoder pulses per wheel rev: {:.2f} pulses/rev'.format(self.config.encoder_pulse_per_wheel_rev))

      # subscriber
      rospy.Subscriber(self.tf_prefix+"cmd_vel", Twist, self.cbSubCmdVelTMsg, queue_size=1)        # command velocity data subscriber
      rospy.Subscriber(self.tf_prefix+"motor_packet", MotorPacket, self.ph.read_packet, queue_size=1)

      # publisher
      self.pub_joint_states = rospy.Publisher(self.tf_prefix+'joint_states', JointState, queue_size=10)
      self.odom_pub = rospy.Publisher(self.tf_prefix+"odom", Odometry, queue_size=10)
      self.odom_broadcaster = TransformBroadcaster()
      self.pub_vel = rospy.Publisher("motor_input", Twist, queue_size=10)

      #service server
      rospy.Service('reset_odom', ResetOdom, self.reset_odom_handle)
      
      # timer
      rospy.Timer(rospy.Duration(0.01), self.cbTimerUpdateDriverData) # 10 hz update
      self.odom_pose.timestamp = rospy.Time.now().to_nsec()
      self.odom_pose.pre_timestamp = rospy.Time.now()
      self.reset_odometry()

      rospy.on_shutdown(self.__del__)

   def reset_odometry(self):
      self.is_enc_offset_set = False
      self.is_imu_offset_set = False

      self.joint.joint_pos = [0.0, 0.0]
      self.joint.joint_vel = [0.0, 0.0]

   def update_odometry(self, odo_l, odo_r, trans_vel, orient_vel):
      odo_l /= 1000.
      odo_r /= 1000.
      trans_vel /= 1000.
      orient_vel /= 1000.

      self.odom_pose.timestamp = rospy.Time.now()
      dt = (self.odom_pose.timestamp - self.odom_pose.pre_timestamp).to_sec()
      self.odom_pose.pre_timestamp = self.odom_pose.timestamp

      self.odom_pose.theta += orient_vel * dt
      d_x = trans_vel * math.cos(self.odom_pose.theta) 
      d_y = trans_vel * math.sin(self.odom_pose.theta) 

      self.odom_pose.x += d_x * dt
      self.odom_pose.y += d_y * dt

      odom_orientation_quat = quaternion_from_euler(0, 0, self.odom_pose.theta)

      self.odom_vel.x = trans_vel
      self.odom_vel.y = 0.
      self.odom_vel.w = orient_vel

      odom = Odometry()
      odom.header.frame_id = "/odom"
      odom.child_frame_id = "/base_link"

      self.odom_broadcaster.sendTransform((self.odom_pose.x, self.odom_pose.y, 0.), 
                                             odom_orientation_quat, self.odom_pose.timestamp, 
                                             odom.child_frame_id, odom.header.frame_id)

      odom.header.stamp = rospy.Time.now()
      odom.pose.pose = Pose(Point(self.odom_pose.x, self.odom_pose.y, 0.), Quaternion(*odom_orientation_quat))
      odom.twist.twist = Twist(Vector3(self.odom_vel.x, self.odom_vel.y, 0), Vector3(0, 0, self.odom_vel.w))

      self.odom_pub.publish(odom)

   def updateJointStates(self, odo_l, odo_r, trans_vel, orient_vel):
      odo_l /= 1000.
      odo_r /= 1000.

      wheel_ang_left = odo_l / self.config.wheel_radius
      wheel_ang_right = odo_r / self.config.wheel_radius

      wheel_ang_vel_left = (trans_vel - (self.config.wheel_separation / 2.0) * orient_vel) / self.config.wheel_radius
      wheel_ang_vel_right = (trans_vel + (self.config.wheel_separation / 2.0) * orient_vel) / self.config.wheel_radius

      self.joint.joint_pos = [wheel_ang_left, wheel_ang_right]
      self.joint.joint_vel = [wheel_ang_vel_left, wheel_ang_vel_right]

      joint_states = JointState()
      joint_states.header.frame_id = self.tf_prefix + "/base_link"
      joint_states.header.stamp = rospy.Time.now()
      joint_states.name = self.joint.joint_name
      joint_states.position = self.joint.joint_pos
      joint_states.velocity = self.joint.joint_vel
      joint_states.effort = []

      self.pub_joint_states.publish(joint_states)

   def cbTimerUpdateDriverData(self, event): 
      odo_l = self.ph._wodom[0]
      odo_r = self.ph._wodom[1]
      trans_vel = self.ph._vel[0]
      orient_vel = self.ph._vel[1]
      rospy.loginfo('V= {}, W= {}, odo_l: {} odo_r:{}'.format(trans_vel, orient_vel, odo_l, odo_r))
      self.update_odometry(odo_l, odo_r, trans_vel, orient_vel)
      self.updateJointStates(odo_l, odo_r, trans_vel, orient_vel)

   def cbSubCmdVelTMsg(self, cmd_vel_msg): ## send to cmd_vel message
      lin_vel_x = cmd_vel_msg.linear.x
      ang_vel_z = cmd_vel_msg.angular.z

      lin_vel_x = max(-self.config.max_lin_vel_x, min(self.config.max_lin_vel_x, lin_vel_x))
      ang_vel_z = max(-self.config.max_ang_vel_z, min(self.config.max_ang_vel_z, ang_vel_z))
      self.pub_vel.publish(Twist(Vector3(lin_vel_x*1000, 0, 0), Vector3(0, 0, ang_vel_z*1000)))

   def reset_odom_handle(self, req):
      self.odom_pose.x = req.x
      self.odom_pose.y = req.y
      self.odom_pose.theta = req.theta

      return ResetOdomResponse()

   def main(self):
      rospy.spin()

   def __del__(self):
      print("terminating eric_a_motor_node")

if __name__ == '__main__':
   rospy.init_node('eric_a_motor_node')
   node = hongdorosMotorNode()
   node.main()
