#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Wrench,Point
from gazebo_msgs.srv import ApplyBodyWrench
from rospy import Duration

class SineWrenchApplier:
    def __init__(self):
        rospy.init_node('apply_sine_body_wrench')
        
        # 等待 Gazebo 服务可用
        rospy.wait_for_service('/gazebo/apply_body_wrench')
        self.apply_wrench = rospy.ServiceProxy('/gazebo/apply_body_wrench', ApplyBodyWrench)
        
        # rospy.sleep(3.0)

        # 获取ROS参数
        self.body_name = rospy.get_param('~body_name', 'bluerov2/base_link')
        self.reference_frame = rospy.get_param('~reference_frame', 'bluerov2/base_link')
        
        # 正弦波参数
        self.amplitude_force = rospy.get_param('~amplitude_force', [0, 0, 0])
        self.amplitude_torque = rospy.get_param('~amplitude_torque', [0, 0, 0])
        self.frequency = rospy.get_param('~frequency', 1.0)  # Hz
        self.phase = rospy.get_param('~phase', 0.0)  # 相位
        
        # 控制参数
        self.update_rate = 50  # Hz
        self.wrench_duration = Duration(0.1)  # 每次施加力的持续时间
        
    def apply_sine_wrench(self):
        rate = rospy.Rate(self.update_rate)
        start_time = rospy.get_time()
        
        while not rospy.is_shutdown():
            current_time = rospy.get_time() - start_time
            
            # 计算正弦波值
            sine_value = math.sin(2 * math.pi * self.frequency * current_time + self.phase)
            
            # 创建力和力矩消息
            wrench = Wrench()
            # 力
            wrench.force.x = self.amplitude_force[0] * sine_value
            wrench.force.y = self.amplitude_force[1] * sine_value
            wrench.force.z = self.amplitude_force[2] * sine_value
            # 力矩
            wrench.torque.x = self.amplitude_torque[0] * sine_value
            wrench.torque.y = self.amplitude_torque[1] * sine_value
            wrench.torque.z = self.amplitude_torque[2] * sine_value
            
            try:
                # 调用Gazebo服务施加力
                self.apply_wrench(
                    body_name=self.body_name,
                    reference_frame=self.reference_frame,
                    reference_point=Point(0, 0, 0),
                    wrench=wrench,
                    start_time=rospy.Time(0),
                    duration=self.wrench_duration
                )
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call failed: {e}")
            
            rate.sleep()

if __name__ == '__main__':
    print("start")
    try:
        print("apply_dynamic_wrench")
        sine_wrench = SineWrenchApplier()
        sine_wrench.apply_sine_wrench()
    except rospy.ROSInterruptException:
        pass