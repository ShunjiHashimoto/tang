#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import tf
from tang_msgs.msg import HumanInfo
from visualization_msgs.msg import Marker
import geometry_msgs.msg

class HumanPosDisplayer():
    def __init__(self):
        self.human_info = HumanInfo()
        self.human_info_sub     = rospy.Subscriber('tang_cmd', HumanInfo, self.cmd_callback, queue_size=1)
        self.humanpos_publisher = rospy.Publisher("human_pos_for_display", Marker, queue_size = 1)
    
    def pub_tf(self):
        broadcaster = tf2_ros.StaticTransformBroadcaster()
        static_transformStamped = geometry_msgs.msg.TransformStamped()
        static_transformStamped.header.stamp = rospy.Time.now()
        static_transformStamped.header.frame_id = "world"
        static_transformStamped.child_frame_id = "map"

        static_transformStamped.transform.translation.x = 0.0
        static_transformStamped.transform.translation.y = 0.0
        static_transformStamped.transform.translation.z = 0.0

        quat = tf.transformations.quaternion_from_euler(float(0), float(0), float(0))
        static_transformStamped.transform.rotation.x = quat[0]
        static_transformStamped.transform.rotation.y = quat[1]
        static_transformStamped.transform.rotation.z = quat[2]
        static_transformStamped.transform.rotation.w = quat[3]

        broadcaster.sendTransform(static_transformStamped)
        return

    def register_humanpos(self, id, scale, marker_type, pose, color):
        marker_data = Marker()
        marker_data.header.frame_id = "world"
        marker_data.header.stamp = rospy.Time.now()
        marker_data.ns = "basic_shapes"
        marker_data.id = id
        marker_data.action = Marker.ADD
        marker_data.pose.position.x = pose.x
        marker_data.pose.position.y = pose.y
        marker_data.pose.position.z = pose.z
        marker_data.pose.orientation.x = -0.5
        marker_data.pose.orientation.y = 0.5
        marker_data.pose.orientation.z = 0.5
        marker_data.pose.orientation.w = -0.5
        marker_data.color.r = color[0]
        marker_data.color.g = color[1]
        marker_data.color.b = color[2]
        marker_data.color.a = color[3]
        marker_data.scale.x = scale[0]
        marker_data.scale.y = scale[1]
        marker_data.scale.z = scale[2]
        marker_data.lifetime = rospy.Duration()
        marker_data.type = marker_type
        marker_data.mesh_resource = "package://tang_detection/obj/penguin.obj"
        return marker_data
    
    def pub_humanpos(self):
        human_scale = [0.00005, 0.00005, 0.00005]
        color_rgba = [1.0, 0.0, 0.0, 0.5]
        marker_data = self.register_humanpos(id=0, scale=human_scale, marker_type=10, \
                                            pose=self.human_info.human_point, color=color_rgba)
        self.humanpos_publisher.publish(marker_data)

    def cmd_callback(self, msg):
        # 人の位置とサイズを得る
        self.human_info = msg

def main():
    rospy.init_node("display_humanpos", anonymous=True)
    human_pos_displayer = HumanPosDisplayer()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        human_pos_displayer.pub_humanpos()
        human_pos_displayer.pub_tf()
        rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    main()
