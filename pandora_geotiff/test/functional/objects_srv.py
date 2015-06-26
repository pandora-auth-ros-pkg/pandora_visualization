#!/usr/bin/env python

from __future__ import print_function

import rospy
from geometry_msgs.msg import PoseStamped

from pandora_data_fusion_msgs.srv import GeotiffSrv, GeotiffSrvResponse


def send_objects(req):
    print("Service called.")

    victims = []
    qrs = []
    hazmats = []

    victim1 = PoseStamped()
    victim2 = PoseStamped()

    qr1 = PoseStamped()
    qr2 = PoseStamped()

    hazmat1 = PoseStamped()
    hazmat2 = PoseStamped()
    hazmat3 = PoseStamped()

    # Create random victims
    victim1.pose.position.x = 1.1
    victim1.pose.position.y = 3.2
    victim1.header.frame_id = 'map'
    victim1.header.stamp = rospy.Time.now()

    victims.append(victim1)

    victim2.pose.position.x = 2.1
    victim2.pose.position.y = 4.2
    victim2.header.frame_id = 'map'
    victim2.header.stamp = rospy.Time.now()

    victims.append(victim2)

    # Create random QRs
    qr1.pose.position.x = 0.5
    qr1.pose.position.y = 1.9
    qr1.header.frame_id = 'map'
    qr1.header.stamp = rospy.Time.now()

    qrs.append(qr1)

    qr2.pose.position.x = 2.1
    qr2.pose.position.y = 3.7
    qr2.header.frame_id = 'map'
    qr2.header.stamp = rospy.Time.now()

    qrs.append(qr2)

    # Create random QRs
    hazmat1.pose.position.x = 2.8
    hazmat1.pose.position.y = 0.6
    hazmat1.header.frame_id = 'map'
    hazmat1.header.stamp = rospy.Time.now()

    hazmats.append(hazmat1)

    hazmat2.pose.position.x = 7.1
    hazmat2.pose.position.y = 1.7
    hazmat2.header.frame_id = 'map'
    hazmat2.header.stamp = rospy.Time.now()

    hazmats.append(hazmat2)

    hazmat3.pose.position.x = 6.0
    hazmat3.pose.position.y = 0.6
    hazmat3.header.frame_id = 'map'
    hazmat3.header.stamp = rospy.Time.now()

    hazmats.append(hazmat3)

    return GeotiffSrvResponse(victims=victims, hazmats=hazmats, qrs=qrs)


def server():
    rospy.init_node("data_fusion_objects")
    rospy.Service('data_fusion_geotiff', GeotiffSrv, send_objects)

    print("Service initialized.")
    rospy.spin()

if __name__ == "__main__":
    server()
