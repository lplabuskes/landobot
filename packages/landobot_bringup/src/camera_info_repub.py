#!/usr/bin/env python3
import yaml

import rospy
from sensor_msgs.msg import CameraInfo


class CameraInfoRepublisher:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        path = rospy.get_param("~extrinsics_path")
        self.homography = None
        with open(path) as file:
            data = yaml.safe_load(file)
            self.homography = data['homography']

        self.pub = rospy.Publisher("~camera_info", CameraInfo, queue_size=1)
        self.sub = rospy.Subscriber("camera_node/camera_info", CameraInfo, self.camera_info_callback)

    def camera_info_callback(self, info: CameraInfo):
        if self.homography is not None and self.pub.get_num_connections() > 0:
            info.R = self.homography
            self.pub.publish(info)

if __name__ == '__main__':
    node = CameraInfoRepublisher(node_name="camera_info_repub")
    rospy.spin()