#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CameraInfo, Image


class CameraPassthroughAdapter:

    def __init__(self) -> None:
        self.input_image_topic = rospy.get_param("~input_image_topic", "/camera/image_raw")
        self.input_camera_info_topic = rospy.get_param("~input_camera_info_topic", "/camera/camera_info")
        self.output_image_topic = rospy.get_param("~output_image_topic", "/camera_undistorted/image")
        self.output_camera_info_topic = rospy.get_param("~output_camera_info_topic",
                                                        "/camera_undistorted/camera_info")

        if self.input_image_topic == self.output_image_topic:
            rospy.logfatal("Camera image input and output topics must be different to avoid a republish loop.")
            raise SystemExit(1)
        if self.input_camera_info_topic == self.output_camera_info_topic:
            rospy.logfatal("Camera info input and output topics must be different to avoid a republish loop.")
            raise SystemExit(1)

        self.image_publisher = rospy.Publisher(self.output_image_topic, Image, queue_size=10)
        self.camera_info_publisher = rospy.Publisher(self.output_camera_info_topic,
                                                     CameraInfo,
                                                     queue_size=10)

        rospy.Subscriber(self.input_image_topic, Image, self._image_callback, queue_size=10)
        rospy.Subscriber(self.input_camera_info_topic,
                         CameraInfo,
                         self._camera_info_callback,
                         queue_size=10)

        rospy.loginfo("Camera passthrough adapter bridging %s -> %s and %s -> %s",
                      self.input_image_topic, self.output_image_topic, self.input_camera_info_topic,
                      self.output_camera_info_topic)

    def _image_callback(self, msg: Image) -> None:
        self.image_publisher.publish(msg)

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        self.camera_info_publisher.publish(msg)


if __name__ == "__main__":
    rospy.init_node("camera_passthrough_adapter")
    CameraPassthroughAdapter()
    rospy.spin()
