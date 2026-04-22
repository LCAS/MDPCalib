#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TransformStamped


class CalibrationResultMonitor:

    def __init__(self) -> None:
        self.received_result = False
        self.timed_out = False

        self.result_topic = rospy.get_param("~result_topic", "/optimizer/refined_transform")
        timeout_sec = float(rospy.get_param("~timeout_sec", 900.0))

        rospy.Subscriber(self.result_topic, TransformStamped, self._result_callback, queue_size=1)
        if timeout_sec > 0:
            rospy.Timer(rospy.Duration(timeout_sec), self._timeout_callback, oneshot=True)

        rospy.loginfo("Waiting for final calibration result on %s", self.result_topic)

    def _result_callback(self, msg: TransformStamped) -> None:
        self.received_result = True
        rospy.loginfo("Received final calibration result %s -> %s. Requesting shutdown.",
                      msg.header.frame_id or "<unset>", msg.child_frame_id or "<unset>")
        rospy.signal_shutdown("Calibration completed")

    def _timeout_callback(self, _event) -> None:
        if self.received_result:
            return
        self.timed_out = True
        rospy.logerr("Timed out waiting for final calibration result on %s", self.result_topic)
        rospy.signal_shutdown("Timed out waiting for calibration result")


if __name__ == "__main__":
    rospy.init_node("calibration_result_monitor")
    monitor = CalibrationResultMonitor()
    rospy.spin()
    if monitor.timed_out:
        raise SystemExit(1)
    raise SystemExit(0)
