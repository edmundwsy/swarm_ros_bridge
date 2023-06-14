#!/usr/bin/env python3

import rospy
from std_srvs.srv import Trigger, TriggerResponse


def triggerCallback(req):
    print("Triggered!")
    return TriggerResponse(True, "Triggered!")


if __name__ == "__main__":
    rospy.init_node("test_service")

    s = rospy.Service("test_service", Trigger, triggerCallback)
    print("[SrvTest] Ready to receive requests.")
    rospy.spin()

    s.shutdown()
