#!/usr/bin/env python3


import rospy
from std_srvs.srv import Trigger, TriggerRequest


def timeCallback(event):
    req = TriggerRequest()
    print("Request sent at %s" % rospy.get_time())
    resp = service_client(req)
    return resp


if __name__ == "__main__":
    rospy.init_node("test_service_client")

    print("[SrvTest] Waiting for service server...")
    rospy.wait_for_service("trigger_srv")
    print("[SrvTest] Service server is up.")

    service_client = rospy.ServiceProxy("trigger_srv", Trigger)

    print("[SrvTest] Ready to send requests.")

    timer = rospy.Timer(rospy.Duration(1), timeCallback)

    rospy.spin()
