#!/usr/bin/env python3
"""Open the official main entrance and publish a latched readiness signal."""

import rospy
from building_generator_interfaces.srv import SetDoorState, SetDoorStateRequest
from std_msgs.msg import Bool


def main():
    rospy.init_node("entrance_door", anonymous=False)
    service_name = rospy.get_param("~service_name", "/set_door_state")
    door_id = rospy.get_param("~door_id", "main_entrance")
    enabled = bool(rospy.get_param("~enabled", True))
    retry_period_s = float(rospy.get_param("~retry_period_s", 1.0))
    ready_pub = rospy.Publisher("/entrance/ready", Bool, queue_size=1, latch=True)
    door_client = rospy.ServiceProxy(service_name, SetDoorState)
    ready_pub.publish(Bool(data=not enabled))

    if not enabled:
        rospy.loginfo("[entrance] automatic door opening disabled")
        rospy.spin()
        return

    while not rospy.is_shutdown():
        try:
            rospy.wait_for_service(service_name, timeout=retry_period_s)
            request = SetDoorStateRequest(door_id=door_id, open=True)
            response = door_client(request)
            if response.accepted:
                ready_pub.publish(Bool(data=True))
                rospy.loginfo(
                    "[entrance] door %s is open: %s", door_id, response.message
                )
                rospy.spin()
                return
            rospy.logwarn_throttle(
                5.0, "[entrance] open request rejected: %s", response.message
            )
        except (rospy.ROSException, rospy.ServiceException) as exc:
            rospy.logwarn_throttle(5.0, "[entrance] waiting for door service: %s", exc)
        rospy.sleep(retry_period_s)


if __name__ == "__main__":
    main()
