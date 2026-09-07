#!/usr/bin/env python3

import rospy
import rosgraph
from rostopic import get_topic_class

'''File used to check which topics are expressed with respect
    the world reference frame'''

TARGET_FRAME = "world"

rospy.init_node("find_topics_by_frame", anonymous=True)

master = rosgraph.Master(rospy.get_name())

topics = master.getPublishedTopics("/")

for topic, msg_type in topics:

    try:
        msg_class, _, _ = get_topic_class(topic, blocking=False)

        if msg_class is None:
            continue

        # Check if the message has an header
        if not hasattr(msg_class, "_slot_types"):
            continue

        # Tries to receive a message
        msg = rospy.wait_for_message(topic, msg_class, timeout=0.5)

        if hasattr(msg, "header") and hasattr(msg.header, "frame_id"):

            if msg.header.frame_id == TARGET_FRAME:
                print(topic)

    except Exception:
        pass