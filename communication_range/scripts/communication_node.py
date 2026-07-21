#!/usr/bin/env python3

import math
from itertools import combinations
import rospy
from nav_msgs.msg import Odometry
from planner_msgs.msg import CommunicationTrigger

class DroneCommNode:
    def __init__(self):
        # Parameters
        self.comm_threshold = rospy.get_param("~communication_threshold", 10.0)
        self.cooldown_time = rospy.get_param("~cooldown_time", 30.0)
        self.update_period = rospy.get_param("~update_period", 0.1)
        self.odometry_topic = rospy.get_param(
            "~odometry_topic", "odometry_sensor1/odometry"
        )
        self.robot_names = self.get_robot_names()
        self.robot_ids = {
            robot_name: self.robot_id_from_name(robot_name)
            for robot_name in self.robot_names
        }

        self.robot_poses = {robot_name: None for robot_name in self.robot_names}
        self.last_trigger_time = {}

        # Generate all the combination of possible robots communication
        for robot_a, robot_b in combinations(self.robot_names, 2):
            pair = self.pair_key(robot_a, robot_b)
            self.last_trigger_time[pair] = rospy.Time(0)

        # Subscribers
        # Update the robots poses iteratively
        self.pose_subscribers = []
        for robot_name in self.robot_names:
            topic = f"/{robot_name}/{self.odometry_topic}"
            self.pose_subscribers.append(
                rospy.Subscriber(
                    topic,
                    Odometry,
                    self.pose_callback,
                    callback_args=robot_name,
                )
            )

        # Publishers
        # Sends the trigger only to the robots pair in communication
        self.comm_trigger_pubs = {
            robot_name: rospy.Publisher(
                f"/{robot_name}/trigger_communication",
                CommunicationTrigger,
                queue_size=10,
            )
            for robot_name in self.robot_names
        }

        rospy.loginfo(
            "[COMMUNICATION NODE] Monitoring %d robots: %s",
            len(self.robot_names),
            ", ".join(self.robot_names),
        )
        rospy.Timer(rospy.Duration(self.update_period), self.update)

    def get_robot_names(self):
        robots_config = rospy.get_param("/robots_config/robots", None)
        if robots_config:
            return [robot["name"] for robot in robots_config if "name" in robot]

    def robot_id_from_name(self, robot_name):
        try:
            return int(robot_name.rsplit("_", 1)[1])
        except (IndexError, ValueError):
            rospy.logwarn(
                "[COMMUNICATION NODE] Could not parse numeric id from robot name '%s'; using 0",
                robot_name,
            )
            return 0

    def pair_key(self, robot_a, robot_b):
        return tuple(sorted((robot_a, robot_b)))

    def pose_callback(self, msg, robot_name):
        self.robot_poses[robot_name] = msg.pose.pose.position

    def distance(self, robot_a, robot_b):
        pose_a = self.robot_poses[robot_a]
        pose_b = self.robot_poses[robot_b]
        if pose_a is None or pose_b is None:
            return None

        dx = pose_a.x - pose_b.x
        dy = pose_a.y - pose_b.y
        dz = pose_a.z - pose_b.z

        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def pair_is_in_cooldown(self, pair, now):
        elapsed = (now - self.last_trigger_time[pair]).to_sec()
        return elapsed <= self.cooldown_time

    def update(self, event):
        now = rospy.Time.now()
        for robot_a, robot_b in combinations(self.robot_names, 2):
            pair = self.pair_key(robot_a, robot_b)
            dist = self.distance(robot_a, robot_b)
            if dist is None:
                continue

            if dist < self.comm_threshold:
                if self.pair_is_in_cooldown(pair, now):
                    continue

                self.trigger_communication(robot_a, robot_b, dist)
                self.last_trigger_time[pair] = now

    def trigger_communication(self, robot_a, robot_b, dist):
        robot_a_id = self.robot_ids[robot_a]
        robot_b_id = self.robot_ids[robot_b]
        rospy.loginfo(
            "[COMMUNICATION NODE] %s and %s in range (%.2fm); triggering exchange",
            robot_a,
            robot_b,
            dist,
        )

        msg = CommunicationTrigger()
        msg.robot_a = robot_a_id
        msg.robot_b = robot_b_id
        self.comm_trigger_pubs[robot_a].publish(msg)
        self.comm_trigger_pubs[robot_b].publish(msg)

if __name__ == "__main__":
    rospy.init_node("communication_node")
    node = DroneCommNode()
    rospy.spin()
