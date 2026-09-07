#!/usr/bin/env python3

"""
Transform Multiple Odometry and Pose Topics from World Frame to Local Frame

This node subscribes to multiple odometry and pose topics (in world frame) and publishes
them transformed to a robot-specific local frame using the tf transform published 
by the static_transform_publisher.

The transform is obtained from:
  static_transform_publisher world -> robot_name/local_frame

Usage with YAML config file (recommended):
  rosrun gbplanner tf_transform_world_to_local.py \
    _config_file:=$(find gbplanner)/config/odometry_transform_config.yaml \
    _target_frame:=robot_name/local_frame

Usage with ROS parameters (multiple odometry and pose topics):
  rosrun gbplanner tf_transform_world_to_local.py \
    _input_topics:="[ground_truth/odometry,odometry_sensor1/odometry]" \
    _output_topics:="[ground_truth/odometry_local,odometry_sensor1/odometry_local]" \
    _input_pose_topics:="[ground_truth/pose_with_covariance]" \
    _output_pose_topics:="[ground_truth/pose_with_covariance_local]" \
    _source_frame:=world \
    _target_frame:=robot_name/local_frame

Usage with single topic (backward compatible):
  rosrun gbplanner tf_transform_world_to_local.py \
    _input_topic:=/odom_world \
    _output_topic:=/odom_local \
    _source_frame:=world \
    _target_frame:=robot_name/local_frame
"""

import rospy
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import numpy as np
import yaml
import os
from scipy.spatial.transform import Rotation

class WorldToLocalTransformer:
    """Transform multiple odometry and pose messages between reference frames using tf."""
    
    def __init__(self):
        """Initialize the odometry transformer node."""
        rospy.init_node('tf_transform_odometry', anonymous=True)
        
        # TF2 buffers and listeners
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # Try to load from YAML config file first
        config_file = rospy.get_param('~config_file', None)
        
        if config_file:
            self._load_from_config_file(config_file)
        else:
            self._load_from_parameters()
    
    def _load_from_config_file(self, config_file):
        """
        Load configuration from a YAML file.
        
        Args:
            config_file: Path to YAML configuration file
        """
        # Expand ROS package paths (e.g., $(find gbplanner)/config/...)
        config_file = os.path.expanduser(config_file)
        
        if not os.path.exists(config_file):
            rospy.logerr(f"Config file not found: {config_file}")
            raise FileNotFoundError(f"Config file not found: {config_file}")
        
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
        except yaml.YAMLError as e:
            rospy.logerr(f"Error parsing YAML config file: {e}")
            raise
        
        # Extract frames
        self.source_frame = rospy.get_param('~source_frame', config.get('source_frame', 'world'))
        self.target_frame = rospy.get_param('~target_frame', config.get('target_frame', 'local_frame'))
        
        # Extract odometry topics
        odometry_config = config.get('odometry_topics', [])
        self.input_topics = []
        self.output_topics = []
        
        if isinstance(odometry_config, list):
            for topic_pair in odometry_config:
                if isinstance(topic_pair, dict) and 'input' in topic_pair and 'output' in topic_pair:
                    self.input_topics.append(topic_pair['input'])
                    self.output_topics.append(topic_pair['output'])
        
        if not self.input_topics:
            rospy.logwarn("No odometry topics configured in YAML file")
        
        # Extract pose topics
        pose_config = config.get('pose_topics', [])
        self.input_pose_topics = []
        self.output_pose_topics = []
        
        if isinstance(pose_config, list):
            for topic_pair in pose_config:
                if isinstance(topic_pair, dict) and 'input' in topic_pair and 'output' in topic_pair:
                    self.input_pose_topics.append(topic_pair['input'])
                    self.output_pose_topics.append(topic_pair['output'])
        
        # Create subscribers and publishers
        self._setup_subscribers_and_publishers()
        
        rospy.loginfo(f"Config loaded from: {config_file}")
        rospy.loginfo(f"Transform: {self.source_frame} -> {self.target_frame}")
    
    def _load_from_parameters(self):
        """
        Load configuration from ROS parameters (backward compatible).
        """
        # Parameters
        self.source_frame = rospy.get_param('~source_frame', 'world')
        self.target_frame = rospy.get_param('~target_frame', 'local_frame')

        # Handle both single topic (backward compatible) and multiple topics
        input_topics_param = rospy.get_param('~input_topics', None)
        output_topics_param = rospy.get_param('~output_topics', None)
        
        if input_topics_param is not None and output_topics_param is not None:
            # Multiple topics mode
            self.input_topics = input_topics_param if isinstance(input_topics_param, list) else [input_topics_param]
            self.output_topics = output_topics_param if isinstance(output_topics_param, list) else [output_topics_param]
        else:
            # Single topic mode (backward compatible)
            input_topic = rospy.get_param('~input_topic', 'odometry_sensor1/odometry')
            output_topic = rospy.get_param('~output_topic', 'odometry_sensor1/odometry_local')
            self.input_topics = [input_topic]
            self.output_topics = [output_topic]
        
        # Handle pose topics (optional)
        input_pose_topics_param = rospy.get_param('~input_pose_topics', None)
        output_pose_topics_param = rospy.get_param('~output_pose_topics', None)
        
        if input_pose_topics_param is not None and output_pose_topics_param is not None:
            self.input_pose_topics = input_pose_topics_param if isinstance(input_pose_topics_param, list) else [input_pose_topics_param]
            self.output_pose_topics = output_pose_topics_param if isinstance(output_pose_topics_param, list) else [output_pose_topics_param]
        else:
            self.input_pose_topics = []
            self.output_pose_topics = []
        
        # Create subscribers and publishers
        self._setup_subscribers_and_publishers()
        
        rospy.loginfo(f"Config loaded from ROS parameters")
        rospy.loginfo(f"Transform: {self.source_frame} -> {self.target_frame}")
    
    def _setup_subscribers_and_publishers(self):
        """
        Create subscribers and publishers for configured topics.
        """
        # Validate topics match
        if len(self.input_topics) != len(self.output_topics):
            rospy.logerr("Number of input topics must match number of output topics!")
            raise ValueError("Input and output topic counts do not match")
        
        # Validate pose topics match
        if len(self.input_pose_topics) != len(self.output_pose_topics):
            rospy.logerr("Number of input pose topics must match number of output pose topics!")
            raise ValueError("Input and output pose topic counts do not match")
        
        # Create publishers and subscribers for odometry topics
        self.subscribers = {}
        
        for input_topic, output_topic in zip(self.input_topics, self.output_topics):
            pub = rospy.Publisher(output_topic, Odometry, queue_size=10)
            # Create callback with closure to capture publisher
            callback = self._create_odometry_callback(pub)
            self.subscribers[input_topic] = rospy.Subscriber(input_topic, Odometry, callback, queue_size=10)
            rospy.loginfo(f"Odometry transformer: {input_topic} -> {output_topic}")
        
        # Create publishers and subscribers for pose topics
        self.pose_subscribers = {}
        
        for input_pose_topic, output_pose_topic in zip(self.input_pose_topics, self.output_pose_topics):
            pub = rospy.Publisher(output_pose_topic, PoseWithCovarianceStamped, queue_size=10)
            # Create callback with closure to capture publisher
            callback = self._create_pose_callback(pub)
            self.pose_subscribers[input_pose_topic] = rospy.Subscriber(input_pose_topic, PoseWithCovarianceStamped, callback, queue_size=10)
            rospy.loginfo(f"Pose transformer: {input_pose_topic} -> {output_pose_topic}")
    
    def _create_odometry_callback(self, publisher):
        """
        Create a callback function for odometry messages.
        
        Args:
            publisher: ROS Publisher for the output odometry topic
            
        Returns:
            Callback function bound to this publisher
        """
        def callback(odom_msg):
            self._transform_and_publish_odometry(odom_msg, publisher)
        return callback
    
    def _create_pose_callback(self, publisher):
        """
        Create a callback function for pose messages.
        
        Args:
            publisher: ROS Publisher for the output pose topic
            
        Returns:
            Callback function bound to this publisher
        """
        def callback(pose_msg):
            self._transform_and_publish_pose(pose_msg, publisher)
        return callback
    
    def _transform_and_publish_odometry(self, odom_msg, publisher):
        """
        Transform odometry message and publish to output topic.
        
        Args:
            odom_msg (nav_msgs/Odometry): Odometry message in source frame
            publisher: ROS Publisher for the output topic
        """
        try:
            # Get the transform from source frame to target frame
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                odom_msg.header.stamp,
                timeout=rospy.Duration(1.0)
            )
            
            # Create transformed odometry message
            transformed_odom = Odometry()
            transformed_odom.header.stamp = odom_msg.header.stamp
            transformed_odom.header.frame_id = self.target_frame
            transformed_odom.child_frame_id = odom_msg.child_frame_id
            
            # Transform position
            pose_stamped = tf2_geometry_msgs.PoseStamped()
            pose_stamped.header = odom_msg.header
            pose_stamped.pose = odom_msg.pose.pose
            
            try:
                transformed_pose_stamped = self.tf_buffer.transform(
                    pose_stamped,
                    self.target_frame,
                    timeout=rospy.Duration(1.0)
                )
                transformed_odom.pose.pose = transformed_pose_stamped.pose
            except tf2_ros.TransformException as e:
                rospy.logwarn(f"Could not transform pose: {e}")
                return
            
            # Transform pose covariance using rotation from tf
            transformed_odom.pose.covariance = self._transform_covariance(
                odom_msg.pose.covariance,
                transform
            )
            
            # Copy twist as-is: twist is expressed in child_frame_id (body frame)
            # which doesn't change when transforming between reference frames
            transformed_odom.twist.twist = odom_msg.twist.twist
            transformed_odom.twist.covariance = odom_msg.twist.covariance
            
            # Publish transformed odometry
            publisher.publish(transformed_odom)
            
        except tf2_ros.TransformException as e:
            rospy.logwarn(f"Could not get transform {self.source_frame} -> {self.target_frame}: {e}")
            return
    
    def _transform_and_publish_pose(self, pose_msg, publisher):
        """
        Transform PoseWithCovarianceStamped message and publish to output topic.
        
        Args:
            pose_msg (geometry_msgs/PoseWithCovarianceStamped): Pose message in source frame
            publisher: ROS Publisher for the output pose topic
        """
        try:
            # Get the transform from source frame to target frame
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.source_frame,
                pose_msg.header.stamp,
                timeout=rospy.Duration(1.0)
            )
            
            # Create transformed pose message
            transformed_pose = PoseWithCovarianceStamped()
            transformed_pose.header.stamp = pose_msg.header.stamp
            transformed_pose.header.frame_id = self.target_frame
            
            # Transform position and orientation
            pose_stamped = tf2_geometry_msgs.PoseStamped()
            pose_stamped.header = pose_msg.header
            pose_stamped.pose = pose_msg.pose.pose
            
            try:
                transformed_pose_stamped = self.tf_buffer.transform(
                    pose_stamped,
                    self.target_frame,
                    timeout=rospy.Duration(1.0)
                )
                transformed_pose.pose.pose = transformed_pose_stamped.pose
            except tf2_ros.TransformException as e:
                rospy.logwarn(f"Could not transform pose: {e}")
                return
            
            # Transform pose covariance using rotation from tf
            transformed_pose.pose.covariance = self._transform_covariance(
                pose_msg.pose.covariance,
                transform
            )
            
            # Publish transformed pose
            publisher.publish(transformed_pose)
            
        except tf2_ros.TransformException as e:
            rospy.logwarn(f"Could not get transform {self.source_frame} -> {self.target_frame}: {e}")
            return
    
    def _get_rotation_matrix(self, transform):
        """
        Extract rotation matrix from tf transform.
        
        Args:
            transform: geometry_msgs/TransformStamped
            
        Returns:
            np.ndarray: 3x3 rotation matrix
        """
        quat = transform.transform.rotation
        rotation = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w])
        return rotation.as_dcm()
    
    def _transform_covariance(self, covariance, transform):
        """
        Transform a 6x6 covariance matrix using rotation from tf transform.
        
        For pose covariance, applies: Cov_transformed = R_block · Cov · R_block^T
        where R_block is a block diagonal matrix with rotation in both position and rotation blocks.
        
        Args:
            covariance: Input 6x6 covariance as flat tuple/list
            transform: geometry_msgs/TransformStamped
            
        Returns:
            Transformed covariance as tuple
        """
        rotation_matrix = self._get_rotation_matrix(transform)
        
        # Reshape flat covariance to 6x6 matrix
        cov_6x6 = np.array(covariance).reshape(6, 6)
        
        # Create block diagonal rotation matrix for 6D (position and orientation)
        R_block = np.eye(6)
        R_block[:3, :3] = rotation_matrix
        R_block[3:, 3:] = rotation_matrix
        
        # Transform covariance: Cov_transformed = R_block · Cov · R_block^T
        transformed_cov = R_block @ cov_6x6 @ R_block.T
        
        return tuple(transformed_cov.flatten())
    
def main():
    """Main function to run the odometry transformer node."""
    transformer = WorldToLocalTransformer()
    rospy.spin()


if __name__ == '__main__':
    main()

