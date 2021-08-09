#!/usr/bin/env python
# -*- coding: utf-8 -*-

# visualizer.py: rviz visualizer
# Author: Ravi Joshi
# Date: 2019/10/01

# import modules
import math
import rospy
from ros_openpose.msg import Frame
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Vector3, Point
from visualization_msgs.msg import Marker, MarkerArray


class RealtimeVisualization():
    def __init__(self, ns, frame_topic, skeleton_frame, id_text_size, id_text_offset, skeleton_hands, skeleton_line_width):
        self.ns = ns
        self.skeleton_frame = skeleton_frame
        self.id_text_size = id_text_size
        self.id_text_offset = id_text_offset
        self.skeleton_hands = skeleton_hands
        self.skeleton_line_width = skeleton_line_width

        # define a few colors we are going to use later on
        self.colors = [ColorRGBA(0.12, 0.63, 0.42, 1.00),
                       ColorRGBA(0.98, 0.30, 0.30, 1.00),
                       ColorRGBA(0.26, 0.09, 0.91, 1.00),
                       ColorRGBA(0.77, 0.44, 0.14, 1.00),
                       ColorRGBA(0.92, 0.73, 0.14, 1.00),
                       ColorRGBA(0.00, 0.61, 0.88, 1.00),
                       ColorRGBA(1.00, 0.65, 0.60, 1.00),
                       ColorRGBA(0.59, 0.00, 0.56, 1.00)]

        '''
        The skeleton is considered as a combination of line strips.
        Hence, the skeleton is decomposed into 3 LINE_STRIP as following:
            1) upper_body : from nose to mid hip
            2) hands : from left-hand wrist to right-hand wrist
            3) legs : from left foot toe to right foot toe

        See the link below to get the id of each joint as defined in Kinect v2
        src: https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/output.md#keypoint-ordering
        Result for BODY_25 (25 body parts consisting of COCO + foot)
        const std::map<unsigned int, std::string> POSE_BODY_25_BODY_PARTS {
            { 0,      "Nose"},    {13,      "LKnee"}
            { 1,      "Neck"},    {14,     "LAnkle"}
            { 2, "RShoulder"},    {15,       "REye"}
            { 3,    "RElbow"},    {16,       "LEye"}
            { 4,    "RWrist"},    {17,       "REar"}
            { 5, "LShoulder"},    {18,       "LEar"}
            { 6,    "LElbow"},    {19,    "LBigToe"}
            { 7,    "LWrist"},    {20,  "LSmallToe"}
            { 8,    "MidHip"},    {21,      "LHeel"}
            { 9,      "RHip"},    {22,    "RBigToe"}
            {10,     "RKnee"},    {23,  "RSmallToe"}
            {11,    "RAnkle"},    {24,      "RHeel"}
            {12,      "LHip"},    {25, "Background"}


        hand output ordering
        src: https://github.com/CMU-Perceptual-Computing-Lab/openpose/raw/master/doc/media/keypoints_hand.png
        We are using 5 LINE_STRIP to draw a hand
        '''

        self.upper_body_ids = [0, 1, 8]
        self.hands_ids = [4, 3, 2, 1, 5, 6, 7]
        self.legs_ids = [22, 11, 10, 9, 8, 12, 13, 14, 19]
        self.body_parts = [self.upper_body_ids, self.hands_ids, self.legs_ids]

        # number of fingers in a hand
        self.fingers = 5

        # number of keypoints to denote a finger
        self.count_keypoints_one_finger = 5

        self.total_finger_kepoints = self.fingers * self.count_keypoints_one_finger

        # write person id on the top of his head
        self.nose_id = 0

        # define a publisher to publish the 3D skeleton of multiple people
        self.skeleton_pub = rospy.Publisher(self.ns, MarkerArray, queue_size=1)

        # define a subscriber to retrive tracked bodies
        rospy.Subscriber(frame_topic, Frame, self.frame_callback)


    def spin(self):
        '''
        We enter in a loop and wait for exit whenever `Ctrl + C` is pressed
        '''
        rospy.spin()


    def create_marker(self, index, color, marker_type, size, time):
        '''
        Function to create a visualization marker which is used inside RViz
        '''
        marker = Marker()
        marker.id = index
        marker.ns = self.ns
        marker.color = color
        marker.action = Marker.ADD
        marker.type = marker_type
        marker.scale = Vector3(size, size, size)
        marker.pose.orientation.w = 1
        marker.header.stamp = time
        marker.header.frame_id = self.skeleton_frame
        marker.lifetime = rospy.Duration(1)  # 1 second
        return marker


    def isValid(self, bodyPart):
        '''
        When should we consider a body part as a valid entity?
        We make sure that the score and z coordinate is a positive number.
        Notice that the z coordinate denotes the distance of the object located
        in front of the camera. Therefore it must be a positive number always.
        '''
        return bodyPart.score > 0 and not math.isnan(bodyPart.point.x) and not math.isnan(bodyPart.point.y) and not math.isnan(bodyPart.point.z) and bodyPart.point.z > 0


    def frame_callback(self, data):
        '''
        This function will be called everytime whenever a message is received by the subscriber
        '''
        marker_counter = 0
        person_counter = 0
        marker_array = MarkerArray()

        for person in data.persons:
            now = rospy.Time.now()
            marker_color = self.colors[person_counter % len(self.colors)]

            # the body_marker contains three markers as mentioned already
            # 1. upper body 2. hands 3. legs
            body_marker = [self.create_marker(marker_counter + idx, marker_color, Marker.LINE_STRIP, self.skeleton_line_width, now) for idx in range(len(self.body_parts))]
            marker_counter += len(self.body_parts)

            # assign 3D positions to each body part
            # make sure to consider only valid body parts
            for index, body_part in enumerate(self.body_parts):
                body_marker[index].points = [person.bodyParts[idx].point for idx in body_part if self.isValid(person.bodyParts[idx])]

            marker_array.markers.extend(body_marker)

            if self.skeleton_hands:
                left_hand = [self.create_marker(marker_counter + idx, marker_color, Marker.LINE_STRIP, self.skeleton_line_width, now) for idx in range(self.fingers)]
                marker_counter += self.fingers

                right_hand = [self.create_marker(marker_counter + idx, marker_color, Marker.LINE_STRIP, self.skeleton_line_width, now) for idx in range(self.fingers)]
                marker_counter += self.fingers

                keypoint_counter = 0
                for idx in range(self.total_finger_kepoints):
                    strip_id = idx / self.count_keypoints_one_finger
                    temp_id = idx % self.count_keypoints_one_finger
                    if temp_id == 0:
                        point_id = temp_id
                    else:
                        keypoint_counter += 1
                        point_id = keypoint_counter

                    leftHandPart = person.leftHandParts[point_id]
                    rightHandPart = person.rightHandParts[point_id]
                    if self.isValid(leftHandPart):
                        left_hand[strip_id].points.append(leftHandPart.point)

                    if self.isValid(rightHandPart):
                        right_hand[strip_id].points.append(rightHandPart.point)
                marker_array.markers.extend(left_hand)
                marker_array.markers.extend(right_hand)

            person_id = self.create_marker(marker_counter, marker_color, Marker.TEXT_VIEW_FACING, self.id_text_size, now)
            marker_counter += 1
            # assign person id and 3D position
            person_id.text = str(person_counter)
            nose = person.bodyParts[self.nose_id]
            if self.isValid(nose):
                person_id.pose.position = Point(nose.point.x, nose.point.y + self.id_text_offset, nose.point.z)
                marker_array.markers.append(person_id)

            # update the counter
            person_counter += 1

        # publish the markers
        self.skeleton_pub.publish(marker_array)


if __name__ == '__main__':
    # define some constants
    ns = 'visualization'

    # initialize ros node
    rospy.init_node('visualizer_node', anonymous=False)

    # read the parameters from ROS parameter server
    frame_topic = rospy.get_param('~pub_topic')
    skeleton_frame = rospy.get_param('~frame_id')
    id_text_size = rospy.get_param('~id_text_size')
    id_text_offset = rospy.get_param('~id_text_offset')
    skeleton_hands = rospy.get_param('~skeleton_hands')
    skeleton_line_width = rospy.get_param('~skeleton_line_width')

    # instantiate the RealtimeVisualization class
    visualization = RealtimeVisualization(ns, frame_topic, skeleton_frame, id_text_size, id_text_offset, skeleton_hands, skeleton_line_width)
    visualization.spin()
