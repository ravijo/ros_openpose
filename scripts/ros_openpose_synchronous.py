#!/usr/bin/env python

# import modules
import sys
import cv2
import rospy
import argparse
import message_filters
from cv_bridge import CvBridge, CvBridgeError
from ros_openpose.msg import Frame, Person, BodyPart, Pixel
from sensor_msgs.msg import Image, CameraInfo


# Import Openpose (Ubuntu)
rospy.init_node('ros_openpose')
py_openpose_path = rospy.get_param("~py_openpose_path")
try:
    # If you run `make install` (default path is `/usr/local/python` for Ubuntu)
    sys.path.append(py_openpose_path)
    from openpose import pyopenpose as op
except ImportError as e:
    rospy.logerr('OpenPose library could not be found. '
                 'Did you enable `BUILD_PYTHON` in CMake and have this Python script in the right folder?')
    raise e


OPENPOSE1POINT7_OR_HIGHER = 'VectorDatum' in op.__dict__


class rosOpenPose:
    def __init__(self, frame_id, no_depth, pub_topic, color_topic, depth_topic, cam_info_topic, op_wrapper, display):
        image_sub = message_filters.Subscriber(color_topic, Image)
        depth_sub = message_filters.Subscriber(depth_topic, Image)
        self.ts = message_filters.ApproximateTimeSynchronizer([image_sub, depth_sub], 1, 0.01)
        self.ts.registerCallback(self.callback)

        self.pub = rospy.Publisher(pub_topic, Frame, queue_size=10)

        self.frame_id = frame_id
        self.no_depth = no_depth

        self.bridge = CvBridge()

        self.op_wrapper = op_wrapper

        self.display = display
        self.frame = None

        # This subscriber is run only once to populate necessary K matrix values.
        self.info_sub = rospy.Subscriber(cam_info_topic, CameraInfo, self.get_info_callback)
        self.fx = False
        self.fy = False
        self.cx = False
        self.cy = False

        """ OpenPose skeleton dictionary
        {0, "Nose"}, {13, "LKnee"}
        {1, "Neck"}, {14, "LAnkle"}
        {2, "RShoulder"}, {15, "REye"}
        {3, "RElbow"}, {16, "LEye"}
        {4, "RWrist"}, {17, "REar"}
        {5, "LShoulder"}, {18, "LEar"}
        {6, "LElbow"}, {19, "LBigToe"}
        {7, "LWrist"}, {20, "LSmallToe"}
        {8, "MidHip"}, {21, "LHeel"}
        {9, "RHip"}, {22, "RBigToe"}
        {10, "RKnee"}, {23, "RSmallToe"}
        {11, "RAnkle"}, {24, "RHeel"}
        {12, "LHip"}, {25, "Background"}
        """

    def get_info_callback(self, cam_info):
        self.fx = cam_info.K[0]
        self.cx = cam_info.K[2]
        self.fy = cam_info.K[4]
        self.cy = cam_info.K[5]
        self.info_sub.unregister()

    def convert_to_3d(self, u, v, depth):
        if self.no_depth: return 0, 0, 0
        z = depth[int(v), int(u)] / 1000
        x = (z / self.fx) * (u - self.cx)
        y = (z / self.fy) * (v - self.cy)
        return x, y, z

    def callback(self, ros_image, ros_depth):
        # Don't process if we have not obtained K matrix yet
        if not (self.fx and self.cx and self.fy and self.cy):
            return

        # Construct a frame with current time !before! pushing to OpenPose
        fr = Frame()
        fr.header.frame_id = self.frame_id
        fr.header.stamp = rospy.Time.now()

        # Convert images to cv2 matrices
        image = depth = None
        try:
            image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
            depth = self.bridge.imgmsg_to_cv2(ros_depth, "32FC1")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        # Push data to OpenPose and block while waiting for results
        datum = op.Datum()
        datum.cvInputData = image

        if OPENPOSE1POINT7_OR_HIGHER:
            self.op_wrapper.emplaceAndPop(op.VectorDatum([datum]))
        else:
            self.op_wrapper.emplaceAndPop([datum])

        pose_kp = datum.poseKeypoints
        lhand_kp = datum.handKeypoints[0]
        rhand_kp = datum.handKeypoints[1]

        # Set number of people detected
        if pose_kp is not None:
            if pose_kp.shape == ():
                num_persons = 0
                body_part_count = 0
            else:
                num_persons = pose_kp.shape[0]
                body_part_count = pose_kp.shape[1]
        else:
            return

        # Check to see if hands were detected
        lhand_detected = False
        rhand_detected = False
        hand_part_count = 0

        if lhand_kp is not None:
            if lhand_kp.shape != ():
                lhand_detected = True
                hand_part_count = lhand_kp.shape[1]

        if rhand_kp is not None:
            if rhand_kp.shape != ():
                rhand_detected = True
                hand_part_count = rhand_kp.shape[1]

        # Handle body points
        fr.persons = [Person() for _ in range(num_persons)]
        try:
            for person in range(num_persons):
                fr.persons[person].bodyParts = [BodyPart() for _ in range(body_part_count)]
                fr.persons[person].leftHandParts = [BodyPart() for _ in range(hand_part_count)]
                fr.persons[person].rightHandParts = [BodyPart() for _ in range(hand_part_count)]

                detected_hands = []
                if lhand_detected:
                    detected_hands.append((lhand_kp, fr.persons[person].leftHandParts))
                if rhand_detected:
                    detected_hands.append((rhand_kp, fr.persons[person].rightHandParts))

                # Process the body
                for bp in range(body_part_count):
                    u, v, s = pose_kp[person, bp]
                    x, y, z = self.convert_to_3d(u, v, depth)
                    arr = fr.persons[person].bodyParts[bp]
                    arr.pixel.x = u
                    arr.pixel.y = v
                    arr.score = s
                    arr.point.x = x
                    arr.point.y = y
                    arr.point.z = z

                # Process left and right hands
                for hp in range(hand_part_count):
                    for kp, harr in detected_hands:
                        u, v, s = kp[person, hp]
                        x, y, z = self.convert_to_3d(u, v, depth)
                        arr = harr[hp]
                        arr.pixel.x = u
                        arr.pixel.y = v
                        arr.score = s
                        arr.point.x = x
                        arr.point.y = y
                        arr.point.z = z

        except IndexError as e:
            rospy.logerr("Indexing error occured: {}".format(e))
            # return

        if self.display: self.frame = datum.cvOutputData.copy()
        self.pub.publish(fr)


def main():
    frame_id = rospy.get_param("~frame_id")
    no_depth = rospy.get_param("~no_depth")
    pub_topic = rospy.get_param("~pub_topic")
    color_topic = rospy.get_param("~color_topic")
    depth_topic = rospy.get_param("~depth_topic")
    cam_info_topic = rospy.get_param("~cam_info_topic")
    try:
        # Flags, refer to include/openpose/flags.hpp for more parameters
        parser = argparse.ArgumentParser()
        args = parser.parse_known_args()

        # Custom Params
        params = dict()
        # Can manually set params like this as well
        # params["model_folder"] = "/home/asjchoi/Programs/openpose-1.6.0/models"

        # Any more obscure flags can be found through this for loop
        for i in range(0, len(args[1])):
            curr_item = args[1][i]
            if i != len(args[1])-1: next_item = args[1][i+1]
            else: next_item = "1"
            if "--" in curr_item and "--" in next_item:
                key = curr_item.replace('-', '')
                if key not in params:  params[key] = "1"
            elif "--" in curr_item and "--" not in next_item:
                key = curr_item.replace('-', '')
                if key not in params: params[key] = next_item

        # Starting OpenPose
        op_wrapper = op.WrapperPython()
        op_wrapper.configure(params)
        op_wrapper.start()

        display = True if 'display' not in params or int(params['display']) > 0 else False

        # Start ros wrapper
        rop = rosOpenPose(frame_id, no_depth, pub_topic, color_topic, depth_topic, cam_info_topic, op_wrapper, display)

        if display:
            while not rospy.is_shutdown():
                if rop.frame is not None:
                    cv2.imshow("Ros OpenPose", rop.frame)
                    cv2.waitKey(1)
        else:
            rospy.spin()

    except Exception as e:
        rospy.logerr(e)
        sys.exit(-1)


if __name__ == "__main__":
    main()
