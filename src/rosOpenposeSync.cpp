#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <openpose/flags.hpp>
#include <openpose/headers.hpp>
#include <ros_openpose/Frame.h>
#include <ros_openpose/BodyPart.h>

// define a macro for compatibility with older versions
#define OPENPOSE1POINT6_OR_HIGHER OpenPose_VERSION_MAJOR >= 1 && OpenPose_VERSION_MINOR >= 6
#define OPENPOSE1POINT7POINT1_OR_HIGHER OpenPose_VERSION_MAJOR >= 1 && OpenPose_VERSION_MINOR >= 7 && OpenPose_VERSION_PATCH >=1

using namespace sensor_msgs;
class rosOpenPose
{
private:
    op::Wrapper* _op_wrapper;
    ros::NodeHandle* _nh;
    ros::Publisher _pub;
    message_filters::Subscriber<Image> _color_sub;
    message_filters::Subscriber<Image> _depth_sub;
    typedef message_filters::sync_policies::ApproximateTime<Image, Image> ColorDepthSyncPolicy;
    typedef message_filters::Synchronizer<ColorDepthSyncPolicy> ColorDepthSync;
    std::shared_ptr<ColorDepthSync> _sync;

    bool _no_depth;
    float _fx, _fy, _cx, _cy;
    float _mm_to_m;

    cv::Mat _color_img, _depth_img;
    ros_openpose::Frame _frame_msg;
public:
    rosOpenPose(ros::NodeHandle* nh, op::Wrapper* op_wrapper, const std::string& color_topic, const std::string& depth_topic,
                const std::string& cam_info_topic, const std::string& pub_topic, const std::string& frame_id, const bool& no_depth):
                _nh(nh), _op_wrapper(op_wrapper), _no_depth(no_depth) {

        _frame_msg.header.frame_id = frame_id;

        // Populate camera intrinsic matrix values.
        auto cam_info = ros::topic::waitForMessage<CameraInfo>(cam_info_topic);
        _fx = cam_info->K.at(0);
        _fy = cam_info->K.at(4);
        _cx = cam_info->K.at(2);
        _cy = cam_info->K.at(5);

        // Obtain depth encoding.
        auto depth_encoding = ros::topic::waitForMessage<Image>(depth_topic)->encoding;
        _mm_to_m = (depth_encoding == image_encodings::TYPE_16UC1) ? 0.001 : 1.;

        // Initialize frame publisher
        _pub = _nh->advertise<ros_openpose::Frame>(pub_topic, 10);

        // Start color & depth subscribers.
        _color_sub.subscribe(*_nh, color_topic, 1);
        _depth_sub.subscribe(*_nh, depth_topic, 1);
        _sync.reset(new ColorDepthSync(ColorDepthSync(10), _color_sub, _depth_sub));
        _sync->registerCallback(boost::bind(&rosOpenPose::callback, this, _1, _2));
    }

    template <typename key_points>
    void assign_msg_vals(ros_openpose::BodyPart& part, const key_points& kp, const int& i) {
        // Assign pixel position and score from Openpose.
        float u = kp[i], v = kp[i+1], s = kp[i+2];
        part.pixel.x = u;
        part.pixel.y = v;
        part.score = s;

        // Compute 3D Pose if depth is provided.
        if (!_no_depth) {
            auto depth = _depth_img.at<float>(static_cast<int>(v), static_cast<int> (u)) * _mm_to_m;
            if (depth <= 0) return;
            part.point.x = (depth / _fx) * (u - _cx);
            part.point.y = (depth / _fy) * (v - _cy);
            part.point.z = depth;
        }
    }

    void callback(const ImageConstPtr& color_msg, const ImageConstPtr& depth_msg) {
        _frame_msg.header.stamp = ros::Time::now();
        _frame_msg.persons.clear();

        _color_img = cv_bridge::toCvShare(color_msg, image_encodings::BGR8)->image;
        _depth_img = cv_bridge::toCvShare(depth_msg, image_encodings::TYPE_32FC1)->image;

        // Fill datum
#if OPENPOSE1POINT6_OR_HIGHER
        auto datum_ptr = _op_wrapper->emplaceAndPop(OP_CV2OPCONSTMAT(_color_img));
#else
        auto datum_ptr = _op_wrapper->emplaceAndPop(_color_img);
#endif

        const auto& pose_kp = datum_ptr->at(0)->poseKeypoints;
        const auto& hand_kp = datum_ptr->at(0)->handKeypoints;

        // get the size
        const auto num_persons = pose_kp.getSize(0);
        const auto body_part_count = pose_kp.getSize(1);
        const auto hand_part_count = hand_kp[0].getSize(1);

        _frame_msg.persons.resize(num_persons);
        int i;
        for (auto p = 0; p < num_persons; p++) {
            auto& curr_person = _frame_msg.persons[p];

            curr_person.bodyParts.resize(body_part_count);
            curr_person.leftHandParts.resize(hand_part_count);
            curr_person.rightHandParts.resize(hand_part_count);

            // Fill body parts
            for (auto bp = 0; bp < body_part_count; bp++) {
                auto& curr_body_part = curr_person.bodyParts[bp];
                i = pose_kp.getSize(2) * (p * body_part_count + bp);
                assign_msg_vals(curr_body_part, pose_kp, i);
            }

            // Fill left and right hands
            for (auto hp = 0; hp < hand_part_count; hp++) {
                i = hand_kp[0].getSize(2) * (p * hand_part_count + hp);

                // Left Hand
                auto& curr_left_hand = curr_person.leftHandParts[hp];
                assign_msg_vals(curr_left_hand, hand_kp[0], i);

                // Right Hand
                auto& curr_right_hand = curr_person.rightHandParts[hp];
                assign_msg_vals(curr_right_hand, hand_kp[1], i);
            }
        }
        _pub.publish(_frame_msg);
    }
};


void configureOpenPose(op::Wrapper& opWrapper) {
    try
    {
#if OPENPOSE1POINT6_OR_HIGHER
        op::checkBool(0 <= FLAGS_logging_level && FLAGS_logging_level <= 255,
#else
        op::check(0 <= FLAGS_logging_level && FLAGS_logging_level <= 255,
#endif
                  "Wrong logging_level value.",
                  __LINE__,
                  __FUNCTION__,
                  __FILE__);

        op::ConfigureLog::setPriorityThreshold((op::Priority)FLAGS_logging_level);
        op::Profiler::setDefaultX(FLAGS_profile_speed);

// Applying user defined configuration - GFlags to program variables
// outputSize
#if OPENPOSE1POINT6_OR_HIGHER
        const auto outputSize = op::flagsToPoint(op::String(FLAGS_output_resolution), "-1x-1");
#else
        const auto outputSize = op::flagsToPoint(FLAGS_output_resolution, "-1x-1");
#endif

// netInputSize
#if OPENPOSE1POINT6_OR_HIGHER
        const auto netInputSize = op::flagsToPoint(op::String(FLAGS_net_resolution), "-1x368");
#else
        const auto netInputSize = op::flagsToPoint(FLAGS_net_resolution, "-1x368");
#endif

// faceNetInputSize
#if OPENPOSE1POINT6_OR_HIGHER
        const auto faceNetInputSize = op::flagsToPoint(op::String(FLAGS_face_net_resolution), "368x368 (multiples of 16)");
#else
        const auto faceNetInputSize = op::flagsToPoint(FLAGS_face_net_resolution, "368x368 (multiples of 16)");
#endif

// handNetInputSize
#if OPENPOSE1POINT6_OR_HIGHER
        const auto handNetInputSize = op::flagsToPoint(op::String(FLAGS_hand_net_resolution), "368x368 (multiples of 16)");
#else
        const auto handNetInputSize = op::flagsToPoint(FLAGS_hand_net_resolution, "368x368 (multiples of 16)");
#endif

        // poseMode
        const auto poseMode = op::flagsToPoseMode(FLAGS_body);

// poseModel
#if OPENPOSE1POINT6_OR_HIGHER
        const auto poseModel = op::flagsToPoseModel(op::String(FLAGS_model_pose));
#else
        const auto poseModel = op::flagsToPoseModel(FLAGS_model_pose);
#endif

        // JSON saving
        if (!FLAGS_write_keypoint.empty())
            ROS_INFO("Flag `write_keypoint` is deprecated and will eventually be removed. Please, use `write_json` instead.");

        // keypointScaleMode
        const auto keypointScaleMode = op::flagsToScaleMode(FLAGS_keypoint_scale);

        // heatmaps to add
        const auto heatMapTypes = op::flagsToHeatMaps(FLAGS_heatmaps_add_parts,
                                                      FLAGS_heatmaps_add_bkg,
                                                      FLAGS_heatmaps_add_PAFs);

        const auto heatMapScaleMode = op::flagsToHeatMapScaleMode(FLAGS_heatmaps_scale);

        // >1 camera view?
        // const auto multipleView = (FLAGS_3d || FLAGS_3d_views > 1 || FLAGS_flir_camera);
        const auto multipleView = false;

        // Face and hand detectors
        const auto faceDetector = op::flagsToDetector(FLAGS_face_detector);
        const auto handDetector = op::flagsToDetector(FLAGS_hand_detector);

        // Enabling Google Logging
        const bool enableGoogleLogging = true;

        // Pose configuration (use WrapperStructPose{} for default and recommended configuration)
        const op::WrapperStructPose wrapperStructPose{poseMode,
                                                      netInputSize,
#if OPENPOSE1POINT7POINT1_OR_HIGHER
                FLAGS_net_resolution_dynamic,
                                                  outputSize,
#else
                                                      outputSize,
#endif
                                                      keypointScaleMode,
                                                      FLAGS_num_gpu,
                                                      FLAGS_num_gpu_start,
                                                      FLAGS_scale_number,
                                                      (float)FLAGS_scale_gap,
                                                      op::flagsToRenderMode(FLAGS_render_pose,
                                                                            multipleView),
                                                      poseModel,
                                                      !FLAGS_disable_blending,
                                                      (float)FLAGS_alpha_pose,
                                                      (float)FLAGS_alpha_heatmap,
                                                      FLAGS_part_to_show,
#if OPENPOSE1POINT6_OR_HIGHER
                op::String(FLAGS_model_folder),
#else
                                                      FLAGS_model_folder,
#endif
                                                      heatMapTypes,
                                                      heatMapScaleMode,
                                                      FLAGS_part_candidates,
                                                      (float)FLAGS_render_threshold,
                                                      FLAGS_number_people_max,
                                                      FLAGS_maximize_positives,
                                                      FLAGS_fps_max,
#if OPENPOSE1POINT6_OR_HIGHER
                op::String(FLAGS_prototxt_path),
                                                  op::String(FLAGS_caffemodel_path),
#else
                                                      FLAGS_prototxt_path,
                                                      FLAGS_caffemodel_path,
#endif
                                                      (float)FLAGS_upsampling_ratio,
                                                      enableGoogleLogging};
        opWrapper.configure(wrapperStructPose);

        // Face configuration (use op::WrapperStructFace{} to disable it)
        const op::WrapperStructFace wrapperStructFace{FLAGS_face,
                                                      faceDetector,
                                                      faceNetInputSize,
                                                      op::flagsToRenderMode(FLAGS_face_render,
                                                                            multipleView,
                                                                            FLAGS_render_pose),
                                                      (float)FLAGS_face_alpha_pose,
                                                      (float)FLAGS_face_alpha_heatmap,
                                                      (float)FLAGS_face_render_threshold};
        opWrapper.configure(wrapperStructFace);

        // Hand configuration (use op::WrapperStructHand{} to disable it)
        const op::WrapperStructHand wrapperStructHand{FLAGS_hand,
                                                      handDetector,
                                                      handNetInputSize,
                                                      FLAGS_hand_scale_number,
                                                      (float)FLAGS_hand_scale_range,
                                                      op::flagsToRenderMode(FLAGS_hand_render,
                                                                            multipleView,
                                                                            FLAGS_render_pose),
                                                      (float)FLAGS_hand_alpha_pose,
                                                      (float)FLAGS_hand_alpha_heatmap,
                                                      (float)FLAGS_hand_render_threshold};
        opWrapper.configure(wrapperStructHand);

        // Extra functionality configuration (use op::WrapperStructExtra{} to disable it)
        const op::WrapperStructExtra wrapperStructExtra{FLAGS_3d,
                                                        FLAGS_3d_min_views,
                                                        FLAGS_identification,
                                                        FLAGS_tracking,
                                                        FLAGS_ik_threads};
        opWrapper.configure(wrapperStructExtra);

        // Output (comment or use default argument to disable any output)
        const op::WrapperStructOutput wrapperStructOutput{FLAGS_cli_verbose,
#if OPENPOSE1POINT6_OR_HIGHER
                op::String(FLAGS_write_keypoint),
#else
                                                          FLAGS_write_keypoint,
#endif
                                                          op::stringToDataFormat(FLAGS_write_keypoint_format),
#if OPENPOSE1POINT6_OR_HIGHER
                op::String(FLAGS_write_json),
                                                      op::String(FLAGS_write_coco_json),
#else
                                                          FLAGS_write_json,
                                                          FLAGS_write_coco_json,
#endif
                                                          FLAGS_write_coco_json_variants,
                                                          FLAGS_write_coco_json_variant,
#if OPENPOSE1POINT6_OR_HIGHER
                op::String(FLAGS_write_images),
                                                      op::String(FLAGS_write_images_format),
                                                      op::String(FLAGS_write_video),
#else
                                                          FLAGS_write_images,
                                                          FLAGS_write_images_format,
                                                          FLAGS_write_video,
#endif
                                                          FLAGS_write_video_fps,
                                                          FLAGS_write_video_with_audio,
#if OPENPOSE1POINT6_OR_HIGHER
                op::String(FLAGS_write_heatmaps),
                                                      op::String(FLAGS_write_heatmaps_format),
                                                      op::String(FLAGS_write_video_3d),
                                                      op::String(FLAGS_write_video_adam),
                                                      op::String(FLAGS_write_bvh),
                                                      op::String(FLAGS_udp_host),
                                                      op::String(FLAGS_udp_port)};
#else
                                                          FLAGS_write_heatmaps,
                                                          FLAGS_write_heatmaps_format,
                                                          FLAGS_write_video_3d,
                                                          FLAGS_write_video_adam,
                                                          FLAGS_write_bvh,
                                                          FLAGS_udp_host,
                                                          FLAGS_udp_port};
#endif
        opWrapper.configure(wrapperStructOutput);

        // GUI (comment or use default argument to disable any visual output)
        const op::WrapperStructGui wrapperStructGui{op::flagsToDisplayMode(FLAGS_display,
                                                                           FLAGS_3d),
                                                    !FLAGS_no_gui_verbose,
                                                    FLAGS_fullscreen};
        opWrapper.configure(wrapperStructGui);
        // clang-format on

        // Set to single-thread (for sequential processing and/or debugging and/or reducing latency)
        if (FLAGS_disable_multi_thread)
            opWrapper.disableMultiThreading();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("Error %s at line number %d on function %s in file %s", e.what(), __LINE__, __FUNCTION__, __FILE__);
    }
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "ros_openpose_synchronous");
    ros::NodeHandle nh("~");

    // Get params
    bool no_depth;
    std::string color_topic, depth_topic, cam_info_topic, pub_topic, frame_id;
    nh.getParam("color_topic", color_topic);
    nh.getParam("depth_topic", depth_topic);
    nh.getParam("cam_info_topic", cam_info_topic);
    nh.getParam("pub_topic", pub_topic);
    nh.getParam("frame_id", frame_id);
    nh.param("no_depth", no_depth, false);  // default value is false

    // Parse Openpose Args
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    try {
        ROS_INFO("Starting ros_openpose...");

        // Initialize Openpose wrapper
        op::Wrapper op_wrapper{op::ThreadManagerMode::Asynchronous};
        configureOpenPose(op_wrapper);
        op_wrapper.start();

        // Start ROS wrapper
        rosOpenPose rop(&nh, &op_wrapper, color_topic, depth_topic, cam_info_topic, pub_topic, frame_id, no_depth);

        ros::spin();

        ROS_INFO("Exiting ros_openpose...");

        op_wrapper.stop();

        return 0;
    }
    catch (const std::exception& e) {
        ROS_ERROR("Error %s at line number %d on function %s in file %s", e.what(), __LINE__, __FUNCTION__, __FILE__);
        return -1;
    }
}