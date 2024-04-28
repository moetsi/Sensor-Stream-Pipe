
//
// Created by adammpolak on 26-09-2021.
//

#include "oakd_xlink_full_reader.h"
#include "human_poses.h"

// Closer-in minimum depth, disparity range is doubled (from 95 to 190):
// static std::atomic<bool> extended_disparity{false};
// // Better accuracy for longer distance, fractional disparity 32-levels:
// static std::atomic<bool> subpixel{false};
// // Better handling for occlusions:
// static std::atomic<bool> lr_check{false};
using namespace std;
using namespace InferenceEngine;

//#define TEST_WITH_IMAGE
//#define VERY_VERBOSE
#define MAX_RETRIAL 10
#define WAIT_AFTER_RETRIAL 5000

namespace moetsi::ssp {
using namespace human_pose_estimation;

// Calculate cosine distance between two vectors
float cos_dist(const std::vector<float>& a, const std::vector<float>& b) {
    float dotProduct = 0.0;
    float normA = 0.0;
    float normB = 0.0;
    for (size_t i = 0; i < a.size(); ++i) {
        dotProduct += a[i] * b[i];
        normA += a[i] * a[i];
        normB += b[i] * b[i];
    }
    return dotProduct / (sqrt(normA) * sqrt(normB));
}

// Normalize frame and bounding box
cv::Rect frame_norm(const int frame_cols, const int frame_rows, const std::vector<float>& bbox) {
    int xMin = std::max(static_cast<int>(bbox[0] * frame_cols), 0);
    int yMin = std::max(static_cast<int>(bbox[1] * frame_rows), 0);
    int xMax = std::min(static_cast<int>(bbox[2] * frame_cols), frame_cols - 1);
    int yMax = std::min(static_cast<int>(bbox[3] * frame_rows), frame_rows - 1);

    return cv::Rect(cv::Point(xMin, yMin), cv::Point(xMax, yMax));
}

void TwoStageHostSeqSync::add_msg(const std::shared_ptr<void>& msg, const std::string& name) {
    try {
        int64_t seq;
        if (name == "color") {
            seq = std::static_pointer_cast<dai::ImgFrame>(msg)->getSequenceNum();
        } else if (name == "depth") {
            seq = std::static_pointer_cast<dai::ImgFrame>(msg)->getSequenceNum();
        } else if (name == "person_detection") {
            seq = std::static_pointer_cast<dai::SpatialImgDetections>(msg)->getSequenceNum();
        } else if (name == "face_detection") {
            seq = std::static_pointer_cast<dai::NNData>(msg)->getSequenceNum();
        } else if (name == "recognition") { 
            seq = std::static_pointer_cast<dai::NNData>(msg)->getSequenceNum();
        } else if (name == "fast_desc") {
            seq = std::static_pointer_cast<dai::ImgFrame>(msg)->getSequenceNum();
        } else {
            throw std::runtime_error("Invalid message name: " + name);
        }

        // Look up the sequence only once
        auto& msgData = msgs[seq];

        if (name == "recognition") {
            msgData.recognitions.push_back(std::static_pointer_cast<dai::NNData>(msg));
        }
        else if (name == "fast_desc") {
            cv::Mat frame = std::static_pointer_cast<dai::ImgFrame>(msg)->getCvFrame();
            std::shared_ptr<cv::Mat> frame_ptr = std::make_shared<cv::Mat>(frame);
            msgData.fast_descriptions.push_back(frame_ptr);
        } else {
            if(name == "color") {
                msgData.color = std::static_pointer_cast<dai::ImgFrame>(msg);
            } else if(name == "depth") {
                msgData.depth = std::static_pointer_cast<dai::ImgFrame>(msg);
            } else if(name == "person_detection") {
                msgData.person_detection = std::static_pointer_cast<dai::SpatialImgDetections>(msg);
            } else if(name == "face_detection") {
                msgData.face_detection = msg;
            }
        }
    } catch (const std::exception &e) {
        std::cerr << "Error in add_msg: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "Unknown error in add_msg" << std::endl;
    }
}

/**
 * @brief Retrieves synchronized messages from the msgs map based on the specified sync_mode.
 *
 * This function iterates over the msgs map to find the maximum sequence number that has synchronized messages.
 * The criteria for a message to be considered synchronized depends on the sync_mode argument:
 *   - color_and_detections (default): It must have both color and person_detection data, and the number of person detections must match the number of recognitions and fast_descriptions.
 *   - color_depth_and_detections: In addition to the color_and_detections criteria, it must also have a synchronized depth frame.
 *   - detections: It must have person_detection data, and the number of person detections must match the number of recognitions and fast_descriptions. Color data is not required.
 *
 * If a synchronized message is found, the function returns the MessageData for the maximum synchronized sequence number.
 * Messages with sequence numbers less than the maximum synchronized sequence are erased from the map.
 * If no synchronized messages are found, an empty MessageData is returned.
 *
 * @param sync_mode The synchronization mode to use for determining message synchronization criteria.
 * @return The synchronized MessageData if found, otherwise an empty MessageData.
 */
MessageData TwoStageHostSeqSync::get_msgs(const std::string& sync_mode) {
    try {
        int64_t maxSyncSeq = INT64_MIN;   // Store max seq number having synchronized messages
        std::unordered_map<int64_t, MessageData>::iterator maxSyncSeqIt;

        // Iterate over the container and identify the maximum synchronized sequence.
        for (auto it = msgs.begin(); it != msgs.end();) {
            bool is_synced = false;

            // Check synchronization criteria based on the sync_mode
            if (sync_mode == "color_and_detections" || sync_mode == "color_depth_and_detections") {
                // Check if both color and person_detection data exist
                if (it->second.color && it->second.person_detection) {
                    // Cast person_detection to SpatialImgDetections
                    auto spatialImgDetPtr = std::static_pointer_cast<dai::SpatialImgDetections>(it->second.person_detection);
                    if (!spatialImgDetPtr) throw std::runtime_error("Failed to cast message to SpatialImgDetections");

                    // Check if the number of detections matches the number of recognitions and fast_descriptions
                    if (spatialImgDetPtr->detections.size() == 0 ||
                        (spatialImgDetPtr->detections.size() ==  it->second.recognitions.size() &&
                         spatialImgDetPtr->detections.size() == it->second.fast_descriptions.size())) {
                        
                        // For color_depth_and_detections mode, also check if depth data exists
                        if (sync_mode == "color_depth_and_detections") {
                            is_synced = it->second.depth != nullptr;
                        } else {
                            is_synced = true;
                        }
                    }
                }
            } else if (sync_mode == "detections") {
                // Check if person_detection data exists
                if (it->second.person_detection) {
                    // Cast person_detection to SpatialImgDetections
                    auto spatialImgDetPtr = std::static_pointer_cast<dai::SpatialImgDetections>(it->second.person_detection);
                    if (!spatialImgDetPtr) throw std::runtime_error("Failed to cast message to SpatialImgDetections");

                    // Check if the number of detections matches the number of recognitions and fast_descriptions
                    if (spatialImgDetPtr->detections.size() == 0 ||
                        (spatialImgDetPtr->detections.size() ==  it->second.recognitions.size() &&
                         spatialImgDetPtr->detections.size() == it->second.fast_descriptions.size())) {
                        is_synced = true;
                    }
                }
            } else {
                throw std::runtime_error("Invalid sync_mode: " + sync_mode);
            }

            // Update the maximum synchronized sequence if the current message is synced
            if (is_synced) {
                if (it->first > maxSyncSeq) {
                    maxSyncSeq = it->first;
                    maxSyncSeqIt = it;
                }
            }

            // Erase messages with sequence numbers less than the maximum synchronized sequence
            if (it->first < maxSyncSeq) {
                it = msgs.erase(it);
            } else {
                ++it;
            }
        }

        // If no synced messages were found, return an empty MessageData
        if (maxSyncSeq == INT64_MIN) {
            return MessageData{};
        }
        
        // Extract the synced messages and erase the entry from the container
        MessageData synced_msgs = std::move(maxSyncSeqIt->second);
        msgs.erase(maxSyncSeqIt);
        return synced_msgs;
    } catch (const std::exception &e) {
        std::cerr << "Error in get_msgs: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "Unknown error in get_msgs" << std::endl;
    }
    return MessageData{}; // No synced messages
}

OakdXlinkFullReader::OakdXlinkFullReader(YAML::Node config, const char* client_key, const char* environment_name, const char* sensor_name) {
    // Convert const char* to std::string, using empty string if nullptr is passed
    client_key_ = client_key ? client_key : "";
    environment_name_ = environment_name ? environment_name : "";
    sensor_name_ = sensor_name ? sensor_name : "";

    frame_template_.client_key = client_key_;
    frame_template_.environment_name = environment_name_;
    frame_template_.sensor_name = sensor_name_;
    frame_template_.static_sensor = true;

    spdlog::debug("Starting to open");
    
    // Pull variables from yaml
    stream_rgb = config["stream_color"].as<bool>();
    stream_depth = config["stream_depth"].as<bool>();
    stream_bodies = config["stream_bodies"].as<bool>();
    fps = config["streaming_rate"].as<unsigned int>();

    Init(config, states[0], 0);
    Init(config, states[1], 1);

    current_frame_counter_ = 0;
    current_rgb_frame_counter_ = 0;
    current_detection_frame_counter_ = 0;
    current_recognition_frame_counter_ = 0;
    frame_template_.stream_id = RandomString(16);
    frame_template_.device_id = config["deviceid"].as<unsigned int>();
    frame_template_.scene_desc = "oakd";

    frame_template_.frame_id = 0;
    frame_template_.message_type = SSPMessageType::MessageTypeDefault; // 0;

    ip_name = config["ip"].as<std::string>();

#ifndef _WIN32    
    std::string rel = "../.."; 
#endif
#ifdef _WIN32    
    std::string rel = "../../..";
#endif

    std::map<std::string,std::string> env;
    env["REL"] = rel;
    model_person_detection_path = config["model_person_detection"].as<std::string>();
    model_person_detection_path = StringInterpolation(env, model_person_detection_path);
    std::cerr << "person detection path is " << model_person_detection_path << std::endl << std::flush;
    model_person_detection_path2 = model_person_detection_path;

    model_reid_path = config["model_reid"].as<std::string>();
    model_reid_path = StringInterpolation(env, model_reid_path);
    std::cerr << "reid path is " << model_reid_path << std::endl << std::flush;
    model_reid_path2 = model_reid_path;

    model_face_detection_path = config["model_face_detection"].as<std::string>();
    model_face_detection_path = StringInterpolation(env, model_face_detection_path);
    std::cerr << "face detection path is " << model_face_detection_path << std::endl << std::flush;
    model_face_detection_path2 = model_face_detection_path;

    model_face_detection_proc_path = config["model_face_detection_proc"].as<std::string>();
    model_face_detection_proc_path = StringInterpolation(env, model_face_detection_proc_path);
    std::cerr << "face detection processing path is " << model_face_detection_proc_path << std::endl << std::flush;
    model_face_detection_proc_path2 = model_face_detection_proc_path;

    model_depth_diff_path = config["model_depth_diff"].as<std::string>();
    model_depth_diff_path = StringInterpolation(env, model_depth_diff_path);
    std::cerr << "face detection processing path is " << model_depth_diff_path << std::endl << std::flush;
    model_depth_diff_path2 = model_depth_diff_path;

    try {
        failed = true;
        SetOrReset();
        SetOrResetState(states[0], 0);
        SetOrResetState(states[1], 1);
        failed = false;
    } catch(std::exception & e) {
        std::cerr << "Failed to initialize OAK-D" << std::endl << std::flush;
        std::cerr << e.what() << std::endl << std::flush;
        std::cerr << "Issue loading blobs" << std::endl << std::flush;
    }
}

void OakdXlinkFullReader::Init(YAML::Node config, std::shared_ptr<State> &st, int n) {
    ResetState(st);

    st->rgb_res = config["rgb_resolution"].as<unsigned int>();
    if (st->rgb_res == 1080)
        st->rgb_dai_res = dai::ColorCameraProperties::SensorResolution::THE_1080_P;
    else if (st->rgb_res == 4000)
        st->rgb_dai_res = dai::ColorCameraProperties::SensorResolution::THE_4_K;
    else if (st->rgb_res == 12000)
        st->rgb_dai_res = dai::ColorCameraProperties::SensorResolution::THE_12_MP;
    else if (st->rgb_res == 13000)
        st->rgb_dai_res = dai::ColorCameraProperties::SensorResolution::THE_13_MP;
    else
        st->rgb_dai_res = dai::ColorCameraProperties::SensorResolution::THE_1080_P;

    st->rgb_dai_preview_y = config["rgb_preview_size_y"].as<unsigned int>();
    rgb_dai_preview_y = config["rgb_preview_size_y"].as<unsigned int>();
    st->rgb_dai_preview_x = config["rgb_preview_size_x"].as<unsigned int>();
    rgb_dai_preview_x = config["rgb_preview_size_x"].as<unsigned int>();
    st->rgb_dai_fps = config["rgb_fps"].as<unsigned int>();

    st->depth_res = config["depth_resolution"].as<unsigned int>();
    if (st->depth_res == 720)
        st->depth_dai_res = dai::MonoCameraProperties::SensorResolution::THE_720_P;
    else if (st->depth_res == 800)
        st->depth_dai_res = dai::MonoCameraProperties::SensorResolution::THE_800_P;
    else if (st->depth_res == 400)
        st->depth_dai_res = dai::MonoCameraProperties::SensorResolution::THE_400_P;
    else if (st->depth_res == 480)
        st->depth_dai_res = dai::MonoCameraProperties::SensorResolution::THE_480_P;
    else
        st->depth_dai_res = dai::MonoCameraProperties::SensorResolution::THE_720_P;
     
    st->depth_dai_preview_y = config["depth_preview_size_y"].as<unsigned int>();
    st->depth_dai_preview_x = config["depth_preview_size_x"].as<unsigned int>();
    st->depth_dai_fps = config["depth_fps"].as<unsigned int>();
    st->depth_dai_sf = config["depth_spatial_filter"].as<bool>();
    st->depth_dai_sf_hfr = config["depth_spatial_hole_filling_radius"].as<unsigned int>();
    st->depth_dai_sf_num_it = config["depth_spatial_filter_num_it"].as<unsigned int>();
    st->depth_dai_df = config["depth_decimation_factor"].as<unsigned int>();
}

void OakdXlinkFullReader::SetOrReset() {
    ResetVino();
    auto st = states[0];

    // 0. We initialize the tracker
    // 1. The pipeline is created and the nodes are added to it.
    // 2. The properties of the nodes are set.
    // 3. The nodes are linked.
    // 4. The device is opened with the pipeline.
    // 5. The output queue is created to receive the frames from the output.
    // 6. The input queue is created to send the control messages.
    // 7. The focus is set to manual and the value is set to 130.
    // 8. The control message is sent to the input queue.

    // 
    // Tracker set up
    //

    // We set the tracker to not drop forgotten tracks and to not limit the number of objects in a track
    params.drop_forgotten_tracks = false;
    params.max_num_objects_in_track = -1;

    // Initialize tracker with the set params
    tracker = std::make_unique<PedestrianTracker>(params, cv::Size(rgb_person_det_nn_in_x_res, rgb_person_det_nn_in_y_res));
    // The tracker currently uses "fast descriptors" of a cropped detection resized to 16x32 and
    // "strong descriptors" which are the cropped detections being resized and put through a ML algo
    // We then also set the distance algorithms to use for fast and for strong
    std::shared_ptr<IImageDescriptor> descriptor_fast =
    std::make_shared<ResizedImageDescriptor>(cv::Size(16, 32), cv::InterpolationFlags::INTER_LINEAR);
    std::shared_ptr<IDescriptorDistance> distance_fast = std::make_shared<MatchTemplateDistance>();
    tracker->set_descriptor_fast(descriptor_fast);
    tracker->set_distance_fast(distance_fast);
    std::shared_ptr<IImageDescriptor> descriptor_strong = std::make_shared<Descriptor>(strong_descriptor_size.height);
    std::shared_ptr<IDescriptorDistance> distance_strong = std::make_shared<CosDistance>(strong_descriptor_size);
    tracker->set_distance_strong(distance_strong);

    // Define source and output
    pipeline = std::make_shared<dai::Pipeline>();
    
    // ColorCamera setup
    camRgb = pipeline->create<dai::node::ColorCamera>();
    camRgb->setPreviewSize(st->rgb_dai_preview_x, st->rgb_dai_preview_y);
    camRgb->setResolution(st->rgb_dai_res);
    camRgb->setInterleaved(false);
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::RGB);
    camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
    camRgb->setFps(st->rgb_dai_fps);
    camRgb->setPreviewNumFramesPool(10);

    // Stereo Depth setup
    left = pipeline->create<dai::node::MonoCamera>();
    right = pipeline->create<dai::node::MonoCamera>();
    stereo = pipeline->create<dai::node::StereoDepth>();
    left->out.link(stereo->left);
    right->out.link(stereo->right); 

    // Depth Properties
    left->setResolution(st->depth_dai_res);
    left->setBoardSocket(dai::CameraBoardSocket::LEFT);
    left->setFps(st->depth_dai_fps);
    right->setResolution(st->depth_dai_res);
    right->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    right->setFps(st->depth_dai_fps);
    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    // stereo->initialConfig.setConfidenceThreshold(250);
    stereo->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_7x7);
    stereo->setSubpixel(true);
    stereo->setLeftRightCheck(true); // LR-check is required for depth alignment
    stereo->setDepthAlign(dai::CameraBoardSocket::RGB);
    stereo->setOutputSize(st->depth_dai_preview_x, st->depth_dai_preview_y);
    auto oakdConfig = stereo->initialConfig.get();
    oakdConfig.postProcessing.spatialFilter.enable = st->depth_dai_sf;
    oakdConfig.postProcessing.speckleFilter.speckleRange = 60;
    oakdConfig.postProcessing.temporalFilter.enable = true;

    oakdConfig.postProcessing.spatialFilter.holeFillingRadius = st->depth_dai_sf_hfr;
    oakdConfig.postProcessing.spatialFilter.numIterations = st->depth_dai_sf_num_it;
    oakdConfig.postProcessing.thresholdFilter.minRange = 700;
    oakdConfig.postProcessing.thresholdFilter.maxRange = 12000;
    oakdConfig.censusTransform.enableMeanMode = true;
    oakdConfig.costMatching.linearEquationParameters.alpha = 0;
    oakdConfig.costMatching.linearEquationParameters.beta = 2;
    oakdConfig.postProcessing.decimationFilter.decimationFactor = st->depth_dai_df;

    stereo->initialConfig.set(oakdConfig);
    stereo->setNumFramesPool(10);

    // PERSON DETECTION SECTION
    // person_det_manip will resize the frame before sending it to the person detection NN node
    // TODO: If the only detection is person detection, can set up camera to be correct size instead of going through manip
    person_det_manip = pipeline->create<dai::node::ImageManip>();
    person_det_manip->initialConfig.setResize(rgb_person_det_nn_in_x_res, rgb_person_det_nn_in_y_res); // requied for nn (544, 320)
    person_det_manip->initialConfig.setFrameType(dai::ImgFrame::Type::RGB888p);
    person_det_manip->setNumFramesPool(10);
    camRgb->preview.link(person_det_manip->inputImage);

    // Person detection spatial nn setup
    cout << "Creating Person Detection Spatial Neural Network..." << endl;
    person_nn = pipeline->create<dai::node::MobileNetSpatialDetectionNetwork>();
    person_nn->setBoundingBoxScaleFactor(0.25);
    person_nn->setDepthLowerThreshold(50);
    person_nn->setDepthUpperThreshold(12000);
    person_nn->setBlobPath(model_person_detection_path);
    person_nn->setNumPoolFrames(10);
    // Link the manip output and the depth output to the person detection nn
    person_det_manip->out.link(person_nn->input);
    stereo->depth.link(person_nn->inputDepth);

    // Send person detections to the host
    person_det_xout = pipeline->create<dai::node::XLinkOut>();
    person_det_xout->setStreamName("person_detection");
    person_nn->out.link(person_det_xout->input);

    // Now we create a node with a script that takes in the rgb (passthrough) and depth (passthroughDepth)
    // it also has an input for commands that are sent by the host
    // it has global variables: send_rgb_as_they_come, send_depth_as_they_come, send_a_depth, send_a_rgb, and send_a_depth_and_rgb_pair set to false
    // it creates a dictionary to link rgb and depth frames by their sequence numbers
    // it adds to the dictionary in a while loop and also checks for commands
    // the commands can be: send_rgb_as_they_come, send_depth_as_they_come, send_a_depth, send_a_rgb, and send_a_depth_and_rgb_pair
    // The commands will set the global variables, and in the while loop the script will do as the global variables mention
    // if the global variables: send_a_rgb, send_a_depth, or send_a_depth_and_rgb_pair are set to true the script will send what was told to rgb_out or depth_out and then reset the variable to false once done
    rgb_and_depth_out_script = pipeline->create<dai::node::Script>();
    person_nn->passthrough.link(rgb_and_depth_out_script->inputs["rgb"]);
    person_nn->passthroughDepth.link(rgb_and_depth_out_script->inputs["depth"]);

    rgb_and_depth_out_script->setScript(R"(
        import time
        import json

        send_rgb_as_they_come = False
        send_depth_as_they_come = False
        send_a_depth = False
        send_a_rgb = False
        send_a_depth_and_rgb_pair = False

        frames = dict()

        while True:
            time.sleep(0.001)
            command = node.io['commands'].tryGet()
            if command is not None:
                node.warn('rgb_and_depth_out_script received command')
                data = command.getData()
                jsonStr = str(data, 'utf-8')
                dict = json.loads(jsonStr)
                commandString = dict['message']
                node.warn('message was: ' + commandString)
                if commandString == "send_rgb_as_they_come":
                    send_rgb_as_they_come = True
                elif commandString == "send_depth_as_they_come":
                    send_depth_as_they_come = True  
                elif commandString == "send_a_depth":
                    send_a_depth = True
                elif commandString == "send_a_rgb":
                    send_a_rgb = True
                elif commandString == "send_a_depth_and_rgb_pair":
                    send_a_depth_and_rgb_pair = True

            rgb_frame = node.io['rgb'].tryGet()
            if rgb_frame is not None:
                seq = rgb_frame.getSequenceNum()
                if seq not in frames:
                    frames[seq] = {}
                frames[seq]["rgb"] = rgb_frame
                if send_rgb_as_they_come:
                    node.io['rgb_out'].send(rgb_frame)

            depth_frame = node.io['depth'].tryGet()  
            if depth_frame is not None:
                seq = depth_frame.getSequenceNum()
                if seq not in frames:
                    frames[seq] = {}
                frames[seq]["depth"] = depth_frame
                if send_depth_as_they_come:
                    node.io['depth_out'].send(depth_frame)
                    
            if len(frames) > 5:
                min_seq = min(frames.keys())
                del frames[min_seq]

            if send_a_rgb:
                if len(frames) > 0:
                    seq, data = frames.popitem()
                    if "rgb" in data:
                        node.io['rgb_out'].send(data["rgb"])
                send_a_rgb = False

            if send_a_depth:  
                if len(frames) > 0:
                    seq, data = frames.popitem()
                    if "depth" in data:
                        node.io['depth_out'].send(data["depth"])
                send_a_depth = False

            if send_a_depth_and_rgb_pair:
                if len(frames) > 0:  
                    seq, data = frames.popitem()
                    if "rgb" in data and "depth" in data:
                        node.io['rgb_out'].send(data["rgb"])
                        node.io['depth_out'].send(data["depth"])
                send_a_depth_and_rgb_pair = False
    )");

    rgbOut = pipeline->create<dai::node::XLinkOut>();
    rgbOut->setStreamName("rgb");
    rgb_and_depth_out_script->outputs["rgb_out"].link(rgbOut->input);

    depthOut = pipeline->create<dai::node::XLinkOut>();  
    depthOut->setStreamName("depth");
    rgb_and_depth_out_script->outputs["depth_out"].link(depthOut->input);

    // Now we create the control node to send to rgb_and_depth_out_script
    control_xlinkin = pipeline->create<dai::node::XLinkIn>();
    control_xlinkin->setStreamName("control");
    control_xlinkin->out.link(rgb_and_depth_out_script->inputs["commands"]);

    // END OF PERSON DETECTION SECTION

    // STRONG DESCRIPTION (NN) AND FAST DESCRIPTION (CROPPED)
    // Set up the person reid strong config manip node for the person reid strong nn
    person_config_manip_for_reid_nn_script = pipeline->create<dai::node::Script>();
    person_nn->passthrough.link(person_config_manip_for_reid_nn_script->inputs["preview"]);
    person_nn->out.link(person_config_manip_for_reid_nn_script->inputs["person_dets_in"]);
    // We create the script including the variables we need for reid size
    std::ostringstream reid_config_script;
    reid_config_script << R"(
    import time
    msgs = dict()
    def add_msg(msg, name, seq = None):
        global msgs
        if seq is None:
            seq = msg.getSequenceNum()
        seq = str(seq)
        if seq not in msgs:
            # node.warn(f"Recving image seq num: {seq}")
            msgs[seq] = dict()
        msgs[seq][name] = msg
        if 15 < len(msgs):
            # node.warn(f"Removing first element! len {len(msgs)}")
            # Check all items in msgs and print incomplete ones
            for seq, syncMsgs in msgs.items():
                if len(syncMsgs) != 2:
                    missing = 'preview' if 'dets' in syncMsgs else 'dets'
                    # node.warn(f"Incomplete set in seq {seq}: missing {missing}")
            # Pop the item and print its sequence number
            popped_item = msgs.popitem()
            # node.warn(f"                       Removed seq: {popped_item[0]}")
    def get_msgs():
        global msgs
        for seq, syncMsgs in msgs.items():
            if len(syncMsgs) == 2:
                result = syncMsgs
                del msgs[seq]
                return result
        return None
    def correct_bb(bb):
        if bb.xmin < 0: bb.xmin = 0.001
        if bb.ymin < 0: bb.ymin = 0.001
        if bb.xmax > 1: bb.xmax = 0.999
        if bb.ymax > 1: bb.ymax = 0.999
        return bb
    while True:
        time.sleep(0.001)
        preview = node.io['preview'].tryGet()
        if preview is not None:
            add_msg(preview, 'preview')
        dets = node.io['person_dets_in'].tryGet()
        if dets is not None:
            seq = dets.getSequenceNum()
            add_msg(dets, 'dets', seq)
        sync_msgs = get_msgs()
        if sync_msgs is not None:
            img = sync_msgs['preview']
            dets = sync_msgs['dets']
            dets_seq = dets.getSequenceNum()
            img_seq = img.getSequenceNum()
            # node.warn(f"   Sending image seq num: {img_seq}")
            for i, det in enumerate(dets.detections):
                cfg = ImageManipConfig()
                correct_bb(det)
                cfg.setCropRect(det.xmin, det.ymin, det.xmax, det.ymax)
                cfg.setResize()";
    reid_config_script << std::to_string(rgb_person_reid_strong_nn_in_x_res) << ", " << std::to_string(rgb_person_reid_strong_nn_in_y_res) << ")";
    reid_config_script << R"(
                cfg.setKeepAspectRatio(False)
                node.io['manip_cfg'].send(cfg)
                node.io['manip_img'].send(img))";
    person_config_manip_for_reid_nn_script->setScript(reid_config_script.str());

    // Take in the configs from the person strong reid config manip node
    recognition_manip = pipeline->create<dai::node::ImageManip>();
    recognition_manip->initialConfig.setResize(rgb_person_reid_strong_nn_in_x_res, rgb_person_reid_strong_nn_in_y_res);
    recognition_manip->setWaitForConfigInput(true);
    person_config_manip_for_reid_nn_script->outputs["manip_cfg"].link(recognition_manip->inputConfig);
    person_config_manip_for_reid_nn_script->outputs["manip_img"].link(recognition_manip->inputImage);
    
    cout << "Creating Recognition Neural Network..." << endl;
    recognition_nn = pipeline->create<dai::node::NeuralNetwork>();
    recognition_nn->setNumPoolFrames(10);
    recognition_nn->setBlobPath(model_reid_path);
    recognition_manip->out.link(recognition_nn->input);

    recognition_nn_xout = pipeline->create<dai::node::XLinkOut>();
    recognition_nn_xout->setStreamName("recognition");
    recognition_nn->out.link(recognition_nn_xout->input);

    //  Person reid fast config manip node for the person reid fast descriptor
    person_config_manip_for_fast_desc_script = pipeline->create<dai::node::Script>();
    person_nn->passthrough.link(person_config_manip_for_fast_desc_script->inputs["preview"]);
    person_nn->out.link(person_config_manip_for_fast_desc_script->inputs["person_dets_in"]);
    // We create the script including the variables we need for reid size
    std::ostringstream fast_desc_script;
    fast_desc_script << R"(
    import time
    msgs = dict()
    def add_msg(msg, name, seq = None):
        global msgs
        if seq is None:
            seq = msg.getSequenceNum()
        seq = str(seq)
        if seq not in msgs:
            msgs[seq] = dict()
        msgs[seq][name] = msg
        if 15 < len(msgs):
            # node.warn(f"Removing first element! len {len(msgs)}")
            msgs.popitem()
    def get_msgs():
        global msgs
        seq_remove = []
        for seq, syncMsgs in msgs.items():
            seq_remove.append(seq)
            if len(syncMsgs) == 2:
                for rm in seq_remove:
                    del msgs[rm]
                return syncMsgs
        return None
    def correct_bb(bb):
        if bb.xmin < 0: bb.xmin = 0.001
        if bb.ymin < 0: bb.ymin = 0.001
        if bb.xmax > 1: bb.xmax = 0.999
        if bb.ymax > 1: bb.ymax = 0.999
        return bb
    while True:
        time.sleep(0.001)
        preview = node.io['preview'].tryGet()
        if preview is not None:
            add_msg(preview, 'preview')
        dets = node.io['person_dets_in'].tryGet()
        if dets is not None:
            seq = dets.getSequenceNum()
            add_msg(dets, 'dets', seq)
        sync_msgs = get_msgs()
        if sync_msgs is not None:
            img = sync_msgs['preview']
            dets = sync_msgs['dets']
            dets_seq = dets.getSequenceNum()
            img_seq = img.getSequenceNum()
            for i, det in enumerate(dets.detections):
                cfg = ImageManipConfig()
                correct_bb(det)
                cfg.setCropRect(det.xmin, det.ymin, det.xmax, det.ymax)
                cfg.setResize()";
    fast_desc_script << std::to_string(rgb_person_reid_fast_nn_in_x_res) << ", " << std::to_string(rgb_person_reid_fast_nn_in_y_res) << ")";
    fast_desc_script << R"(
                cfg.setKeepAspectRatio(False)
                node.io['manip_cfg'].send(cfg)
                node.io['manip_img'].send(img))";
    person_config_manip_for_fast_desc_script->setScript(fast_desc_script.str());

    // Take in the configs from the config manip node
    fast_desc_manip = pipeline->create<dai::node::ImageManip>();
    fast_desc_manip->initialConfig.setResize(rgb_person_reid_fast_nn_in_x_res, rgb_person_reid_fast_nn_in_y_res);
    fast_desc_manip->setWaitForConfigInput(true);
    person_config_manip_for_fast_desc_script->outputs["manip_cfg"].link(fast_desc_manip->inputConfig);
    person_config_manip_for_fast_desc_script->outputs["manip_img"].link(fast_desc_manip->inputImage);

    fast_desc_xout = pipeline->create<dai::node::XLinkOut>();
    fast_desc_xout->setStreamName("fast_desc");
    fast_desc_manip->out.link(fast_desc_xout->input);


    std::cerr << "Trying to create pipeline" << std::endl << std::flush;  
    device_info = std::make_shared<dai::DeviceInfo>(ip_name);
    device = std::make_shared<dai::Device>(*pipeline, *device_info, true); // usb 2 mode
    std::cerr << "Created pipeline" << std::endl << std::flush;  

    // Connect to device and start pipeline
    std::cerr << "Connected cameras: " << std::endl << std::flush;
    std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
    for(const auto& cam : device->getConnectedCameras()) {
        std::cerr << static_cast<int>(cam) << " ";
        std::cerr << cam << " ";
    }
    std::cerr << endl;

    // Set up the output queues
    for (const auto &name : {"rgb", "depth", "person_detection", "recognition", "fast_desc"}) {
        queues[name] = device->getOutputQueue(name, 25, false);
    }

    control_queue =  device->getInputQueue("control");
    if (send_rgb_to_host || stream_rgb) {
        nlohmann::json dict{{"message", "send_rgb_as_they_come"}};
        auto buf = dai::Buffer();
        auto data = dict.dump();
        buf.setData({data.begin(), data.end()});
        control_queue->send(buf);
    }
    if (send_depth_to_host || stream_depth) {
        nlohmann::json dict{{"message", "send_depth_as_they_come"}};
        auto buf = dai::Buffer();
        auto data = dict.dump();
        buf.setData({data.begin(), data.end()});
        control_queue->send(buf);
    }
    spdlog::debug("Done opening");

}
void OakdXlinkFullReader::SetOrResetState(const std::shared_ptr<State> &st, int n) {

    start_time = CurrentTimeMs();
}

OakdXlinkFullReader::~OakdXlinkFullReader() {

}

// Function to process the inference output
std::vector<std::vector<float>> postprocess(const dai::NNData& inference, int x_width, int y_width) {

    // Get inference results as a 1D float vector
    std::vector<float> dets = inference.getLayerFp16("dets");

    // Get number of valid faces
    int nb_valid_faces = inference.getLayerInt32("dets@shape")[0];
    // std::cerr << "nb_valid_faces: " << nb_valid_faces << std::endl << std::flush;

    // Convert the 1D float vector to a 2D float vector
    std::vector<std::vector<float>> faces(nb_valid_faces, std::vector<float>(15));
    for(int i = 0; i < nb_valid_faces; ++i) {
        for(int j = 0; j < 15; ++j) {
            faces[i][j] = dets[i*15+j];
        }
    }

    // Replace (x2,y2) by (w,h)
    for(auto& face : faces) {
        face[2] -= face[0];
        face[3] -= face[1];
    }

    // Scale the coordinates with the padded size
    // This is not working correctly, as you move to edges of rgb the face is off
    // TODO: This is probably because of the thumbnail resize imagemanip performed for the face detection nnf
    // TODO: We are not scaling back to the original image size correctly 
    for(auto& face : faces) {
        for(int i = 0; i < 14; ++i) {
            face[i] *= ((i % 2 == 0) ? x_width : y_width);
        }
    }

    return faces;
}

void OakdXlinkFullReader::NextFrame(const std::vector<std::string> frame_types_to_pull) {


    for (int kk=0; kk<MAX_RETRIAL; ++kk) {
        try {
            if (failed) {

                SetOrReset();
                SetOrResetState(states[0], 0);
                SetOrResetState(states[1], 1);

                // SetOrResetInternals();
                failed = false;
            }
            current_frame_.clear();
            current_frame_counter_++;
            uint64_t capture_timestamp = CurrentTimeMs();

            // Oakd reader must be able to respond to a "c" input from keyboard which would request a rgb and depth frame to be returned
            // This will cause frame_types_to_pull to be populated with "rgb" and "depth"

            bool message_pulled = false; //we will set this to true if we pull a message (so we don't call get_msgs unnecessarily)
            if (queues["rgb"]->has()) {
                auto rgb_message = queues["rgb"]->get();
                sync.add_msg(rgb_message, "color");
                message_pulled = true;
            }
            if (queues["depth"]->has()) {
                auto depth_message = queues["depth"]->get();
                sync.add_msg(depth_message, "depth");
                message_pulled = true;
            }
            if (queues["person_detection"]->has()) {
                auto det_message = queues["person_detection"]->get();
                sync.add_msg(det_message, "person_detection");
                message_pulled = true;
            }
            if (queues["recognition"]->has()) {
                auto rec_message = queues["recognition"]->get();
                sync.add_msg(rec_message, "recognition");
                message_pulled = true;
            }
            if (queues["fast_desc"]->has()) {
                auto det_fast_desc_message = queues["fast_desc"]->get();
                sync.add_msg(det_fast_desc_message, "fast_desc");
                message_pulled = true;
            }
            // Declare msgs to make it available downstream
            MessageData msgs;
            // Now we check if all messages have come in for a given color (it checks if all messages have the same sequence number)
            if (message_pulled) {
                msgs = sync.get_msgs("color_and_detections");
            }
            if (!msgs.color) {
                // If we don't have a color message returned in the sync, we don't have all the messages
                return;
            }

            // Now that we have the messages for a given rgb frame, we can process them for the tracker algo
            // We grab the rgb frame as a cv::Mat
            auto frame = msgs.color->getCvFrame();
            // std::cerr << "RGB Synched Seq Num: " << msgs.color->getSequenceNum() << std::endl << std::flush;
            // First we grab the timestamp of the rgb frame
            auto timestamp = msgs.color->getTimestamp();
            uint64_t epoch_time = std::chrono::duration_cast<std::chrono::milliseconds>(timestamp.time_since_epoch()).count();
            
            // This will store all the TrackedObject detections
            TrackedObjects detections;

            // We grab the detections that we must turn into TrackedObject as well as fast and strong descriptors
            std::vector<dai::SpatialImgDetection> person_detections = msgs.person_detection->detections;
            std::vector<std::shared_ptr<dai::NNData>>& recognitions = msgs.recognitions;
            std::vector<std::shared_ptr<cv::Mat>> fast_descs = msgs.fast_descriptions;

            // The seq_num will be used as the frame_idx for the TrackedObject
            auto seq_num = msgs.person_detection->getSequenceNum();
            auto detection_timestamp = msgs.person_detection->getTimestamp();
            uint64_t detection_epoch_time = std::chrono::duration_cast<std::chrono::milliseconds>(detection_timestamp.time_since_epoch()).count();

            // Now we prepare the person detections to meet tracker requirements of TrackedObject
            for (size_t i = 0; i < person_detections.size(); ++i) {
                // We create a TrackedObject for each person detection
                TrackedObject obj;

                // We grab the detection for ease of reading
                auto detection = person_detections[i];

                // We set the frame_idx to the seq number of the person detection
                obj.frame_idx = seq_num;
                obj.timestamp = detection_epoch_time;

                // We set the confidence of the TrackedObject
                obj.confidence = detection.confidence;

                // Now we set the center coordinates of the TrackedObject
                obj.center_x = detection.spatialCoordinates.x;
                obj.center_y = detection.spatialCoordinates.y;
                obj.center_z = detection.spatialCoordinates.z;

                // Now we set the strong and fast descriptors
                std::vector<float> reid_result = (recognitions)[i]->getFirstLayerFp16();
                cv::Mat strong_desc(reid_result.size(), 1, CV_32F, reid_result.data());
                obj.strong_descriptor = strong_desc;
                std::shared_ptr<cv::Mat> fast_desc = fast_descs[i];
                obj.fast_descriptor = *fast_desc;

                // Now we set the rect of the TrackedObject
                auto bbox = frame_norm(rgb_person_det_nn_in_x_res, rgb_person_det_nn_in_y_res, {detection.xmin, detection.ymin, detection.xmax, detection.ymax});
                obj.rect = bbox;

                // If the bounding box is greater than 0 we add to detections (we removed label flags from example)
                if (obj.rect.area() > 0 ) {
                    detections.emplace_back(obj);
                }
            }
            std:cerr << "RAW detections size: " << detections.size() << std::endl << std::flush;
            
            tracker->Process(detections, epoch_time);

            auto recent_detections = tracker->GetMostRecentDetections();

            // We print the number of detections
            std::cerr << "PROCESSED detection size: " << recent_detections.size() << std::endl << std::flush;

            if (true)
            {
                // Drawing colored "worms" (tracks).
                frame = tracker->DrawActiveTracks(frame);

                // Drawing all detected objects on a frame by BLUE COLOR
                for (const auto& detection : detections) {
                    cv::rectangle(frame, detection.rect, cv::Scalar(255, 0, 0), 3);
                    // Now we put the confidence label on the bounding box with same information as below
                    std::string text =
                        std::to_string(detection.object_id) + " conf: " + std::to_string(detection.confidence);
                    putHighlightedText(frame,
                                    text,
                                    detection.rect.tl() - cv::Point{10, 10},
                                    cv::FONT_HERSHEY_COMPLEX,
                                    0.65,
                                    cv::Scalar(255, 0, 0),
                                    2);
                }

                // Drawing tracked detections only by RED color and print ID and detection
                // confidence level.
                for (const auto& detection : tracker->TrackedDetections()) {
                    cv::rectangle(frame, detection.rect, cv::Scalar(0, 0, 255), 3);
                    // std::string text =
                    //     std::to_string(detection.object_id) + " conf: " + std::to_string(detection.confidence);
                    // putHighlightedText(frame,
                    //                    text,
                    //                    detection.rect.tl() - cv::Point{10, 10},
                    //                    cv::FONT_HERSHEY_COMPLEX,
                    //                    0.65,
                    //                    cv::Scalar(0, 0, 255),
                    //                    2);
                }
                // presenter.drawGraphs(frame);
                // metrics.update(startTime, frame, {10, 22}, cv::FONT_HERSHEY_COMPLEX, 0.65);

                videoWriter.write(frame);
                if (true) {
                    cv::namedWindow("dbg", cv::WINDOW_NORMAL);
                    cv::imshow("dbg", frame);
                    char k = cv::waitKey(1);
                    if (k == 27)
                        break;
                    // presenter.handleKey(k);
                }

                if (true && (seq_num % 100 == 0)) {
                    DetectionLog log = tracker->GetDetectionLog(true);
                    // SaveDetectionLogToTrajFile(detlog_out, log);
                }

            }
            startTime = std::chrono::steady_clock::now();

            if (stream_bodies)
            {
                // Detections frame
                std::shared_ptr<FrameStruct> detections_frame_struct = 
                    std::shared_ptr<FrameStruct>(new FrameStruct(frame_template_));
                
                detections_frame_struct->sensor_id = 0;
                detections_frame_struct->frame_type = FrameType::FrameTypeDetection;
                detections_frame_struct-> frame_data_type = FrameDataType::FrameDataTypeTrackedObjects;
                detections_frame_struct->frame_id = seq_num;
                detections_frame_struct->frame_device_timestamp = epoch_time;
                detections_frame_struct->timestamps.push_back(detection_epoch_time);

                // Now we add the device_id to all the detection structs in recent_detections from detections_frame_struct.device_id
                for (auto& detection : recent_detections) {
                    detection.device_id = detections_frame_struct->device_id;
                }
                
                // We make the frame the size of the number of detections and an int to hold the number of detections
                int32_t num_detections = recent_detections.size();
                detections_frame_struct->frame = std::vector<uchar>();
                detections_frame_struct->frame.resize(sizeof(detection_struct_t) * num_detections + sizeof(int32_t));

                // Now we memcopy into detections_frame_struct->frame first the number of detections then the detection_struct_ts
                memcpy(detections_frame_struct->frame.data(), &num_detections, sizeof(int32_t));
                memcpy(detections_frame_struct->frame.data() + sizeof(int32_t), recent_detections.data(), sizeof(detection_struct_t) * num_detections);
                
                // Now we push the frame into the current frame
                current_frame_.push_back(detections_frame_struct);
            }


        // //    THIS IS ALL COMMENTED OUT STARTS
        //       //Increment counter and grab time
        //     current_frame_counter_++;
        //     uint64_t capture_timestamp = CurrentTimeMs();
        //     auto framesASecond = (float)current_frame_counter_/((float)(capture_timestamp - start_time)*.001);
        //     auto fps_string = std::to_string(framesASecond);

        //     cv::Mat frameRgbOpenCv = std::static_pointer_cast<dai::ImgFrame>(msgs.color)->getCvFrame();
        //     // resizing frames
        //     cv::Mat resizedFrame;
        //     cv::Size size(1632, 960); 
        //     cv::resize(frameRgbOpenCv, resizedFrame, size); // resize image

        //     // commented out because already done above
        //     // auto person_detections = std::static_pointer_cast<dai::SpatialImgDetections>(msgs.person_detection)->detections;
        //     auto face_detections = std::static_pointer_cast<dai::NNData>(msgs.face_detection);
        //     auto faces = postprocess(*face_detections, resizedFrame.cols, resizedFrame.rows);

        //     cv::putText(resizedFrame, fps_string, cv::Point(10, 30), cv::FONT_HERSHEY_TRIPLEX, 1, cv::Scalar(0, 0, 0), 8);
        //     cv::putText(resizedFrame, fps_string, cv::Point(10, 30), cv::FONT_HERSHEY_TRIPLEX, 1, cv::Scalar(255, 255, 255), 2);

        //     // Person detections bounding boxes, depth and reid labeling
        //     for (size_t i = 0; i < person_detections.size(); ++i) {
        //         auto detection = person_detections[i];
        //         auto bbox = frame_norm(1632, 960, {detection.xmin, detection.ymin, detection.xmax, detection.ymax});

        //         auto reid_result = (recognitions)[i]->getFirstLayerFp16();

        //         bool found = false;
        //         int reid_id;
        //         for (size_t j = 0; j < reid_results.size(); ++j) {
        //             auto dist = cos_dist(reid_result, reid_results[j]);
        //             if (dist > 0.7) {
        //                 reid_results[j] = reid_result;
        //                 reid_id = j;
        //                 found = true;
        //                 break;
        //             }
        //         }

        //         if (!found) {
        //             reid_results.push_back(reid_result);
        //             reid_id = reid_results.size() - 1;
        //         }

        //         cv::rectangle(resizedFrame, cv::Point(bbox.x, bbox.y), cv::Point(bbox.x + bbox.width, bbox.y + bbox.height), cv::Scalar(10, 245, 10), 2);
        //         int y = (bbox.y + bbox.y + bbox.height) / 2;
        //         auto depth_x = std::to_string(detection.spatialCoordinates.x/1000.0f);
        //         auto depth_y = std::to_string(detection.spatialCoordinates.y/1000.0fdetections_frame_struct cv::FONT_HERSHEY_TRIPLEX, 1.5, cv::Scalar(0, 0, 0), 8);
        //         cv::putText(resizedFrame, depth_text, cv::Point(bbox.x, y + 30), cv::FONT_HERSHEY_TRIPLEX, 1.5, cv::Scalar(255, 255, 255), 2);
        //     }

        //     // Face detections bounding boxes
        //     for(const auto &face : faces) {
        //         // Extract bounding box coordinates
        //         int x = static_cast<int>(face[0]);
        //         int y = static_cast<int>(face[1]);
        //         int w = static_cast<int>(face[2]);
        //         int h = static_cast<int>(face[3]);

        //         // Draw bounding box for each face
        //         cv::rectangle(resizedFrame, cv::Point(x, y), cv::Point(x + w, y + h), cv::Scalar(0, 255, 0), 2);
        //     }

        //     cv::imshow("Camera", resizedFrame);
        //     cv::waitKey(1);

        //     // Color frame
        //     std::shared_ptr<FrameStruct> rgbFrame =
        //         std::shared_ptr<FrameStruct>(new FrameStruct(frame_template_));
        //         rgbFrame->sensor_id = 0; 
        //     rgbFrame->frame_type = FrameType::FrameTypeColor; // 0;
        //     rgbFrame->frame_data_type = FrameDataType::FrameDataTypeCvMat; // 9;
        //     rgbFrame->frame_id = current_frame_counter_;
        //     rgbFrame->timestamps.push_back(capture_timestamp);

        //     // convert the raw buffer to cv::Mat
        //     int32_t colorCols = resizedFrame.cols;                                                        
        //     int32_t colorRows = resizedFrame.rows;                                                        
        //     size_t colorSize = colorCols*colorRows*3*sizeof(uchar); //This assumes that oakd color always returns CV_8UC3

        //     rgbFrame->frame.resize(colorSize + 2 * sizeof(int32_t));                                        

        //     memcpy(&rgbFrame->frame[0], &colorCols, sizeof(int32_t));                                       
        //     memcpy(&rgbFrame->frame[4], &colorRows, sizeof(int32_t));                                       
        //     memcpy(&rgbFrame->frame[8], (unsigned char*)(resizedFrame.data), colorSize);

        //     //Depth frame
        //     // std::shared_ptr<FrameStruct> depthFrame =
        //     //     std::shared_ptr<FrameStruct>(new FrameStruct(frame_template_));
        //     // depthFrame->sensor_id = 1;
        //     // depthFrame->frame_type = FrameType::FrameTypeDepth; // 1;
        //     // depthFrame->frame_data_type = FrameDataType::FrameDataTypeDepthAIStereoDepth; // 11; It is not when not subpixel
        //     // depthFrame->frame_id = current_frame_counter_;
        //     // depthFrame->timestamps.push_back(capture_timestamp);

        //     // // convert the raw buffer to cv::Mat
        //     // int32_t depthCols = frameDepthMat.cols;                                                        
        //     // int32_t depthRows = frameDepthMat.rows;                                                        
        //     // size_t depthSize = depthCols*depthRows*sizeof(uint16_t); // DepthAI StereoDepth outputs ImgFrame message that carries RAW16 encoded (0..65535) depth data in millimeters.

        //     // depthFrame->frame.resize(depthSize + 2 * sizeof(int32_t));                                        

        //     // memcpy(&depthFrame->frame[0], &depthCols, sizeof(int32_t));                                       
        //     // memcpy(&depthFrame->frame[4], &depthRows, sizeof(int32_t));                                       
        //     // memcpy(&depthFrame->frame[8], (unsigned char*)(frameDepthMat.data), depthSize);              

        //     // if (stream_depth)
        //     //     current_frame_.push_back(depthFrame);

        // // THIS IS WEHRE ALL COMENTED OUT ENDS

        } catch(std::exception &e) {
            std::cerr << "FAILED ON NEXTFRAME" << std::endl;
            std::cerr << "TRY/CATCH CATCH " << e.what() << std::endl << std::flush;
            failed = true;
            std::this_thread::sleep_for(std::chrono::milliseconds(WAIT_AFTER_RETRIAL));
        }

        break;
    }
}

bool OakdXlinkFullReader::HasNextFrame() { return true; }

void OakdXlinkFullReader::Reset() {}

std::vector<std::shared_ptr<FrameStruct>> OakdXlinkFullReader::GetCurrentFrame() {
  return current_frame_;
}

unsigned int OakdXlinkFullReader::GetFps() {
  return 30;
}

std::vector<FrameType> OakdXlinkFullReader::GetType() {
  std::vector<FrameType> types;

    if (stream_rgb)
        types.push_back(FrameType::FrameTypeColor);
    if (stream_depth)
        types.push_back(FrameType::FrameTypeDepth);
    if (stream_bodies)
        types.push_back(FrameType::FrameTypeDetection); // 5

    return types;

}

void OakdXlinkFullReader::GoToFrame(unsigned int frame_id) {}
unsigned int OakdXlinkFullReader::GetCurrentFrameId() {return current_frame_counter_;}

}
