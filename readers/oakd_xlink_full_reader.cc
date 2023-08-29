
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

// TO-DO: Fix so only the lower seq num keys get deleted
MessageData TwoStageHostSeqSync::get_msgs() {
    try {
        int64_t maxSyncSeq = INT64_MIN;   // Store max seq number having synchronized messages
        std::unordered_map<int64_t, MessageData>::iterator maxSyncSeqIt;

        // Iterate over the container and identify the maximum synchronized sequence.
        for (auto it = msgs.begin(); it != msgs.end();) {
            if (!it->second.color || !it->second.person_detection || !it->second.face_detection) {
                ++it;
                continue;
            }

            auto spatialImgDetPtr = std::static_pointer_cast<dai::SpatialImgDetections>(it->second.person_detection);
            if (!spatialImgDetPtr) throw std::runtime_error("Failed to cast message to SpatialImgDetections");

            if (spatialImgDetPtr->detections.size() == 0 ||
                (spatialImgDetPtr->detections.size() ==  it->second.recognitions.size() &&
                 spatialImgDetPtr->detections.size() == it->second.fast_descriptions.size())) {
                if (it->first > maxSyncSeq) {
                    maxSyncSeq = it->first;
                    maxSyncSeqIt = it;
                }
            }

            // Check if current sequence number is less than maxSyncSeq, if yes erase it.
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

OakdXlinkFullReader::OakdXlinkFullReader(YAML::Node config) {

    // std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
    spdlog::debug("Starting to open");
    // std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
    
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

    // We keep track of info 
    params.drop_forgotten_tracks = false;
    params.max_num_objects_in_track = -1;

    // Initialize tracker
    tracker = std::make_unique<PedestrianTracker>(params);
    // cv::Size graphSize = cv::Size(static_cast<int>(frame.cols / 4), 60);

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

    // FACE DETECTION SECTION
    // 
    // // face_det_manip will resize the frame before sending it to the face detection NN node
    face_det_manip = pipeline->create<dai::node::ImageManip>();
    face_det_manip->initialConfig.setResizeThumbnail(rgb_face_det_nn_in_x_res, rgb_face_det_nn_in_y_res);
    face_det_manip->initialConfig.setFrameType(dai::ImgFrame::Type::RGB888p);
    camRgb->preview.link(face_det_manip->inputImage);

    // Face detection nn setup
    cout << "Creating Face Detection Neural Network..." << endl;
    face_nn = pipeline->create<dai::node::NeuralNetwork>();
    face_nn->setBlobPath(model_face_detection_path);
    face_det_manip->out.link(face_nn->input);
    // Link the face detection nn to the post processing nn
    cout << "Creating Face Detection Processing Neural Network..." << endl;
    face_post_proc_nn = pipeline->create<dai::node::NeuralNetwork>();
    face_post_proc_nn->setBlobPath(model_face_detection_proc_path);
    face_nn->out.link(face_post_proc_nn->input);

    // // Send face detections to the host (for bounding boxes)
    face_det_xout = pipeline->create<dai::node::XLinkOut>();
    face_det_xout->setStreamName("face_detection");
    face_post_proc_nn->out.link(face_det_xout->input);

    // PERSON DETECTION SECTION
    // person_det_manip will resize the frame before sending it to the person detection NN node
    person_det_manip = pipeline->create<dai::node::ImageManip>();
    person_det_manip->initialConfig.setResize(rgb_person_det_nn_in_x_res, rgb_person_det_nn_in_y_res); // requied for nn (544, 320)
    person_det_manip->initialConfig.setFrameType(dai::ImgFrame::Type::RGB888p);
    person_det_manip->setNumFramesPool(10);
    // camRgb->preview.link(person_det_manip->inputImage); !!!

    // !!Logging test!!
    // We are going to create a script node that takes in the person_det_manip, prints seq_num, and outputs to person_nn
    log_image_manip_input_script = pipeline->create<dai::node::Script>();
    camRgb->preview.link(log_image_manip_input_script->inputs["input_preview"]);
    log_image_manip_input_script->setScript(R"(
    import time
    while True:
        time.sleep(0.001)
        input_preview = node.io['input_preview'].tryGet()
        if input_preview is not None:
            # node.warn(f"CamRgb Out SeqNum: {input_preview.getSequenceNum()}")
            node.io['output_preview'].send(input_preview)
    )");
    log_image_manip_input_script->outputs["output_preview"].link(person_det_manip->inputImage);

    log_person_nn_input_script = pipeline->create<dai::node::Script>();
    person_det_manip->out.link(log_person_nn_input_script->inputs["input_manip"]);
    log_person_nn_input_script->setScript(R"(
    import time
    while True:
        time.sleep(0.001)
        input_manip = node.io['input_manip'].tryGet()
        if input_manip is not None:
            # node.warn(f"ImageManip Output SeqNum: {input_manip.getSequenceNum()}")
            node.io['output_manip'].send(input_manip)
    )");

    log_person_nn_input_depth_script = pipeline->create<dai::node::Script>();
    stereo->depth.link(log_person_nn_input_depth_script->inputs["input_depth"]);
    log_person_nn_input_depth_script->setScript(R"(
    import time
    while True:
        time.sleep(0.001)
        input_depth = node.io['input_depth'].tryGet()
        if input_depth is not None:
            # node.warn(f"Depth Output SeqNum: {input_depth.getSequenceNum()}")
            node.io['output_depth'].send(input_depth)
    )");

    // Person detection spatial nn setup
    cout << "Creating Person Detection Spatial Neural Network..." << endl;
    person_nn = pipeline->create<dai::node::MobileNetSpatialDetectionNetwork>();
    person_nn->setBoundingBoxScaleFactor(0.25);
    person_nn->setDepthLowerThreshold(50);
    person_nn->setDepthUpperThreshold(12000);
    person_nn->setBlobPath(model_person_detection_path);
    person_nn->setNumPoolFrames(10);
    // person_nn->setBlocking(true);
    // Link the manip output and the depth output to the person detection nn
    // person_det_manip->out.link(person_nn->input);
    // Now we link the log
    log_person_nn_input_script->outputs["output_manip"].link(person_nn->input);
    // stereo->depth.link(person_nn->inputDepth);
    log_person_nn_input_depth_script->outputs["output_depth"].link(person_nn->inputDepth);

    log_person_nn_output_script = pipeline->create<dai::node::Script>();
    person_nn->out.link(log_person_nn_output_script->inputs["input_dets"]);
    log_person_nn_output_script->setScript(R"(
    import time
    while True:
        time.sleep(0.001)
        input_dets = node.io['input_dets'].tryGet()
        if input_dets is not None:
            # node.warn(f"Person Detection Output SeqNum: {input_dets.getSequenceNum()}")
            node.io['output_dets'].send(input_dets)
    )");

    // Send person detections to the host (for bounding boxes)
    person_det_xout = pipeline->create<dai::node::XLinkOut>();
    person_det_xout->setStreamName("person_detection");
    person_nn->out.link(person_det_xout->input);
    // Send the person detection pass through to the host (for rgb frames)
    rgbOut = pipeline->create<dai::node::XLinkOut>();
    rgbOut->setStreamName("rgb");
    person_nn->passthrough.link(rgbOut->input);

    // Set up the person reid strong config manip node for the person reid strong nn
    person_config_manip_for_reid_nn_script = pipeline->create<dai::node::Script>();
    // using preview, but now going to use manip
    // camRgb->preview.link(person_config_manip_for_reid_nn_script->inputs["preview"]);
    person_nn->passthrough.link(person_config_manip_for_reid_nn_script->inputs["preview"]);
    // person_nn->out.link(person_config_manip_for_reid_nn_script->inputs["person_dets_in"]); !!!
    log_person_nn_output_script->outputs["output_dets"].link(person_config_manip_for_reid_nn_script->inputs["person_dets_in"]);
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
    camRgb->preview.link(person_config_manip_for_fast_desc_script->inputs["preview"]);
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

    // DEPTH CONTROL SECTION
    // Depth control frame and new depth frame script setup
    // depth_control_script = pipeline->create<dai::node::Script>();
    // stereo->depth.link(depth_control_script->inputs["depth_in"]);
    // depth_control_script->setScript(R"(
    //     import time

    //     control_depth = node.io['depth_in'].get()
    //     current_time = time.time()

    //     # we wait an arbitrary 5 seconds for the depth to stabilize
    //     while time.time() - current_time < 8:
    //         time.sleep(0.001) # Avoid lazy looping
    //         control_depth = node.io['depth_in'].get()

    //     while True:
    //         time.sleep(0.001) # Avoid lazy looping
    //         new_depth = node.io['depth_in'].get()
    //         if new_depth is not None:
    //             node.io['depth_control'].send(control_depth)
    //             node.io['depth_new'].send(new_depth)
    //     )");

    // Depth diff nn setup
    cout << "Creating Depth Diff Neural Network..." << endl;
    // depth_diff_nn = pipeline->create<dai::node::NeuralNetwork>();
    // depth_diff_nn->setBlobPath(model_depth_diff_path);
    // depth_control_script->outputs["depth_control"].link(depth_diff_nn->inputs["input1"]);
    // depth_control_script->outputs["depth_new"].link(depth_diff_nn->inputs["input2"]);
    depth_diff_xout = pipeline->create<dai::node::XLinkOut>();
    depth_diff_xout->setStreamName("depth_diff");
    // depth_diff_nn->out.link(depth_diff_xout->input);

    //Select which device through ip address and create pipeline
    // auto deviceInfoVec = dai::Device::getAllAvailableDevices();
    // Now we print all available devices
    // std::cerr << "Available devices: " << std::endl << std::flush;
    // for(const auto& deviceInfo : deviceInfoVec) {
    //     std::cerr << "Device info: " << deviceInfo.getMxId() << std::endl << std::flush;
    // }

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
    for (const auto &name : {"rgb", "face_detection", "depth_diff", "person_detection", "recognition", "fast_desc"}) {
        queues[name] = device->getOutputQueue(name, 25, false);
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

void OakdXlinkFullReader::NextFrame() {


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

            if (queues["rgb"]->has()) {
                auto rgb_message = queues["rgb"]->get();
                sync.add_msg(rgb_message, "color");
            }
            if (queues["person_detection"]->has()) {
                auto det_message = queues["person_detection"]->get();
                sync.add_msg(det_message, "person_detection");
            }
            if (queues["recognition"]->has()) {
                auto rec_message = queues["recognition"]->get();
                sync.add_msg(rec_message, "recognition");
            }
            if (queues["fast_desc"]->has()) {
                auto det_fast_desc_message = queues["fast_desc"]->get();
                sync.add_msg(det_fast_desc_message, "fast_desc");
            }
            if (queues["face_detection"]->has()) {
                auto det_message = queues["face_detection"]->get<dai::NNData>();
                sync.add_msg(det_message, "face_detection");
            }
            if (queues["depth_diff"]->has()) {
                int width = 544;
                int height = 320;
                auto depth_diff_message = queues["depth_diff"]->get<dai::NNData>();
                std::vector<float> depth_diff_data = depth_diff_message->getFirstLayerFp16();
                // std::cerr << "depth diff data size: " << depth_diff_data.size() << std::endl << std::flush;
                
                // Now we go through depth_diff data and for any values between -1500 and 1500 we set it to 0
                // TODO: This is a hack to get rid of the noise in the depth diff image
                // TODO: We should probably do this in the model
                for (int i = 0; i < depth_diff_data.size(); i++) {
                    if (depth_diff_data[i] > -500 && depth_diff_data[i] < 500) {
                        depth_diff_data[i] = 0;
                    }
                    
                }
                // Convert your data to Mat
                cv::Mat depth_diff_mat(height, width, CV_32F, depth_diff_data.data());
                depth_diff_mat.convertTo(depth_diff_mat, CV_8U, 255.0);

                // Now we normalize
                cv::Mat depth_diff_mat_normalized;
                cv::normalize(depth_diff_mat, depth_diff_mat_normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);
                // Now we colorize

                cv::namedWindow("Depth Difference Image", cv::WINDOW_NORMAL);
                cv::imshow("Depth Difference Image", depth_diff_mat_normalized);
                cv::waitKey(1);

            }

            // Now we check if all messages have come in for a given color (it checks if all messages have the same sequence number)
            auto msgs = sync.get_msgs();
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
            // std:cerr << "Detections size: " << detections.size() << std::endl << std::flush;
            
            tracker->Process(frame, detections, epoch_time);

            auto recent_detections = tracker->GetMostRecentDetections();

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
