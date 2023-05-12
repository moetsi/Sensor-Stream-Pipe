
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
cv::Rect frame_norm(const cv::Mat& frame, const std::vector<float>& bbox) {
    int xMin = std::max(static_cast<int>(bbox[0] * frame.cols), 0);
    int yMin = std::max(static_cast<int>(bbox[1] * frame.rows), 0);
    int xMax = std::min(static_cast<int>(bbox[2] * frame.cols), frame.cols - 1);
    int yMax = std::min(static_cast<int>(bbox[3] * frame.rows), frame.rows - 1);

    return cv::Rect(cv::Point(xMin, yMin), cv::Point(xMax, yMax));
}

void TwoStageHostSeqSync::add_msg(const std::shared_ptr<void>& msg, const std::string& name) {
    try {
        int64_t seq;
        if (name == "color") {
            seq = std::static_pointer_cast<dai::ImgFrame>(msg)->getSequenceNum();
        } else if (name == "detection") {
            seq = std::static_pointer_cast<dai::ImgDetections>(msg)->getSequenceNum();
        } else if (name == "recognition") {
            seq = std::static_pointer_cast<dai::NNData>(msg)->getSequenceNum();
        } else {
            throw std::runtime_error("Invalid message name: " + name);
        }

        // std::cerr << "SEQ Number for " << name << " is " << seq << std::endl << std::flush;

        std::string seq_str = std::to_string(seq);
        if (msgs.find(seq_str) == msgs.end()) {
            msgs[seq_str] = {};
        }

        if (name == "recognition") {
            if (msgs[seq_str].find("recognition") == msgs[seq_str].end()) {
                msgs[seq_str]["recognition"] = std::make_shared<std::vector<std::shared_ptr<dai::NNData>>>();
            }
            std::static_pointer_cast<std::vector<std::shared_ptr<dai::NNData>>>(msgs[seq_str]["recognition"])->push_back(std::static_pointer_cast<dai::NNData>(msg));
        } else {
            msgs[seq_str][name] = msg;
            if (name == "detection") {
                msgs[seq_str]["len"] = std::make_shared<size_t>(std::static_pointer_cast<dai::ImgDetections>(msg)->detections.size());
            }
        }
    } catch (const std::exception &e) {
        std::cerr << "Error in add_msg: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "Unknown error in add_msg" << std::endl;
    }
}

std::unordered_map<std::string, std::shared_ptr<void>> TwoStageHostSeqSync::get_msgs() {
    try {
        std::vector<std::string> seq_remove;

        for (auto& it : msgs) {

            seq_remove.push_back(it.first);

            if (it.second.find("color") != it.second.end() && it.second.find("len") != it.second.end() && it.second.find("recognition") != it.second.end()) {
                try {
                    size_t len = *std::static_pointer_cast<size_t>(it.second["len"]);
                    std::vector<std::shared_ptr<dai::NNData>> recognition_vec = *std::static_pointer_cast<std::vector<std::shared_ptr<dai::NNData>>>(it.second["recognition"]);
                    if (len == recognition_vec.size()) {
                        std::unordered_map<std::string, std::shared_ptr<void>> synced_msgs = it.second;
                        for (const std::string& rm : seq_remove) {
                            msgs.erase(rm);
                        }

                        return synced_msgs;
                    }
                } catch (const std::exception &e) {
                    std::cerr << "Error in get_msgs: " << e.what() << std::endl;
                }
            }
        }
        return {}; // No synced messages
    } catch (const std::exception &e) {
        std::cerr << "Error in add_msg: " << e.what() << std::endl;
    } catch (...) {
        std::cerr << "Unknown error in add_msg" << std::endl;
    }
}

OakdXlinkFullReader::OakdXlinkFullReader(YAML::Node config) {

    outputFile = std::ofstream("output.txt");
    std::streambuf* coutbuf = std::cout.rdbuf(); // save old buf
    // std::cout.rdbuf(outputFile.rdbuf()); // redirect std::cout to output.txt


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
    model_detection_path = config["model_detection"].as<std::string>();
    model_detection_path = StringInterpolation(env, model_detection_path);
    std::cerr << "detection path is " << model_detection_path << std::endl << std::flush;
    model_detection_path2 = model_detection_path;

    model_reid_path = config["model_reid"].as<std::string>();
    model_reid_path = StringInterpolation(env, model_reid_path);
    std::cerr << "reid path is " << model_reid_path << std::endl << std::flush;
    model_reid_path2 = model_reid_path;

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
    st->rgb_dai_preview_x = config["rgb_preview_size_x"].as<unsigned int>();
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

    // 1. The pipeline is created and the nodes are added to it.
    // 2. The properties of the nodes are set.
    // 3. The nodes are linked.
    // 4. The device is opened with the pipeline.
    // 5. The output queue is created to receive the frames from the output.
    // 6. The input queue is created to send the control messages.
    // 7. The focus is set to manual and the value is set to 130.
    // 8. The control message is sent to the input queue. 

    // Define source and output
    pipeline = std::make_shared<dai::Pipeline>();
    
    // ColorCamera setup
    camRgb = pipeline->create<dai::node::ColorCamera>();
    camRgb->setPreviewSize(st->rgb_dai_preview_x, st->rgb_dai_preview_y);
    camRgb->setResolution(st->rgb_dai_res);
    camRgb->setInterleaved(false);
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::RGB);
    camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);

    // camRgbManip = pipeline->create<dai::node::ImageManip>();
    // camRgbManip-> initialConfig.setResize(408, 240);
    // camRgbManip->initialConfig.setFrameType(dai::ImgFrame::Type::BGR888p);
    // camRgb->preview.link(camRgbManip->inputImage);

    rgbOut = pipeline->create<dai::node::XLinkOut>();
    // Here I will take the camRgb frames and downscale them before sending them out

    rgbOut->setStreamName("rgb");
    // camRgbManip->out.link(rgbOut->input);
    // camRgb->preview.link(rgbOut->input);

    // Color Properties
    camRgb->setFps(st->rgb_dai_fps);
    camRgb->setPreviewNumFramesPool(10);

    // ImageManip will resize the frame before sending it to the person detection NN node
    person_det_manip = pipeline->create<dai::node::ImageManip>();
    person_det_manip->initialConfig.setResize(544, 320); // This seems to downscale it by 1/3 (544x320)
    person_det_manip->initialConfig.setFrameType(dai::ImgFrame::Type::RGB888p);
    camRgb->preview.link(person_det_manip->inputImage);
    // person_det_manip->out.link(rgbOut->input);

    // Person detection nn setup
    cout << "Creating Person Detection Neural Network..." << endl;
    person_nn = pipeline->create<dai::node::MobileNetDetectionNetwork>();
    person_nn->setConfidenceThreshold(0.5);
    person_nn->setBlobPath(model_detection_path);
    person_det_manip->out.link(person_nn->input);

    // Send person detections to the host (for bounding boxes)
    person_det_xout = pipeline->create<dai::node::XLinkOut>();
    person_det_xout->setStreamName("detection");
    person_nn->out.link(person_det_xout->input);
    person_nn->passthrough.link(rgbOut->input);

    // // We set up the script node that takes the detections from the nn and
    // // sets the ImageManipConfig on the device to send to the recognition_manip to cro
    image_manip_script = pipeline->create<dai::node::Script>();
    camRgb->preview.link(image_manip_script->inputs["preview"]);
    person_nn->out.link(image_manip_script->inputs["dets_in"]);
    // Only sending metadata for syncing with color frames
    person_nn->passthrough.link(image_manip_script->inputs["passthrough"]);
    image_manip_script->setScript(R"(
    import time
    msgs = dict()
    def add_msg(msg, name, seq = None):
        global msgs
        if seq is None:
            seq = msg.getSequenceNum()
        seq = str(seq)
        # node.warn(f"New msg {name}, seq {seq}")
        # Each seq number has it's own dict of msgs
        if seq not in msgs:
            msgs[seq] = dict()
        msgs[seq][name] = msg
        # To avoid freezing (not necessary for this ObjDet model)
        if 15 < len(msgs):
            node.warn(f"Removing first element! len {len(msgs)}")
            msgs.popitem() # Remove first element
    def get_msgs():
        global msgs
        seq_remove = [] # Arr of sequence numbers to get deleted
        for seq, syncMsgs in msgs.items():
            seq_remove.append(seq) # Will get removed from dict if we find synced msgs pair
            # node.warn(f"Checking sync {seq}")
            # Check if we have both detections and color frame with this sequence number
            if len(syncMsgs) == 2: # 1 frame, 1 detection
                for rm in seq_remove:
                    del msgs[rm]
                # node.warn(f"synced {seq}. Removed older sync values. len {len(msgs)}")
                return syncMsgs # Returned synced msgs
        return None
    def correct_bb(bb):
        if bb.xmin < 0: bb.xmin = 0.001
        if bb.ymin < 0: bb.ymin = 0.001
        if bb.xmax > 1: bb.xmax = 0.999
        if bb.ymax > 1: bb.ymax = 0.999
        return bb
    while True:
        time.sleep(0.001) # Avoid lazy looping
        preview = node.io['preview'].tryGet()
        if preview is not None:
            add_msg(preview, 'preview')
        dets = node.io['dets_in'].tryGet()
        if dets is not None:
            # TODO: in 2.18.0.0 use dets.getSequenceNum()
            passthrough = node.io['passthrough'].get()
            seq = passthrough.getSequenceNum()
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
                # node.warn(f"Sending {i + 1}. person det. Seq {img_seq}. Det {det.xmin}, {det.ymin}, {det.xmax}, {det.ymax}")
                cfg.setResize(128, 256)
                cfg.setKeepAspectRatio(False)
                node.io['manip_cfg'].send(cfg)
                node.io['manip_img'].send(img)
    )");

    recognition_manip = pipeline->create<dai::node::ImageManip>();
    recognition_manip->initialConfig.setResize(128, 256);
    recognition_manip->setWaitForConfigInput(true);
    image_manip_script->outputs["manip_cfg"].link(recognition_manip->inputConfig);
    image_manip_script->outputs["manip_img"].link(recognition_manip->inputImage);
    
    
    cout << "Creating Recognition Neural Network..." << endl;
    recognition_nn = pipeline->create<dai::node::NeuralNetwork>();
    recognition_nn->setBlobPath(model_reid_path);
    recognition_manip->out.link(recognition_nn->input);

    recognition_nn_xout = pipeline->create<dai::node::XLinkOut>();
    recognition_nn_xout->setStreamName("recognition");
    recognition_nn->out.link(recognition_nn_xout->input);

    // // Stereo Depth setup
    // left = pipeline->create<dai::node::MonoCamera>();
    // right = pipeline->create<dai::node::MonoCamera>();
    // stereo = pipeline->create<dai::node::StereoDepth>();
    // depthOut = pipeline->create<dai::node::XLinkOut>();
    // depthOut->setStreamName("depth");
    // left->out.link(stereo->left);
    // right->out.link(stereo->right);
    // stereo->depth.link(depthOut->input);
 
    // // Depth Properties
    // left->setResolution(st->depth_dai_res);
    // left->setBoardSocket(dai::CameraBoardSocket::LEFT);
    // left->setFps(st->depth_dai_fps);
    // right->setResolution(st->depth_dai_res);
    // right->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    // right->setFps(st->depth_dai_fps);
    // stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_ACCURACY);
    // stereo->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_7x7);
    // stereo->setSubpixel(true);
    // stereo->setLeftRightCheck(true); // LR-check is required for depth alignment
    // stereo->setDepthAlign(dai::CameraBoardSocket::RGB);
    // stereo->setOutputSize(st->depth_dai_preview_x, st->depth_dai_preview_y);
    // auto oakdConfig = stereo->initialConfig.get();
    // oakdConfig.postProcessing.spatialFilter.enable = st->depth_dai_sf;
    // oakdConfig.postProcessing.spatialFilter.holeFillingRadius = st->depth_dai_sf_hfr;
    // oakdConfig.postProcessing.spatialFilter.numIterations = st->depth_dai_sf_num_it;
    // oakdConfig.postProcessing.decimationFilter.decimationFactor = st->depth_dai_df;
    // stereo->initialConfig.set(oakdConfig);


    //Select which device through ip address and create pipeline
    std::cerr << "Trying to create pipeline" << std::endl << std::flush;  
    device_info = std::make_shared<dai::DeviceInfo>(ip_name);
    device = std::make_shared<dai::Device>(*pipeline, *device_info, true); // usb 2 mode
    std::cerr << "Created pipeline" << std::endl << std::flush;  

    // Capture the log output
    // std::ofstream outputFile("output.txt");
    // std::cout.rdbuf(outputFile.rdbuf());

    // Connect to device and start pipeline
    std::cerr << "Connected cameras: " << std::endl << std::flush;        
    std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;                     
    for(const auto& cam : device->getConnectedCameras()) {
        std::cerr << static_cast<int>(cam) << " ";
        std::cerr << cam << " ";
    }
    std::cerr << endl;

    // Set up the output queues
    for (const auto &name : {"rgb", "detection", "recognition"}) {
        queues[name] = device->getOutputQueue(name, 10, true);
    }

    spdlog::debug("Done opening");

    // Close the file
    // outputFile.close();
}
void OakdXlinkFullReader::SetOrResetState(const std::shared_ptr<State> &st, int n) {

    start_time = CurrentTimeMs();
}

OakdXlinkFullReader::~OakdXlinkFullReader() {



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
 
            // std::cerr << "FRAMES A SECOND: " << framesASecond << std::endl << std::flush;

                // std::cerr << "rgb" << std::endl << std::flush;
            if (queues["rgb"]->has()) {
                auto rgb_message = queues["rgb"]->get();
                int64_t seq = std::static_pointer_cast<dai::ImgFrame>(rgb_message)->getSequenceNum();
                sync.add_msg(rgb_message, "color");
                current_rgb_frame_counter_++;
                uint64_t capture_timestamp = CurrentTimeMs();
                auto framesASecond = (float)current_rgb_frame_counter_/((float)(capture_timestamp - start_time)*.001);
                auto fps_string = std::to_string(framesASecond);
                std::cerr << "color fps: " << fps_string << std::endl << std::flush;
                // std::cerr << "color seq num: " << seq << std::endl << std::flush;
            }
            if (queues["detection"]->has()) {
                // std::cerr << "detection" << std::endl << std::flush;q
                auto det_message = queues["detection"]->get();
                sync.add_msg(det_message, "detection");
                int64_t seq = std::static_pointer_cast<dai::ImgDetections>(det_message)->getSequenceNum();
                current_detection_frame_counter_++;
                uint64_t capture_timestamp = CurrentTimeMs();
                auto framesASecond = (float)current_detection_frame_counter_/((float)(capture_timestamp - start_time)*.001);
                auto fps_string = std::to_string(framesASecond);
                std::cerr << "detection fps: " << fps_string << std::endl << std::flush;
                // std::cerr << "detection seq num: " << seq << std::endl << std::flush;
            }
            if (queues["recognition"]->has()) {
                auto rec_message = queues["recognition"]->get();
                sync.add_msg(rec_message, "recognition");
                int64_t seq = std::static_pointer_cast<dai::NNData>(rec_message)->getSequenceNum();
                current_recognition_frame_counter_++;
                uint64_t capture_timestamp = CurrentTimeMs();
                auto framesASecond = (float)current_recognition_frame_counter_/((float)(capture_timestamp - start_time)*.001);
                auto fps_string = std::to_string(framesASecond);
                // std::cerr << "recognition fps: " << fps_string << std::endl << std::flush;
                // std::cerr << "recognition seq num: " << seq << std::endl << std::flush;
            }
            // for (auto& it : queues) {
            //     const std::string& name = it.first;
            //     std::shared_ptr<dai::DataOutputQueue>& q = it.second;
            //     std::cerr << "QUEUE " << name << std::endl << std::flush;
            //     if (q->has()) {
            //         std::cerr << "Q HAS" << std::endl << std::flush;            cv::Mat frameRgbOpenCv = std::static_pointer_cast<dai::ImgFrame>(msgs["color"])->getCvFrame();
            //         sync.add_msg(q->get(), name);
            //     }
            // }

            auto msgs = sync.get_msgs();
            if (msgs.empty()) {
                return;
            }
           //Increment counter and grab time
            current_frame_counter_++;
            uint64_t capture_timestamp = CurrentTimeMs();
            auto framesASecond = (float)current_frame_counter_/((float)(capture_timestamp - start_time)*.001);
            auto fps_string = std::to_string(framesASecond);

            cv::Mat frameRgbOpenCv = std::static_pointer_cast<dai::ImgFrame>(msgs["color"])->getCvFrame();
            // resizing frames
            cv::Mat resizedFrame;
            cv::Size size(1632, 960); 
            cv::resize(frameRgbOpenCv, resizedFrame, size); // resize image

            auto detections = std::static_pointer_cast<dai::ImgDetections>(msgs["detection"])->detections;
            auto recognitions = std::static_pointer_cast<std::vector<std::shared_ptr<dai::NNData>>>(msgs["recognition"]);


            cv::putText(resizedFrame, fps_string, cv::Point(10, 30), cv::FONT_HERSHEY_TRIPLEX, 1, cv::Scalar(0, 0, 0), 8);
            cv::putText(resizedFrame, fps_string, cv::Point(10, 30), cv::FONT_HERSHEY_TRIPLEX, 1, cv::Scalar(255, 255, 255), 2);

            for (size_t i = 0; i < detections.size(); ++i) {
                auto detection = detections[i];
                auto bbox = frame_norm(resizedFrame, {detection.xmin, detection.ymin, detection.xmax, detection.ymax});

                auto reid_result = (*recognitions)[i]->getFirstLayerFp16();

                bool found = false;
                int reid_id;
                for (size_t j = 0; j < reid_results.size(); ++j) {
                    auto dist = cos_dist(reid_result, reid_results[j]);
                    if (dist > 0.7) {
                        reid_results[j] = reid_result;
                        reid_id = j;
                        found = true;
                        break;
                    }
                }

                if (!found) {
                    reid_results.push_back(reid_result);
                    reid_id = reid_results.size() - 1;
                }

                cv::rectangle(resizedFrame, cv::Point(bbox.x, bbox.y), cv::Point(bbox.x + bbox.width, bbox.y + bbox.height), cv::Scalar(10, 245, 10), 2);
                int y = (bbox.y + bbox.y + bbox.height) / 2;
                std::string person_text = "Person reid " + std::to_string(reid_id);
                cv::putText(resizedFrame, person_text, cv::Point(bbox.x, y), cv::FONT_HERSHEY_TRIPLEX, 1.5, cv::Scalar(0, 0, 0), 8);
                cv::putText(resizedFrame, person_text, cv::Point(bbox.x, y), cv::FONT_HERSHEY_TRIPLEX, 1.5, cv::Scalar(255, 255, 255), 2);
            }

            cv::imshow("Camera", resizedFrame);
            cv::waitKey(1);

            // Color frame
            std::shared_ptr<FrameStruct> rgbFrame =
                std::shared_ptr<FrameStruct>(new FrameStruct(frame_template_));
                rgbFrame->sensor_id = 0; 
            rgbFrame->frame_type = FrameType::FrameTypeColor; // 0;
            rgbFrame->frame_data_type = FrameDataType::FrameDataTypeCvMat; // 9;
            rgbFrame->frame_id = current_frame_counter_;
            rgbFrame->timestamps.push_back(capture_timestamp);

            // convert the raw buffer to cv::Mat
            int32_t colorCols = resizedFrame.cols;                                                        
            int32_t colorRows = resizedFrame.rows;                                                        
            size_t colorSize = colorCols*colorRows*3*sizeof(uchar); //This assumes that oakd color always returns CV_8UC3

            rgbFrame->frame.resize(colorSize + 2 * sizeof(int32_t));                                        

            memcpy(&rgbFrame->frame[0], &colorCols, sizeof(int32_t));                                       
            memcpy(&rgbFrame->frame[4], &colorRows, sizeof(int32_t));                                       
            memcpy(&rgbFrame->frame[8], (unsigned char*)(resizedFrame.data), colorSize);

            //Depth frame
            // std::shared_ptr<FrameStruct> depthFrame =
            //     std::shared_ptr<FrameStruct>(new FrameStruct(frame_template_));
            // depthFrame->sensor_id = 1;
            // depthFrame->frame_type = FrameType::FrameTypeDepth; // 1;
            // depthFrame->frame_data_type = FrameDataType::FrameDataTypeDepthAIStereoDepth; // 11; It is not when not subpixel
            // depthFrame->frame_id = current_frame_counter_;
            // depthFrame->timestamps.push_back(capture_timestamp);

            // // convert the raw buffer to cv::Mat
            // int32_t depthCols = frameDepthMat.cols;                                                        
            // int32_t depthRows = frameDepthMat.rows;                                                        
            // size_t depthSize = depthCols*depthRows*sizeof(uint16_t); // DepthAI StereoDepth outputs ImgFrame message that carries RAW16 encoded (0..65535) depth data in millimeters.

            // depthFrame->frame.resize(depthSize + 2 * sizeof(int32_t));                                        

            // memcpy(&depthFrame->frame[0], &depthCols, sizeof(int32_t));                                       
            // memcpy(&depthFrame->frame[4], &depthRows, sizeof(int32_t));                                       
            // memcpy(&depthFrame->frame[8], (unsigned char*)(frameDepthMat.data), depthSize);              

            // if (stream_depth)
            //     current_frame_.push_back(depthFrame);
            outputFile.close();
              

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
        types.push_back(FrameType::FrameTypeHumanPose); // 4;

    return types;

}

void OakdXlinkFullReader::GoToFrame(unsigned int frame_id) {}
unsigned int OakdXlinkFullReader::GetCurrentFrameId() {return current_frame_counter_;}

}
