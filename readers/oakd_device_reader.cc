//
// Created by adammpolak on 26-09-2021.
//

#include "oakd_device_reader.h"
#include "human_poses.h"
#include <cmath>
#include <filesystem>
using namespace std;
using namespace InferenceEngine;

//#define VERY_VERBOSE

namespace moetsi::ssp {
using namespace human_pose_estimation;

OakdDeviceReader::OakdDeviceReader(YAML::Node config) {

std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
    spdlog::debug("Starting to open");
std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
    
    fps = config["streaming_rate"].as<unsigned int>();
    
    current_frame_counter_ = 0;
    frame_template_.stream_id = RandomString(16);
    frame_template_.device_id = config["deviceid"].as<unsigned int>();
    frame_template_.scene_desc = "oakd";

    frame_template_.frame_id = 0;
    frame_template_.message_type = SSPMessageType::MessageTypeDefault; // 0;

std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;

    // Define source and output
    camRgb = pipeline.create<dai::node::ColorCamera>();
    left = pipeline.create<dai::node::MonoCamera>();
    right = pipeline.create<dai::node::MonoCamera>();
    stereo = pipeline.create<dai::node::StereoDepth>();
    // SETTING UP THE ON-DEVICE INFERENCE
    // HARDWARID BLOB WAS MADE BY FOLLOWING DEPTHAI HOW-TO
    // https://docs.luxonis.com/en/latest/pages/model_conversion/
    nn = pipeline.create<dai::node::NeuralNetwork>();
    nn->setBlobPath(input_blob);
    nn->setNumInferenceThreads(2);
    
    // # Only send metadata, we are only interested in timestamp, so we can sync
    // # depth frames with NN output

    // rgbOut = pipeline.create<dai::node::XLinkOut>();
    depthOut = pipeline.create<dai::node::XLinkOut>();
    nnXout = pipeline.create<dai::node::XLinkOut>();

    // rgbOut->setStreamName("rgb");
    depthOut->setStreamName("depth");
    nnXout->setStreamName("nn");

    // Color Properties
    camRgb->setPreviewSize(384, 256);
    camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setFps(7);
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::RGB);
    camRgb->setInterleaved(false);

    // Depth Properties
    auto monoRes = dai::MonoCameraProperties::SensorResolution::THE_720_P;
    left->setResolution(monoRes);
    left->setBoardSocket(dai::CameraBoardSocket::LEFT);
    left->setFps(7);
    right->setResolution(monoRes);
    right->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    right->setFps(7);
    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_ACCURACY);
    stereo->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_7x7);
    stereo->setSubpixel(true);
    stereo->setLeftRightCheck(true); // LR-check is required for depth alignment
    stereo->setDepthAlign(dai::CameraBoardSocket::RGB);
    stereo->setOutputSize(640,360);
    stereo->setFocalLengthFromCalibration(true);
    auto oakdConfig = stereo->initialConfig.get();
    oakdConfig.postProcessing.spatialFilter.enable = true;
    oakdConfig.postProcessing.spatialFilter.holeFillingRadius = 2;
    oakdConfig.postProcessing.spatialFilter.numIterations = 1;
    stereo->initialConfig.set(oakdConfig);
    
    // Linking
    // camRgb->preview.link(rgbOut->input);
    camRgb->preview.link(nn->input);
    left->out.link(stereo->left);
    right->out.link(stereo->right);
    stereo->depth.link(depthOut->input);
    nn->out.link(nnXout->input);

    // Changing the IP address to the correct depthai format (const char*)
    char chText[48];
    std::string ip_name = config["ip"].as<std::string>();
    ip_name = StringInterpolation(ip_name);
    ip_name.copy(chText, ip_name.size(), 0);
    chText[ip_name.size()] = '\0';
    //Which sensor
    device_info = dai::DeviceInfo();
    strcpy(device_info.desc.name, chText);
    device_info.state = X_LINK_BOOTLOADER; 
    device_info.desc.protocol = X_LINK_TCP_IP;
    std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
 
    device = std::make_shared<dai::Device>(pipeline, device_info, true); // usb 2 mode   // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT
    std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
    deviceCalib = device->readCalibration();
    cameraIntrinsics = deviceCalib.getCameraIntrinsics(dai::CameraBoardSocket::RGB, 1280, 720);
    horizontalFocalLengthPixels = cameraIntrinsics[0][0];
    verticalFocalLengthPixels =  cameraIntrinsics[1][1];
    cameraHFOVInRadians = ((deviceCalib.getFov(dai::CameraBoardSocket::RGB) * pi) / 180.0) * (3840.0/4056.0); // Must scale for cropping: https://discordapp.com/channels/790680891252932659/924798503270625290/936746213691228260

    std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
    // Connect to device and start pipeline
    cout << "Connected cameras: ";
       std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;                     // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT
       for(const auto& cam : device->getConnectedCameras()) {
          cout << static_cast<int>(cam) << " ";
          cout << cam << " ";
      }
   cout << endl;

    // Output queue will be used to get the rgb frames from the output defined above
    // Sets queues size and behavior
    // qRgb = device->getOutputQueue("rgb", 4, false);                                      // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT
    qDepth = device->getOutputQueue("depth", 10, false);    
    qNn = device->getOutputQueue("nn", 10, false);

#ifndef _WIN32    
    std::string rel = "../../"; 
#endif
#ifdef _WIN32    
    std::string rel = "../../../";
#endif

    // start_time_in_ms = CurrentTimeMs();
}

OakdDeviceReader::~OakdDeviceReader() {

}

const float magic = 0.84381;
auto dumpVec = [](const SizeVector& vec) -> std::string {
    if (vec.empty())
        return "[]";
    std::stringstream oss;
    oss << "[" << vec[0];
    for (size_t i = 1; i < vec.size(); i++)
        oss << "," << vec[i];
    oss << "]";
    return oss.str();
};

auto toSizeVector = [](auto v) -> SizeVector {
    SizeVector rv;
    for(auto &x: v) {
        rv.push_back((size_t) x);
    }
    return rv;
};

void OakdDeviceReader::NextFrame() {

    // auto current_frame_time = CurrentTimeMs();
    // auto avg_rate = current_frame_counter_/((current_frame_time - start_time_in_ms)/1000.0);
    // std::cerr << "Avg Frame Rate:  " << avg_rate << std::endl << std::flush;
    
    current_frame_.clear();
    //Increment counter and grab time
    current_frame_counter_++;
    uint64_t capture_timestamp = CurrentTimeMs();

    // Here we try until we get a synchronized color and depth frame
    bool haveSyncedFrames = false;
    std::shared_ptr<dai::NNData> synchedNnFrame;
    std::shared_ptr<dai::ImgFrame> synchedDepthFrame;
    int seqNum;
    struct moetsi::ssp::human_pose_estimation::poses posesStruct;
    // Now we pull frames until we get a synchronized nn and depth frame
    // The dictionary has a key of seqNum (int) and value is a img::frame pointer
    // std::unordered_map<int, struct nn_and_depth { std::shared_ptr<dai::NNData> nn; std::shared_ptr<dai::ImgFrame> depth };>> frames_dictionary;
    while (!haveSyncedFrames)
    {
        auto nnFromQueue = qNn->tryGet<dai::NNData>();
        if (nnFromQueue != nullptr)
        {
            try{
            //We are going to run parse_poses on every NN data message to help with body tracking
            // DO WHAT YOU NEED TO DO TO RUN PARSE_POSES ON THE INFERENCE RESULTS
            std::vector<float> features_output_new = nnFromQueue->getLayerFp16("features");
            std::vector<float> heatmaps_output_new = nnFromQueue->getLayerFp16("heatmaps");
            std::vector<float> pafs_output_new = nnFromQueue->getLayerFp16("pafs");
            
            dai::TensorInfo featureInfo;
            dai::TensorInfo heatmapsInfo;
            dai::TensorInfo pafsInfo; 
            nnFromQueue->getLayer("features", featureInfo);
            nnFromQueue->getLayer("heatmaps", heatmapsInfo);
            nnFromQueue->getLayer("pafs", pafsInfo);

            const SizeVector features_output_shape = toSizeVector(featureInfo.dims);
            const SizeVector heatmaps_output_shape = toSizeVector(heatmapsInfo.dims);
            const SizeVector pafs_output_shape = toSizeVector(pafsInfo.dims);

            const float *features_result = &features_output_new[0]; // features_outputMapped.as<float *>();
            const float *heatmaps_result = &heatmaps_output_new[0]; // heatmaps_outputMapped.as<float *>();
            const float *pafs_result = &pafs_output_new[0]; // pafs_outputMapped.as<float *>();


            int featureMapSize[] = { (int)features_output_shape[1], (int)features_output_shape[2], (int)features_output_shape[3] }; // { 57,32,48 }; 
            cv::Mat featuresMat = cv::Mat(3, featureMapSize, CV_32FC1, const_cast<float*>(features_result));
            int heatmapMatSize[] =  { (int)heatmaps_output_shape[1], (int)heatmaps_output_shape[2], (int)heatmaps_output_shape[3] }; // { 19,32,48 };
            cv::Mat heatmapMat = cv::Mat(3, heatmapMatSize, CV_32FC1, const_cast<float*>(heatmaps_result));
            int paf_mapMatSize[] =  { (int) pafs_output_shape[1], (int) pafs_output_shape[2], (int) pafs_output_shape[3] };  // {38,32,48};
            cv::Mat paf_mapMat = cv::Mat(3, paf_mapMatSize, CV_32FC1, const_cast<float*>(pafs_result));

            matrix3x4 R = matrix3x4{ { {     
                    0.1656794936,
                    0.0336560618,
                    -0.9856051821,
                    0.0
                },
                {
                    -0.09224101321,
                    0.9955650135,
                    0.01849052095,
                    0.0
                },
                {
                    0.9818563545,
                    0.08784972047,
                    0.1680491765,
                    0.0
            } } };

            auto input_scale2 = input_scale * magic; // * 0.84; //  /1.04065; // -> 0.8*1280/984 || *0.83;
            posesStruct = parse_poses(previous_poses_2d, common, R, 
                featuresMat, heatmapMat, paf_mapMat, input_scale2, stride, fx,
                1080 //1080
                , true); //true);

            } catch(...) {
                return;
            }

            //Grab nn data and add to dictionary
            seqNum = nnFromQueue->getSequenceNum();
            if (frames_dictionary.find(seqNum) != frames_dictionary.end() && !haveSyncedFrames)
            {
                synchedNnFrame = nnFromQueue;
                synchedDepthFrame = frames_dictionary[seqNum].depth;
                haveSyncedFrames = true;
                break;
            }
            else
            {
                frames_dictionary[seqNum].nn = nnFromQueue;
            }
        }

        auto depthFromQueue = qDepth->tryGet<dai::ImgFrame>();
        if (depthFromQueue != nullptr)
        {
            //Grab depth and add to dictionary
            seqNum = depthFromQueue->getSequenceNum();
            if (frames_dictionary.find(seqNum) != frames_dictionary.end() && !haveSyncedFrames)
            {
                synchedDepthFrame = depthFromQueue;
                synchedNnFrame = frames_dictionary[seqNum].nn;
                haveSyncedFrames = true;
                break;
            }
            else
            {
                frames_dictionary[seqNum].depth = depthFromQueue;
            }
        }
    }
    //Now we remove all sequence numbers that are less than or equal the sequence number
    for (auto it = frames_dictionary.cbegin(), next_it = it; it != frames_dictionary.cend(); it = next_it)
    {
        ++next_it;
        if (it->first <= seqNum)
        {
            frames_dictionary.erase(it);
        }
    }

        auto frameDepthMat = synchedDepthFrame->getFrame();
        cv::resize(frameDepthMat, frameDepthMat, cv::Size(1280, 720), 0, 0, cv::INTER_NEAREST);


//   // TODO FIXME x big/little endian hazard ~

//   //Color frame
// //   std::shared_ptr<FrameStruct> rgbFrame =
// //       std::shared_ptr<FrameStruct>(new FrameStruct(frame_template_));
// //     rgbFrame->sensor_id = 0; 
// //   rgbFrame->frame_type = FrameType::FrameTypeColor; // 0;
// //   rgbFrame->frame_data_type = FrameDataType::FrameDataTypeCvMat; // 9;
// //   rgbFrame->frame_id = current_frame_counter_;
// //   rgbFrame->timestamps.push_back(capture_timestamp);

//    // convert the raw buffer to cv::Mat
// //    int32_t colorCols = frameRgbOpenCv.cols;                                                        // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT
// //    int32_t colorRows = frameRgbOpenCv.rows;                                                        // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT
// //    size_t colorSize = colorCols*colorRows*3*sizeof(uchar); //This assumes that oakd color always returns CV_8UC3// UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT

// //    rgbFrame->frame.resize(colorSize + 2 * sizeof(int32_t));                                        // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT

// //    memcpy(&rgbFrame->frame[0], &colorCols, sizeof(int32_t));                                       // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT
// //    memcpy(&rgbFrame->frame[4], &colorRows, sizeof(int32_t));                                       // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT
// //    memcpy(&rgbFrame->frame[8], (unsigned char*)(frameRgbOpenCv.data), colorSize);              // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT

//     std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;

// //   Depth frame
//     std::shared_ptr<FrameStruct> depthFrame =
//         std::shared_ptr<FrameStruct>(new FrameStruct(frame_template_));
//         depthFrame->sensor_id = 1;
//     depthFrame->frame_type = FrameType::FrameTypeDepth; // 1;
//     depthFrame->frame_data_type = FrameDataType::FrameDataTypeDepthAIStereoDepth; // 11; It is not when not subpixel
//     depthFrame->frame_id = current_frame_counter_;
//     depthFrame->timestamps.push_back(capture_timestamp);


//     int32_t depthCols = frameDepthMat.cols;                                                        // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT
//     int32_t depthRows = frameDepthMat.rows;                                                        // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT
//     size_t depthSize = depthCols*depthRows*sizeof(uint16_t); //  DepthAI StereoDepth outputs ImgFrame message that carries RAW16 encoded (0..65535) depth data in millimeters.

//     depthFrame->frame.resize(depthSize + 2 * sizeof(int32_t));                                        // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT

//     memcpy(&depthFrame->frame[0], &depthCols, sizeof(int32_t));                                       // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT
//     memcpy(&depthFrame->frame[4], &depthRows, sizeof(int32_t));                                       // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT
//     memcpy(&depthFrame->frame[8], (unsigned char*)(frameDepthMat.data), depthSize);              // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT

//     current_frame_.push_back(depthFrame);

    try {
    /////////////////////////////////////////////
    // HERE WE TAKE THE PARSE_POSES RESULT AND CREATE BODYSTRUCTS

    int32_t bodyCount = (int)posesStruct.poses_3d.size();
    // If 0, no need to send structs
    if (bodyCount > 0)
    {
        return;
    }

    //Prepare the bodies frame
    std::shared_ptr<FrameStruct> s =
        std::shared_ptr<FrameStruct>(new FrameStruct(frame_template_));
    s->sensor_id = 4;
    s->frame_id = current_frame_counter_;
    s->frame_type = FrameType::FrameTypeHumanPose; // 4;
    s->frame_data_type = FrameDataType::FrameDataTypeObjectHumanData; // 8;
    s->timestamps.push_back(capture_timestamp);

    s->frame = std::vector<uchar>();

    std::cerr << "bodyCount: " << bodyCount << std::endl << std::flush; 

    //Resize the frame to hold all the detected COCO bodies detected in this frame
    s->frame.resize(sizeof(coco_human_t)*bodyCount + sizeof(int32_t));

    //Copy the number of COCO bodies detected into the first 4 bytes of the bodies frame
    auto nBodyCount = bodyCount;
    inplace_hton(nBodyCount);  
    memcpy(&s->frame[0], &nBodyCount, sizeof(int32_t));

    //Now we iterate through all detected bodies in poses_3d, create a coco_human_t, and then copy the data into the frame
    for (size_t i = 0; i < bodyCount; i++) {

        //Create a COCO body Struct
        coco_human_t bodyStruct;
        bodyStruct.Id = posesStruct.poses_id[i];
        std::cerr << "BODY: " << current_frame_counter_ << " # " <<  i << " ... " << bodyStruct.Id << std::endl << std::flush;
        // Map of joints to array index
        // neck 0 
        // nose 1 
        // pelvis (can't use it!) 2 
        // left shoulder 3 
        // left elbow 4 
        // left wrist 5 
        // left hip 6 
        // left knee 7 
        // left ankle 8 
        // right shoulder 9 
        // right elbow 10 
        // right wrist 11 
        // right hip 12 
        // right knee 13 
        // right ankle 14 
        // left eye 15
        // left ear 16
        // right eye 17
        // right ear 18

        //Now we set the data of the COCO body struct
        // posesStruct.poses_3d [ {body number } ][ {body joint index}*4 + {0, 1, 2 ,3 for x, y, z, probability}]
        auto zCorrection = [](float x) -> float {
            return x; // / 0.84381;
        };

        bodyStruct.neck_x = posesStruct.poses_3d[i][0 * 4 + 0];
        bodyStruct.neck_y = posesStruct.poses_3d[i][0 * 4 + 1];
        bodyStruct.neck_z = zCorrection(posesStruct.poses_3d[i][0 * 4 + 2]);
        bodyStruct.neck_conf = posesStruct.poses_3d[i][0 * 4 + 3];
        bodyStruct.nose_x = posesStruct.poses_3d[i][1 * 4 + 0];
        bodyStruct.nose_y = posesStruct.poses_3d[i][1 * 4 + 1];
        bodyStruct.nose_z = zCorrection(posesStruct.poses_3d[i][1 * 4 + 2]);
        bodyStruct.nose_conf = posesStruct.poses_3d[i][1 * 4 + 3];
        bodyStruct.pelvis_x = posesStruct.poses_3d[i][2 * 4 + 0];
        bodyStruct.pelvis_y = posesStruct.poses_3d[i][2 * 4 + 1];
        bodyStruct.pelvis_z = zCorrection(posesStruct.poses_3d[i][2 * 4 + 2]);
        bodyStruct.pelvis_conf = posesStruct.poses_3d[i][2 * 4 + 3];
        bodyStruct.shoulder_left_x = posesStruct.poses_3d[i][3 * 4 + 0];
        bodyStruct.shoulder_left_y = posesStruct.poses_3d[i][3 * 4 + 1];
        bodyStruct.shoulder_left_z = zCorrection(posesStruct.poses_3d[i][3 * 4 + 2]);
        bodyStruct.shoulder_left_conf = posesStruct.poses_3d[i][3 * 4 + 3];
        bodyStruct.elbow_left_x = posesStruct.poses_3d[i][4 * 4 + 0];
        bodyStruct.elbow_left_y = posesStruct.poses_3d[i][4 * 4 + 1];
        bodyStruct.elbow_left_z = zCorrection(posesStruct.poses_3d[i][4 * 4 + 2]);
        bodyStruct.elbow_left_conf = posesStruct.poses_3d[i][4 * 4 + 3];
        bodyStruct.wrist_left_x = posesStruct.poses_3d[i][5 * 4 + 0];
        bodyStruct.wrist_left_y = posesStruct.poses_3d[i][5 * 4 + 1];
        bodyStruct.wrist_left_z = zCorrection(posesStruct.poses_3d[i][5 * 4 + 2]);
        bodyStruct.wrist_left_conf = posesStruct.poses_3d[i][5 * 4 + 3];
        bodyStruct.hip_left_x = posesStruct.poses_3d[i][6 * 4 + 0];
        bodyStruct.hip_left_y = posesStruct.poses_3d[i][6 * 4 + 1];
        bodyStruct.hip_left_z = zCorrection(posesStruct.poses_3d[i][6 * 4 + 2]);
        bodyStruct.hip_left_conf = posesStruct.poses_3d[i][6 * 4 + 3];
        bodyStruct.knee_left_x = posesStruct.poses_3d[i][7 * 4 + 0];
        bodyStruct.knee_left_y = posesStruct.poses_3d[i][7 * 4 + 1];
        bodyStruct.knee_left_z = zCorrection(posesStruct.poses_3d[i][7 * 4 + 2]);
        bodyStruct.knee_left_conf = posesStruct.poses_3d[i][7 * 4 + 3];
        bodyStruct.ankle_left_x = posesStruct.poses_3d[i][8 * 4 + 0];
        bodyStruct.ankle_left_y = posesStruct.poses_3d[i][8 * 4 + 1];
        bodyStruct.ankle_left_z = zCorrection(posesStruct.poses_3d[i][8 * 4 + 2]);
        bodyStruct.ankle_left_conf = posesStruct.poses_3d[i][8 * 4 + 3];
        bodyStruct.shoulder_right_x = posesStruct.poses_3d[i][9 * 4 + 0];
        bodyStruct.shoulder_right_y = posesStruct.poses_3d[i][9 * 4 + 1];
        bodyStruct.shoulder_right_z = zCorrection(posesStruct.poses_3d[i][9 * 4 + 2]);
        bodyStruct.shoulder_right_conf = posesStruct.poses_3d[i][9 * 4 + 3];
        bodyStruct.elbow_right_x = posesStruct.poses_3d[i][10 * 4 + 0];
        bodyStruct.elbow_right_y = posesStruct.poses_3d[i][10 * 4 + 1];
        bodyStruct.elbow_right_z = zCorrection(posesStruct.poses_3d[i][10 * 4 + 2]);
        bodyStruct.elbow_right_conf = posesStruct.poses_3d[i][10 * 4 + 3];
        bodyStruct.wrist_right_x = posesStruct.poses_3d[i][11 * 4 + 0];
        bodyStruct.wrist_right_y = posesStruct.poses_3d[i][11 * 4 + 1];
        bodyStruct.wrist_right_z = zCorrection(posesStruct.poses_3d[i][11 * 4 + 2]);
        bodyStruct.wrist_right_conf = posesStruct.poses_3d[i][11 * 4 + 3];
        bodyStruct.hip_right_x = posesStruct.poses_3d[i][12 * 4 + 0];
        bodyStruct.hip_right_y = posesStruct.poses_3d[i][12 * 4 + 1];
        bodyStruct.hip_right_z = zCorrection(posesStruct.poses_3d[i][12 * 4 + 2]);
        bodyStruct.hip_right_conf = posesStruct.poses_3d[i][12 * 4 + 3];
        bodyStruct.knee_right_x = posesStruct.poses_3d[i][13 * 4 + 0];
        bodyStruct.knee_right_y = posesStruct.poses_3d[i][13 * 4 + 1];
        bodyStruct.knee_right_z = zCorrection(posesStruct.poses_3d[i][13 * 4 + 2]);
        bodyStruct.knee_right_conf = posesStruct.poses_3d[i][13 * 4 + 3];
        bodyStruct.ankle_right_x = posesStruct.poses_3d[i][14 * 4 + 0];
        bodyStruct.ankle_right_y = posesStruct.poses_3d[i][14 * 4 + 1];
        bodyStruct.ankle_right_z = zCorrection(posesStruct.poses_3d[i][14 * 4 + 2]);
        bodyStruct.ankle_right_conf = posesStruct.poses_3d[i][14 * 4 + 3];
        bodyStruct.eye_left_x = posesStruct.poses_3d[i][15 * 4 + 0];
        bodyStruct.eye_left_y = posesStruct.poses_3d[i][15 * 4 + 1];
        bodyStruct.eye_left_z = zCorrection(posesStruct.poses_3d[i][15 * 4 + 2]);
        bodyStruct.eye_left_conf = posesStruct.poses_3d[i][15 * 4 + 3];
        bodyStruct.ear_left_x = posesStruct.poses_3d[i][16 * 4 + 0];
        bodyStruct.ear_left_y = posesStruct.poses_3d[i][16 * 4 + 1];
        bodyStruct.ear_left_z = zCorrection(posesStruct.poses_3d[i][16 * 4 + 2]);
        bodyStruct.ear_left_conf = posesStruct.poses_3d[i][16 * 4 + 3];
        bodyStruct.eye_right_x = posesStruct.poses_3d[i][17 * 4 + 0];
        bodyStruct.eye_right_y = posesStruct.poses_3d[i][17 * 4 + 1];
        bodyStruct.eye_right_z = zCorrection(posesStruct.poses_3d[i][17 * 4 + 2]);
        bodyStruct.eye_right_conf = posesStruct.poses_3d[i][17 * 4 + 3];
        bodyStruct.ear_right_x = posesStruct.poses_3d[i][18 * 4 + 0];
        bodyStruct.ear_right_y = posesStruct.poses_3d[i][18 * 4 + 1];
        bodyStruct.ear_right_z = zCorrection(posesStruct.poses_3d[i][18 * 4 + 2]);
        bodyStruct.ear_right_conf = posesStruct.poses_3d[i][18 * 4 + 3];

        auto to2D = [](float x) -> int16_t {
            // std::cerr << "to2D: value = " << x << std::endl << std::flush;
            return std::min(std::max(int64_t(0L), int64_t(x )), int64_t(1280L - 1L));
        };

        auto to2Dy = [](float x) -> int16_t {
            // std::cerr << "to2D/y: value = " << x << std::endl << std::flush;
            return std::min(std::max(int64_t(0L), int64_t(x )), int64_t(720L-1L)); //  *0.84381); /// 0.84381);
        };

        auto to2Dd = [](float x) -> int16_t {
            // std::cerr << "to2D/d: value = " << x << std::endl << std::flush;
            return int64_t(x); //  *0.84381); /// 0.84381);
        };

        bodyStruct.neck_2d_x = to2D(posesStruct.poses_2d[i][0 * 3 + 0]);
        bodyStruct.neck_2d_y = to2Dy(posesStruct.poses_2d[i][0 * 3 + 1]);
        bodyStruct.neck_2d_depth = to2Dd(frameDepthMat.at<ushort>(bodyStruct.neck_2d_y,bodyStruct.neck_2d_x));
        bodyStruct.neck_2d_conf = posesStruct.poses_2d[i][0 * 3 + 2];
        bodyStruct.nose_2d_x = to2D(posesStruct.poses_2d[i][1 * 3 + 0]);
        bodyStruct.nose_2d_y = to2Dy(posesStruct.poses_2d[i][1 * 3 + 1]);
        bodyStruct.nose_2d_depth = to2Dd(frameDepthMat.at<ushort>(bodyStruct.nose_2d_y,bodyStruct.nose_2d_x));
        bodyStruct.nose_2d_conf = posesStruct.poses_2d[i][1 * 3 + 2];
        bodyStruct.pelvis_2d_x = to2D(posesStruct.poses_2d[i][2 * 3 + 0]);
        bodyStruct.pelvis_2d_y = to2Dy(posesStruct.poses_2d[i][2 * 3 + 1]);
        bodyStruct.pelvis_2d_depth = to2Dd(frameDepthMat.at<ushort>(bodyStruct.pelvis_2d_y,bodyStruct.pelvis_2d_x));
        bodyStruct.pelvis_2d_conf = posesStruct.poses_2d[i][2 * 3 + 2];
        bodyStruct.shoulder_left_2d_x = to2D(posesStruct.poses_2d[i][3 * 3 + 0]);
        bodyStruct.shoulder_left_2d_y = to2Dy(posesStruct.poses_2d[i][3 * 3 + 1]);
        bodyStruct.shoulder_left_2d_depth = to2Dd(frameDepthMat.at<ushort>(bodyStruct.shoulder_left_2d_y,bodyStruct.shoulder_left_2d_x));
        bodyStruct.shoulder_left_2d_conf = posesStruct.poses_2d[i][3 * 3 + 2];
        bodyStruct.elbow_left_2d_x = to2D(posesStruct.poses_2d[i][4 * 3 + 0]);
        bodyStruct.elbow_left_2d_y = to2Dy(posesStruct.poses_2d[i][4 * 3 + 1]);
        bodyStruct.elbow_left_2d_depth = to2Dd(frameDepthMat.at<ushort>(bodyStruct.elbow_left_2d_y,bodyStruct.elbow_left_2d_x));
        bodyStruct.elbow_left_2d_conf = posesStruct.poses_2d[i][4 * 3 + 2];
        bodyStruct.wrist_left_2d_x = to2D(posesStruct.poses_2d[i][5 * 3 + 0]);
        bodyStruct.wrist_left_2d_y = to2Dy(posesStruct.poses_2d[i][5 * 3 + 1]);
        bodyStruct.wrist_left_2d_depth = to2Dd(frameDepthMat.at<ushort>(bodyStruct.wrist_left_2d_y,bodyStruct.wrist_left_2d_x));
        bodyStruct.wrist_left_2d_conf = posesStruct.poses_2d[i][5 * 3 + 2];
        bodyStruct.hip_left_2d_x = to2D(posesStruct.poses_2d[i][6 * 3 + 0]);
        bodyStruct.hip_left_2d_y = to2Dy(posesStruct.poses_2d[i][6 * 3 + 1]);
        bodyStruct.hip_left_2d_depth = to2Dd(frameDepthMat.at<ushort>(bodyStruct.hip_left_2d_y,bodyStruct.hip_left_2d_x));
        bodyStruct.hip_left_2d_conf = posesStruct.poses_2d[i][6 * 3 + 2];
        bodyStruct.knee_left_2d_x = to2D(posesStruct.poses_2d[i][7 * 3 + 0]);
        bodyStruct.knee_left_2d_y = to2Dy(posesStruct.poses_2d[i][7 * 3 + 1]);
        bodyStruct.knee_left_2d_depth = to2Dd(frameDepthMat.at<ushort>(bodyStruct.knee_left_2d_y,bodyStruct.knee_left_2d_x));
        bodyStruct.knee_left_2d_conf = posesStruct.poses_2d[i][7 * 3 + 2];
        bodyStruct.ankle_left_2d_x = to2D(posesStruct.poses_2d[i][8 * 3 + 0]);
        bodyStruct.ankle_left_2d_y = to2Dy(posesStruct.poses_2d[i][8 * 3 + 1]);
        bodyStruct.ankle_left_2d_depth = to2Dd(frameDepthMat.at<ushort>(bodyStruct.ankle_left_2d_y,bodyStruct.ankle_left_2d_x));
        bodyStruct.ankle_left_2d_conf = posesStruct.poses_2d[i][8 * 3 + 2];
        bodyStruct.shoulder_right_2d_x = to2D(posesStruct.poses_2d[i][9 * 3 + 0]);
        bodyStruct.shoulder_right_2d_y = to2Dy(posesStruct.poses_2d[i][9 * 3 + 1]);
        bodyStruct.shoulder_right_2d_depth = to2Dd(frameDepthMat.at<ushort>(bodyStruct.shoulder_right_2d_y,bodyStruct.shoulder_right_2d_x));
        bodyStruct.shoulder_right_2d_conf = posesStruct.poses_2d[i][9 * 3 + 2];
        bodyStruct.elbow_right_2d_x = to2D(posesStruct.poses_2d[i][10 * 3 + 0]);
        bodyStruct.elbow_right_2d_y = to2Dy(posesStruct.poses_2d[i][10 * 3 + 1]);
        bodyStruct.elbow_right_2d_depth = to2Dd(frameDepthMat.at<ushort>(bodyStruct.elbow_right_2d_y,bodyStruct.elbow_right_2d_x));
        bodyStruct.elbow_right_2d_conf = posesStruct.poses_2d[i][10 * 3 + 2];
        bodyStruct.wrist_right_2d_x = to2D(posesStruct.poses_2d[i][11 * 3 + 0]);
        bodyStruct.wrist_right_2d_y = to2Dy(posesStruct.poses_2d[i][11 * 3 + 1]);
        bodyStruct.wrist_right_2d_depth = to2Dd(frameDepthMat.at<ushort>(bodyStruct.wrist_right_2d_y,bodyStruct.wrist_right_2d_x));
        bodyStruct.wrist_right_2d_conf = posesStruct.poses_2d[i][11 * 3 + 2];
        bodyStruct.hip_right_2d_x = to2D(posesStruct.poses_2d[i][12 * 3 + 0]);
        bodyStruct.hip_right_2d_y = to2Dy(posesStruct.poses_2d[i][12 * 3 + 1]);
        bodyStruct.hip_right_2d_depth = to2Dd(frameDepthMat.at<ushort>(bodyStruct.hip_right_2d_y,bodyStruct.hip_right_2d_x));
        bodyStruct.hip_right_2d_conf = posesStruct.poses_2d[i][12 * 3 + 2];
        bodyStruct.knee_right_2d_x = to2D(posesStruct.poses_2d[i][13 * 3 + 0]);
        bodyStruct.knee_right_2d_y = to2Dy(posesStruct.poses_2d[i][13 * 3 + 1]);
        bodyStruct.knee_right_2d_depth = to2Dd(frameDepthMat.at<ushort>(bodyStruct.knee_right_2d_y,bodyStruct.knee_right_2d_x));
        bodyStruct.knee_right_2d_conf = posesStruct.poses_2d[i][13 * 3 + 2];
        bodyStruct.ankle_right_2d_x = to2D(posesStruct.poses_2d[i][14 * 3 + 0]);
        bodyStruct.ankle_right_2d_y = to2Dy(posesStruct.poses_2d[i][14 * 3 + 1]);
        bodyStruct.ankle_right_2d_depth = to2Dd(frameDepthMat.at<ushort>(bodyStruct.ankle_right_2d_y,bodyStruct.ankle_right_2d_x));
        bodyStruct.ankle_right_2d_conf = posesStruct.poses_2d[i][14 * 3 + 2];
        bodyStruct.eye_left_2d_x = to2D(posesStruct.poses_2d[i][15 * 3 + 0]);
        bodyStruct.eye_left_2d_y = to2Dy(posesStruct.poses_2d[i][15 * 3 + 1]);
        bodyStruct.eye_left_2d_depth = to2Dd(frameDepthMat.at<ushort>(bodyStruct.eye_left_2d_y,bodyStruct.eye_left_2d_x));
        bodyStruct.eye_left_2d_conf = posesStruct.poses_2d[i][15 * 3 + 2];
        bodyStruct.ear_left_2d_x = to2D(posesStruct.poses_2d[i][16 * 3 + 0]);
        bodyStruct.ear_left_2d_y = to2Dy(posesStruct.poses_2d[i][16 * 3 + 1]);
        bodyStruct.ear_left_2d_depth = to2Dd(frameDepthMat.at<ushort>(bodyStruct.ear_left_2d_y,bodyStruct.ear_left_2d_x));
        bodyStruct.ear_left_2d_conf = posesStruct.poses_2d[i][16 * 3 + 2];
        bodyStruct.eye_right_2d_x = to2D(posesStruct.poses_2d[i][17 * 3 + 0]);
        bodyStruct.eye_right_2d_y = to2Dy(posesStruct.poses_2d[i][17 * 3 + 1]);
        bodyStruct.eye_right_2d_depth = to2Dd(frameDepthMat.at<ushort>(bodyStruct.eye_right_2d_y,bodyStruct.eye_right_2d_x));
        bodyStruct.eye_right_2d_conf = posesStruct.poses_2d[i][17 * 3 + 2];
        bodyStruct.ear_right_2d_x = to2D(posesStruct.poses_2d[i][18 * 3 + 0]);
        bodyStruct.ear_right_2d_y = to2Dy(posesStruct.poses_2d[i][18 * 3 + 1]);
        bodyStruct.ear_right_2d_depth = to2Dd(frameDepthMat.at<ushort>(bodyStruct.ear_right_2d_y,bodyStruct.ear_right_2d_x));
        bodyStruct.ear_right_2d_conf = posesStruct.poses_2d[i][18 * 3 + 2];

        //Now we find the spatial coordinates of using StereoDepth and intrinsics
        //We first find the x,y coordinate in the image of the body that is most confident
        int xPoint;
        int yPoint;
        if(bodyStruct.neck_2d_conf > .5)
        {
            xPoint = bodyStruct.neck_2d_x;
            yPoint = bodyStruct.neck_2d_y;
        }
        //If neck not confident but shoulders are confident, choose half way point of shoulders
        else if (bodyStruct.shoulder_left_2d_conf > .5 && bodyStruct.shoulder_right_2d_conf > .5)
        {
            xPoint = (bodyStruct.shoulder_left_2d_x + bodyStruct.shoulder_right_2d_x)/2;
            yPoint = (bodyStruct.shoulder_left_2d_y + bodyStruct.shoulder_right_2d_y)/2;
        }
        //If that isn't true do midpoint between hips
        else if (bodyStruct.hip_left_2d_conf > .5 && bodyStruct.hip_right_2d_conf > .5)
        {
            xPoint = (bodyStruct.hip_left_2d_x + bodyStruct.hip_right_2d_x)/2;
            yPoint = (bodyStruct.hip_left_2d_y + bodyStruct.hip_right_2d_y)/2;
        }
        //If head, shoulders, hips aren't confident, then try nose
        else if (bodyStruct.nose_2d_conf > .5)
        {
            xPoint = bodyStruct.nose_2d_x;
            yPoint = bodyStruct.nose_2d_y;
        }
        //Finally, if none of those are confident, we will grab the average x/y location of all detected joints
        else
        {
            int countOfDetectedJoints = 0;
            int xValueOfDetectedJoints = 0;
            int yValueOfDetectedJoints = 0;

            if(bodyStruct.neck_2d_conf > 0)
            {
                countOfDetectedJoints++;
                xValueOfDetectedJoints += bodyStruct.neck_2d_x;
                yValueOfDetectedJoints += bodyStruct.neck_2d_y;
            }
            if(bodyStruct.nose_2d_conf > 0)
            {
                countOfDetectedJoints++;
                xValueOfDetectedJoints += bodyStruct.nose_2d_x;
                yValueOfDetectedJoints += bodyStruct.nose_2d_y;
            }
            // if(bodyStruct.pelvis_2d_conf > 0)
            // {
            //     countOfDetectedJoints++;
            //     xValueOfDetectedJoints += bodyStruct.pelvis_2d_x;
            //     yValueOfDetectedJoints += bodyStruct.pelvis_2d_y;
            // }
            if(bodyStruct.shoulder_left_2d_conf > 0)
            {
                countOfDetectedJoints++;
                xValueOfDetectedJoints += bodyStruct.shoulder_left_2d_x;
                yValueOfDetectedJoints += bodyStruct.shoulder_left_2d_y;
            }
            if(bodyStruct.elbow_left_2d_conf > 0)
            {
                countOfDetectedJoints++;
                xValueOfDetectedJoints += bodyStruct.elbow_left_2d_x;
                yValueOfDetectedJoints += bodyStruct.elbow_left_2d_y;
            }
            if(bodyStruct.wrist_left_2d_conf > 0)
            {
                countOfDetectedJoints++;
                xValueOfDetectedJoints += bodyStruct.wrist_left_2d_x;
                yValueOfDetectedJoints += bodyStruct.wrist_left_2d_y;
            }
            if(bodyStruct.hip_left_2d_conf > 0)
            {
                countOfDetectedJoints++;
                xValueOfDetectedJoints += bodyStruct.hip_left_2d_x;
                yValueOfDetectedJoints += bodyStruct.hip_left_2d_y;
            }
            if(bodyStruct.knee_left_2d_conf > 0)
            {
                countOfDetectedJoints++;
                xValueOfDetectedJoints += bodyStruct.knee_left_2d_x;
                yValueOfDetectedJoints += bodyStruct.knee_left_2d_y;
            }
            if(bodyStruct.ankle_left_2d_conf > 0)
            {
                countOfDetectedJoints++;
                xValueOfDetectedJoints += bodyStruct.ankle_left_2d_x;
                yValueOfDetectedJoints += bodyStruct.ankle_left_2d_y;
            }
            if(bodyStruct.shoulder_right_2d_conf > 0)
            {
                countOfDetectedJoints++;
                xValueOfDetectedJoints += bodyStruct.shoulder_right_2d_x;
                yValueOfDetectedJoints += bodyStruct.shoulder_right_2d_y;
            }
            if(bodyStruct.elbow_right_2d_conf > 0)
            {
                countOfDetectedJoints++;
                xValueOfDetectedJoints += bodyStruct.elbow_right_2d_x;
                yValueOfDetectedJoints += bodyStruct.elbow_right_2d_y;
            }
            if(bodyStruct.wrist_right_2d_conf > 0)
            {
                countOfDetectedJoints++;
                xValueOfDetectedJoints += bodyStruct.wrist_right_2d_x;
                yValueOfDetectedJoints += bodyStruct.wrist_right_2d_y;
            }
            if(bodyStruct.hip_right_2d_conf > 0)
            {
                countOfDetectedJoints++;
                xValueOfDetectedJoints += bodyStruct.hip_right_2d_x;
                yValueOfDetectedJoints += bodyStruct.hip_right_2d_y;
            }
            if(bodyStruct.knee_right_2d_conf > 0)
            {
                countOfDetectedJoints++;
                xValueOfDetectedJoints += bodyStruct.knee_right_2d_x;
                yValueOfDetectedJoints += bodyStruct.knee_right_2d_y;
            }
            if(bodyStruct.ankle_right_2d_conf > 0)
            {
                countOfDetectedJoints++;
                xValueOfDetectedJoints += bodyStruct.ankle_right_2d_x;
                yValueOfDetectedJoints += bodyStruct.ankle_right_2d_y;
            }
            if(bodyStruct.eye_left_2d_conf > 0)
            {
                countOfDetectedJoints++;
                xValueOfDetectedJoints += bodyStruct.eye_left_2d_x;
                yValueOfDetectedJoints += bodyStruct.eye_left_2d_y;
            }
            if(bodyStruct.ear_left_2d_conf > 0)
            {
                countOfDetectedJoints++;
                xValueOfDetectedJoints += bodyStruct.ear_left_2d_x;
                yValueOfDetectedJoints += bodyStruct.ear_left_2d_y;
            }
            if(bodyStruct.eye_right_2d_conf > 0)
            {
                countOfDetectedJoints++;
                xValueOfDetectedJoints += bodyStruct.eye_right_2d_x;
                yValueOfDetectedJoints += bodyStruct.eye_right_2d_y;
            }
            if(bodyStruct.ear_right_2d_conf > 0)
            {
                countOfDetectedJoints++;
                xValueOfDetectedJoints += bodyStruct.ear_right_2d_x;
                yValueOfDetectedJoints += bodyStruct.ear_right_2d_y;
            }

            xPoint = xValueOfDetectedJoints/countOfDetectedJoints;
            yPoint = yValueOfDetectedJoints/countOfDetectedJoints;
        }

        //Now we grab a the average depth value of a 6x6 grid of pixels surrounding the xPoint and yPoint
        cv::Scalar mean, stddev;
        //Region square radius
        int regionRadius = 3;

        //We grab a region of interest, we need to make sure it is not asking for pixels outside of the frame
        int xPointMin  = (xPoint - regionRadius >= 0) ? xPoint - regionRadius : 0;
        xPointMin  = (xPointMin <= frameDepthMat.cols) ? xPointMin : frameDepthMat.cols;
        int xPointMax  = (xPoint + regionRadius <= frameDepthMat.cols) ? xPoint + regionRadius : frameDepthMat.cols;
        xPointMax  = (xPointMax >= 0) ? xPointMax : 0;
        int yPointMin  = (yPoint - regionRadius >= 0) ? yPoint - regionRadius : 0;
        yPointMin  = (yPointMin <= frameDepthMat.rows) ? yPointMin : frameDepthMat.rows;
        int yPointMax   = (yPoint + regionRadius <= frameDepthMat.rows) ? yPoint + regionRadius : frameDepthMat.rows;
        yPointMax   = (yPointMax >= 0) ? yPointMax : 0;

        // std::cerr << "xPoint: " << xPoint << std::endl << std::flush;
        // std::cerr << "yPoint: " << yPoint << std::endl << std::flush;
        // std::cerr << "xPointMin: " << xPointMin << std::endl << std::flush;
        // std::cerr << "xPointMax: " << xPointMax << std::endl << std::flush;
        // std::cerr << "yPointMin: " << yPointMin << std::endl << std::flush;
        // std::cerr << "yPointMax: " << yPointMax << std::endl << std::flush;

        //We grab a reference to the cropped image and calculate the mean, and then store it as pointDepth
        cv::Rect myROI(cv::Point(xPointMin, yPointMin), cv::Point(xPointMax , yPointMax ));
        cv::Mat croppedDepth = frameDepthMat(myROI);
        cv::meanStdDev(croppedDepth, mean, stddev);
        int pointDepth = int(mean[0]);

        //Now we use the HFOV and VFOV to find the x and y coordinates in millimeters
        // https://github.com/luxonis/depthai-experiments/blob/377c50c13931a082825d457f69893c1bf3f24aa2/gen2-calc-spatials-on-host/calc.py#L10
        int midDepthXCoordinate = frameDepthMat.cols / 2;    // middle of depth image x across (columns) - this is depth origin
        int midDepthYCoordinate = frameDepthMat.rows / 2;    // middle of depth image y across (rows) - this is depth origin
        
        int xPointInDepthCenterCoordinates = xPoint - midDepthXCoordinate; //This is the xPoint but if the depth map center is the origin
        int yPointInDepthCenterCoordinates = yPoint - midDepthYCoordinate; //This is the yPoint but if the depth map center is the origin

        float angle_x = atan(tan(cameraHFOVInRadians / 2.0f) * float(xPointInDepthCenterCoordinates) / float(midDepthXCoordinate));
        float angle_y = atan(tan(cameraHFOVInRadians / 2.0f) * float(yPointInDepthCenterCoordinates) / float(midDepthYCoordinate));

        //We will save this depth value as the pelvis depth
        bodyStruct.pelvis_2d_depth = pointDepth;
        bodyStruct.pelvis_2d_x = int(float(pointDepth) * tan(angle_x));
        bodyStruct.pelvis_2d_y = int(-1.0f * float(pointDepth) * tan(angle_y));

        std::cerr << "NECK DEPTH: " << int(frameDepthMat.at<ushort>(bodyStruct.neck_2d_y,bodyStruct.neck_2d_x))  << std::endl << std::flush;
        std::cerr << "AVERAGE NECK DEPTH: " << pointDepth  << std::endl << std::flush; //UNCOMMENT
        //Finally we copy the COCO body struct memory to the frame
        bodyStruct.hton();
        
        memcpy(&s->frame[(i*sizeof(coco_human_t))+4], &bodyStruct, sizeof(coco_human_t));
    }

    //Now that we have copied all memory to the frame we can push it back
    if (bodyCount > 0)
    {
        current_frame_.push_back(s);
    }
    //We push the rgb frame to the back (this helps when running ssp_client_opencv)
    // current_frame_.push_back(rgbFrame);

    } catch(...) {
        return;
    }


}

bool OakdDeviceReader::HasNextFrame() { return true; }

void OakdDeviceReader::Reset() {}

std::vector<std::shared_ptr<FrameStruct>> OakdDeviceReader::GetCurrentFrame() {
  return current_frame_;
}

unsigned int OakdDeviceReader::GetFps() {
  return 30;
}

std::vector<FrameType> OakdDeviceReader::GetType() {
  std::vector<FrameType> types;

//   types.push_back(FrameType::FrameTypeColor);
//   types.push_back(FrameType::FrameTypeDepth);
  types.push_back(FrameType::FrameTypeHumanPose); // 4;

  return types;
}

void OakdDeviceReader::GoToFrame(unsigned int frame_id) {}
unsigned int OakdDeviceReader::GetCurrentFrameId() {return current_frame_counter_;}

}