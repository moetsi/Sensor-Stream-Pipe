//
// Created by adammpolak on 26-09-2021.
//

#include "oakd_xlink_reader.h"

// Closer-in minimum depth, disparity range is doubled (from 95 to 190):
// static std::atomic<bool> extended_disparity{false};
// // Better accuracy for longer distance, fractional disparity 32-levels:
// static std::atomic<bool> subpixel{false};
// // Better handling for occlusions:
// static std::atomic<bool> lr_check{false};
using namespace std;
using namespace InferenceEngine;

namespace moetsi::ssp {

OakdXlinkReader::OakdXlinkReader(YAML::Node config) {

std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
    spdlog::debug("Starting to open");
std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
    current_frame_counter_ = 0;

    frame_template_.sensor_id = 0;
    frame_template_.stream_id = RandomString(16);
    frame_template_.device_id = config["deviceid"].as<unsigned int>();
    frame_template_.scene_desc = "oakd";

    frame_template_.frame_id = 0;
    frame_template_.message_type = SSPMessageType::MessageTypeDefault; // 0;

std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
    // Define source and output
    camRgb = pipeline.create<dai::node::ColorCamera>();
    xoutRgb = pipeline.create<dai::node::XLinkOut>();
std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
    xoutRgb->setStreamName("rgb");

    // Properties
    camRgb->setPreviewSize(300, 300);
    camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setInterleaved(false);
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::RGB);
std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
    // Linking
    camRgb->preview.link(xoutRgb->input);
std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
    // Changing the IP address to the correct depthai format (const char*)
    char chText[48];
    std::string ip_name = config["ip"].as<std::string>();
    ip_name.copy(chText, ip_name.size(), 0);
    chText[ip_name.size()] = '\0';
  std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
    //Which sensor
    device_info = dai::DeviceInfo();
    strcpy(device_info.desc.name, chText);
    device_info.state = X_LINK_BOOTLOADER;
    device_info.desc.protocol = X_LINK_TCP_IP;
std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;    
    // device = std::make_shared<dai::Device>(pipeline, device_info, true); // usb 2 mode   // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT
std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
    // Connect to device and start pipeline
    cout << "Connected cameras: ";
// std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;                     // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT
//     for(const auto& cam : device->getConnectedCameras()) {
//         cout << static_cast<int>(cam) << " ";
//         cout << cam << " ";
//     }
// cout << endl;
std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
    // Print USB speed
    // cout << "Usb speed: " << device->getUsbSpeed() << endl;                              // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT
std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;

    // Output queue will be used to get the rgb frames from the output defined above
    // qRgb = device->getOutputQueue("rgb", 4, false);                                      // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT
std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
    spdlog::debug("Done opening");
std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
    //Now setting up Body model
    // input_model = {"../models/human-pose-estimation-3d.xml"};
    // input_image_path = "../openvino/fart";
    // device_name = simpleConvert("CPU");
    //Generating bodies
    // -----------------------------------------------------------------------------------------------------

    // --------------------------- Step 1. Initialize inference engine core
    // -------------------------------------
    // Core ie;
    // -----------------------------------------------------------------------------------------------------

    // Step 2. Read a model in OpenVINO Intermediate Representation (.xml and
    // .bin files) or ONNX (.onnx file) format
#ifndef _WIN32    
    network = ie.ReadNetwork("../../models/human-pose-estimation-3d-0001.xml");
#endif
#ifdef _WIN32    
    network = ie.ReadNetwork("../../../models/human-pose-estimation-3d-0001.xml");
#endif
    // if (network.getOutputsInfo().size() != 1)
    //     throw std::logic_error("Sample supports topologies with 1 output only");
    if (network.getInputsInfo().size() != 1)
        throw std::logic_error("Sample supports topologies with 1 input only");
    // -----------------------------------------------------------------------------------------------------

    // --------------------------- Step 3. Configure input & output
    // ---------------------------------------------
    // --------------------------- Prepare input blobs
    // -----------------------------------------------------
    input_info = network.getInputsInfo().begin()->second;
    input_name = network.getInputsInfo().begin()->first;

    /* Mark input as resizable by setting of a resize algorithm.
      * In this case we will be able to set an input blob of any shape to an
      * infer request. Resize and layout conversions are executed automatically
      * during inference */
    input_info->getPreProcess().setResizeAlgorithm(RESIZE_BILINEAR);
    input_info->setLayout(Layout::NHWC);
    input_info->setPrecision(Precision::U8);
    // --------------------------- Prepare output blobs
    // ----------------------------------------------------
    if (network.getOutputsInfo().empty()) {
        std::cerr << "Network outputs info is empty" << std::endl;
        return;
    }
    features_output_info = network.getOutputsInfo()["features"];
    heatmaps_output_info = network.getOutputsInfo()["heatmaps"];
    pafs_output_info = network.getOutputsInfo()["pafs"];

    features_output_info->setPrecision(Precision::FP32);
    heatmaps_output_info->setPrecision(Precision::FP32);
    pafs_output_info->setPrecision(Precision::FP32);
    // -----------------------------------------------------------------------------------------------------

    // --------------------------- Step 4. Loading a model to the device
    // ------------------------------------------
    executable_network = ie.LoadNetwork(network, "CPU");
    // -----------------------------------------------------------------------------------------------------

}

OakdXlinkReader::~OakdXlinkReader() {

}

void OakdXlinkReader::NextFrame() {
  current_frame_.clear();

  //Increment counter and grab time
  current_frame_counter_++;
  uint64_t capture_timestamp = CurrentTimeMs();

  //Color frame
//   auto frameFromOakD = qRgb->get<dai::ImgFrame>();                                       // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT
//   auto frameRgbOpenCv = frameFromOakD->getCvFrame();                                     // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT

  //Color frame
  std::shared_ptr<FrameStruct> rgbFrame =
      std::shared_ptr<FrameStruct>(new FrameStruct(frame_template_));
  rgbFrame->frame_type = FrameType::FrameTypeColor; // 0;
  rgbFrame->frame_data_type = FrameDataType::FrameDataTypeCvMat; // 9;
  rgbFrame->frame_id = current_frame_counter_;
  rgbFrame->timestamps.push_back(capture_timestamp);

  // convert the raw buffer to cv::Mat
//   int cols = frameRgbOpenCv.cols;                                                        // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT
//   int rows = frameRgbOpenCv.rows;                                                        // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT
//   size_t size = cols*rows*3*sizeof(uchar); //This assumes that oakd always returns CV_8UC3// UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT

//   rgbFrame->frame.resize(size + 2 * sizeof(int));                                        // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT

//   memcpy(&rgbFrame->frame[0], &cols, sizeof(int));                                       // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT
//   memcpy(&rgbFrame->frame[4], &rows, sizeof(int));                                       // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT
//   memcpy(&rgbFrame->frame[8], (unsigned char*)(frameRgbOpenCv.data), size);              // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT

  current_frame_.push_back(rgbFrame);


            


  // --------------------------- Step 5. Create an infer request
  // -------------------------------------------------
  InferRequest infer_request = executable_network.CreateInferRequest();
  // -----------------------------------------------------------------------------------------------------

  // --------------------------- Step 6. Prepare input
  // --------------------------------------------------------
  /* Read input image to a blob and set it to an infer request without resize
    * and layout conversions. */

    cv::Mat image = cv::imread("../../../models/pointing_close_of_view.jpg"); //This is a hardwired image only to help Renaud with parsing
    Blob::Ptr imgBlob = wrapMat2Blob(image);
    infer_request.SetBlob(input_name, imgBlob);  // infer_request accepts input blob of any size


//   Blob::Ptr imgBlob = wrapMat2Blob(frameRgbOpenCv);     // just wrap Mat data by Blob::Ptr// UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT
  // -----------------------------------------------------------------------------------------------------

  // --------------------------- Step 7. Do inference
  // --------------------------------------------------------

  /* Running the request synchronously */
    infer_request.Infer();
  // -----------------------------------------------------------------------------------------------------

  // --------------------------- Step 8. Process output
  // ------------------------------------------------------

    Blob::Ptr features_output = infer_request.GetBlob("features");
    Blob::Ptr heatmaps_output = infer_request.GetBlob("heatmaps");
    Blob::Ptr pafs_output = infer_request.GetBlob("pafs");

    const SizeVector features_output_shape = features_output_info->getTensorDesc().getDims();
    const SizeVector heatmaps_output_shape = heatmaps_output_info->getTensorDesc().getDims();
    const SizeVector pafs_output_shape = pafs_output_info->getTensorDesc().getDims();

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
    std::cerr << "Resulting output shape dumpVec(features_output_shape) = " << dumpVec(features_output_shape) << std::endl << std::flush;
    std::cerr << "Resulting output shape dumpVec(heatmaps_output_shape) = " << dumpVec(heatmaps_output_shape) << std::endl << std::flush;
    std::cerr << "Resulting output shape dumpVec(pafs_output_shape) = " << dumpVec(pafs_output_shape) << std::endl << std::flush;

    InferenceEngine::MemoryBlob::CPtr features_moutput = InferenceEngine::as<InferenceEngine::MemoryBlob>(features_output);
    InferenceEngine::MemoryBlob::CPtr heatmaps_moutput = InferenceEngine::as<InferenceEngine::MemoryBlob>(heatmaps_output);
    InferenceEngine::MemoryBlob::CPtr pafs_moutput = InferenceEngine::as<InferenceEngine::MemoryBlob>(pafs_output);

    InferenceEngine::LockedMemory<const void> features_outputMapped = features_moutput->rmap();
    InferenceEngine::LockedMemory<const void> heatmaps_outputMapped = heatmaps_moutput->rmap();
    InferenceEngine::LockedMemory<const void> pafs_outputMapped = pafs_moutput->rmap();

    // --------------------------- RENAUD TO TAKE THE FEATURES, HEATMAPS, AND PAFS, AND CHECK THAT THE MULTI-DIMENSIONAL
    // --------------------------- ARRAYS ARE THE SAME AS BEING OUTPUT BY THE PYTHON DEMO 

    const float *features_result = features_outputMapped.as<float *>();
    const float *heatmaps_result = heatmaps_outputMapped.as<float *>();
    const float *pafs_result = pafs_outputMapped.as<float *>();

    std::cerr << "features_result[0] = " << features_result[0] << std::endl << std::flush;
    std::cerr << "heatmaps_result[0] = " << heatmaps_result[0] << std::endl << std::flush;
    std::cerr << "pafs_result[0] = " << pafs_result[0] << std::endl << std::flush;


  std::shared_ptr<FrameStruct> s =
        std::shared_ptr<FrameStruct>(new FrameStruct(frame_template_));
  s->frame_id = current_frame_counter_;
  s->frame_type = FrameType::FrameTypeHumanPose; // 4;
  s->frame_data_type = FrameDataType::FrameDataTypeObjectHumanData; // 8;
  s->timestamps.push_back(capture_timestamp);

  s->frame = std::vector<uchar>();
  s->frame.resize(sizeof(object_human_t) + sizeof(int32_t));
  
  //here we will say we detected 1 body
  // TODO :: Change bodyCount to number found from inference
  int32_t bodyCount = 1;
  object_human_t bodyStruct;

  bodyStruct.Id = 1;
  bodyStruct.pelvis_x = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
  // bodyStruct.pelvis_x = 1.4;
  bodyStruct.pelvis_y = 0.1;
  bodyStruct.pelvis_z = 0.1;
  bodyStruct.pelvis_QX = 0.1;
  bodyStruct.pelvis_QY = 0.1;
  bodyStruct.pelvis_QZ = 0.1;
  bodyStruct.pelvis_QW = 0.1;
  bodyStruct.pelvis_conf = 1;
  bodyStruct.spine_navel_x = 0.1;
  bodyStruct.spine_navel_y = 0.1;
  bodyStruct.spine_navel_z = 0.1;
  bodyStruct.spine_navel_QX = 0.1;
  bodyStruct.spine_navel_QY = 0.1;
  bodyStruct.spine_navel_QZ = 0.1;
  bodyStruct.spine_navel_QW = 0.1;
  bodyStruct.spine_navel_conf = 1;
  bodyStruct.spine_chest_x = 0.1;
  bodyStruct.spine_chest_y = 0.1;
  bodyStruct.spine_chest_z = 0.1;
  bodyStruct.spine_chest_QX = 0.1;
  bodyStruct.spine_chest_QY = 0.1;
  bodyStruct.spine_chest_QZ = 0.1;
  bodyStruct.spine_chest_QW = 0.1;
  bodyStruct.spine_chest_conf = 1;
  bodyStruct.neck_x = 0.1;
  bodyStruct.neck_y = 0.1;
  bodyStruct.neck_z = 0.1;
  bodyStruct.neck_QX = 0.1;
  bodyStruct.neck_QY = 0.1;
  bodyStruct.neck_QZ = 0.1;
  bodyStruct.neck_QW = 0.1;
  bodyStruct.neck_conf = 1;
  bodyStruct.clavicle_left_x = 0.1;
  bodyStruct.clavicle_left_y = 0.1;
  bodyStruct.clavicle_left_z = 0.1;
  bodyStruct.clavicle_left_QX = 0.1;
  bodyStruct.clavicle_left_QY = 0.1;
  bodyStruct.clavicle_left_QZ = 0.1;
  bodyStruct.clavicle_left_QW = 0.1;
  bodyStruct.clavicle_left_conf = 1;
  bodyStruct.shoulder_left_x = 0.1;
  bodyStruct.shoulder_left_y = 0.1;
  bodyStruct.shoulder_left_z = 0.1;
  bodyStruct.shoulder_left_QX = 0.1;
  bodyStruct.shoulder_left_QY = 0.1;
  bodyStruct.shoulder_left_QZ = 0.1;
  bodyStruct.shoulder_left_QW = 0.1;
  bodyStruct.shoulder_left_conf = 1;
  bodyStruct.elbow_left_x = 0.1;
  bodyStruct.elbow_left_y = 0.1;
  bodyStruct.elbow_left_z = 0.1;
  bodyStruct.elbow_left_QX = 0.1;
  bodyStruct.elbow_left_QY = 0.1;
  bodyStruct.elbow_left_QZ = 0.1;
  bodyStruct.elbow_left_QW = 0.1;
  bodyStruct.elbow_left_conf = 1;
  bodyStruct.wrist_left_x = 0.1;
  bodyStruct.wrist_left_y = 0.1;
  bodyStruct.wrist_left_z = 0.1;
  bodyStruct.wrist_left_QX = 0.1;
  bodyStruct.wrist_left_QY = 0.1;
  bodyStruct.wrist_left_QZ = 0.1;
  bodyStruct.wrist_left_QW = 0.1;
  bodyStruct.wrist_left_conf = 1;
  bodyStruct.hand_left_x = 0.1;
  bodyStruct.hand_left_y = 0.1;
  bodyStruct.hand_left_z = 0.1;
  bodyStruct.hand_left_QX = 0.1;
  bodyStruct.hand_left_QY = 0.1;
  bodyStruct.hand_left_QZ = 0.1;
  bodyStruct.hand_left_QW = 0.1;
  bodyStruct.hand_left_conf = 1;
  bodyStruct.handtip_left_x = 0.1;
  bodyStruct.handtip_left_y = 0.1;
  bodyStruct.handtip_left_z = 0.1;
  bodyStruct.handtip_left_QX = 0.1;
  bodyStruct.handtip_left_QY = 0.1;
  bodyStruct.handtip_left_QZ = 0.1;
  bodyStruct.handtip_left_QW = 0.1;
  bodyStruct.handtip_left_conf = 1;
  bodyStruct.thumb_left_x = 0.1;
  bodyStruct.thumb_left_y = 0.1;
  bodyStruct.thumb_left_z = 0.1;
  bodyStruct.thumb_left_QX = 0.1;
  bodyStruct.thumb_left_QY = 0.1;
  bodyStruct.thumb_left_QZ = 0.1;
  bodyStruct.thumb_left_QW = 0.1;
  bodyStruct.thumb_left_conf = 1;
  bodyStruct.clavicle_right_x = 0.1;
  bodyStruct.clavicle_right_y = 0.1;
  bodyStruct.clavicle_right_z = 0.1;
  bodyStruct.clavicle_right_QX = 0.1;
  bodyStruct.clavicle_right_QY = 0.1;
  bodyStruct.clavicle_right_QZ = 0.1;
  bodyStruct.clavicle_right_QW = 0.1;
  bodyStruct.clavicle_right_conf = 1;
  bodyStruct.shoulder_right_x = 0.1;
  bodyStruct.shoulder_right_y = 0.1;
  bodyStruct.shoulder_right_z = 0.1;
  bodyStruct.shoulder_right_QX = 0.1;
  bodyStruct.shoulder_right_QY = 0.1;
  bodyStruct.shoulder_right_QZ = 0.1;
  bodyStruct.shoulder_right_QW = 0.1;
  bodyStruct.shoulder_right_conf = 1;
  bodyStruct.elbow_right_x = 0.1;
  bodyStruct.elbow_right_y = 0.1;
  bodyStruct.elbow_right_z = 0.1;
  bodyStruct.elbow_right_QX = 0.1;
  bodyStruct.elbow_right_QY = 0.1;
  bodyStruct.elbow_right_QZ = 0.1;
  bodyStruct.elbow_right_QW = 0.1;
  bodyStruct.elbow_right_conf = 1;
  bodyStruct.wrist_right_x = 0.1;
  bodyStruct.wrist_right_y = 0.1;
  bodyStruct.wrist_right_z = 0.1;
  bodyStruct.wrist_right_QX = 0.1;
  bodyStruct.wrist_right_QY = 0.1;
  bodyStruct.wrist_right_QZ = 0.1;
  bodyStruct.wrist_right_QW = 0.1;
  bodyStruct.wrist_right_conf = 1;
  bodyStruct.hand_right_x = 0.1;
  bodyStruct.hand_right_y = 0.1;
  bodyStruct.hand_right_z = 0.1;
  bodyStruct.hand_right_QX = 0.1;
  bodyStruct.hand_right_QY = 0.1;
  bodyStruct.hand_right_QZ = 0.1;
  bodyStruct.hand_right_QW = 0.1;
  bodyStruct.hand_right_conf = 1;
  bodyStruct.handtip_right_x = 0.1;
  bodyStruct.handtip_right_y = 0.1;
  bodyStruct.handtip_right_z = 0.1;
  bodyStruct.handtip_right_QX = 0.1;
  bodyStruct.handtip_right_QY = 0.1;
  bodyStruct.handtip_right_QZ = 0.1;
  bodyStruct.handtip_right_QW = 0.1;
  bodyStruct.handtip_right_conf = 1;
  bodyStruct.thumb_right_x = 0.1;
  bodyStruct.thumb_right_y = 0.1;
  bodyStruct.thumb_right_z = 0.1;
  bodyStruct.thumb_right_QX = 0.1;
  bodyStruct.thumb_right_QY = 0.1;
  bodyStruct.thumb_right_QZ = 0.1;
  bodyStruct.thumb_right_QW = 0.1;
  bodyStruct.thumb_right_conf = 1;
  bodyStruct.hip_left_x = 0.1;
  bodyStruct.hip_left_y = 0.1;
  bodyStruct.hip_left_z = 0.1;
  bodyStruct.hip_left_QX = 0.1;
  bodyStruct.hip_left_QY = 0.1;
  bodyStruct.hip_left_QZ = 0.1;
  bodyStruct.hip_left_QW = 0.1;
  bodyStruct.hip_left_conf = 1;
  bodyStruct.knee_left_x = 0.1;
  bodyStruct.knee_left_y = 0.1;
  bodyStruct.knee_left_z = 0.1;
  bodyStruct.knee_left_QX = 0.1;
  bodyStruct.knee_left_QY = 0.1;
  bodyStruct.knee_left_QZ = 0.1;
  bodyStruct.knee_left_QW = 0.1;
  bodyStruct.knee_left_conf = 1;
  bodyStruct.ankle_left_x = 0.1;
  bodyStruct.ankle_left_y = 0.1;
  bodyStruct.ankle_left_z = 0.1;
  bodyStruct.ankle_left_QX = 0.1;
  bodyStruct.ankle_left_QY = 0.1;
  bodyStruct.ankle_left_QZ = 0.1;
  bodyStruct.ankle_left_QW = 0.1;
  bodyStruct.ankle_left_conf = 1;
  bodyStruct.foot_left_x = 0.1;
  bodyStruct.foot_left_y = 0.1;
  bodyStruct.foot_left_z = 0.1;
  bodyStruct.foot_left_QX = 0.1;
  bodyStruct.foot_left_QY = 0.1;
  bodyStruct.foot_left_QZ = 0.1;
  bodyStruct.foot_left_QW = 0.1;
  bodyStruct.foot_left_conf = 1;
  bodyStruct.hip_right_x = 0.1;
  bodyStruct.hip_right_y = 0.1;
  bodyStruct.hip_right_z = 0.1;
  bodyStruct.hip_right_QX = 0.1;
  bodyStruct.hip_right_QY = 0.1;
  bodyStruct.hip_right_QZ = 0.1;
  bodyStruct.hip_right_QW = 0.1;
  bodyStruct.hip_right_conf = 1;
  bodyStruct.knee_right_x = 0.1;
  bodyStruct.knee_right_y = 0.1;
  bodyStruct.knee_right_z = 0.1;
  bodyStruct.knee_right_QX = 0.1;
  bodyStruct.knee_right_QY = 0.1;
  bodyStruct.knee_right_QZ = 0.1;
  bodyStruct.knee_right_QW = 0.1;
  bodyStruct.knee_right_conf = 1;
  bodyStruct.ankle_right_x = 0.1;
  bodyStruct.ankle_right_y = 0.1;
  bodyStruct.ankle_right_z = 0.1;
  bodyStruct.ankle_right_QX = 0.1;
  bodyStruct.ankle_right_QY = 0.1;
  bodyStruct.ankle_right_QZ = 0.1;
  bodyStruct.ankle_right_QW = 0.1;
  bodyStruct.ankle_right_conf = 1;
  bodyStruct.foot_right_x = 0.1;
  bodyStruct.foot_right_y = 0.1;
  bodyStruct.foot_right_z = 0.1;
  bodyStruct.foot_right_QX = 0.1;
  bodyStruct.foot_right_QY = 0.1;
  bodyStruct.foot_right_QZ = 0.1;
  bodyStruct.foot_right_QW = 0.1;
  bodyStruct.foot_right_conf = 1;
  bodyStruct.head_x = 0.1;
  bodyStruct.head_y = 0.1;
  bodyStruct.head_z = 0.1;
  bodyStruct.head_QX = 0.1;
  bodyStruct.head_QY = 0.1;
  bodyStruct.head_QZ = 0.1;
  bodyStruct.head_QW = 0.1;
  bodyStruct.head_conf = 1;
  bodyStruct.nose_x = 0.1;
  bodyStruct.nose_y = 0.1;
  bodyStruct.nose_z = 0.1;
  bodyStruct.nose_QX = 0.1;
  bodyStruct.nose_QY = 0.1;
  bodyStruct.nose_QZ = 0.1;
  bodyStruct.nose_QW = 0.1;
  bodyStruct.nose_conf = 1;
  bodyStruct.eye_left_x = 0.1;
  bodyStruct.eye_left_y = 0.1;
  bodyStruct.eye_left_z = 0.1;
  bodyStruct.eye_left_QX = 0.1;
  bodyStruct.eye_left_QY = 0.1;
  bodyStruct.eye_left_QZ = 0.1;
  bodyStruct.eye_left_QW = 0.1;
  bodyStruct.eye_left_conf = 1;
  bodyStruct.ear_left_x = 0.1;
  bodyStruct.ear_left_y = 0.1;
  bodyStruct.ear_left_z = 0.1;
  bodyStruct.ear_left_QX = 0.1;
  bodyStruct.ear_left_QY = 0.1;
  bodyStruct.ear_left_QZ = 0.1;
  bodyStruct.ear_left_QW = 0.1;
  bodyStruct.ear_left_conf = 1;
  bodyStruct.eye_right_x = 0.1;
  bodyStruct.eye_right_y = 0.1;
  bodyStruct.eye_right_z = 0.1;
  bodyStruct.eye_right_QX = 0.1;
  bodyStruct.eye_right_QY = 0.1;
  bodyStruct.eye_right_QZ = 0.1;
  bodyStruct.eye_right_QW = 0.1;
  bodyStruct.eye_right_conf = 1;
  bodyStruct.ear_right_x = 0.1;
  bodyStruct.ear_right_y = 0.1;
  bodyStruct.ear_right_z = 0.1;
  bodyStruct.ear_right_QX = 0.1;
  bodyStruct.ear_right_QY = 0.1;
  bodyStruct.ear_right_QZ = 0.1;
  bodyStruct.ear_right_QW = 0.1;
  bodyStruct.ear_right_conf = 1;
  inplace_hton(bodyCount);
  // std::cerr << "dummy: bodyCount after hton " << bodyCount << std::endl << std::flush;
  memcpy(&s->frame[0], &bodyCount, sizeof(int32_t));
  memcpy(&s->frame[4], &bodyStruct, sizeof(object_human_t));


  current_frame_.push_back(s);

}

bool OakdXlinkReader::HasNextFrame() { return true; }

void OakdXlinkReader::Reset() {}

std::vector<std::shared_ptr<FrameStruct>> OakdXlinkReader::GetCurrentFrame() {
  return current_frame_;
}

unsigned int OakdXlinkReader::GetFps() {
  return 10;
}

std::vector<FrameType> OakdXlinkReader::GetType() {
  std::vector<FrameType> types;

  types.push_back(FrameType::FrameTypeColor);
  // later => types.push_back(FrameType::FrameTypeDepth);
  types.push_back(FrameType::FrameTypeHumanPose); // 4;

  return types;
}

void OakdXlinkReader::GoToFrame(unsigned int frame_id) {}
unsigned int OakdXlinkReader::GetCurrentFrameId() {return current_frame_counter_;}

}