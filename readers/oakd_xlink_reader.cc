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

OakdXlinkReader::OakdXlinkReader(YAML::Node config) {

    spdlog::debug("Starting to open");
    current_frame_counter_ = 0;

    frame_template_.sensor_id = 0;
    frame_template_.stream_id = RandomString(16);
    frame_template_.device_id = config["deviceid"].as<unsigned int>();
    frame_template_.scene_desc = "oakd";

    frame_template_.frame_id = 0;
    frame_template_.message_type = 0;


    // Define source and output
    camRgb = pipeline.create<dai::node::ColorCamera>();
    xoutRgb = pipeline.create<dai::node::XLinkOut>();

    xoutRgb->setStreamName("rgb");

    // Properties
    camRgb->setPreviewSize(300, 300);
    camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setInterleaved(false);
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::RGB);

    // Linking
    camRgb->preview.link(xoutRgb->input);

    // Changing the IP address to the correct depthai format (const char*)
    char chText[48];
    std::string ip_name = config["ip"].as<std::string>();
    ip_name.copy(chText, ip_name.size(), 0);
    chText[ip_name.size()] = '\0';
    
    //Which sensor
    device_info = dai::DeviceInfo();
    strcpy(device_info.desc.name, chText);
    device_info.state = X_LINK_BOOTLOADER;
    device_info.desc.protocol = X_LINK_TCP_IP;
    device = std::make_shared<dai::Device>(pipeline, device_info);

    // Connect to device and start pipeline
    cout << "Connected cameras: ";
    for(const auto& cam : device->getConnectedCameras()) {
        cout << static_cast<int>(cam) << " ";
        cout << cam << " ";
    }
    cout << endl;

    // Print USB speed
    cout << "Usb speed: " << device->getUsbSpeed() << endl;


    // Output queue will be used to get the rgb frames from the output defined above
    qRgb = device->getOutputQueue("rgb", 4, false);

    spdlog::debug("Done opening");

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
    network = ie.ReadNetwork("../../models/human-pose-estimation-3d.xml");
#endif
#ifdef _WIN32    
    network = ie.ReadNetwork("../../../models/human-pose-estimation-3d.xml");
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
    output_info = network.getOutputsInfo().begin()->second;
    output_name = network.getOutputsInfo().begin()->first;

    output_info->setPrecision(Precision::FP32);
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
  auto frameFromOakD = qRgb->get<dai::ImgFrame>();
  auto frameRgbOpenCv = frameFromOakD->getCvFrame();

  //Color frame
  std::shared_ptr<FrameStruct> rgbFrame =
      std::shared_ptr<FrameStruct>(new FrameStruct(frame_template_));
  rgbFrame->frame_type = 0;
  rgbFrame->frame_data_type = 9;
  rgbFrame->frame_id = current_frame_counter_;
  rgbFrame->timestamps.push_back(capture_timestamp);

  // convert the raw buffer to cv::Mat
  int cols = frameRgbOpenCv.cols;
  int rows = frameRgbOpenCv.rows;
  size_t size = cols*rows*3*sizeof(uchar); //This assumes that oakd always returns CV_8UC3

  rgbFrame->frame.resize(size + 2 * sizeof(int));

  memcpy(&rgbFrame->frame[0], &cols, sizeof(int));
  memcpy(&rgbFrame->frame[4], &rows, sizeof(int));
  memcpy(&rgbFrame->frame[8], (unsigned char*)(frameRgbOpenCv.data), size);

  current_frame_.push_back(rgbFrame);

  // --------------------------- Step 5. Create an infer request
  // -------------------------------------------------
  InferRequest infer_request = executable_network.CreateInferRequest();
  // -----------------------------------------------------------------------------------------------------

  // --------------------------- Step 6. Prepare input
  // --------------------------------------------------------
  /* Read input image to a blob and set it to an infer request without resize
    * and layout conversions. */
  // cv::Mat image = imread_t(input_image_path);
  Blob::Ptr imgBlob = wrapMat2Blob(frameRgbOpenCv);     // just wrap Mat data by Blob::Ptr
                                                // without allocating of new memory
  infer_request.SetBlob(input_name, imgBlob);  // infer_request accepts input blob of any size
  // -----------------------------------------------------------------------------------------------------

  // --------------------------- Step 7. Do inference
  // --------------------------------------------------------
  /* Running the request synchronously */
  infer_request.Infer();
  // -----------------------------------------------------------------------------------------------------

  // --------------------------- Step 8. Process output
  // ------------------------------------------------------
  Blob::Ptr output = infer_request.GetBlob(output_name);
  // Print classification results
  ClassificationResult_t classificationResult(output, {{input_image_path}});
  classificationResult.print();

  //TODO :: Change open vino model to Human Pose

  //TODO :: Take output of body detection inference and use utilities
  //        to convert to array of human keypoints

  //TODO :: Update the dummy creator below to change from a single body
  //        to as many bodies that have been detected, will need to make integer
  //        value dynamic to bodies, and add more arrays of bodies

  std::shared_ptr<FrameStruct> s =
        std::shared_ptr<FrameStruct>(new FrameStruct(frame_template_));
  s->frame_id = current_frame_counter_;
  s->frame_type = 4;
  s->frame_data_type = 8;
  s->timestamps.push_back(capture_timestamp);

  s->frame = std::vector<uchar>();
  s->frame.resize(sizeof(_object_human_t) + sizeof(int));
  
  //here we will say we detected 1 body
  // TODO :: Change bodyCount to number found from inference
  int bodyCount = 1;
  _object_human_t bodyStruct;

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


  memcpy(&s->frame[0], &bodyCount, sizeof(int));
  memcpy(&s->frame[4], &bodyStruct, sizeof(_object_human_t));


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

std::vector<unsigned int> OakdXlinkReader::GetType() {
  std::vector<unsigned int> types;

  types.push_back(0);
  // types.push_back(1);
  types.push_back(4);

  return types;
}

void OakdXlinkReader::GoToFrame(unsigned int frame_id) {}
unsigned int OakdXlinkReader::GetCurrentFrameId() {return current_frame_counter_;}
