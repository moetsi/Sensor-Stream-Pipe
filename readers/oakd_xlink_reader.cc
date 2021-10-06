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

OakdXlinkReader::OakdXlinkReader() {

    spdlog::debug("Starting to open");
    current_frame_counter_ = 0;

    frame_template_.sensor_id = 0;
    frame_template_.stream_id = RandomString(16);
    frame_template_.device_id = 0;
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

    // Connect to device and start pipeline
    device = std::make_shared<dai::Device>(pipeline);
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
    network = ie.ReadNetwork("../../models/human-pose-estimation-3d.xml");
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
  ClassificationResult_t classificationResult(output);
  classificationResult.print();

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
  // types.push_back(4)

  return types;
}

void OakdXlinkReader::GoToFrame(unsigned int frame_id) {}
unsigned int OakdXlinkReader::GetCurrentFrameId() {return current_frame_counter_;}
