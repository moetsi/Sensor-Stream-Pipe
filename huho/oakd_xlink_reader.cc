//
// Created by adammpolak on 26-09-2021.
//

#include "oakd_xlink_reader.h"
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

namespace moetsi::ssp {
using namespace human_pose_estimation;

OakdXlinkReader::OakdXlinkReader(YAML::Node config) {

std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
    spdlog::debug("Starting to open");
std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
    
    fps = config["streaming_rate"].as<unsigned int>();
    
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
    left = pipeline.create<dai::node::MonoCamera>();
    right = pipeline.create<dai::node::MonoCamera>();
    stereo = pipeline.create<dai::node::StereoDepth>();

    rgbOut = pipeline.create<dai::node::XLinkOut>();
    depthOut = pipeline.create<dai::node::XLinkOut>();

    rgbOut->setStreamName("rgb");
    queueNames.push_back("rgb");
    depthOut->setStreamName("depth");
    queueNames.push_back("depth");

std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;

    // Color Properties
    camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setFps(fps);
    camRgb->setIspScale(2, 3); //this downscales from 1080p to 720p
    camRgb->initialControl.setManualFocus(135); // requires this focus to align depth
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::RGB);
    camRgb->setInterleaved(false);

    // Depth Properties
    auto monoRes = dai::MonoCameraProperties::SensorResolution::THE_400_P;
    left->setResolution(monoRes);
    left->setBoardSocket(dai::CameraBoardSocket::LEFT);
    left->setFps(fps);
    right->setResolution(monoRes);
    right->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    right->setFps(fps);

    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    // depth->setSubpixel(true);
    stereo->setLeftRightCheck(true); // LR-check is required for depth alignment
    stereo->setDepthAlign(dai::CameraBoardSocket::RGB);

std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;


    // Linking
    camRgb->isp.link(rgbOut->input);
    left->out.link(stereo->left);
    right->out.link(stereo->right);
    stereo->disparity.link(depthOut->input);

std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
    // Changing the IP address to the correct depthai format (const char*)
    char chText[48];
    std::string ip_name = config["ip"].as<std::string>();
    ip_name = StringInterpolation(ip_name);
    ip_name.copy(chText, ip_name.size(), 0);
    chText[ip_name.size()] = '\0';
std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
    //Which sensor
    device_info = dai::DeviceInfo();
    strcpy(device_info.desc.name, chText);
    device_info.state = X_LINK_BOOTLOADER;
    device_info.desc.protocol = X_LINK_TCP_IP;
std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;  

#ifndef TEST_WITH_IMAGE  
    device = std::make_shared<dai::Device>(pipeline, device_info, true); // usb 2 mode   // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT
#endif

std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
    // Connect to device and start pipeline
    cout << "Connected cameras: ";
#ifndef TEST_WITH_IMAGE
       std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;                     // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT
       for(const auto& cam : device->getConnectedCameras()) {
          cout << static_cast<int>(cam) << " ";
          cout << cam << " ";
      }
   cout << endl;
std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
    spdlog::debug(std::string(__FILE__) + ":" + std::to_string(__LINE__));
    // Print USB speed
    cout << "Usb speed: " << device->getUsbSpeed() << endl;                              // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT
    spdlog::debug(std::string(__FILE__) + ":" + std::to_string(__LINE__));
std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;

    // Output queue will be used to get the rgb frames from the output defined above
    // Sets queues size and behavior

    qRgb = device->getOutputQueue("rgb", 4, true); // false);                                      // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT
    qDepth = device->getOutputQueue("depth", 4, true); // false);    

std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
    spdlog::debug("Done opening");
std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
#endif
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
    std::string rel = "../../"; 
#endif
#ifdef _WIN32    
    std::string rel = "../../../";
#endif

    std::map<std::string,std::string> env;
    env["REL"] = rel;
    std::string model_path = config["model"].as<std::string>();
    model_path = StringInterpolation(env, model_path);
    std::cerr << "model_path = " << model_path << std::endl << std::flush;
    network = ie.ReadNetwork(model_path);
    //#ifndef _WIN32    
    //    network = ie.ReadNetwork("../../models/human-pose-estimation-3d-0001.xml");
    //#endif
    //#ifdef _WIN32    
    //    network = ie.ReadNetwork("../../../models/human-pose-estimation-3d-0001.xml");
    //#endif
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
    
    // TODO keep?
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

    // TODO needed?
    ICNNNetwork::InputShapes inputShape {{ "data", std::vector<size_t>{1,3,256,384} }};
    network.reshape(inputShape); 

    // -----------------------------------------------------------------------------------------------------

    // --------------------------- Step 4. Loading a model to the device
    // ------------------------------------------
    executable_network = ie.LoadNetwork(network, "CPU");
    // -----------------------------------------------------------------------------------------------------
    std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
}

OakdXlinkReader::~OakdXlinkReader() {

}

std::string type2str(int type) {
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

void OakdXlinkReader::NextFrame() {
  current_frame_.clear();

  std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
  //Increment counter and grab time
  current_frame_counter_++;
  uint64_t capture_timestamp = CurrentTimeMs();

#ifndef TEST_WITH_IMAGE
    
    //Here we try until we get a synchronized color and depth frame
    bool haveSyncedFrames = false;
    int seqNum;
    //std::shared_ptr<dai::ImgFrame> synchedRgbFrame;
    std::shared_ptr<FrameStruct> synchedRgbFrameStruct;
    //std::shared_ptr<dai::ImgFrame> synchedDepthFrame;
    std::shared_ptr<FrameStruct> synchedDepthFrameStruct;

cv::Mat frameRgbOpenCv ; // ugly!

    auto convertRgbFrame = [&](const std::shared_ptr<dai::ImgFrame> &synchedRgbFrame) -> std::shared_ptr<FrameStruct> {
        // auto  <- todo
        frameRgbOpenCv = synchedRgbFrame->getCvFrame();    
        std::shared_ptr<FrameStruct> rgbFrame =
            std::shared_ptr<FrameStruct>(new FrameStruct(frame_template_));
        rgbFrame->frame_type = FrameType::FrameTypeColor; // 0;
        rgbFrame->frame_data_type = FrameDataType::FrameDataTypeCvMat; // 9;
        rgbFrame->frame_id = current_frame_counter_;
        rgbFrame->timestamps.push_back(capture_timestamp);
        int32_t colorCols = frameRgbOpenCv.cols;
        int32_t colorRows = frameRgbOpenCv.rows; 
        size_t colorSize = colorCols*colorRows*3*sizeof(uchar); //This assumes that oakd color always returns CV_8UC3
        rgbFrame->frame.resize(colorSize + 2 * sizeof(int32_t));   
        std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;                         
        memcpy(&rgbFrame->frame[0], &colorCols, sizeof(int32_t)); 
        std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
        memcpy(&rgbFrame->frame[4], &colorRows, sizeof(int32_t)); 
        std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
        memcpy(&rgbFrame->frame[8], (unsigned char*)(frameRgbOpenCv.data), colorSize);   
        std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
        return rgbFrame;
    };

    auto convertDepthFrame = [&](const std::shared_ptr<dai::ImgFrame> &synchedDepthFrame) ->  std::shared_ptr<FrameStruct> {
        auto frameDepthOpenCv = synchedDepthFrame->getCvFrame();
  std::shared_ptr<FrameStruct> depthFrame =
      std::shared_ptr<FrameStruct>(new FrameStruct(frame_template_));
  depthFrame->frame_type = FrameType::FrameTypeDepth; // 1;
  depthFrame->frame_data_type = FrameDataType::FrameDataTypeGRAY16LE; // 3;
  depthFrame->frame_id = current_frame_counter_;
  depthFrame->timestamps.push_back(capture_timestamp);
  std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;

  std::cerr << int(frameDepthOpenCv.isContinuous()) << std::endl;
  std::cerr << type2str(frameDepthOpenCv.type()) << std::endl;
   // convert the raw buffer to cv::Mat
   int32_t depthCols = frameDepthOpenCv.cols;                                                        // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT
   int32_t depthRows = frameDepthOpenCv.rows;  
   std::cerr << "depth: " << depthCols << " x " << depthRows << std::endl << std::flush;                                                      // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT
   size_t depthSize = depthCols*depthRows*1; // ???? *sizeof(uint16_t); //This assumes that oakd depth always returns CV_16U// UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT

   depthFrame->frame.resize(depthSize + 2 * sizeof(int32_t));                                        // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT

   std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
   memcpy(&depthFrame->frame[0], &depthCols, sizeof(int32_t));                                       // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT
   std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
   memcpy(&depthFrame->frame[4], &depthRows, sizeof(int32_t));                                       // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT
   std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
   std::cerr << (void*)(frameDepthOpenCv.data) << std::endl << std::flush;
   memcpy(&depthFrame->frame[8], (unsigned char*)(frameDepthOpenCv.data), depthSize);              // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT
   std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
    return depthFrame;
    };


    //Now we pull frames until we get a synchronized color and depth frame
    while (!haveSyncedFrames)
    {
        std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
        //Grab rgb and add to dictionary
        auto rgbFromQueue = qRgb->get<dai::ImgFrame>();
        if (!rgbFromQueue) {
            std::cerr << "rgb alert"<< std::endl << std::flush;
        } else {      
        seqNum = rgbFromQueue->getSequenceNum();
        auto rgbFromQueueC = convertRgbFrame(rgbFromQueue);
        frames_dictionary[seqNum].push_back(std::make_tuple("rgb", rgbFromQueueC));
        std::cerr << "rgb " << seqNum << std::endl << std::flush; 
        }
        //Grab depth and add to dictionary
        auto depthFromQueue = qDepth->get<dai::ImgFrame>();
        if (!depthFromQueue) {
            std::cerr << "alert"<< std::endl << std::flush;
        } else {
        seqNum = depthFromQueue->getSequenceNum();
        std::cerr << seqNum	<< std::endl << std::flush;
        auto depthFromQueueC = convertDepthFrame(depthFromQueue);
        frames_dictionary[seqNum].push_back(std::make_tuple("depth", depthFromQueueC));
        std::cerr << "depth " << seqNum << std::endl << std::flush;
        }
        //Check if any dictionary values have an array of length 2
        for (const auto& kv: frames_dictionary) {
	        // We check if any sequence numbers have delivered both their rgb and depth frame (so list would have size 2)
            if (kv.second.size() > 1)
            {
                //This sequence number has 2 elements in the list, so now we grab the rgb and depth frames
                std::cerr << kv.first << " " << std::get<0>(kv.second[0]) << " " << std::get<0>(kv.second[1]) << std::endl << std::flush;
                //Check if this tuple element in list is an rgb or depth, by checkign for the first index int the tuple which stores type
                if (std::get<0>(kv.second[0]) == "rgb")
                {
                    //This means first element was rgb
                    synchedRgbFrameStruct = std::get<1>(kv.second[0]);
                    synchedDepthFrameStruct = std::get<1>(kv.second[1]);
                }
                else
                {
                    // This means first element was depth
                    synchedRgbFrameStruct = std::get<1>(kv.second[1]);
                    synchedDepthFrameStruct = std::get<1>(kv.second[0]);
                }
                //We have the frames
                haveSyncedFrames = true;
                //We got the frames at this sequence number
                seqNum = kv.first;
                std::cerr << seqNum << std::endl;
                //Now we can stop the loop
                break;
            }
        }

        std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;

        //Now we remove all sequence numbers that are less than or equal the sequence number
        std::vector<int> rem;
        for (auto it = frames_dictionary.cbegin(), next_it = it; it != frames_dictionary.cend(); it = next_it)
        {
            ++next_it;
            std::cerr << "seq "<< it->first << std::endl << std::flush;
            if (it->first <= seqNum)
            {
                std::cerr << "erase" << std::endl << std::flush; 
                rem.push_back(it->first);
            //    frames_dictionary.erase(it);
            }
        }

        for (auto &r : rem) {
            frames_dictionary.erase(r);
        }
    }
    
    //This is the dictionary that we use to check if a RGB and Depth frame of the same sequence number have arrived0
    //Every "next frame" we pull a depth and a color until we get a match of depth and color
    //pull color
    //assign to proper key (if doesn't exist make one)
    //pull depth
    //assign to proper key (if doesn't exist make one)
    //check if a key has a value that is length 2
        // if yes, save those as rgbFrame and depthFrame
        // if no, start from top
 //   auto frameRgbOpenCv = synchedRgbFrame->getCvFrame();    
 //   auto frameDepthOpenCv = synchedDepthFrame->getCvFrame();
    // auto frameDepthMat = synchedDepthFrame->getFrame();
    std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
#endif

  // TODO FIXME x big/little endian hazard ~

  //Color frame
//  std::shared_ptr<FrameStruct> rgbFrame =
//      std::shared_ptr<FrameStruct>(new FrameStruct(frame_template_));
//  rgbFrame->frame_type = FrameType::FrameTypeColor; // 0;
//  rgbFrame->frame_data_type = FrameDataType::FrameDataTypeCvMat; // 9;
//  rgbFrame->frame_id = current_frame_counter_;
//  rgbFrame->timestamps.push_back(capture_timestamp);

//  std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;

#ifndef TEST_WITH_IMAGE
   // convert the raw buffer to cv::Mat
//   int32_t colorCols = frameRgbOpenCv.cols;                                                        // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT
//   int32_t colorRows = frameRgbOpenCv.rows;                                                        // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT
//   size_t colorSize = colorCols*colorRows*3*sizeof(uchar); //This assumes that oakd color always returns CV_8UC3// UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT

//   rgbFrame->frame.resize(colorSize + 2 * sizeof(int32_t));                                        // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT

//   memcpy(&rgbFrame->frame[0], &colorCols, sizeof(int32_t));                                       // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT
//   memcpy(&rgbFrame->frame[4], &colorRows, sizeof(int32_t));                                       // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT
//   memcpy(&rgbFrame->frame[8], (unsigned char*)(frameRgbOpenCv.data), colorSize);              // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT
#endif
auto rgbFrame = synchedRgbFrameStruct;
rgbFrame->frame_id = current_frame_counter_;
  current_frame_.push_back(rgbFrame);

//  std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
  //Depth frame
//  std::shared_ptr<FrameStruct> depthFrame =
//      std::shared_ptr<FrameStruct>(new FrameStruct(frame_template_));
//  depthFrame->frame_type = FrameType::FrameTypeDepth; // 1;
//  depthFrame->frame_data_type = FrameDataType::FrameDataTypeGRAY16LE; // 3;
//  depthFrame->frame_id = current_frame_counter_;
//  depthFrame->timestamps.push_back(capture_timestamp);
//  std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
#ifndef TEST_WITH_IMAGE
   // convert the raw buffer to cv::Mat
//   int32_t depthCols = frameDepthOpenCv.cols;                                                        // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT
//   int32_t depthRows = frameDepthOpenCv.rows;  
//   std::cerr << "depth: " << depthCols << " x " << depthRows << std::endl << std::flush;                                                      // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT
//   size_t depthSize = depthCols*depthRows*1*sizeof(ushort); //This assumes that oakd depth always returns CV_16U// UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT

//   depthFrame->frame.resize(depthSize + 2 * sizeof(int32_t));                                        // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT

//   std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
//   memcpy(&depthFrame->frame[0], &depthCols, sizeof(int32_t));                                       // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT
//   std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
//   memcpy(&depthFrame->frame[4], &depthRows, sizeof(int32_t));                                       // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT
//   std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
//   std::cerr << (void*)(frameDepthOpenCv.data) << std::endl << std::flush;
//   memcpy(&depthFrame->frame[8], (unsigned char*)(frameDepthOpenCv.data), depthSize);              // UNCOMMENT ONCE INFERENCE PARSING IS FIGURED OUT
//   std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
#endif
auto depthFrame = synchedDepthFrameStruct;
depthFrame->frame_id = current_frame_counter_;
  current_frame_.push_back(depthFrame);
  std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
  // --------------------------- Step 5. Create an infer request
  // -------------------------------------------------
  InferRequest infer_request = executable_network.CreateInferRequest();
  // -----------------------------------------------------------------------------------------------------
  std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
  // --------------------------- Step 6. Prepare input
  // --------------------------------------------------------
  /* Read input image to a blob and set it to an infer request without resize
   * and layout conversions. */

#ifdef TEST_WITH_IMAGE

#ifndef _WIN32 
    cv::Mat image = cv::imread("../../models/two_bodies_in_middle.jpg"); //This is a hardwired image only to help Renaud with parsing
#endif
#ifdef _WIN32
    cv::Mat image = cv::imread("../../../models/two_bodies_in_middle.jpg"); //This is a hardwired image only to help Renaud with parsing
#endif
#else
    auto &image = frameRgbOpenCv;
#endif

    // compare to:
    // python demo.py --model human-pose-estimation-3d-0001.xml --use-openvino -d CPU --images pointing_close_of_view.jpg 

    cv::Mat image2;
    int stride = 8;
    // scale to 256 ~
    double input_scale = 256.0 / image.size[0];
    std::cerr << "input_scale = " << input_scale << std::endl << std::flush;
    cv::resize(image, image2, cv::Size(), input_scale, input_scale, cv::INTER_LINEAR);

#ifdef VERY_VERBOSE
{
    // https://stackoverflow.com/questions/26681713/convert-mat-to-array-vector-in-opencv
    std::vector<uchar> array;
    auto &mat = image2;
    if (mat.isContinuous()) {
        // array.assign(mat.datastart, mat.dataend); // <- has problems for sub-matrix like mat = big_mat.row(i)
        array.assign(mat.data, mat.data + mat.total()*mat.channels());
    } else {
        for (int i = 0; i < mat.rows; ++i) {
            array.insert(array.end(), mat.ptr<uchar>(i), mat.ptr<uchar>(i)+mat.cols*mat.channels());
        }
    }

    auto dumpU8 = [](const uchar *vec, size_t n) -> std::string {
        if (n == 0)
            return "[]";
        std::stringstream oss;
        oss << "[" << int(vec[0]);
        for (size_t i = 1; i < n; i++)
            oss << "," << int(vec[i]);
        oss << "]";
        return oss.str();
    };

    std::cerr << "SCALED_IMAGE_DUMP " << dumpU8(&array[0], array.size()) << std::endl << std::flush;
}
#endif

    std::cerr << image2.size[0] << std::endl << std::flush; 
    std::cerr << (image2.size[1] - (image2.size[1] % stride)) << std::endl << std::flush;  

    cv::Mat image3 = cv::Mat(image2, cv::Rect(0, 0, 
                                                    image2.size[1] - (image2.size[1] % stride),
                                                    image2.size[0]));

#ifdef VERY_VERBOSE
{
    // https://stackoverflow.com/questions/26681713/convert-mat-to-array-vector-in-opencv
    std::vector<uchar> array;
    auto &mat = image3;
    if (mat.isContinuous()) {
        std::cerr << "is continuous" << std::endl << std::flush;

        // array.assign(mat.datastart, mat.dataend); // <- has problems for sub-matrix like mat = big_mat.row(i)
        array.assign(mat.data, mat.data + mat.total()*mat.channels());
    } else {
        for (int i = 0; i < mat.rows; ++i) {
            array.insert(array.end(), mat.ptr<uchar>(i), mat.ptr<uchar>(i)+mat.cols*mat.channels());
        }
    }

    auto dumpU8 = [](const uchar *vec, size_t n) -> std::string {
        if (n == 0)
            return "[]";
        std::stringstream oss;
        oss << "[" << int(vec[0]);
        for (size_t i = 1; i < n; i++)
            oss << "," << int(vec[i]);
        oss << "]";
        return oss.str();
    };

    std::cerr << "IMAGE_DUMP " << dumpU8(&array[0], array.size()) << std::endl << std::flush;
}
#endif

std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;

    cv::Mat image4(image3.size[0], image3.size[1], CV_8UC3);
    for (int i=0; i< image3.size[0]; ++i) {
        for (int j=0; j< image3.size[1]; ++j) {
            auto v = image3.at<cv::Vec3b>(i,j);
            auto v2 = cv::Vec3b{ v[0], v[1], v[2] }; // 0,1,2
            image4.at<cv::Vec3b>(i,j) = v2;
        }
    }

std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;

   Blob::Ptr imgBlobX = wrapMat2Blob(image4);

std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;

    infer_request.SetBlob(input_name, imgBlobX);  // infer_request accepts input blob of any size

  // -----------------------------------------------------------------------------------------------------

  // --------------------------- Step 7. Do inference
  // --------------------------------------------------------
std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
  /* Running the request synchronously */
    infer_request.Infer();
  // -----------------------------------------------------------------------------------------------------
std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
  // --------------------------- Step 8. Process output
  // ------------------------------------------------------

    Blob::Ptr features_output = infer_request.GetBlob("features");
    Blob::Ptr heatmaps_output = infer_request.GetBlob("heatmaps");
    Blob::Ptr pafs_output = infer_request.GetBlob("pafs");

    const SizeVector features_output_shape = features_output_info->getTensorDesc().getDims();
    auto l = features_output_info->getTensorDesc().getLayout();
    auto p = features_output_info->getTensorDesc().getPrecision(); 
    std::cerr << "lp " << l << " " << p << std::endl << std::flush;
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

    const float *features_result = features_outputMapped.as<float *>();
    const float *heatmaps_result = heatmaps_outputMapped.as<float *>();
    const float *pafs_result = pafs_outputMapped.as<float *>();

    std::cerr << "features_result[0] = " << features_result[0] << std::endl << std::flush;

    for (int i=0;i<10;++i) {
      std::cerr << "features_result[" << i << "] = " << features_result[i] << std::endl << std::flush;
    }

#ifdef VERY_VERBOSE
    auto dumpRes = [](const float *vec, size_t n) -> std::string {
        if (n == 0)
            return "[]";
        std::stringstream oss;
        oss << "[" << vec[0];
        for (size_t i = 1; i < n; i++)
            oss << "," << vec[i];
        oss << "]";
        return oss.str();
    };

    // grep FEATURES_RESULT_DUMP log | tail +2 | cut -d " " -f 2 > features.txt 
    std::cerr << "FEATURES_RESULT_DUMP " << dumpRes(features_result, 57*32*48) << std::endl << std::flush; 
    std::cerr << "HEATMAPS_RESULT_DUMP " << dumpRes(heatmaps_result, 19*32*48) << std::endl << std::flush; 
    std::cerr << "PAFS_RESULT_DUMP " << dumpRes(pafs_result, 38*32*48) << std::endl << std::flush; 
#endif
    std::cerr << "heatmaps_result[0] = " << heatmaps_result[0] << std::endl << std::flush;
    std::cerr << "pafs_result[0] = " << pafs_result[0] << std::endl << std::flush;

    float fx = 984.344; // -1;

    // needed?
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

    int featureMapSize[] = { (int)features_output_shape[1], (int)features_output_shape[2], (int)features_output_shape[3] }; // { 57,32,48 }; 
    cv::Mat featuresMat = cv::Mat(3, featureMapSize, CV_32FC1, const_cast<float*>(features_result));
    int heatmapMatSize[] =  { (int)heatmaps_output_shape[1], (int)heatmaps_output_shape[2], (int)heatmaps_output_shape[3] }; // { 19,32,48 };
    cv::Mat heatmapMat = cv::Mat(3, heatmapMatSize, CV_32FC1, const_cast<float*>(heatmaps_result));
    int paf_mapMatSize[] =  { (int) pafs_output_shape[1], (int) pafs_output_shape[2], (int) pafs_output_shape[3] };  // {38,32,48};
    cv::Mat paf_mapMat = cv::Mat(3, paf_mapMatSize, CV_32FC1, const_cast<float*>(pafs_result));

    std::cerr << "image size " << image.size[0] << " " << image.size[1] << std::endl << std::flush; 

    auto posesStruct = parse_poses(previous_poses_2d, common, R, 
        featuresMat, heatmapMat, paf_mapMat, input_scale, stride, fx,
        image.size[1] //1080
        , true); //true);

#ifdef TEST_WITH_IMAGE

        std::cerr << "poses3d" << std::endl << std::flush;
        for (auto &l: posesStruct.poses_3d) {
            std::cerr << "pose3d_line";
            for (auto &x: l) {
                std::cerr << ", " << x;
            }
            std::cerr << std::endl << std::flush;
        }
        std::cerr << "poses2d" << std::endl << std::flush;
        for (auto &l: posesStruct.poses_2d) {
            std::cerr << "pose2d_line";
            for (auto &x: l) {
                std::cerr << ", " << x;
            }
            std::cerr << std::endl << std::flush;
        }

        // 3D data
        std::vector<std::vector<float>> d1 = 
            std::vector<std::vector<float>> { 
                std::vector<float> { -5.8116764e-01,  1.3135418e-01, -2.6623271e-02},
                std::vector<float> { -4.1665617e-01,  9.0584852e-02,  1.0973391e-01},
                std::vector<float> { -6.3156098e-01,  1.0134139e-01, -5.2170849e-01},
                std::vector<float> { -5.7664627e-01,  2.7422610e-01, -1.6360372e-02},
                std::vector<float> { -5.7326126e-01,  3.8949987e-01, -2.4231449e-01},
                std::vector<float> { -4.7172031e-01,  3.3503777e-01, -3.6468127e-01},
                std::vector<float> { -6.1248940e-01,  2.0294990e-01, -5.3791904e-01},
                std::vector<float> { -6.3211733e-01,  1.8192437e-01, -8.9180791e-01},
                std::vector<float> { -7.2299236e-01,  1.6886683e-01, -1.2353224e+00},
                std::vector<float> { -6.3673156e-01,  7.0862949e-02, -8.2087860e-04},
                std::vector<float> { -7.6713055e-01, -1.1119901e-01, -7.6769933e-02},
                std::vector<float> { -7.2015315e-01, -2.5040415e-01, -1.9094884e-02},
                std::vector<float> { -6.5249604e-01,  3.2747467e-03, -5.1337701e-01},
                std::vector<float> { -6.6003913e-01, -1.6922930e-02, -8.6318237e-01},
                std::vector<float> { -7.5892818e-01,  5.9486399e-03, -1.2117190e+00},
                std::vector<float> { -4.1849327e-01,  1.1638151e-01,  1.1316368e-01},
                std::vector<float> { -5.0609869e-01,  1.7822178e-01,  1.0136506e-01},
                std::vector<float> { -4.3404469e-01,  5.1015873e-02,  1.4532641e-01},
                std::vector<float> { -4.9512008e-01,  6.2296622e-02,  1.3576485e-01}
            };

        std::vector<float> d2 = std::vector<float> {  7.3100000e+02,  3.9300000e+02,  6.6418380e-01,  6.8600000e+02,
                    1.9100000e+02, 7.7300525e-01, -1.0000000e+00, -1.0000000e+00,
                    -1.0000000e+00,  9.1100000e+02,  4.1600000e+02,  5.5604172e-01,
                    -1.0000000e+00, -1.0000000e+00, -1.0000000e+00, -1.0000000e+00,
                    -1.0000000e+00, -1.0000000e+00, -1.0000000e+00, -1.0000000e+00,
                    -1.0000000e+00, -1.0000000e+00, -1.0000000e+00, -1.0000000e+00,
                    -1.0000000e+00, -1.0000000e+00, -1.0000000e+00,  5.6800000e+02,
                    3.8800000e+02,  5.8045483e-01,  3.7100000e+02,  4.8300000e+02,
                    6.5866584e-01,  1.9100000e+02,  3.8800000e+02,  5.9422147e-01,
                    -1.0000000e+00, -1.0000000e+00, -1.0000000e+00, -1.0000000e+00,
                    -1.0000000e+00, -1.0000000e+00, -1.0000000e+00, -1.0000000e+00,
                    -1.0000000e+00,  6.6300000e+02,  1.6300000e+02,  8.0384231e-01,
                    7.3100000e+02,  1.4600000e+02,  7.8739011e-01,  6.3500000e+02,
                    1.6800000e+02,  2.3887357e-01,  8.2100000e+02,  1.6800000e+02,
                    7.9140055e-01,  1.2366208e+01 };

        for (unsigned i=0; i<d1.size(); ++i) {
            std::cerr << posesStruct.poses_3d[0][i*4] - d1[i][0] 
                << " " << posesStruct.poses_3d[0][i*4+1] - d1[i][1] 
                << " " << posesStruct.poses_3d[0][i*4+2] - d1[i][2] 
                << std::endl << std::flush;
        }

        std::cerr << "=======" << std::endl << std::flush;

        for (unsigned i =0; i< d2.size() / 3; ++i) {
            std::cerr << posesStruct.poses_2d[0][i*3] - d2[i*3] 
                << " " << posesStruct.poses_2d[0][i*3+1] - d2[i*3+1] 
                << std::endl << std::flush; 
        }
#endif
    /////////////////////////////////////////////

    //Prepare the bodies frame
    std::shared_ptr<FrameStruct> s =
        std::shared_ptr<FrameStruct>(new FrameStruct(frame_template_));
    s->frame_id = current_frame_counter_;
    s->frame_type = FrameType::FrameTypeHumanPose; // 4;
    s->frame_data_type = FrameDataType::FrameDataTypeObjectHumanData; // 8;
    s->timestamps.push_back(capture_timestamp);

    s->frame = std::vector<uchar>();
    
    //Grab the amount of COCO bodies detected in this rgb frame
    int32_t bodyCount = (int)posesStruct.poses_3d.size();

    std::cerr << "bodyCount: " << bodyCount << std::endl << std::flush; 

    //Resize the frame to hold all the detected COCO bodies detected in this frame
    s->frame.resize(sizeof(coco_human_t)*bodyCount + sizeof(int32_t));

    //Copy the number of COCO bodies detected into the first 4 bytes of the bodies frame
    auto nBodyCount = bodyCount;
    inplace_hton(nBodyCount);  
    memcpy(&s->frame[0], &nBodyCount, sizeof(int32_t));

    //Now we iterate through all detected bodies in poses_3d, create a coco_human_t, and then copy the data into the frame
    for (size_t i = 0; i < bodyCount; i++) {

        //Here we see the 2D joints detected of this body
        //We grab the depth at the x,y location in the image
        // posesStruct.poses_2d[i]; //how do I know what joints are being provided? (will need this for bodyStruct depth values)
        //depth value = (int)frameDepthMat.at<ushort>(yvalue,xvalue);

        

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

        // bodyStruct.Id = ; Currently id is not supported
        bodyStruct.neck_x = posesStruct.poses_3d[i][0 * 4 + 0];
        bodyStruct.neck_y = posesStruct.poses_3d[i][0 * 4 + 1];
        bodyStruct.neck_z = posesStruct.poses_3d[i][0 * 4 + 2];
        bodyStruct.neck_conf = posesStruct.poses_3d[i][0 * 4 + 3];
        bodyStruct.nose_x = posesStruct.poses_3d[i][1 * 4 + 0];
        bodyStruct.nose_y = posesStruct.poses_3d[i][1 * 4 + 1];
        bodyStruct.nose_z = posesStruct.poses_3d[i][1 * 4 + 2];
        bodyStruct.nose_conf = posesStruct.poses_3d[i][1 * 4 + 3];
        bodyStruct.pelvis_x = posesStruct.poses_3d[i][2 * 4 + 0];
        bodyStruct.pelvis_y = posesStruct.poses_3d[i][2 * 4 + 1];
        bodyStruct.pelvis_z = posesStruct.poses_3d[i][2 * 4 + 2];
        bodyStruct.pelvis_conf = posesStruct.poses_3d[i][2 * 4 + 3];
        bodyStruct.shoulder_left_x = posesStruct.poses_3d[i][3 * 4 + 0];
        bodyStruct.shoulder_left_y = posesStruct.poses_3d[i][3 * 4 + 1];
        bodyStruct.shoulder_left_z = posesStruct.poses_3d[i][3 * 4 + 2];
        bodyStruct.shoulder_left_conf = posesStruct.poses_3d[i][3 * 4 + 3];
        bodyStruct.elbow_left_x = posesStruct.poses_3d[i][4 * 4 + 0];
        bodyStruct.elbow_left_y = posesStruct.poses_3d[i][4 * 4 + 1];
        bodyStruct.elbow_left_z = posesStruct.poses_3d[i][4 * 4 + 2];
        bodyStruct.elbow_left_conf = posesStruct.poses_3d[i][4 * 4 + 3];
        bodyStruct.wrist_left_x = posesStruct.poses_3d[i][5 * 4 + 0];
        bodyStruct.wrist_left_y = posesStruct.poses_3d[i][5 * 4 + 1];
        bodyStruct.wrist_left_z = posesStruct.poses_3d[i][5 * 4 + 2];
        bodyStruct.wrist_left_conf = posesStruct.poses_3d[i][5 * 4 + 3];
        bodyStruct.hip_left_x = posesStruct.poses_3d[i][6 * 4 + 0];
        bodyStruct.hip_left_y = posesStruct.poses_3d[i][6 * 4 + 1];
        bodyStruct.hip_left_z = posesStruct.poses_3d[i][6 * 4 + 2];
        bodyStruct.hip_left_conf = posesStruct.poses_3d[i][6 * 4 + 3];
        bodyStruct.knee_left_x = posesStruct.poses_3d[i][7 * 4 + 0];
        bodyStruct.knee_left_y = posesStruct.poses_3d[i][7 * 4 + 1];
        bodyStruct.knee_left_z = posesStruct.poses_3d[i][7 * 4 + 2];
        bodyStruct.knee_left_conf = posesStruct.poses_3d[i][7 * 4 + 3];
        bodyStruct.ankle_left_x = posesStruct.poses_3d[i][8 * 4 + 0];
        bodyStruct.ankle_left_y = posesStruct.poses_3d[i][8 * 4 + 1];
        bodyStruct.ankle_left_z = posesStruct.poses_3d[i][8 * 4 + 2];
        bodyStruct.ankle_left_conf = posesStruct.poses_3d[i][8 * 4 + 3];
        bodyStruct.shoulder_right_x = posesStruct.poses_3d[i][9 * 4 + 0];
        bodyStruct.shoulder_right_y = posesStruct.poses_3d[i][9 * 4 + 1];
        bodyStruct.shoulder_right_z = posesStruct.poses_3d[i][9 * 4 + 2];
        bodyStruct.shoulder_right_conf = posesStruct.poses_3d[i][9 * 4 + 3];
        bodyStruct.elbow_right_x = posesStruct.poses_3d[i][10 * 4 + 0];
        bodyStruct.elbow_right_y = posesStruct.poses_3d[i][10 * 4 + 1];
        bodyStruct.elbow_right_z = posesStruct.poses_3d[i][10 * 4 + 2];
        bodyStruct.elbow_right_conf = posesStruct.poses_3d[i][10 * 4 + 3];
        bodyStruct.wrist_right_x = posesStruct.poses_3d[i][11 * 4 + 0];
        bodyStruct.wrist_right_y = posesStruct.poses_3d[i][11 * 4 + 1];
        bodyStruct.wrist_right_z = posesStruct.poses_3d[i][11 * 4 + 2];
        bodyStruct.wrist_right_conf = posesStruct.poses_3d[i][11 * 4 + 3];
        bodyStruct.hip_right_x = posesStruct.poses_3d[i][12 * 4 + 0];
        bodyStruct.hip_right_y = posesStruct.poses_3d[i][12 * 4 + 1];
        bodyStruct.hip_right_z = posesStruct.poses_3d[i][12 * 4 + 2];
        bodyStruct.hip_right_conf = posesStruct.poses_3d[i][12 * 4 + 3];
        bodyStruct.knee_right_x = posesStruct.poses_3d[i][13 * 4 + 0];
        bodyStruct.knee_right_y = posesStruct.poses_3d[i][13 * 4 + 1];
        bodyStruct.knee_right_z = posesStruct.poses_3d[i][13 * 4 + 2];
        bodyStruct.knee_right_conf = posesStruct.poses_3d[i][13 * 4 + 3];
        bodyStruct.ankle_right_x = posesStruct.poses_3d[i][14 * 4 + 0];
        bodyStruct.ankle_right_y = posesStruct.poses_3d[i][14 * 4 + 1];
        bodyStruct.ankle_right_z = posesStruct.poses_3d[i][14 * 4 + 2];
        bodyStruct.ankle_right_conf = posesStruct.poses_3d[i][14 * 4 + 3];
        bodyStruct.eye_left_x = posesStruct.poses_3d[i][15 * 4 + 0];
        bodyStruct.eye_left_y = posesStruct.poses_3d[i][15 * 4 + 1];
        bodyStruct.eye_left_z = posesStruct.poses_3d[i][15 * 4 + 2];
        bodyStruct.eye_left_conf = posesStruct.poses_3d[i][15 * 4 + 3];
        bodyStruct.ear_left_x = posesStruct.poses_3d[i][16 * 4 + 0];
        bodyStruct.ear_left_y = posesStruct.poses_3d[i][16 * 4 + 1];
        bodyStruct.ear_left_z = posesStruct.poses_3d[i][16 * 4 + 2];
        bodyStruct.ear_left_conf = posesStruct.poses_3d[i][16 * 4 + 3];
        bodyStruct.eye_right_x = posesStruct.poses_3d[i][17 * 4 + 0];
        bodyStruct.eye_right_y = posesStruct.poses_3d[i][17 * 4 + 1];
        bodyStruct.eye_right_z = posesStruct.poses_3d[i][17 * 4 + 2];
        bodyStruct.eye_right_conf = posesStruct.poses_3d[i][17 * 4 + 3];
        bodyStruct.ear_right_x = posesStruct.poses_3d[i][18 * 4 + 0];
        bodyStruct.ear_right_y = posesStruct.poses_3d[i][18 * 4 + 1];
        bodyStruct.ear_right_z = posesStruct.poses_3d[i][18 * 4 + 2];
        bodyStruct.ear_right_conf = posesStruct.poses_3d[i][18 * 4 + 3];

        auto to2D = [](float x) -> int16_t {
            std::cerr << "to2D: value = " << x << std::endl << std::flush;
            return int64_t(x);
        };

        bodyStruct.neck_2DX = to2D(posesStruct.poses_2d[i][0 * 2 + 0]);
        bodyStruct.neck_2DY = to2D(posesStruct.poses_2d[i][0 * 2 + 1]);
        bodyStruct.nose_2DX = to2D(posesStruct.poses_2d[i][1 * 2 + 0]);
        bodyStruct.nose_2DY = to2D(posesStruct.poses_2d[i][1 * 2 + 1]);
        bodyStruct.pelvis_2DX = to2D(posesStruct.poses_2d[i][2 * 2 + 0]);
        bodyStruct.pelvis_2DY = to2D(posesStruct.poses_2d[i][2 * 2 + 1]);
        bodyStruct.shoulder_left_2DX = to2D(posesStruct.poses_2d[i][3 * 2 + 0]);
        bodyStruct.shoulder_left_2DY = to2D(posesStruct.poses_2d[i][3 * 2 + 1]);
        bodyStruct.elbow_left_2DX = to2D(posesStruct.poses_2d[i][4 * 2 + 0]);
        bodyStruct.elbow_left_2DY = to2D(posesStruct.poses_2d[i][4 * 2 + 1]);
        bodyStruct.wrist_left_2DX = to2D(posesStruct.poses_2d[i][5 * 2 + 0]);
        bodyStruct.wrist_left_2DY = to2D(posesStruct.poses_2d[i][5 * 2 + 1]);
        bodyStruct.hip_left_2DX = to2D(posesStruct.poses_2d[i][6 * 2 + 0]);
        bodyStruct.hip_left_2DY = to2D(posesStruct.poses_2d[i][6 * 2 + 1]);
        bodyStruct.knee_left_2DX = to2D(posesStruct.poses_2d[i][7 * 2 + 0]);
        bodyStruct.knee_left_2DY = to2D(posesStruct.poses_2d[i][7 * 2 + 1]);
        bodyStruct.ankle_left_2DX = to2D(posesStruct.poses_2d[i][8 * 2 + 0]);
        bodyStruct.ankle_left_2DY = to2D(posesStruct.poses_2d[i][8 * 2 + 1]);
        bodyStruct.shoulder_right_2DX = to2D(posesStruct.poses_2d[i][9 * 2 + 0]);
        bodyStruct.shoulder_right_2DY = to2D(posesStruct.poses_2d[i][9 * 2 + 1]);
        bodyStruct.elbow_right_2DX = to2D(posesStruct.poses_2d[i][10 * 2 + 0]);
        bodyStruct.elbow_right_2DY = to2D(posesStruct.poses_2d[i][10 * 2 + 1]);
        bodyStruct.wrist_right_2DX = to2D(posesStruct.poses_2d[i][11 * 2 + 0]);
        bodyStruct.wrist_right_2DY = to2D(posesStruct.poses_2d[i][11 * 2 + 1]);
        bodyStruct.hip_right_2DX = to2D(posesStruct.poses_2d[i][12 * 2 + 0]);
        bodyStruct.hip_right_2DY = to2D(posesStruct.poses_2d[i][12 * 2 + 1]);
        bodyStruct.knee_right_2DX = to2D(posesStruct.poses_2d[i][13 * 2 + 0]);
        bodyStruct.knee_right_2DY = to2D(posesStruct.poses_2d[i][13 * 2 + 1]);
        bodyStruct.ankle_right_2DX = to2D(posesStruct.poses_2d[i][14 * 2 + 0]);
        bodyStruct.ankle_right_2DY = to2D(posesStruct.poses_2d[i][14 * 2 + 1]);
        bodyStruct.eye_left_2DX = to2D(posesStruct.poses_2d[i][15 * 2 + 0]);
        bodyStruct.eye_left_2DY = to2D(posesStruct.poses_2d[i][15 * 2 + 1]);
        bodyStruct.ear_left_2DX = to2D(posesStruct.poses_2d[i][16 * 2 + 0]);
        bodyStruct.ear_left_2DY = to2D(posesStruct.poses_2d[i][16 * 2 + 1]);
        bodyStruct.eye_right_2DX = to2D(posesStruct.poses_2d[i][17 * 2 + 0]);
        bodyStruct.eye_right_2DY = to2D(posesStruct.poses_2d[i][17 * 2 + 1]);
        bodyStruct.ear_right_2DX = to2D(posesStruct.poses_2d[i][18 * 2 + 0]);
        bodyStruct.ear_right_2DY = to2D(posesStruct.poses_2d[i][18 * 2 + 1]);

        bodyStruct.hton();
        //Finally we copy the COCO body struct memory to the frame
        memcpy(&s->frame[(i*sizeof(coco_human_t))+4], &bodyStruct, sizeof(coco_human_t));
    }

    //Now that we have copied all memory to the frame we can push it back
    current_frame_.push_back(s);

}

bool OakdXlinkReader::HasNextFrame() { return true; }

void OakdXlinkReader::Reset() {}

std::vector<std::shared_ptr<FrameStruct>> OakdXlinkReader::GetCurrentFrame() {
  return current_frame_;
}

unsigned int OakdXlinkReader::GetFps() {
  return fps;
}

std::vector<FrameType> OakdXlinkReader::GetType() {
  std::vector<FrameType> types;

  types.push_back(FrameType::FrameTypeColor);
  types.push_back(FrameType::FrameTypeDepth);
  types.push_back(FrameType::FrameTypeHumanPose); // 4;

  return types;
}

void OakdXlinkReader::GoToFrame(unsigned int frame_id) {}
unsigned int OakdXlinkReader::GetCurrentFrameId() {return current_frame_counter_;}

}