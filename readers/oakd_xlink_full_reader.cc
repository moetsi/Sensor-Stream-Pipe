
//
// Created by adammpolak on 26-09-2021.
//

#include "oakd_xlink_full_reader.h"
#include "human_poses.h"
#include <cmath>
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
    frame_template_.stream_id = RandomString(16);
    frame_template_.device_id = config["deviceid"].as<unsigned int>();
    frame_template_.scene_desc = "oakd";

    frame_template_.frame_id = 0;
    frame_template_.message_type = SSPMessageType::MessageTypeDefault; // 0;

    ip_name = config["ip"].as<std::string>();

#ifndef _WIN32    
    std::string rel = "../../"; 
#endif
#ifdef _WIN32    
    std::string rel = "../../../";
#endif

    std::map<std::string,std::string> env;
    env["REL"] = rel;
    model_path = config["model"].as<std::string>();
    model_path = StringInterpolation(env, model_path);

    model_path2 = model_path;
    try {
        model_path2 = config["model2"].as<std::string>();
        model_path2 = StringInterpolation(env, model_path2);
    } catch(std::exception & e) {
        std::cerr << e.what() << std::endl << std::flush;
    }

    //double scale_multiplier1 = MAGIC;
    //double scale_multiplier2 = MAGIC;
    //double xmagic = MAGIC;
    //int sz_x = 256;
    //int sz_y = 384;
    //bool compute_alt = false;
    //double scale_multiplier1_alt = MAGIC;
    //double scale_multiplier2_alt = MAGIC;
    //double xmagic_alt;
    //int sz_x_alt = 256;
    //int sz_y_alt = 384;

    try {
        scale_multiplier1 = config["scale_multiplier1"].as<double>();
        scale_multiplier2 = config["scale_multiplier2"].as<double>();
        xmagic = config["xmagic"].as<double>();
        sz_x = config["sz_x"].as<int>();
        sz_y = config["sz_y"].as<int>();
        compute_alt = config["compute_alt"].as<bool>();
        scale_multiplier1_alt = config["scale_multiplier1_alt"].as<double>();
        scale_multiplier2_alt = config["scale_multiplier2_alt"].as<double>();
        xmagic_alt = config["xmagic_alt"].as<double>();
        sz_x_alt = config["sz_x_alt"].as<int>();
        sz_y_alt = config["sz_y_alt"].as<int>();        
    } catch(std::exception & e) {
        std::cerr << e.what() << std::endl << std::flush;
    }

    try {
        failed = true;
        SetOrReset();
        SetOrResetState(states[0], 0);
        SetOrResetState(states[1], 1);
        failed = false;
    } catch(std::exception & e) {
        std::cerr << e.what() << std::endl << std::flush;
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
    camRgb = pipeline->create<dai::node::ColorCamera>();
    left = pipeline->create<dai::node::MonoCamera>();
    right = pipeline->create<dai::node::MonoCamera>();
    stereo = pipeline->create<dai::node::StereoDepth>();

    controlIn = pipeline->create<dai::node::XLinkIn>();
    rgbOut = pipeline->create<dai::node::XLinkOut>();
    depthOut = pipeline->create<dai::node::XLinkOut>();

    controlIn->setStreamName("control");
    rgbOut->setStreamName("rgb");
    depthOut->setStreamName("depth");


    // Color Properties
    camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
    camRgb->setResolution(st->rgb_dai_res);
    camRgb->setPreviewSize(st->rgb_dai_preview_x, st->rgb_dai_preview_y);
    camRgb->setFps(st->rgb_dai_fps);
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::RGB);
    camRgb->setInterleaved(false);
    // camRgb->setIspScale(2, 3); //this downscales from 1080p to 720p

    // Depth Properties
    left->setResolution(st->depth_dai_res);
    left->setBoardSocket(dai::CameraBoardSocket::LEFT);
    left->setFps(st->depth_dai_fps);
    right->setResolution(st->depth_dai_res);
    right->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    right->setFps(st->depth_dai_fps);
    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_ACCURACY);
    stereo->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_7x7);
    stereo->setSubpixel(true);
    stereo->setLeftRightCheck(true); // LR-check is required for depth alignment
    stereo->setDepthAlign(dai::CameraBoardSocket::RGB);
    stereo->setOutputSize(st->depth_dai_preview_x, st->depth_dai_preview_y);
    auto oakdConfig = stereo->initialConfig.get();
    oakdConfig.postProcessing.spatialFilter.enable = st->depth_dai_sf;
    oakdConfig.postProcessing.spatialFilter.holeFillingRadius = st->depth_dai_sf_hfr;
    oakdConfig.postProcessing.spatialFilter.numIterations = st->depth_dai_sf_num_it;
    oakdConfig.postProcessing.decimationFilter.decimationFactor = st->depth_dai_df;
    // oakdConfig.postProcessing.speckleFilter.enable = false;
    // oakdConfig.postProcessing.speckleFilter.speckleRange = 50;
    // oakdConfig.postProcessing.temporalFilter.enable = true;
    // oakdConfig.postProcessing.thresholdFilter.minRange = 400;
    // oakdConfig.postProcessing.thresholdFilter.maxRange = 15000;
    stereo->initialConfig.set(oakdConfig);


    // Linking
    controlIn->out.link(camRgb->inputControl);
    camRgb->preview.link(rgbOut->input);
    left->out.link(stereo->left);
    right->out.link(stereo->right);
    stereo->depth.link(depthOut->input);

    //Select which sensor through ip address
    device_info = std::make_shared<dai::DeviceInfo>(ip_name);
 
    device = std::make_shared<dai::Device>(*pipeline, *device_info, true); // usb 2 mode     
    deviceCalib = std::make_shared<dai::CalibrationHandler>(device->readCalibration());  
    cameraIntrinsics = deviceCalib->getCameraIntrinsics(dai::CameraBoardSocket::RGB, 1280, 720);
    auto lensPos = deviceCalib->getLensPosition(dai::CameraBoardSocket::RGB);
    std::cerr << "LENS POSITION: " << (int)lensPos << std::endl << std::flush;  
    // camRgb->initialControl.setManualFocus(lensPos);  
    horizontalFocalLengthPixels = cameraIntrinsics[0][0];
    verticalFocalLengthPixels =  cameraIntrinsics[1][1];  
    cameraHFOVInRadians = ((deviceCalib->getFov(dai::CameraBoardSocket::RGB) * pi) / 180.0) * (3840.0/4056.0); // Must scale for cropping: https://discordapp.com/channels/790680891252932659/924798503270625290/936746213691228260

                     
    for(const auto& cam : device->getConnectedCameras()) {
          std::cerr << static_cast<int>(cam) << " ";
          std::cerr << cam << " ";
    }
    std::cerr << endl;

    // Output queue will be used to get the rgb frames from the output defined above
    // Sets queues size and behavior
    qRgb = device->getOutputQueue("rgb", 4, false);                                      
    qDepth = device->getOutputQueue("depth", 4, false);
    controlQueue = device->getInputQueue("control");

    dai::CameraControl ctrl;
    ctrl.setManualFocus(lensPos);
    controlQueue->send(ctrl);

    spdlog::debug("Done opening");

}
void OakdXlinkFullReader::SetOrResetState(const std::shared_ptr<State> &st, int n) {
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

    // if (n > 0) {                                                                             ****
    //     st->network = st->ie.ReadNetwork(model_path2);
    // } else {
    //     st->network = st->ie.ReadNetwork(model_path);
    // }

    if (n > 0) {
        st->model = st->ie2.read_model(model_path2);
    } else {
        st->model = st->ie2.read_model(model_path);
    }
    printInputAndOutputsInfo(*(st->model));
    //#ifndef _WIN32    
    //    network = ie.ReadNetwork("../../models/human-pose-estimation-3d-0001.xml");
    //#endif
    //#ifdef _WIN32    
    //    network = ie.ReadNetwork("../../../models/human-pose-estimation-3d-0001.xml");
    //#endif
    // if (network.getOutputsInfo().size() != 1)                                                ****
    //     throw std::logic_error("Sample supports topologies with 1 output only");

    if (st->model->get_parameters().size() != 1) {
        throw std::logic_error("Model must have only one input");
    }
    // -----------------------------------------------------------------------------------------------------

    // --------------------------- Step 3. Configure input & output
    // ---------------------------------------------
    // --------------------------- Prepare input blobs
    // -----------------------------------------------------
    // input_info = network.getInputsInfo().begin()->second;                                    ****                                    
    // input_name = network.getInputsInfo().begin()->first;

    /* Mark input as resizable by setting of a resize algorithm.
      * In this case we will be able to set an input blob of any shape to an
      * infer request. Resize and layout conversions are executed automatically
      * during inference */

    
    ov::preprocess::PrePostProcessor ppp = ov::preprocess::PrePostProcessor(st->model);
    
    // st->input_info->getPreProcess().setResizeAlgorithm(RESIZE_BILINEAR);                     ****                            
    ppp.input().preprocess().resize(ov::preprocess::ResizeAlgorithm::RESIZE_LINEAR);
    
    // st->input_info->setLayout(Layout::NHWC);                                                 ****
    const ov::Layout tensor_layout{"NHWC"};
    ppp.input().tensor().set_layout(tensor_layout);
    // st->input_info->setPrecision(Precision::U8);                                             ****
    ov::element::Type input_type = ov::element::u8;
    ppp.input().tensor().set_element_type(input_type);
    // set model layout
    ppp.input().model().set_layout("NCHW");

    // --------------------------- Prepare output blobs
    // ----------------------------------------------------
    // if (st->network.getOutputsInfo().empty()) {                                              ****
    //     std::cerr << "Network outputs info is empty" << std::endl;
    //     return;
    // }
    // st->features_output_info = st->network.getOutputsInfo()["features"];                     ****
    // st->heatmaps_output_info = st->network.getOutputsInfo()["heatmaps"];
    // st->pafs_output_info = st->network.getOutputsInfo()["pafs"];

    // st->features_output_info->setPrecision(Precision::FP32);                                 ****
    // st->heatmaps_output_info->setPrecision(Precision::FP32);
    // st->pafs_output_info->setPrecision(Precision::FP32);

    // We are setting the outputs of the network to be FP32
    for (const ov::Output<ov::Node>& out : st->model->outputs()) {
        ppp.output(out.get_any_name()).tensor().set_element_type(ov::element::f32);
        std::cerr << "!!! OUTPUT NAME !!!!" << std::endl << std::flush;
        std::cerr << out.get_any_name() << std::endl << std::flush;
    }

    // -----------------------------------------------------------------------------------------------------

    // --------------------------- Step 4. Loading a model to the device
    // ------------------------------------------
    // st->executable_network = st->ie.LoadNetwork(st->network, "CPU");                         ****
    // Apply preprocessing modifying the original 'model'
    st->model = ppp.build();
    // Loading a model to the device
    st->compiled_model = st->ie2.compile_model(st->model, "CPU");
    // -----------------------------------------------------------------------------------------------------
    start_time = CurrentTimeMs();
}

OakdXlinkFullReader::~OakdXlinkFullReader() {

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

int findMedianXlinkFull(vector<uint16_t> a,
                  int n)
{
  
    // If size of the arr[] is even
    if (n % 2 == 0) {
  
        // Applying nth_element
        // on n/2th index
        nth_element(a.begin(),
                    a.begin() + n / 2,
                    a.end());
  
        // Applying nth_element
        // on (n-1)/2 th index
        nth_element(a.begin(),
                    a.begin() + (n - 1) / 2,
                    a.end());
  
        // Find the average of value at
        // index N/2 and (N-1)/2
        return (int)(a[(n - 1) / 2]
                        + a[n / 2])
               / 2.0;
    }
  
    // If size of the arr[] is odd
    else {
  
        // Applying nth_element
        // on n/2
        nth_element(a.begin(),
                    a.begin() + n / 2,
                    a.end());
  
        // Value at index (N/2)th
        // is the median
        return (uint16_t)a[n / 2];
    }
}

vector<uint16_t> returnVectorOfNonZeroValuesInRoiXlinkFull(cv::Mat &frameDepthMat, int xPoint, int yPoint, int roiRadius)
{
    //Region square radius
    int regionRadius = roiRadius;

    //We grab a region of interest, we need to make sure it is not asking for pixels outside of the frame
    int xPointMin  = (xPoint - regionRadius >= 0) ? xPoint - regionRadius : 0; //make sure the x min isn't less than 0
    xPointMin  = (xPointMin <= frameDepthMat.cols) ? xPointMin : frameDepthMat.cols; //make sure the x min isn't more than amount of columns
    int xPointMax  = (xPoint + regionRadius <= frameDepthMat.cols) ? xPoint + regionRadius : frameDepthMat.cols; //make sure the x max isn't more than number of columns
    xPointMax  = (xPointMax >= 0) ? xPointMax : 0; //make sure the x max isn't less than 0
    int yPointMin  = (yPoint - regionRadius >= 0) ? yPoint - regionRadius : 0; //make sure the y min isn't less than 0
    yPointMin  = (yPointMin <= frameDepthMat.rows) ? yPointMin : frameDepthMat.rows; //make sure the y min isn't more than amount of rows
    int yPointMax   = (yPoint + regionRadius <= frameDepthMat.rows) ? yPoint + regionRadius : frameDepthMat.rows; //make sure the y max isn't more than amount of rows
    yPointMax   = (yPointMax >= 0) ? yPointMax : 0; //make sure the y max isn't less than 0

    cv::Rect myROI(cv::Point(xPointMin, yPointMin), cv::Point(xPointMax , yPointMax));
    cv::Mat croppedDepth = frameDepthMat(myROI);
    std::vector<uint16_t> nonZeroDepthValues;
    int limit = croppedDepth.rows * croppedDepth.cols;
    if (!croppedDepth.isContinuous())
    {
        croppedDepth = croppedDepth.clone();
    }
    ushort* ptr = reinterpret_cast<ushort*>(croppedDepth.data);
    // std::cerr << "ROI 2 - Limit: " << limit  << std::endl << std::flush;
    for (int i = 0; i < limit; i++, ptr++)
    {
        // std::cerr << "ROI 2 - i: " << i  << "   |  *ptr value: " << *ptr  << std::endl << std::flush;
        if(*ptr != 0)
        {
            nonZeroDepthValues.push_back((uint16_t)(*ptr));
        }
        
    }
    return nonZeroDepthValues;

}

struct moetsi::ssp::human_pose_estimation::poses OakdXlinkFullReader::getPosesAfterImageResize(int targetX, int targetY, const std::shared_ptr<State> &st, cv::Mat &image, bool isFlex)  {
    struct moetsi::ssp::human_pose_estimation::poses posesStruct;
    // InferRequest infer_request = st->executable_network.CreateInferRequest();                ****
    std::cerr << "RIGHT BEFORE CREATING INFER REQUEST" << std::endl << std::flush;
    ov::InferRequest infer_request = st->compiled_model.create_infer_request();
    std::cerr << "RIGHT AFTER CREATING INFER REQUEST" << std::endl << std::flush;
    cv::Mat image2;

    std::cerr << image.size[0] << " x " << image.size[1] << std::endl << std::flush; 

    double scale =  double(targetX) / image.size[1];
    double scale2 = double(targetY) / image.size[0];
    std::cerr << "scales= " << scale << ", " << scale2 << std::endl << std::flush;
    cv::resize(image, image2, cv::Size(), scale, scale2, cv::INTER_LINEAR);
    cv::Mat image3 = cv::Mat(image2, cv::Rect(0, 0, 
                                                    image2.size[1] - (image2.size[1] % st->stride),
                                                    image2.size[0]));

    cv::Mat image4(image3.size[0], image3.size[1], CV_8UC3);
    for (int i=0; i< image3.size[0]; ++i) {
        for (int j=0; j< image3.size[1]; ++j) {
            auto v = image3.at<cv::Vec3b>(i,j);
            auto v2 = cv::Vec3b{ v[0], v[1], v[2] };
            image4.at<cv::Vec3b>(i,j) = v2;
        }
    }

    std::cerr << image4.size[0] << " x " << image4.size[1] << std::endl << std::flush;
    // Blob::Ptr imgBlobX = wrapMat2Blob(image4);                                               ****
    std::cerr << "RIGHT BEFORE CREATING WARP 2 TENSOR" << std::endl << std::flush;
    ov::Tensor imgBlobX = wrapMat2Tensor(image);
    std::cerr << "RIGHT AFTER CREATING WRAP 2 TENSOR" << std::endl << std::flush;

    // infer_request.SetBlob(st->input_name, imgBlobX);                                         ****
    try
    {
        infer_request.set_input_tensor(0, imgBlobX);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    
    
    std::cerr << "RIGHT AFTER SET INPUT TENSOR" << std::endl << std::flush;

    
    try
    {
        // infer_request.Infer();                                                                   ****
        infer_request.infer();
        std::cerr << "ACHIEVED INFERENCE" << std::endl << std::flush;
    }
    catch(...) {
        std::cerr << "TRY/CATCH CATCH AFTER REAL INFERENCE" << std::endl << std::flush;
    }

    // Blob::Ptr features_output = infer_request.GetBlob("features");                           ****
    // Blob::Ptr heatmaps_output = infer_request.GetBlob("heatmaps");
    // Blob::Ptr pafs_output = infer_request.GetBlob("pafs");

    const ov::Tensor& features_output = infer_request.get_tensor("features");
    const ov::Tensor& heatmaps_output = infer_request.get_tensor("heatmaps");
    const ov::Tensor& pafs_output = infer_request.get_tensor("pafs");

    // It seemed like this was all used to check the sizes of output
    // const SizeVector features_output_shape = st->features_output_info->getTensorDesc().getDims();
    // auto l = st->features_output_info->getTensorDesc().getLayout();
    // auto p = st->features_output_info->getTensorDesc().getPrecision(); 

    // const SizeVector heatmaps_output_shape = st->heatmaps_output_info->getTensorDesc().getDims();
    // const SizeVector pafs_output_shape = st->pafs_output_info->getTensorDesc().getDims();

    const ov::Shape features_output_shape = features_output.get_shape();
    const ov::Shape heatmaps_output_shape = heatmaps_output.get_shape();
    const ov::Shape pafs_output_shape = pafs_output.get_shape();

    std::cerr << "!!!! PRINTING SHAPE OF OUTPUTS !!!!" << std::endl << std::flush;
    std::cerr << dumpVec(features_output_shape) << std::endl << std::flush;
    std::cerr << dumpVec(heatmaps_output_shape) << std::endl << std::flush;
    std::cerr << dumpVec(pafs_output_shape) << std::endl << std::flush;

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

    // InferenceEngine::MemoryBlob::CPtr features_moutput = InferenceEngine::as<InferenceEngine::MemoryBlob>(features_output);  ****
    // InferenceEngine::MemoryBlob::CPtr heatmaps_moutput = InferenceEngine::as<InferenceEngine::MemoryBlob>(heatmaps_output);  ****
    // InferenceEngine::MemoryBlob::CPtr pafs_moutput = InferenceEngine::as<InferenceEngine::MemoryBlob>(pafs_output);          ****

    // InferenceEngine::LockedMemory<const void> features_outputMapped = features_moutput->rmap();                              ****
    // InferenceEngine::LockedMemory<const void> heatmaps_outputMapped = heatmaps_moutput->rmap();
    // InferenceEngine::LockedMemory<const void> pafs_outputMapped = pafs_moutput->rmap();

    // const float *features_result = features_outputMapped.as<float *>();                                                      ****
    // const float *heatmaps_result = heatmaps_outputMapped.as<float *>();
    // const float *pafs_result = pafs_outputMapped.as<float *>();

    // Access tensor data directly
    const float* features_result = features_output.data<float>();
    const float* heatmaps_result = heatmaps_output.data<float>();
    const float* pafs_result = pafs_output.data<float>();

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
    int paf_mapMatSize[] =  { (int) pafs_output_shape[1], (int) pafs_output_shape[2], (int) pafs_output_shape[3] }; // {38,32,48};
    cv::Mat paf_mapMat = cv::Mat(3, paf_mapMatSize, CV_32FC1, const_cast<float*>(pafs_result));

    // double scale_multiplier1 = MAGIC;
    // double scale_multiplier2 = MAGIC;
    // double xmagic = MAGIC;
    // int sz_x = 256;
    // int sz_y = 384;
    // bool compute_alt = false;
    // double scale_multiplier1_alt = MAGIC;
    // double scale_multiplier2_alt = MAGIC;
    // double xmagic_alt;
    // int sz_x_alt = 256;
    // int sz_y_alt = 384;

    auto input_scale2 = 256.0/720.0 * magic;
    auto input_scale3 = 256.0/720.0;

    if (!isFlex) {
        // magic, magic, magic
        // posesStruct = parse_poses(st->previous_poses_2d, st->common, R, featuresMat, heatmapMat, paf_mapMat, input_scale2, input_scale2, st->stride, st->fx, 1080, true, magic);
        double is2 = 256.0/720.0 * scale_multiplier1;
        double is3 = 256.0/720.0 * scale_multiplier2;
        double mg = xmagic;
        posesStruct = parse_poses(st->previous_poses_2d, st->common, R, featuresMat, heatmapMat, paf_mapMat, is2, is3, st->stride, st->fx, 1080, true, mg);
    } else {
        // 1, magic, 1
        // posesStruct = parse_poses_flex(st->previous_poses_2d, st->common, R, featuresMat, heatmapMat, paf_mapMat, input_scale3, input_scale2, st->stride, st->fx, 1080, true, 1.0);
        double is2 = 256.0/720.0 * scale_multiplier1_alt;
        double is3 = 256.0/720.0 * scale_multiplier2_alt;
        double mg = xmagic_alt;
        posesStruct = parse_poses(st->previous_poses_2d, st->common, R, featuresMat, heatmapMat, paf_mapMat, is2, is3, st->stride, st->fx, 1080, true, mg);
    }
    return posesStruct;
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

            // if (kk=0) 
            //{
            //    throw std::logic_error("BOO");
            //}

            current_frame_.clear();

            //Increment counter and grab time
            current_frame_counter_++;
            uint64_t capture_timestamp = CurrentTimeMs();
            auto framesASecond = (float)current_frame_counter_/((float)(capture_timestamp - start_time)*.001);
            std::cerr << "FRAMES A SECOND: " << framesASecond << std::endl << std::flush;

            //Here we try until we get a synchronized color and depth frame
            bool haveSyncedFrames = false;
            std::shared_ptr<dai::ImgFrame> synchedRgbFrame;
            std::shared_ptr<dai::ImgFrame> synchedDepthFrame;
            int rgbSeqNum;
            int depthSeqNum;
            struct moetsi::ssp::human_pose_estimation::poses synchedPosesStruct;
            // Now we pull frames until we get a synchronized nn and depth frame
            // The dictionary has a key of seqNum (int) and value is a img::frame pointer
            // std::unordered_map<int, struct color_poses_and_depth { std::shared_ptr<dai::ImgFrame> rgb; moetsi::ssp::human_pose_estimation::poses poses; std::shared_ptr<dai::ImgFrame> depth;};
            while (!haveSyncedFrames)
            {
                auto rgbFromQueue = qRgb->tryGet<dai::ImgFrame>();
                if (rgbFromQueue != nullptr)
                {
                    auto testRgbSeqNum = rgbFromQueue->getSequenceNum();
                    struct moetsi::ssp::human_pose_estimation::poses posesStruct;
                    struct moetsi::ssp::human_pose_estimation::poses posesStruct2;
                    // We run inference and parse poses on every received RGB to help with body tracking
                    try
                    {
                        // --------------------------------------------------------
                        // Read input image to a blob and set it to an infer request without resize and layout conversions.
                        auto rgbCvMat = rgbFromQueue->getCvFrame();
                        auto &image = rgbCvMat;

                        // double scale_multiplier1 = MAGIC;
                        // double scale_multiplier2 = MAGIC;
                        // double xmagic = MAGIC;
                        // int sz_x = 256;
                        // int sz_y = 384;
                        // bool compute_alt = false;
                        // double scale_multiplier1_alt = MAGIC;
                        // double scale_multiplier2_alt = MAGIC;
                        // double xmagic_alt;
                        // int sz_x_alt = 256;
                        // int sz_y_alt = 384;
                        std::cerr << "RIGHT BEFORE GET POSES AFTER IMAGE RESIZE" << framesASecond << std::endl << std::flush;
                        posesStruct =  getPosesAfterImageResize(
                            // 256, 384, 
                            sz_x, sz_y,
                            states[0], image, false);
                        if (compute_alt) {
                            posesStruct2 = getPosesAfterImageResize(
                                // 256, 448,
                                sz_x_alt, sz_y_alt,
                                states[1], image, true); 
                        }
                        std::cerr << "RIGHT AFTER GET POSES AFTER IMAGE RESIZE" << framesASecond << std::endl << std::flush;
                        std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
                        {
                            int c = 0;
                            for (auto & x : posesStruct.poses_2d) {
                                std::cerr << x.size() << std::endl << std::flush;
                                if (posesStruct2.poses_2d.size() > unsigned(c)) {
                                    auto &y = posesStruct2.poses_2d[c];
                                    std::cerr << y.size() << std::endl << std::flush;

                                    if (x.size() == y.size()) {
                                        std::cerr << "cmp" << std::endl << std::flush;

                                        for (unsigned k=0; k+2< x.size(); k += 3) {
                                            std::cerr << "k=" << k << std::endl << std::flush;
                                            std::cerr << x[k] << "," << x[k+1] << "," << x[k+2] << std::endl << std::flush;
                                            std::cerr << y[k] << "," << y[k+1] << "," << y[k+2] << std::endl << std::flush;
                                            std::cerr << (y[k]-x[k]) << "," << (y[k+1]-x[k+1]) << "," << (y[k+2]-x[k+2]) << std::endl << std::flush;
                                        }
                                    }
                                }
                                ++ c;
                            }

                            c = 0;
                            for (auto & x: posesStruct.poses_3d) {
                                std::cerr << x.size() << std::endl << std::flush;
                                if (posesStruct2.poses_3d.size() > unsigned(c)) {
                                    auto &y = posesStruct2.poses_3d[c];
                                    std::cerr << y.size() << std::endl << std::flush;

                                    if (x.size() == y.size()) {
                                        std::cerr << "cmp/3d" << std::endl << std::flush;

                                        for (unsigned k=0; k+3< x.size(); k += 4) {
                                            std::cerr << "k=" << k << std::endl << std::flush;
                                            std::cerr << x[k] << "," << x[k+1] << "," << x[k+2] << "," << x[k+3] << std::endl << std::flush;
                                            std::cerr << y[k] << "," << y[k+1] << "," << y[k+2] << "," << y[k+3] << std::endl << std::flush;
                                            std::cerr << (y[k]-x[k]) << "," << (y[k+1]-x[k+1]) << "," << (y[k+2]-x[k+2]) << "," << (y[k+3]-x[k+3]) << std::endl << std::flush;
                                        }                                    
                                    }
                                }
                                ++c;
                            }
#if 0
                    posesStruct = posesStruct2;
#endif


                        }
                        std::cerr << __FILE__ << ":" << __LINE__ << std::endl << std::flush;
                    } catch(...) {
                        std::cerr << "TRY/CATCH CATCH AFTER INFERENCE " << std::endl << std::flush;
                        return;
                    }

                    //Grab rgb data and add to dictionary
                    rgbSeqNum = rgbFromQueue->getSequenceNum();
                    if (frames_dictionary.find(rgbSeqNum) != frames_dictionary.end() && !haveSyncedFrames)
                    {
                        synchedRgbFrame = rgbFromQueue;
                        synchedDepthFrame = frames_dictionary[rgbSeqNum].depth;
                        synchedPosesStruct = posesStruct;
                        haveSyncedFrames = true;
                        break;
                    }
                    else
                    {
                        frames_dictionary[rgbSeqNum].rgb = rgbFromQueue;
                        frames_dictionary[rgbSeqNum].poses = posesStruct;
                    }
                }

                auto depthFromQueue = qDepth->tryGet<dai::ImgFrame>();
                if (depthFromQueue != nullptr)
                {
                    //Grab depth and add to dictionary
                    depthSeqNum = depthFromQueue->getSequenceNum();
                    if (frames_dictionary.find(depthSeqNum) != frames_dictionary.end() && !haveSyncedFrames)
                    {
                        synchedDepthFrame = depthFromQueue;
                        synchedRgbFrame = frames_dictionary[depthSeqNum].rgb;
                        synchedPosesStruct = frames_dictionary[depthSeqNum].poses;
                        haveSyncedFrames = true;
                        break;
                    }
                    else
                    {
                        frames_dictionary[depthSeqNum].depth = depthFromQueue;
                    }
                }
            }

            //Now we remove all sequence numbers that are less than or equal the sequence number
            for (auto it = frames_dictionary.cbegin(), next_it = it; it != frames_dictionary.cend(); it = next_it)
            {
                ++next_it;
                if (it->first <= depthSeqNum)
                {
                    frames_dictionary.erase(it);
                }
            }

            auto frameRgbOpenCv = synchedRgbFrame->getCvFrame();
            cv::resize(frameRgbOpenCv, frameRgbOpenCv, cv::Size(1280, 720), 0, 0, cv::INTER_NEAREST);

            // auto frameDepthOpenCv = synchedDepthFrame->getCvFrame();
            auto frameDepthMat = synchedDepthFrame->getFrame();
            cv::resize(frameDepthMat, frameDepthMat, cv::Size(1280, 720), 0, 0, cv::INTER_NEAREST);

            auto sz1depth = frameDepthMat.size[1];
            auto sz2depth = frameDepthMat.size[0];

            std::cerr << sz1depth << " x " << sz2depth << std::endl << std::flush;

            // TODO FIXME x big/little endian hazard ~
            // Color frame
            std::shared_ptr<FrameStruct> rgbFrame =
                std::shared_ptr<FrameStruct>(new FrameStruct(frame_template_));
                rgbFrame->sensor_id = 0; 
            rgbFrame->frame_type = FrameType::FrameTypeColor; // 0;
            rgbFrame->frame_data_type = FrameDataType::FrameDataTypeCvMat; // 9;
            rgbFrame->frame_id = current_frame_counter_;
            rgbFrame->timestamps.push_back(capture_timestamp);

            // convert the raw buffer to cv::Mat
            int32_t colorCols = frameRgbOpenCv.cols;                                                        
            int32_t colorRows = frameRgbOpenCv.rows;                                                        
            size_t colorSize = colorCols*colorRows*3*sizeof(uchar); //This assumes that oakd color always returns CV_8UC3

            rgbFrame->frame.resize(colorSize + 2 * sizeof(int32_t));                                        

            memcpy(&rgbFrame->frame[0], &colorCols, sizeof(int32_t));                                       
            memcpy(&rgbFrame->frame[4], &colorRows, sizeof(int32_t));                                       
            memcpy(&rgbFrame->frame[8], (unsigned char*)(frameRgbOpenCv.data), colorSize);

            //Depth frame
            std::shared_ptr<FrameStruct> depthFrame =
                std::shared_ptr<FrameStruct>(new FrameStruct(frame_template_));
            depthFrame->sensor_id = 1;
            depthFrame->frame_type = FrameType::FrameTypeDepth; // 1;
            depthFrame->frame_data_type = FrameDataType::FrameDataTypeDepthAIStereoDepth; // 11; It is not when not subpixel
            depthFrame->frame_id = current_frame_counter_;
            depthFrame->timestamps.push_back(capture_timestamp);

            // convert the raw buffer to cv::Mat
            int32_t depthCols = frameDepthMat.cols;                                                        
            int32_t depthRows = frameDepthMat.rows;                                                        
            size_t depthSize = depthCols*depthRows*sizeof(uint16_t); // DepthAI StereoDepth outputs ImgFrame message that carries RAW16 encoded (0..65535) depth data in millimeters.

            depthFrame->frame.resize(depthSize + 2 * sizeof(int32_t));                                        

            memcpy(&depthFrame->frame[0], &depthCols, sizeof(int32_t));                                       
            memcpy(&depthFrame->frame[4], &depthRows, sizeof(int32_t));                                       
            memcpy(&depthFrame->frame[8], (unsigned char*)(frameDepthMat.data), depthSize);              

            if (stream_depth)
                current_frame_.push_back(depthFrame);
              
            try {

                /////////////////////////////////////////////
                //Grab the amount of COCO bodies detected in this rgb frame
                int32_t bodyCount = (int)synchedPosesStruct.poses_3d.size();
                //If 0, no need to send structs
                if (bodyCount < 1)
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
                
                std::cerr << "!!BODY COUNT: " << bodyCount << std::endl << std::flush; 

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
                    // synchedPosesStruct.poses_2d[i]; //how do I know what joints are being provided? (will need this for bodyStruct depth values)
                    // depth value = (int)frameDepthMat.at<ushort>(yvalue,xvalue);

                    //Create a COCO body Struct
                    coco_human_t bodyStruct;
                    bodyStruct.Id = synchedPosesStruct.poses_id[i];
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
                    // synchedPosesStruct.poses_3d [ {body number } ][ {body joint index}*4 + {0, 1, 2 ,3 for x, y, z, probability}]

                    auto zCorrection = [](float x) -> float {
                        return x;
                    };

                    bodyStruct.neck_x = synchedPosesStruct.poses_3d[i][0 * 4 + 0];
                    bodyStruct.neck_y = synchedPosesStruct.poses_3d[i][0 * 4 + 1];
                    bodyStruct.neck_z = zCorrection(synchedPosesStruct.poses_3d[i][0 * 4 + 2]);
                    bodyStruct.neck_conf = synchedPosesStruct.poses_3d[i][0 * 4 + 3];
                    bodyStruct.nose_x = synchedPosesStruct.poses_3d[i][1 * 4 + 0];
                    bodyStruct.nose_y = synchedPosesStruct.poses_3d[i][1 * 4 + 1];
                    bodyStruct.nose_z = zCorrection(synchedPosesStruct.poses_3d[i][1 * 4 + 2]);
                    bodyStruct.nose_conf = synchedPosesStruct.poses_3d[i][1 * 4 + 3];
                    bodyStruct.pelvis_x = synchedPosesStruct.poses_3d[i][2 * 4 + 0];
                    bodyStruct.pelvis_y = synchedPosesStruct.poses_3d[i][2 * 4 + 1];
                    bodyStruct.pelvis_z = zCorrection(synchedPosesStruct.poses_3d[i][2 * 4 + 2]);
                    bodyStruct.pelvis_conf = synchedPosesStruct.poses_3d[i][2 * 4 + 3];
                    bodyStruct.shoulder_left_x = synchedPosesStruct.poses_3d[i][3 * 4 + 0];
                    bodyStruct.shoulder_left_y = synchedPosesStruct.poses_3d[i][3 * 4 + 1];
                    bodyStruct.shoulder_left_z = zCorrection(synchedPosesStruct.poses_3d[i][3 * 4 + 2]);
                    bodyStruct.shoulder_left_conf = synchedPosesStruct.poses_3d[i][3 * 4 + 3];
                    bodyStruct.elbow_left_x = synchedPosesStruct.poses_3d[i][4 * 4 + 0];
                    bodyStruct.elbow_left_y = synchedPosesStruct.poses_3d[i][4 * 4 + 1];
                    bodyStruct.elbow_left_z = zCorrection(synchedPosesStruct.poses_3d[i][4 * 4 + 2]);
                    bodyStruct.elbow_left_conf = synchedPosesStruct.poses_3d[i][4 * 4 + 3];
                    bodyStruct.wrist_left_x = synchedPosesStruct.poses_3d[i][5 * 4 + 0];
                    bodyStruct.wrist_left_y = synchedPosesStruct.poses_3d[i][5 * 4 + 1];
                    bodyStruct.wrist_left_z = zCorrection(synchedPosesStruct.poses_3d[i][5 * 4 + 2]);
                    bodyStruct.wrist_left_conf = synchedPosesStruct.poses_3d[i][5 * 4 + 3];
                    bodyStruct.hip_left_x = synchedPosesStruct.poses_3d[i][6 * 4 + 0];
                    bodyStruct.hip_left_y = synchedPosesStruct.poses_3d[i][6 * 4 + 1];
                    bodyStruct.hip_left_z = zCorrection(synchedPosesStruct.poses_3d[i][6 * 4 + 2]);
                    bodyStruct.hip_left_conf = synchedPosesStruct.poses_3d[i][6 * 4 + 3];
                    bodyStruct.knee_left_x = synchedPosesStruct.poses_3d[i][7 * 4 + 0];
                    bodyStruct.knee_left_y = synchedPosesStruct.poses_3d[i][7 * 4 + 1];
                    bodyStruct.knee_left_z = zCorrection(synchedPosesStruct.poses_3d[i][7 * 4 + 2]);
                    bodyStruct.knee_left_conf = synchedPosesStruct.poses_3d[i][7 * 4 + 3];
                    bodyStruct.ankle_left_x = synchedPosesStruct.poses_3d[i][8 * 4 + 0];
                    bodyStruct.ankle_left_y = synchedPosesStruct.poses_3d[i][8 * 4 + 1];
                    bodyStruct.ankle_left_z = zCorrection(synchedPosesStruct.poses_3d[i][8 * 4 + 2]);
                    bodyStruct.ankle_left_conf = synchedPosesStruct.poses_3d[i][8 * 4 + 3];
                    bodyStruct.shoulder_right_x = synchedPosesStruct.poses_3d[i][9 * 4 + 0];
                    bodyStruct.shoulder_right_y = synchedPosesStruct.poses_3d[i][9 * 4 + 1];
                    bodyStruct.shoulder_right_z = zCorrection(synchedPosesStruct.poses_3d[i][9 * 4 + 2]);
                    bodyStruct.shoulder_right_conf = synchedPosesStruct.poses_3d[i][9 * 4 + 3];
                    bodyStruct.elbow_right_x = synchedPosesStruct.poses_3d[i][10 * 4 + 0];
                    bodyStruct.elbow_right_y = synchedPosesStruct.poses_3d[i][10 * 4 + 1];
                    bodyStruct.elbow_right_z = zCorrection(synchedPosesStruct.poses_3d[i][10 * 4 + 2]);
                    bodyStruct.elbow_right_conf = synchedPosesStruct.poses_3d[i][10 * 4 + 3];
                    bodyStruct.wrist_right_x = synchedPosesStruct.poses_3d[i][11 * 4 + 0];
                    bodyStruct.wrist_right_y = synchedPosesStruct.poses_3d[i][11 * 4 + 1];
                    bodyStruct.wrist_right_z = zCorrection(synchedPosesStruct.poses_3d[i][11 * 4 + 2]);
                    bodyStruct.wrist_right_conf = synchedPosesStruct.poses_3d[i][11 * 4 + 3];
                    bodyStruct.hip_right_x = synchedPosesStruct.poses_3d[i][12 * 4 + 0];
                    bodyStruct.hip_right_y = synchedPosesStruct.poses_3d[i][12 * 4 + 1];
                    bodyStruct.hip_right_z = zCorrection(synchedPosesStruct.poses_3d[i][12 * 4 + 2]);
                    bodyStruct.hip_right_conf = synchedPosesStruct.poses_3d[i][12 * 4 + 3];
                    bodyStruct.knee_right_x = synchedPosesStruct.poses_3d[i][13 * 4 + 0];
                    bodyStruct.knee_right_y = synchedPosesStruct.poses_3d[i][13 * 4 + 1];
                    bodyStruct.knee_right_z = zCorrection(synchedPosesStruct.poses_3d[i][13 * 4 + 2]);
                    bodyStruct.knee_right_conf = synchedPosesStruct.poses_3d[i][13 * 4 + 3];
                    bodyStruct.ankle_right_x = synchedPosesStruct.poses_3d[i][14 * 4 + 0];
                    bodyStruct.ankle_right_y = synchedPosesStruct.poses_3d[i][14 * 4 + 1];
                    bodyStruct.ankle_right_z = zCorrection(synchedPosesStruct.poses_3d[i][14 * 4 + 2]);
                    bodyStruct.ankle_right_conf = synchedPosesStruct.poses_3d[i][14 * 4 + 3];
                    bodyStruct.eye_left_x = synchedPosesStruct.poses_3d[i][15 * 4 + 0];
                    bodyStruct.eye_left_y = synchedPosesStruct.poses_3d[i][15 * 4 + 1];
                    bodyStruct.eye_left_z = zCorrection(synchedPosesStruct.poses_3d[i][15 * 4 + 2]);
                    bodyStruct.eye_left_conf = synchedPosesStruct.poses_3d[i][15 * 4 + 3];
                    bodyStruct.ear_left_x = synchedPosesStruct.poses_3d[i][16 * 4 + 0];
                    bodyStruct.ear_left_y = synchedPosesStruct.poses_3d[i][16 * 4 + 1];
                    bodyStruct.ear_left_z = zCorrection(synchedPosesStruct.poses_3d[i][16 * 4 + 2]);
                    bodyStruct.ear_left_conf = synchedPosesStruct.poses_3d[i][16 * 4 + 3];
                    bodyStruct.eye_right_x = synchedPosesStruct.poses_3d[i][17 * 4 + 0];
                    bodyStruct.eye_right_y = synchedPosesStruct.poses_3d[i][17 * 4 + 1];
                    bodyStruct.eye_right_z = zCorrection(synchedPosesStruct.poses_3d[i][17 * 4 + 2]);
                    bodyStruct.eye_right_conf = synchedPosesStruct.poses_3d[i][17 * 4 + 3];
                    bodyStruct.ear_right_x = synchedPosesStruct.poses_3d[i][18 * 4 + 0];
                    bodyStruct.ear_right_y = synchedPosesStruct.poses_3d[i][18 * 4 + 1];
                    bodyStruct.ear_right_z = zCorrection(synchedPosesStruct.poses_3d[i][18 * 4 + 2]);
                    bodyStruct.ear_right_conf = synchedPosesStruct.poses_3d[i][18 * 4 + 3];

                    auto to2D = [sz1depth](float x) -> int16_t {
                        // std::cerr << "to2D: value = " << x << std::endl << std::flush;
                        auto x2 = x;
                        return std::min(std::max(int64_t(0L), int64_t(x2 )), int64_t(sz1depth - 1L)); // 1280
                    };

                    auto to2Dy = [sz2depth](float x) -> int16_t {
                        // std::cerr << "to2D/y: value = " << x << std::endl << std::flush;
                        auto x2 = x;
                        return std::min(std::max(int64_t(0L), int64_t(x2 )), int64_t(sz2depth-1L));
                    };

                    auto to2Dd = [](float x) -> int16_t {
                        // std::cerr << "to2D/d: value = " << x << std::endl << std::flush;
                        return int64_t(x);
                    };

                    bodyStruct.neck_2d_x = to2D(synchedPosesStruct.poses_2d[i][0 * 3 + 0]);
                    bodyStruct.neck_2d_y = to2Dy(synchedPosesStruct.poses_2d[i][0 * 3 + 1]);
                    bodyStruct.neck_2d_depth = to2Dd(frameDepthMat.at<ushort>(bodyStruct.neck_2d_y,bodyStruct.neck_2d_x));
                    bodyStruct.neck_2d_conf = synchedPosesStruct.poses_2d[i][0 * 3 + 2];
                    bodyStruct.nose_2d_x = to2D(synchedPosesStruct.poses_2d[i][1 * 3 + 0]);
                    bodyStruct.nose_2d_y = to2Dy(synchedPosesStruct.poses_2d[i][1 * 3 + 1]);
                    bodyStruct.nose_2d_depth = to2Dd(frameDepthMat.at<ushort>(bodyStruct.nose_2d_y,bodyStruct.nose_2d_x));
                    bodyStruct.nose_2d_conf = synchedPosesStruct.poses_2d[i][1 * 3 + 2];
                    bodyStruct.pelvis_2d_x = to2D(synchedPosesStruct.poses_2d[i][2 * 3 + 0]);
                    bodyStruct.pelvis_2d_y = to2Dy(synchedPosesStruct.poses_2d[i][2 * 3 + 1]);
                    bodyStruct.pelvis_2d_depth = to2Dd(frameDepthMat.at<ushort>(bodyStruct.pelvis_2d_y,bodyStruct.pelvis_2d_x));
                    bodyStruct.pelvis_2d_conf = synchedPosesStruct.poses_2d[i][2 * 3 + 2];
                    bodyStruct.shoulder_left_2d_x = to2D(synchedPosesStruct.poses_2d[i][3 * 3 + 0]);
                    bodyStruct.shoulder_left_2d_y = to2Dy(synchedPosesStruct.poses_2d[i][3 * 3 + 1]);
                    bodyStruct.shoulder_left_2d_depth = to2Dd(frameDepthMat.at<ushort>(bodyStruct.shoulder_left_2d_y,bodyStruct.shoulder_left_2d_x));
                    bodyStruct.shoulder_left_2d_conf = synchedPosesStruct.poses_2d[i][3 * 3 + 2];
                    bodyStruct.elbow_left_2d_x = to2D(synchedPosesStruct.poses_2d[i][4 * 3 + 0]);
                    bodyStruct.elbow_left_2d_y = to2Dy(synchedPosesStruct.poses_2d[i][4 * 3 + 1]);
                    bodyStruct.elbow_left_2d_depth = to2Dd(frameDepthMat.at<ushort>(bodyStruct.elbow_left_2d_y,bodyStruct.elbow_left_2d_x));
                    bodyStruct.elbow_left_2d_conf = synchedPosesStruct.poses_2d[i][4 * 3 + 2];
                    bodyStruct.wrist_left_2d_x = to2D(synchedPosesStruct.poses_2d[i][5 * 3 + 0]);
                    bodyStruct.wrist_left_2d_y = to2Dy(synchedPosesStruct.poses_2d[i][5 * 3 + 1]);
                    bodyStruct.wrist_left_2d_depth = to2Dd(frameDepthMat.at<ushort>(bodyStruct.wrist_left_2d_y,bodyStruct.wrist_left_2d_x));
                    bodyStruct.wrist_left_2d_conf = synchedPosesStruct.poses_2d[i][5 * 3 + 2];
                    bodyStruct.hip_left_2d_x = to2D(synchedPosesStruct.poses_2d[i][6 * 3 + 0]);
                    bodyStruct.hip_left_2d_y = to2Dy(synchedPosesStruct.poses_2d[i][6 * 3 + 1]);
                    std::cerr << bodyStruct.hip_left_2d_y << " " << bodyStruct.hip_left_2d_x << std::endl << std::flush;
                    bodyStruct.hip_left_2d_depth = to2Dd(frameDepthMat.at<ushort>(bodyStruct.hip_left_2d_y,bodyStruct.hip_left_2d_x));
                    bodyStruct.hip_left_2d_conf = synchedPosesStruct.poses_2d[i][6 * 3 + 2];
                    bodyStruct.knee_left_2d_x = to2D(synchedPosesStruct.poses_2d[i][7 * 3 + 0]);
                    bodyStruct.knee_left_2d_y = to2Dy(synchedPosesStruct.poses_2d[i][7 * 3 + 1]);
                    std::cerr << bodyStruct.knee_left_2d_y << " " << bodyStruct.knee_left_2d_x << std::endl << std::flush;
                    bodyStruct.knee_left_2d_depth = to2Dd(frameDepthMat.at<ushort>(bodyStruct.knee_left_2d_y,bodyStruct.knee_left_2d_x));
                    bodyStruct.knee_left_2d_conf = synchedPosesStruct.poses_2d[i][7 * 3 + 2];
                    bodyStruct.ankle_left_2d_x = to2D(synchedPosesStruct.poses_2d[i][8 * 3 + 0]);
                    bodyStruct.ankle_left_2d_y = to2Dy(synchedPosesStruct.poses_2d[i][8 * 3 + 1]);
                    bodyStruct.ankle_left_2d_depth = to2Dd(frameDepthMat.at<ushort>(bodyStruct.ankle_left_2d_y,bodyStruct.ankle_left_2d_x));
                    bodyStruct.ankle_left_2d_conf = synchedPosesStruct.poses_2d[i][8 * 3 + 2];
                    bodyStruct.shoulder_right_2d_x = to2D(synchedPosesStruct.poses_2d[i][9 * 3 + 0]);
                    bodyStruct.shoulder_right_2d_y = to2Dy(synchedPosesStruct.poses_2d[i][9 * 3 + 1]);
                    bodyStruct.shoulder_right_2d_depth = to2Dd(frameDepthMat.at<ushort>(bodyStruct.shoulder_right_2d_y,bodyStruct.shoulder_right_2d_x));
                    bodyStruct.shoulder_right_2d_conf = synchedPosesStruct.poses_2d[i][9 * 3 + 2];
                    bodyStruct.elbow_right_2d_x = to2D(synchedPosesStruct.poses_2d[i][10 * 3 + 0]);
                    bodyStruct.elbow_right_2d_y = to2Dy(synchedPosesStruct.poses_2d[i][10 * 3 + 1]);
                    bodyStruct.elbow_right_2d_depth = to2Dd(frameDepthMat.at<ushort>(bodyStruct.elbow_right_2d_y,bodyStruct.elbow_right_2d_x));
                    bodyStruct.elbow_right_2d_conf = synchedPosesStruct.poses_2d[i][10 * 3 + 2];
                    bodyStruct.wrist_right_2d_x = to2D(synchedPosesStruct.poses_2d[i][11 * 3 + 0]);
                    bodyStruct.wrist_right_2d_y = to2Dy(synchedPosesStruct.poses_2d[i][11 * 3 + 1]);
                    bodyStruct.wrist_right_2d_depth = to2Dd(frameDepthMat.at<ushort>(bodyStruct.wrist_right_2d_y,bodyStruct.wrist_right_2d_x));
                    bodyStruct.wrist_right_2d_conf = synchedPosesStruct.poses_2d[i][11 * 3 + 2];
                    bodyStruct.hip_right_2d_x = to2D(synchedPosesStruct.poses_2d[i][12 * 3 + 0]);
                    bodyStruct.hip_right_2d_y = to2Dy(synchedPosesStruct.poses_2d[i][12 * 3 + 1]);
                    bodyStruct.hip_right_2d_depth = to2Dd(frameDepthMat.at<ushort>(bodyStruct.hip_right_2d_y,bodyStruct.hip_right_2d_x));
                    bodyStruct.hip_right_2d_conf = synchedPosesStruct.poses_2d[i][12 * 3 + 2];
                    bodyStruct.knee_right_2d_x = to2D(synchedPosesStruct.poses_2d[i][13 * 3 + 0]);
                    bodyStruct.knee_right_2d_y = to2Dy(synchedPosesStruct.poses_2d[i][13 * 3 + 1]);
                    bodyStruct.knee_right_2d_depth = to2Dd(frameDepthMat.at<ushort>(bodyStruct.knee_right_2d_y,bodyStruct.knee_right_2d_x));
                    bodyStruct.knee_right_2d_conf = synchedPosesStruct.poses_2d[i][13 * 3 + 2];
                    bodyStruct.ankle_right_2d_x = to2D(synchedPosesStruct.poses_2d[i][14 * 3 + 0]);
                    bodyStruct.ankle_right_2d_y = to2Dy(synchedPosesStruct.poses_2d[i][14 * 3 + 1]);
                    bodyStruct.ankle_right_2d_depth = to2Dd(frameDepthMat.at<ushort>(bodyStruct.ankle_right_2d_y,bodyStruct.ankle_right_2d_x));
                    bodyStruct.ankle_right_2d_conf = synchedPosesStruct.poses_2d[i][14 * 3 + 2];
                    bodyStruct.eye_left_2d_x = to2D(synchedPosesStruct.poses_2d[i][15 * 3 + 0]);
                    bodyStruct.eye_left_2d_y = to2Dy(synchedPosesStruct.poses_2d[i][15 * 3 + 1]);
                    bodyStruct.eye_left_2d_depth = to2Dd(frameDepthMat.at<ushort>(bodyStruct.eye_left_2d_y,bodyStruct.eye_left_2d_x));
                    bodyStruct.eye_left_2d_conf = synchedPosesStruct.poses_2d[i][15 * 3 + 2];
                    bodyStruct.ear_left_2d_x = to2D(synchedPosesStruct.poses_2d[i][16 * 3 + 0]);
                    bodyStruct.ear_left_2d_y = to2Dy(synchedPosesStruct.poses_2d[i][16 * 3 + 1]);
                    bodyStruct.ear_left_2d_depth = to2Dd(frameDepthMat.at<ushort>(bodyStruct.ear_left_2d_y,bodyStruct.ear_left_2d_x));
                    bodyStruct.ear_left_2d_conf = synchedPosesStruct.poses_2d[i][16 * 3 + 2];
                    bodyStruct.eye_right_2d_x = to2D(synchedPosesStruct.poses_2d[i][17 * 3 + 0]);
                    bodyStruct.eye_right_2d_y = to2Dy(synchedPosesStruct.poses_2d[i][17 * 3 + 1]);
                    bodyStruct.eye_right_2d_depth = to2Dd(frameDepthMat.at<ushort>(bodyStruct.eye_right_2d_y,bodyStruct.eye_right_2d_x));
                    bodyStruct.eye_right_2d_conf = synchedPosesStruct.poses_2d[i][17 * 3 + 2];
                    bodyStruct.ear_right_2d_x = to2D(synchedPosesStruct.poses_2d[i][18 * 3 + 0]);
                    bodyStruct.ear_right_2d_y = to2Dy(synchedPosesStruct.poses_2d[i][18 * 3 + 1]);
                    bodyStruct.ear_right_2d_depth = to2Dd(frameDepthMat.at<ushort>(bodyStruct.ear_right_2d_y,bodyStruct.ear_right_2d_x));
                    bodyStruct.ear_right_2d_conf = synchedPosesStruct.poses_2d[i][18 * 3 + 2];

                    //Now we find the depth value to use to mark where the detected human body is located, depends on confidence of 2D joints and the amount of stereo depth regions
                    uint16_t medianPointDepth = 0;     //This will store the end depth value
                    bool foundGoodDepth = false;        //This will be used to stop the conditionals when good depth is found
                    int usedXPoint;                     //This will save what x point was used (for visualization purposes)
                    int usedYPoint;                     //This will save what y point was used (for visualization purposes)
                    int usedRadius;                     //This will save what radius was used (for visualization purposes)

                    // If there is good depth around neck, use that
                    if(bodyStruct.neck_2d_conf > 0)
                    {
                        auto nonZeroVector = returnVectorOfNonZeroValuesInRoiXlinkFull(frameDepthMat, bodyStruct.neck_2d_x, bodyStruct.neck_2d_y, 6);
                        if (nonZeroVector.size() > 9)
                        {
                            medianPointDepth = findMedianXlinkFull(nonZeroVector, nonZeroVector.size());
                            usedXPoint = bodyStruct.neck_2d_x;
                            usedYPoint = bodyStruct.neck_2d_y;
                            usedRadius = 6;
                            foundGoodDepth = true;
                        }
                        std::cerr << "OPTION 1 TRIGGERED (neck)" << std::endl << std::flush;
                    }
                    // If there is good depth in between shoulders, use that
                    if (bodyStruct.shoulder_left_2d_conf > 0 && bodyStruct.shoulder_right_2d_conf > 0 && !foundGoodDepth)
                    {
                        int xPoint = (bodyStruct.shoulder_left_2d_x + bodyStruct.shoulder_right_2d_x)/2;
                        int yPoint = (bodyStruct.shoulder_left_2d_y + bodyStruct.shoulder_right_2d_y)/2;

                        auto nonZeroVector = returnVectorOfNonZeroValuesInRoiXlinkFull(frameDepthMat, xPoint, yPoint, 6);
                        if (nonZeroVector.size() > 9)
                        {
                            medianPointDepth = findMedianXlinkFull(nonZeroVector, nonZeroVector.size());
                            usedXPoint = xPoint;
                            usedYPoint = yPoint;
                            usedRadius = 6;
                            foundGoodDepth = true;
                        }
                        std::cerr << "OPTION 2 TRIGGERED (between shoulders)" << std::endl << std::flush;
                    }
                    //If there is good depth in the center of shoulders and hips, use that, increase the ROI size 2 times if needed
                    if (bodyStruct.hip_left_2d_conf > 0 && bodyStruct.hip_right_2d_conf > 0 && bodyStruct.shoulder_left_2d_conf > 0 && bodyStruct.shoulder_right_2d_conf && !foundGoodDepth)
                    {
                        int xPoint1 = (bodyStruct.shoulder_left_2d_x + bodyStruct.shoulder_right_2d_x)/2;
                        int yPoint1 = (bodyStruct.shoulder_left_2d_y + bodyStruct.shoulder_right_2d_y)/2;
                        int xPoint2 = (bodyStruct.hip_left_2d_x + bodyStruct.hip_right_2d_x)/2;
                        int yPoint2 = (bodyStruct.hip_left_2d_y + bodyStruct.hip_right_2d_y)/2;
                        int xPoint3 = (xPoint1 + xPoint2)/2;
                        int yPoint3 = (yPoint1 + yPoint2)/2;
                        bool foundDepth = false;

                        auto nonZeroVector = returnVectorOfNonZeroValuesInRoiXlinkFull(frameDepthMat, xPoint3, yPoint3, 6);
                        if (nonZeroVector.size() > 9)
                        {
                            medianPointDepth = findMedianXlinkFull(nonZeroVector, nonZeroVector.size());
                            usedXPoint = xPoint3;
                            usedYPoint = yPoint3;
                            usedRadius = 6;
                            foundDepth = true;
                            foundGoodDepth = true;
                            std::cerr << "OPTION 3A TRIGGERED (between shoulders and hips) - 6 roi" << std::endl << std::flush;
                        }
                        else
                        {
                            nonZeroVector = returnVectorOfNonZeroValuesInRoiXlinkFull(frameDepthMat, xPoint3, yPoint3, 12);
                        }
                        if (nonZeroVector.size() > 9 && !foundDepth)
                        {
                            medianPointDepth = findMedianXlinkFull(nonZeroVector, nonZeroVector.size());
                            usedXPoint = xPoint3;
                            usedYPoint = yPoint3;
                            usedRadius = 12;
                            foundDepth = true;
                            foundGoodDepth = true;
                            std::cerr << "OPTION 3B TRIGGERED (between shoulders and hips) - 12 roi" << std::endl << std::flush;
                        }
                        else
                        {
                            nonZeroVector = returnVectorOfNonZeroValuesInRoiXlinkFull(frameDepthMat, xPoint3, yPoint3, 18);
                        }
                        if (nonZeroVector.size() > 9 && !foundDepth)
                        {
                            medianPointDepth = findMedianXlinkFull(nonZeroVector, nonZeroVector.size());
                            usedXPoint = xPoint3;
                            usedYPoint = yPoint3;
                            usedRadius = 18;
                            std::cerr << "OPTION 3C TRIGGERED (between shoulders and hips) - 18 roi" << std::endl << std::flush;
                            foundGoodDepth = true;
                        }
                    }
                    if (!foundGoodDepth)
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

                        int xPoint = xValueOfDetectedJoints/countOfDetectedJoints;
                        int yPoint = yValueOfDetectedJoints/countOfDetectedJoints;

                        std::vector<uint16_t> nonZeroVector;
                        int roiRadius = 6;
                        bool noGoodReadingsAnywhere = false;
                        while (nonZeroVector.size() < 10)
                        {
                            nonZeroVector = returnVectorOfNonZeroValuesInRoiXlinkFull(frameDepthMat, xPoint, yPoint, roiRadius);
                            roiRadius += 6;
                            // There can be the case where there are 0 depth values anywhere reasonable in which case we must return 0
                            if (roiRadius > 60)
                            {
                                noGoodReadingsAnywhere = true;
                                break;
                            }
                        }
                        if (!noGoodReadingsAnywhere)
                        {
                            medianPointDepth = findMedianXlinkFull(nonZeroVector, nonZeroVector.size());
                            usedXPoint = xPoint;
                            usedYPoint = yPoint;
                            usedRadius = roiRadius;
                            foundGoodDepth = true;
                            std::cerr << "OPTION 4 TRIGGERED (all joints) - " << roiRadius <<  " roi radius" << std::endl << std::flush;
                        }
                        else
                        {
                            medianPointDepth = 0;
                            usedXPoint = frameDepthMat.cols/2;
                            usedYPoint = frameDepthMat.rows/2;
                            usedRadius = roiRadius;
                            foundGoodDepth = true;
                            std::cerr << "OPTION 5 TRIGGERED - NO GOOD DEPTH, RETURNING 0" << std::endl << std::flush;
                        }
                    }

                    // //Now we find the spatial coordinates of using StereoDepth and intrinsics
                    // //We first find the x,y coordinate in the image of the body that is most confident
                    // int xPoint;
                    // int yPoint;
                    // if(bodyStruct.neck_2d_conf > .5)
                    // {
                    //     xPoint = bodyStruct.neck_2d_x;
                    //     yPoint = bodyStruct.neck_2d_y;
                    // }
                    // //If neck not confident but shoulders are confident, choose half way point of shoulders
                    // else if (bodyStruct.shoulder_left_2d_conf > .5 && bodyStruct.shoulder_right_2d_conf > .5)
                    // {
                    //     xPoint = (bodyStruct.shoulder_left_2d_x + bodyStruct.shoulder_right_2d_x)/2;
                    //     yPoint = (bodyStruct.shoulder_left_2d_y + bodyStruct.shoulder_right_2d_y)/2;
                    // }
                    // //If that isn't true do midpoint between hips
                    // else if (bodyStruct.hip_left_2d_conf > .5 && bodyStruct.hip_right_2d_conf > .5)
                    // {
                    //     xPoint = (bodyStruct.hip_left_2d_x + bodyStruct.hip_right_2d_x)/2;
                    //     yPoint = (bodyStruct.hip_left_2d_y + bodyStruct.hip_right_2d_y)/2;
                    // }
                    // //If head, shoulders, hips aren't confident, then try nose
                    // else if (bodyStruct.nose_2d_conf > .5)
                    // {
                    //     xPoint = bodyStruct.nose_2d_x;
                    //     yPoint = bodyStruct.nose_2d_y;
                    // }
                    // //Finally, if none of those are confident, we will grab the average x/y location of all detected joints
                    // else
                    // {
                    //     int countOfDetectedJoints = 0;
                    //     int xValueOfDetectedJoints = 0;
                    //     int yValueOfDetectedJoints = 0;

                    //     if(bodyStruct.neck_2d_conf > 0)
                    //     {
                    //         countOfDetectedJoints++;
                    //         xValueOfDetectedJoints += bodyStruct.neck_2d_x;
                    //         yValueOfDetectedJoints += bodyStruct.neck_2d_y;
                    //     }
                    //     if(bodyStruct.nose_2d_conf > 0)
                    //     {
                    //         countOfDetectedJoints++;
                    //         xValueOfDetectedJoints += bodyStruct.nose_2d_x;
                    //         yValueOfDetectedJoints += bodyStruct.nose_2d_y;
                    //     }
                    //     // if(bodyStruct.pelvis_2d_conf > 0)
                    //     // {
                    //     //     countOfDetectedJoints++;
                    //     //     xValueOfDetectedJoints += bodyStruct.pelvis_2d_x;
                    //     //     yValueOfDetectedJoints += bodyStruct.pelvis_2d_y;
                    //     // }
                    //     if(bodyStruct.shoulder_left_2d_conf > 0)
                    //     {
                    //         countOfDetectedJoints++;
                    //         xValueOfDetectedJoints += bodyStruct.shoulder_left_2d_x;
                    //         yValueOfDetectedJoints += bodyStruct.shoulder_left_2d_y;
                    //     }
                    //     if(bodyStruct.elbow_left_2d_conf > 0)
                    //     {
                    //         countOfDetectedJoints++;
                    //         xValueOfDetectedJoints += bodyStruct.elbow_left_2d_x;
                    //         yValueOfDetectedJoints += bodyStruct.elbow_left_2d_y;
                    //     }
                    //     if(bodyStruct.wrist_left_2d_conf > 0)
                    //     {
                    //         countOfDetectedJoints++;
                    //         xValueOfDetectedJoints += bodyStruct.wrist_left_2d_x;
                    //         yValueOfDetectedJoints += bodyStruct.wrist_left_2d_y;
                    //     }
                    //     if(bodyStruct.hip_left_2d_conf > 0)
                    //     {
                    //         countOfDetectedJoints++;
                    //         xValueOfDetectedJoints += bodyStruct.hip_left_2d_x;
                    //         yValueOfDetectedJoints += bodyStruct.hip_left_2d_y;
                    //     }
                    //     if(bodyStruct.knee_left_2d_conf > 0)
                    //     {
                    //         countOfDetectedJoints++;
                    //         xValueOfDetectedJoints += bodyStruct.knee_left_2d_x;
                    //         yValueOfDetectedJoints += bodyStruct.knee_left_2d_y;
                    //     }
                    //     if(bodyStruct.ankle_left_2d_conf > 0)
                    //     {
                    //         countOfDetectedJoints++;
                    //         xValueOfDetectedJoints += bodyStruct.ankle_left_2d_x;
                    //         yValueOfDetectedJoints += bodyStruct.ankle_left_2d_y;
                    //     }
                    //     if(bodyStruct.shoulder_right_2d_conf > 0)
                    //     {
                    //         countOfDetectedJoints++;
                    //         xValueOfDetectedJoints += bodyStruct.shoulder_right_2d_x;
                    //         yValueOfDetectedJoints += bodyStruct.shoulder_right_2d_y;
                    //     }
                    //     if(bodyStruct.elbow_right_2d_conf > 0)
                    //     {
                    //         countOfDetectedJoints++;
                    //         xValueOfDetectedJoints += bodyStruct.elbow_right_2d_x;
                    //         yValueOfDetectedJoints += bodyStruct.elbow_right_2d_y;
                    //     }
                    //     if(bodyStruct.wrist_right_2d_conf > 0)
                    //     {
                    //         countOfDetectedJoints++;
                    //         xValueOfDetectedJoints += bodyStruct.wrist_right_2d_x;
                    //         yValueOfDetectedJoints += bodyStruct.wrist_right_2d_y;
                    //     }
                    //     if(bodyStruct.hip_right_2d_conf > 0)
                    //     {
                    //         countOfDetectedJoints++;
                    //         xValueOfDetectedJoints += bodyStruct.hip_right_2d_x;
                    //         yValueOfDetectedJoints += bodyStruct.hip_right_2d_y;
                    //     }
                    //     if(bodyStruct.knee_right_2d_conf > 0)
                    //     {
                    //         countOfDetectedJoints++;
                    //         xValueOfDetectedJoints += bodyStruct.knee_right_2d_x;
                    //         yValueOfDetectedJoints += bodyStruct.knee_right_2d_y;
                    //     }
                    //     if(bodyStruct.ankle_right_2d_conf > 0)
                    //     {
                    //         countOfDetectedJoints++;
                    //         xValueOfDetectedJoints += bodyStruct.ankle_right_2d_x;
                    //         yValueOfDetectedJoints += bodyStruct.ankle_right_2d_y;
                    //     }
                    //     if(bodyStruct.eye_left_2d_conf > 0)
                    //     {
                    //         countOfDetectedJoints++;
                    //         xValueOfDetectedJoints += bodyStruct.eye_left_2d_x;
                    //         yValueOfDetectedJoints += bodyStruct.eye_left_2d_y;
                    //     }
                    //     if(bodyStruct.ear_left_2d_conf > 0)
                    //     {
                    //         countOfDetectedJoints++;
                    //         xValueOfDetectedJoints += bodyStruct.ear_left_2d_x;
                    //         yValueOfDetectedJoints += bodyStruct.ear_left_2d_y;
                    //     }
                    //     if(bodyStruct.eye_right_2d_conf > 0)
                    //     {
                    //         countOfDetectedJoints++;
                    //         xValueOfDetectedJoints += bodyStruct.eye_right_2d_x;
                    //         yValueOfDetectedJoints += bodyStruct.eye_right_2d_y;
                    //     }
                    //     if(bodyStruct.ear_right_2d_conf > 0)
                    //     {
                    //         countOfDetectedJoints++;
                    //         xValueOfDetectedJoints += bodyStruct.ear_right_2d_x;
                    //         yValueOfDetectedJoints += bodyStruct.ear_right_2d_y;
                    //     }

                    //     xPoint = xValueOfDetectedJoints/countOfDetectedJoints;
                    //     yPoint = yValueOfDetectedJoints/countOfDetectedJoints;
                    // }

                    // //Region square radius
                    // int regionRadius = 6;

                    // //We grab a region of interest, we need to make sure it is not asking for pixels outside of the frame
                    // int xPointMin  = (xPoint - regionRadius >= 0) ? xPoint - regionRadius : 0;
                    // xPointMin  = (xPointMin <= frameDepthMat.cols) ? xPointMin : frameDepthMat.cols;
                    // int xPointMax  = (xPoint + regionRadius <= frameDepthMat.cols) ? xPoint + regionRadius : frameDepthMat.cols;
                    // xPointMax  = (xPointMax >= 0) ? xPointMax : 0;
                    // int yPointMin  = (yPoint - regionRadius >= 0) ? yPoint - regionRadius : 0;
                    // yPointMin  = (yPointMin <= frameDepthMat.rows) ? yPointMin : frameDepthMat.rows;
                    // int yPointMax   = (yPoint + regionRadius <= frameDepthMat.rows) ? yPoint + regionRadius : frameDepthMat.rows;
                    // yPointMax   = (yPointMax >= 0) ? yPointMax : 0;

                    // // std::cerr << "xPoint: " << xPoint << std::endl << std::flush;
                    // // std::cerr << "yPoint: " << yPoint << std::endl << std::flush;
                    // // std::cerr << "xPointMin: " << xPointMin << std::endl << std::flush;
                    // // std::cerr << "xPointMax: " << xPointMax << std::endl << std::flush;
                    // // std::cerr << "yPointMin: " << yPointMin << std::endl << std::flush;
                    // // std::cerr << "yPointMax: " << yPointMax << std::endl << std::flush;

                    // // //We grab a reference to the cropped image and calculate the mean, and then store it as pointDepth
                    // //Now we grab a the average depth value of a 12x12 grid of pixels surrounding the xPoint and yPoint
                    // cv::Scalar mean, stddev;
                    // cv::Rect myROI(cv::Point(xPointMin, yPointMin), cv::Point(xPointMax , yPointMax ));
                    // cv::Mat croppedDepth = frameDepthMat(myROI);
                    // cv::meanStdDev(croppedDepth, mean, stddev);
                    // int pointDepth = int(mean[0]);        

                    // // Now we will try to get the non-0 median
                    // // Keep changing until we get a non-0 point depth 2 value
                    //     // go neck bigger 3 times
                    //     // then between neck and pelvis
                    //     // then go bigger 3 times
                    //     // then go full torso, shoulder shoulder hip hip, entire thing
                    // cv::Rect myROI2(cv::Point(xPointMin, yPointMin), cv::Point(xPointMax , yPointMax ));
                    // cv::Mat croppedDepth2 = frameDepthMat(myROI2);
                    // std::vector<uint16_t> nonZeroDepthValues;
                    // // ushort table[];
                    // int limit = croppedDepth2.rows * croppedDepth2.cols;
                    // ushort* ptr = reinterpret_cast<ushort*>(croppedDepth2.data);
                    // if (!croppedDepth2.isContinuous())
                    // {
                    //     croppedDepth2 = croppedDepth2.clone();
                    // }
                    // // std::cerr << "ROI 2 - Limit: " << limit  << std::endl << std::flush;
                    // for (int i = 0; i < limit; i++, ptr++)
                    // {
                    //     // *ptr = table[*ptr];
                    //     // std::cerr << "ROI 2 - i: " << i  << "   |  *ptr value: " << *ptr  << std::endl << std::flush;
                    //     if(*ptr != 0)
                    //     {
                    //         nonZeroDepthValues.push_back((uint16_t)(*ptr));
                    //     }
                    // }
                    // uint16_t pointDepth2 = 0;
                    // if (nonZeroDepthValues.size() > 0)
                    //     pointDepth2 = findMedianXlinkFull(nonZeroDepthValues, nonZeroDepthValues.size());

                    //Now we use the HFOV and VFOV to find the x and y coordinates in millimeters
                    // https://github.com/luxonis/depthai-experiments/blob/377c50c13931a082825d457f69893c1bf3f24aa2/gen2-calc-spatials-on-host/calc.py#L10
                    int midDepthXCoordinate = frameDepthMat.cols / 2;    // middle of depth image x across (columns) - this is depth origin
                    int midDepthYCoordinate = frameDepthMat.rows / 2;    // middle of depth image y across (rows) - this is depth origin
                    
                    int xPointInDepthCenterCoordinates = usedXPoint - midDepthXCoordinate; //This is the xPoint but if the depth map center is the origin
                    int yPointInDepthCenterCoordinates = usedYPoint - midDepthYCoordinate; //This is the yPoint but if the depth map center is the origin

                    float angle_x = atan(tan(cameraHFOVInRadians / 2.0f) * float(xPointInDepthCenterCoordinates) / float(midDepthXCoordinate));
                    float angle_y = atan(tan(cameraHFOVInRadians / 2.0f) * float(yPointInDepthCenterCoordinates) / float(midDepthYCoordinate));

                    //We will save this depth value as the pelvis depth
                    bodyStruct.pelvis_2d_depth = medianPointDepth;
                    bodyStruct.pelvis_2d_x = int(float(medianPointDepth) * tan(angle_x));
                    bodyStruct.pelvis_2d_y = int(-1.0f * float(medianPointDepth) * tan(angle_y));

                    std::cerr << "NECK DEPTH: " << int(frameDepthMat.at<ushort>(bodyStruct.neck_2d_y,bodyStruct.neck_2d_x))  << std::endl << std::flush;
                    // std::cerr << "AVERAGE NECK DEPTH: " << pointDepth  << std::endl << std::flush; //UNCOMMENT
                    std::cerr << "MEDIAN NON-0 NECK DEPTH: " << medianPointDepth << std::endl << std::flush;
                    //Finally we copy the COCO body struct memory to the frame
                    bodyStruct.hton();
                    
                    memcpy(&s->frame[(i*sizeof(coco_human_t))+4], &bodyStruct, sizeof(coco_human_t));

                    // cv::Mat depthFrameColor;
                    // // cv::medianBlur(frameDepthMat, frameDepthMat, 25);
                    // cv::normalize(frameDepthMat, depthFrameColor, 255, 0, cv::NORM_INF, CV_8UC1);
                    // cv::equalizeHist(depthFrameColor, depthFrameColor);
                    // cv::applyColorMap(depthFrameColor, depthFrameColor, cv::COLORMAP_HOT);
                    // rectangle(depthFrameColor, cv::Point(usedXPoint - usedRadius, usedYPoint - usedRadius), cv::Point(usedXPoint + usedRadius, usedYPoint + usedRadius), cv::Scalar( 255, 0, 255 ), cv::FILLED, cv::LINE_8 );
                    // cv::imshow("depth", depthFrameColor);
                    // cv::waitKey(1);
                }

                //Now that we have copied all memory to the frame we can push it back
                // ALL THE TIME (HEARTBEAT) if (bodyCount > 0)
                {
                    if (stream_bodies)
                        current_frame_.push_back(s);
                }
                //We push the rgb frame to the back (this helps when running ssp_client_opencv)
                if (stream_rgb)
                    current_frame_.push_back(rgbFrame);

                } catch(...) {
                std::cerr << "TRY/CATCH CATCH AFTER PARSE POSES " << std::endl << std::flush;
                return;
            }

        } catch(std::exception &e) {
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
