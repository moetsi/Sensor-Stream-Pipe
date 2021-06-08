//
// Created by David Geldreich on 1-06-2021.
//

#include "iphone_reader.h"

#include <opencv2/imgproc.hpp>

#import <ARKit/ARKit.h>
#include <mach/mach_init.h>
#include <mach/mach_error.h>
#include <mach/semaphore.h>
#include <mach/task.h>

using namespace std;

@interface SessionDelegate : NSObject<ARSessionDelegate>
{
  @public semaphore_t _semaphore;
  @public pthread_mutex_t _mutex;
  @public CVPixelBufferRef _pixelBuffer;
  @public CVPixelBufferRef _depthBuffer;
  @public CVPixelBufferRef _confidenceBuffer;
  @public unsigned long _timestamp;
}
@end

@implementation SessionDelegate

- (instancetype)init
{
    self = [super init];
    if (self)
    {
      semaphore_create(mach_task_self(), &_semaphore, SYNC_POLICY_FIFO, 0);
      pthread_mutex_init(&_mutex, NULL);
      _pixelBuffer = nil;
      _depthBuffer = nil;
      _confidenceBuffer = nil;
      _timestamp = 0;
    }
    
    return self;
}

- (void)session:(ARSession *)session didUpdateFrame:(ARFrame *)frame
{
  pthread_mutex_lock(&_mutex);
  CVPixelBufferRelease(_pixelBuffer);
  _pixelBuffer = CVPixelBufferRetain(frame.capturedImage);
  _timestamp = CurrentTimeMs();
    
  if (@available(iOS 14.0, *))
  {
    if (frame.sceneDepth != nil)
    {
      CVPixelBufferRelease(_depthBuffer);
      _depthBuffer = CVPixelBufferRetain(frame.sceneDepth.depthMap);
      CVPixelBufferRelease(_confidenceBuffer);
      _confidenceBuffer = CVPixelBufferRetain(frame.sceneDepth.confidenceMap);
    }
  }
  pthread_mutex_unlock(&_mutex);

  semaphore_signal(_semaphore);
}
@end

class iPhoneReaderImpl
{
public:
  ARSession* session;
  SessionDelegate* delegate;
  unsigned int fps;
  std::shared_ptr<FrameStruct> image;
  std::shared_ptr<FrameStruct> depth;
  std::shared_ptr<FrameStruct> confidence;
};

iPhoneReader::iPhoneReader()
{
  pImpl = new iPhoneReaderImpl;
  
  frame_template_.frame_id = 0;
  frame_template_.device_id = 0;
  
  frame_template_.message_type = 0;
  
  frame_template_.frame_data_type = 0;
  frame_template_.scene_desc = "iphone";
  frame_template_.stream_id = RandomString(16);
  
  pImpl->image = std::shared_ptr<FrameStruct>(new FrameStruct(frame_template_));
  pImpl->image->sensor_id = 0;
  pImpl->image->frame_type = 0;      // image
  pImpl->image->frame_data_type = 6; // YUV
  pImpl->image->timestamps.push_back(CurrentTimeMs());
  pImpl->image->timestamps.push_back(CurrentTimeMs());

  pImpl->depth = std::shared_ptr<FrameStruct>(new FrameStruct(frame_template_));
  pImpl->depth->sensor_id = 1;
  pImpl->depth->frame_type = 1;      // depth
  pImpl->depth->frame_data_type = 5; // float
  pImpl->depth->timestamps.push_back(CurrentTimeMs());
  pImpl->depth->timestamps.push_back(CurrentTimeMs());
  
  pImpl->confidence = std::shared_ptr<FrameStruct>(new FrameStruct(frame_template_));
  pImpl->confidence->sensor_id = 2;
  pImpl->confidence->frame_type = 3;      // confidence
  pImpl->confidence->frame_data_type = 7; // U8C1
  pImpl->confidence->timestamps.push_back(CurrentTimeMs());
  pImpl->confidence->timestamps.push_back(CurrentTimeMs());

  @autoreleasepool
  {
    pImpl->session = [ARSession new];

    // Need WorldTracking to get reconstruction of depth buffer from LiDAR
    // but disable unused features to lower power usage
    ARWorldTrackingConfiguration* configuration = [ARWorldTrackingConfiguration new];
    if (@available(iOS 13.0, *))
    {
      configuration.collaborationEnabled = NO;
      configuration.userFaceTrackingEnabled = NO;
      configuration.wantsHDREnvironmentTextures = NO;
    }
    if (@available(iOS 13.4, *))
      configuration.sceneReconstruction = ARSceneReconstructionNone;
    if (@available(iOS 14.3, *))
      configuration.appClipCodeTrackingEnabled = NO;
    if (@available(iOS 12.0, *))
    {
      configuration.environmentTexturing = AREnvironmentTexturingNone;
      configuration.maximumNumberOfTrackedImages = 0;
    }
    configuration.planeDetection = ARPlaneDetectionNone;
    configuration.lightEstimationEnabled = NO;
    configuration.providesAudioData = NO;

    pImpl->delegate = [[SessionDelegate alloc] init];
    pImpl->session.delegate = pImpl->delegate;

    pImpl->fps = 60;
    
    if (@available(iOS 11.3, *))
      pImpl->fps = (unsigned int)configuration.videoFormat.framesPerSecond;
    
    // Depth is only supported on iOS 14 and above
    if (@available(iOS 14.0, *))
    {
      if ([ARWorldTrackingConfiguration supportsFrameSemantics:ARFrameSemanticSceneDepth])
      {
        configuration.frameSemantics = ARFrameSemanticSceneDepth;
        spdlog::debug("Adding SceneDepth to configuration");
      }
    }
    
    [pImpl->session runWithConfiguration:configuration];
  }
}

iPhoneReader::~iPhoneReader()
{
  [pImpl->session pause];
  pImpl->session = nil;
  pImpl->delegate = nil;
  delete pImpl;
  pImpl = nullptr;
}

void iPhoneReader::Reset()
{
}

bool iPhoneReader::HasNextFrame()
{
  // Block until a new frame is available
  semaphore_wait(pImpl->delegate->_semaphore);
  return true;
}

void iPhoneReader::NextFrame()
{
}

vector<shared_ptr<FrameStruct>> iPhoneReader::GetCurrentFrame()
{
  vector<shared_ptr<FrameStruct>> res;
  
  // Copy data from capture thread
  pthread_mutex_lock(&pImpl->delegate->_mutex);
  
  CVPixelBufferRef pixelBuffer = pImpl->delegate->_pixelBuffer;
  if (pixelBuffer != nil && CVPixelBufferIsPlanar(pixelBuffer) &&
      CVPixelBufferGetPixelFormatType(pixelBuffer) == kCVPixelFormatType_420YpCbCr8BiPlanarFullRange &&
      CVPixelBufferGetPlaneCount(pixelBuffer) >= 2)
  {
    std::shared_ptr<FrameStruct> s = pImpl->image;
    
    int cols = (int)CVPixelBufferGetWidth(pixelBuffer);
    int rows = (int)CVPixelBufferGetHeight(pixelBuffer);
    
    size_t h_y = CVPixelBufferGetHeightOfPlane(pixelBuffer, 0);
    size_t bpr_y = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 0);
    size_t len_y = h_y * bpr_y;
    size_t h_uv = CVPixelBufferGetHeightOfPlane(pixelBuffer, 1);
    size_t bpr_uv = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 1);
    size_t len_uv = h_uv * bpr_uv;
    s->frame.resize(len_y + len_uv + 2 * sizeof(int));
    
    s->timestamps[0] = pImpl->delegate->_timestamp;
    s->timestamps[1] = CurrentTimeMs();

    memcpy(&s->frame[0], &cols, sizeof(int));
    memcpy(&s->frame[4], &rows, sizeof(int));

    CVPixelBufferLockBaseAddress(pixelBuffer, kCVPixelBufferLock_ReadOnly);
    // Y part
    void *baseaddress = CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 0);
    memcpy(&s->frame[8], baseaddress, len_y);
    // UV/CbCr part
    baseaddress = CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 1);
    memcpy(&s->frame[8+len_y], baseaddress, len_uv);
    CVPixelBufferUnlockBaseAddress(pixelBuffer, kCVPixelBufferLock_ReadOnly);

    res.push_back(s);
  }
  
  if (pImpl->delegate->_depthBuffer != nil &&
      CVPixelBufferGetPixelFormatType(pImpl->delegate->_depthBuffer) == kCVPixelFormatType_DepthFloat32)
  {
    CVPixelBufferRef depthBuffer = pImpl->delegate->_depthBuffer;
    
    std::shared_ptr<FrameStruct> s = pImpl->depth;
    int cols = (int)CVPixelBufferGetWidth(depthBuffer);
    int rows = (int)CVPixelBufferGetHeight(depthBuffer);
    size_t size = cols*rows*sizeof(float);
    s->frame.resize(size + 2 * sizeof(int));

    s->timestamps[0] = pImpl->delegate->_timestamp;
    s->timestamps[1] = CurrentTimeMs();
    
    memcpy(&s->frame[0], &cols, sizeof(int));
    memcpy(&s->frame[4], &rows, sizeof(int));
    
    CVPixelBufferLockBaseAddress(depthBuffer, kCVPixelBufferLock_ReadOnly);
    void *baseaddress = CVPixelBufferGetBaseAddress(depthBuffer);
    memcpy(&s->frame[8], baseaddress, size);
    CVPixelBufferUnlockBaseAddress(depthBuffer, kCVPixelBufferLock_ReadOnly);
    
    res.push_back(s);
  }

  if (pImpl->delegate->_confidenceBuffer != nil &&
      CVPixelBufferGetPixelFormatType(pImpl->delegate->_confidenceBuffer) == kCVPixelFormatType_OneComponent8)
  {
    CVPixelBufferRef confidenceBuffer = pImpl->delegate->_confidenceBuffer;
    
    std::shared_ptr<FrameStruct> s = pImpl->confidence;
    int cols = (int)CVPixelBufferGetWidth(confidenceBuffer);
    int rows = (int)CVPixelBufferGetHeight(confidenceBuffer);
    size_t size = cols*rows*sizeof(unsigned char);
    s->frame.resize(size + 2 * sizeof(int));

    s->timestamps[0] = pImpl->delegate->_timestamp;
    s->timestamps[1] = CurrentTimeMs();
    
    memcpy(&s->frame[0], &cols, sizeof(int));
    memcpy(&s->frame[4], &rows, sizeof(int));
    
    CVPixelBufferLockBaseAddress(confidenceBuffer, kCVPixelBufferLock_ReadOnly);
    void *baseaddress = CVPixelBufferGetBaseAddress(confidenceBuffer);
    memcpy(&s->frame[8], baseaddress, size);
    CVPixelBufferUnlockBaseAddress(confidenceBuffer, kCVPixelBufferLock_ReadOnly);
    
    res.push_back(s);
  }

  pthread_mutex_unlock(&pImpl->delegate->_mutex);
  
  return res;
}

unsigned int iPhoneReader::GetCurrentFrameId()
{
  return 0;
}

void iPhoneReader::GoToFrame(unsigned int frame_id)
{
}

unsigned int iPhoneReader::GetFps()
{
  return pImpl->fps;
}

vector<unsigned int> iPhoneReader::GetType()
{
  vector<unsigned int> res;
  res.push_back(0);
  
  if (@available(iOS 14.0, *))
  {
    if ([ARWorldTrackingConfiguration supportsFrameSemantics:ARFrameSemanticSceneDepth])
    {
      res.push_back(1); // Depth
      res.push_back(3); // Confidence
    }
  }
  return res;
}
