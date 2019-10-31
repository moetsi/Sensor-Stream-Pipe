//
// Created by amourao on 28-10-2019.
//

#include <NvPipe.h>
#include <memory>

#pragma once

struct NVPipeDeleter {
  void operator()(NvPipe *ptr) const {
    if (ptr)
      NvPipe_Destroy(ptr);
  }
};

typedef std::unique_ptr<NvPipe, NVPipeDeleter> NvPipeSafeP;
