/**
 * \file nvpipe_types.h @brief Types for NvPipe support
 */
// Created by amourao on 28-10-2019.
#pragma once

#include <NvPipe.h>
#include <memory>

namespace moetsi::ssp {

struct NVPipeDeleter {
  void operator()(NvPipe *ptr) const {
    if (ptr)
      NvPipe_Destroy(ptr);
  }
};

typedef std::unique_ptr<NvPipe, NVPipeDeleter> NvPipeSafeP;

} // namespace moetsi::ssp
