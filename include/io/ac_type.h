#ifndef AC_TYPE_H
#define AC_TYPE_H

#include "common_include.h"

namespace pc {
  class ACFileType
  {
  public:
      std::shared_ptr<ACFileType> Ptr;

      int numVerts_, tableX_, tableY_, numHeaderByte_;
      float *pVertex_;
      short *pOrder_;

      ACFileType(const std::string& filename);
      ~ACFileType();
  };
}

#endif
