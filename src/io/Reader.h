//
// Created by jojo on 06.12.19.
//

#ifndef BAMAPPING_READER_H
#define BAMAPPING_READER_H

#include "../Frame.h"
namespace Reconstruction
{
    namespace io
    {
        class Reader
        {
        public:
            static Reconstruction::FrameVector readITEFrames(const char *cam_file,const char *dataset_path);
        };
    }


}


#endif //BAMAPPING_READER_H
