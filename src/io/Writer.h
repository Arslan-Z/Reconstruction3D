//
// Created by jojo on 04.01.20.
//

#ifndef RECONSTRUCTION3D_WRITER_H
#define RECONSTRUCTION3D_WRITER_H

#include "../Frame.h"
#include "../util/Parser.h"

#include "Open3D/Registration/PoseGraph.h"
#include "Open3D/IO/ClassIO/PoseGraphIO.h"
namespace Reconstruction
{
    class Writer
    {
    public:
        static void writeGlobalPoseGraphTo_ITE_Format(const FrameVector frameVector_global, const Parser config);
        static void writeLocalPoseGraphTo_ITE_Format(const open3d::registration::PoseGraph poseGraph_local,
                                                     const FrameVector frameVector_global,
                                                     const std::string camera_file,
                                                     const std::string image_paths_file,
                                                     size_t fragment_id,
                                                     Eigen::Matrix4d Twc0,
                                                     Parser config);
    };
}



#endif //RECONSTRUCTION3D_WRITER_H
