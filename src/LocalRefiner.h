//
// Created by jojo on 01.01.20.
//

#ifndef RECONSTRUCTION3D_LOCALREFINER_H
#define RECONSTRUCTION3D_LOCALREFINER_H

#include "Frame.h"
#include "util/Parser.h"
#include "Open3D/Open3D.h"
namespace Reconstruction
{
    class LocalRefiner
    {
    public:
        static open3d::geometry::TriangleMesh refineAndCreateMesh(std::vector<std::reference_wrapper<Frame>> frameVectorRef, const Parser config);
    private:
        static open3d::geometry::TriangleMesh createMesh(FrameVector frameVector);
        static bool createRGBDImageFromFrame(const Frame frame, const Parser config, open3d::geometry::RGBDImage& rgbd);
    };
}



#endif //RECONSTRUCTION3D_LOCALREFINER_H
