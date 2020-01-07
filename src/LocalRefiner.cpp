//
// Created by jojo on 01.01.20.
//

#include "LocalRefiner.h"

using namespace Reconstruction;

open3d::geometry::TriangleMesh  LocalRefiner::refineAndCreateMesh(std::vector<std::reference_wrapper<Frame>> frameVectorRef, Parser config)
{
    using namespace open3d;
}

bool LocalRefiner::createRGBDImageFromFrame(Frame frame, Parser config, open3d::geometry::RGBDImage& rgbd)
{
    using namespace open3d;
    geometry::Image depth;
    geometry::Image infraRed;
    double depth_factor = config.getValue<double>("depth_factor");
    double depth_truncate = config.getValue<double>("depth_truncate");

    bool read = false;
    read = io::ReadImage(frame.getDepthImagePath().c_str(), depth);
    if(!read)
        return false;
    read = io::ReadImageFromPNG(frame.getInfraRedImagePath().c_str(),infraRed);
    if(!read)
        return false;
    rgbd = *geometry::RGBDImage::CreateFromColorAndDepth(
            infraRed, depth, depth_factor,
            depth_truncate, true);
    return true;
}
