//
// Created by jojo on 07.01.20.
//

#include "GeometryMethods.h"

using namespace Reconstruction;

bool GeometryMethods::createRGBDImageFromFrame(const Frame frame, Parser config, open3d::geometry::RGBDImage &rgbd, bool useIRImg)
{
    using namespace open3d;
    geometry::Image depth;
    geometry::Image infraRed;
    geometry::Image rgb;
    double depth_factor = config.getValue<double>("depth_factor");
    double depth_truncate = config.getValue<double>("depth_truncate");

    bool read = false;
    read = io::ReadImage(frame.getDepthImagePath().c_str(), depth);
    if(!read)
        return false;
    if(useIRImg)
    {
        read = io::ReadImageFromPNG(frame.getInfraRedImagePath().c_str(),infraRed);
        if(!read)
            return false;
        rgbd = *geometry::RGBDImage::CreateFromColorAndDepth(
                infraRed, depth, depth_factor,
                depth_truncate, true);
    }
    else
    {
        read = io::ReadImage(frame.getRGBImagePath().c_str(), rgb);
        if(!read)
            return false;

        rgbd = *geometry::RGBDImage::CreateFromColorAndDepth(
                rgb, depth, depth_factor,
                depth_truncate, false);
    }

    return true;
}

bool GeometryMethods::createMeshFromFrames(
        const FrameVector frameVector,
        Parser config,
        std::shared_ptr<open3d::geometry::TriangleMesh>& mesh,
        bool color)
{
    using namespace open3d;
    if(frameVector.empty())
        return false;
    double voxel_size = config.getValue<double>("volume_size")/config.getValue<double>("resolution");
    double sdf_trunc = config.getValue<double>("sdf_trunc");
    integration::TSDFVolumeColorType type;
    if(color)
    {
        type = integration::TSDFVolumeColorType::RGB8;
    }
    else
    {
        type = integration::TSDFVolumeColorType::Gray32;
    }
    integration::ScalableTSDFVolume volume(voxel_size,sdf_trunc,
                                           type);
    auto Twc0 = frameVector[0].getConstTwc(); //set frame 0 as base

    for(auto frame : frameVector)
    {
        open3d::geometry::RGBDImage rgbd;
        bool success = createRGBDImageFromFrame(frame,config,rgbd,!color);
        if(!success)
            break;

        int width = config.getValue<int>("Camera.width");
        int height = config.getValue<int>("Camera.height");
        double fx = config.getValue<double>("Camera.fx");
        double fy = config.getValue<double>("Camera.fy");
        double cx = config.getValue<double>("Camera.cx");
        double cy = config.getValue<double>("Camera.cy");

        camera::PinholeCameraIntrinsic intrinsic;

        intrinsic.SetIntrinsics(width,height,fx,fy,cx,cy);
        auto extrinsic = frame.getConstTcw() * Twc0;

        volume.Integrate(rgbd,intrinsic,extrinsic);
    }
    mesh = volume.ExtractTriangleMesh();
    if(mesh->vertices_.empty())
        return false;
    else
        return true;
}

bool GeometryMethods::createMeshAndPointCloudFromFrames(const FrameVector frameVector, Parser config,
                                                        std::shared_ptr<open3d::geometry::TriangleMesh> &mesh,
                                                        std::shared_ptr<open3d::geometry::PointCloud>& pcd,
                                                        bool color)
{
    using namespace open3d;
    double voxel_size = config.getValue<double>("volume_size")/config.getValue<double>("resolution");
    double sdf_trunc = config.getValue<double>("sdf_trunc");
    integration::TSDFVolumeColorType type;
    if(color)
    {
        type = integration::TSDFVolumeColorType::RGB8;
    }
    else
    {
        type = integration::TSDFVolumeColorType::Gray32;
    }
    integration::ScalableTSDFVolume volume(voxel_size,sdf_trunc,
                                           type);
    for(auto frame : frameVector)
    {
        open3d::geometry::RGBDImage rgbd;
        bool success = createRGBDImageFromFrame(frame,config,rgbd,!color);
        if(!success)
            break;

        int width = config.getValue<int>("Camera.width");
        int height = config.getValue<int>("Camera.height");
        double fx = config.getValue<double>("Camera.fx");
        double fy = config.getValue<double>("Camera.fy");
        double cx = config.getValue<double>("Camera.cx");
        double cy = config.getValue<double>("Camera.cy");

        camera::PinholeCameraIntrinsic intrinsic;

        intrinsic.SetIntrinsics(width,height,fx,fy,cx,cy);
        auto extrinsic = frame.getConstTcw();
        volume.Integrate(rgbd,intrinsic,extrinsic);
    }
    mesh = volume.ExtractTriangleMesh();
    pcd = volume.ExtractPointCloud();
    if(mesh->vertices_.empty())
        return false;
    else
        return true;
}

bool GeometryMethods::createPointCloundFromFrames(const FrameVector frameVector, Parser config,
                                                  std::shared_ptr<open3d::geometry::PointCloud> &pcd, bool color)
{
    using namespace open3d;
    if(frameVector.empty())
        return false;

    double voxel_size = config.getValue<double>("volume_size")/config.getValue<double>("resolution");
    double sdf_trunc = config.getValue<double>("sdf_trunc");
    integration::TSDFVolumeColorType type;
    if(color)
    {
        type = integration::TSDFVolumeColorType::RGB8;
    }
    else
    {
        type = integration::TSDFVolumeColorType::Gray32;
    }
    integration::ScalableTSDFVolume volume(voxel_size,sdf_trunc,
                                           type);
    auto Twc0 = frameVector[0].getConstTwc(); //set frame 0 as base
    for(auto frame : frameVector)
    {
        open3d::geometry::RGBDImage rgbd;
        bool success = createRGBDImageFromFrame(frame,config,rgbd,!color);
        if(!success)
            break;

        int width = config.getValue<int>("Camera.width");
        int height = config.getValue<int>("Camera.height");
        double fx = config.getValue<double>("Camera.fx");
        double fy = config.getValue<double>("Camera.fy");
        double cx = config.getValue<double>("Camera.cx");
        double cy = config.getValue<double>("Camera.cy");

        camera::PinholeCameraIntrinsic intrinsic;

        intrinsic.SetIntrinsics(width,height,fx,fy,cx,cy);
        auto extrinsic = frame.getConstTcw() * Twc0;

        volume.Integrate(rgbd,intrinsic,extrinsic);
    }
    pcd = volume.ExtractPointCloud();
    if(pcd->points_.empty())
        return false;
    else
        return true;
}
