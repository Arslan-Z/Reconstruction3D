//
// Created by jojo on 01.01.20.
//

#include "LocalRefiner.h"
#include "GeometryMethods.h"
using namespace Reconstruction;

open3d::geometry::TriangleMesh  LocalRefiner::refineAndCreateMesh(std::vector<std::reference_wrapper<Frame>> frameVectorRef, Parser config)
{
    using namespace open3d;
    std::vector<std::shared_ptr<geometry::RGBDImage>> rgbdVec;
    camera::PinholeCameraTrajectory camera;
    FrameVector frameVector;
    for(auto frame : frameVectorRef)
    {
        geometry::RGBDImage rgbd;
        bool success = GeometryMethods::createRGBDImageFromFrame(frame,config,rgbd,false);
        if(!success)
            break;
        rgbdVec.push_back(std::make_shared<geometry::RGBDImage>(rgbd));
        frameVector.push_back(frame);
    }
    bool success = createCameraTrajectoryFromFrames(config,frameVector,camera);
    if(!success)
        return geometry::TriangleMesh();

    std::shared_ptr<geometry::TriangleMesh> mesh;
    Reconstruction::GeometryMethods::createMeshFromFrames(frameVector,config,mesh,true);

    auto option = color_map::ColorMapOptimizationOption();
    option.maximum_iteration_ = 300;
    option.non_rigid_camera_coordinate_ = true;

    visualization::DrawGeometries({mesh});
    utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);
    color_map::ColorMapOptimization(*mesh,rgbdVec,camera,option);
    visualization::DrawGeometries({mesh});


}


bool LocalRefiner::createCameraTrajectoryFromFrames(
        Parser config,
        const FrameVector frameVector,
        open3d::camera::PinholeCameraTrajectory& camera)
{
    using namespace open3d;
    if(frameVector.empty())
        return false;
    camera.parameters_.clear();
    for(auto frame : frameVector)
    {
        auto param = camera::PinholeCameraParameters();
        int width = config.getValue<int>("Camera.width");
        int height = config.getValue<int>("Camera.height");
        double fx = config.getValue<double>("Camera.fx");
        double fy = config.getValue<double>("Camera.fy");
        double cx = config.getValue<double>("Camera.cx");
        double cy = config.getValue<double>("Camera.cy");

        camera::PinholeCameraIntrinsic intrinsic;
        Eigen::Matrix4d extrinsic;

        intrinsic.SetIntrinsics(width,height,fx,fy,cx,cy);
        extrinsic = frame.getConstTcw();

        param.intrinsic_ = intrinsic;
        param.extrinsic_ = extrinsic;

        camera.parameters_.push_back(param);
    }
    return true;

}
