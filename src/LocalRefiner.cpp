//
// Created by jojo on 01.01.20.
//

#include "LocalRefiner.h"
#include "GeometryMethods.h"
using namespace Reconstruction;

open3d::geometry::PointCloud LocalRefiner::refineAndCreateMPointCloud(std::vector<std::reference_wrapper<Frame> > frameVectorRef, Parser config)
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
        return geometry::PointCloud();

    std::shared_ptr<geometry::TriangleMesh> mesh;
    std::shared_ptr<geometry::PointCloud> pcd(new geometry::PointCloud);

    GeometryMethods::createMeshFromFrames(frameVector,config,mesh,true);
    mesh->ComputeVertexNormals();

    auto max_itr = config.getValue<int>("max_itr");
    auto option = color_map::ColorMapOptimizationOption();
    option.maximum_iteration_ = max_itr;
    option.non_rigid_camera_coordinate_ = false;

    bool debug = config.getValue<bool>("debug_mode");
    if(debug) {
        utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);
    }

    color_map::ColorMapOptimization(*mesh,rgbdVec,camera,option);

    for(size_t frame_id = 0; frame_id < frameVectorRef.size(); frame_id++)
    {
        auto Tcw = camera.parameters_[frame_id].extrinsic_;
        Eigen::Affine3d Tcw_affine;
        Tcw_affine.matrix() = Tcw;
        frameVectorRef[frame_id].get().setFromAffine3d(Tcw_affine);
//        frameVector[frame_id].setFromAffine3d(Tcw_affine);
    }
//    GeometryMethods::createPointCloundFromFrames(frameVector,config,pcd,true);
    pcd->points_ = mesh->vertices_;
    pcd->colors_ = mesh->vertex_colors_;
    pcd->normals_ = mesh->vertex_normals_;

//    visualization::DrawGeometries({mesh});
//    visualization::DrawGeometries({pcd});

    return *pcd;
}

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
    option.non_rigid_camera_coordinate_ = false;

    bool debug = config.getValue<bool>("debug_mode");
    if(debug)
        utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);

    color_map::ColorMapOptimization(*mesh,rgbdVec,camera,option);

    for(size_t frame_id = 0; frame_id < frameVectorRef.size(); frame_id++)
    {
        auto Tcw = camera.parameters_[frame_id].extrinsic_;
        Eigen::Affine3d Tcw_affine;
        Tcw_affine.matrix() = Tcw;
        frameVectorRef[frame_id].get().setFromAffine3d(Tcw_affine);
    }
    return *mesh;

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

    auto Twc0 = frameVector[0].getConstTwc(); //set frame 0 as base

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
        extrinsic = frame.getConstTcw() * Twc0;

        param.intrinsic_ = intrinsic;
        param.extrinsic_ = extrinsic;

        camera.parameters_.push_back(param);
    }
    return true;

}
