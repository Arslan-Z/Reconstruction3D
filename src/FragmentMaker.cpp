//
// Created by jojo on 01.01.20.
//

#include "FragmentMaker.h"

using namespace Reconstruction;

void FragmentMaker::processSingleFragment(size_t fragment_id, FrameVector frameVector, const std::string config_file)
{
    Parser config;
    config.load(config_file);
    makePoseGraph(fragment_id, frameVector, config);
    optimizePoseGraph(fragment_id,config);
    makePointCloud(fragment_id,frameVector,config);
}

void FragmentMaker::makePoseGraph(size_t fragment_id, FrameVector frameVector, Parser config)
{
    using namespace open3d;
    registration::PoseGraph poseGraph;
    if(frameVector.empty())
        return;

    size_t n_keyFrames_per_fragment = config.getValue<int>("n_keyFrames_per_fragment");//to do
    size_t end_id = frameVector.size();
    size_t keyFrame_id_base = frameVector.size()/n_keyFrames_per_fragment;
    if(keyFrame_id_base == 0)
        keyFrame_id_base = 1;

    auto Tc0w = frameVector[0].getConstTcw();
    poseGraph.nodes_.push_back(registration::PoseGraphNode(Eigen::Matrix4d::Identity()));

    for(size_t s = 0; s < end_id; s++)
    {
        for(size_t t = s + 1; t < end_id; t++)
        {

//            auto source_pcd = createPoinCloudFromFrame(frameVector[s], config);
//            auto target_pcd = createPoinCloudFromFrame(frameVector[t], config);
//            auto voxel_size = config.getValue<double>("voxel_size");

            //odometry
            if(t == s + 1)
            {
                auto Tctw = frameVector[t].getConstTcw();
                auto Twcs = frameVector[s].getConstTwc();
                auto Tctcs = Tctw * Twcs;
                auto Tc0ct = Tc0w * frameVector[t].getConstTwc();

//                auto information = registration::GetInformationMatrixFromPointClouds(*source_pcd->VoxelDownSample(voxel_size),
//                                                                                     *target_pcd->VoxelDownSample(voxel_size),
//                                                                                     voxel_size,
//                                                                                     Tctcs);
                auto information = Eigen::Matrix6d::Identity();
                poseGraph.nodes_.push_back(registration::PoseGraphNode(Tc0ct));

                poseGraph.edges_.push_back(registration::PoseGraphEdge(s,t,Tctcs,information,false));
            }
            //local loop closure
            else if(s%keyFrame_id_base == 0 &&
                    t%keyFrame_id_base == 0)
            {
                //todo loopclosure detection
                auto Tctw = frameVector[t].getConstTcw();
                auto Twcs = frameVector[s].getConstTwc();
                auto Tctcs = Tctw * Twcs;

//                auto information = registration::GetInformationMatrixFromPointClouds(*source_pcd->VoxelDownSample(voxel_size),
//                                                                                     *target_pcd->VoxelDownSample(voxel_size),
//                                                                                     voxel_size,
//                                                                                     Tctcs);
                auto information = Eigen::Matrix6d::Identity();
                registration::PoseGraphEdge edge(s,t,Tctcs,information,true);
                poseGraph.edges_.push_back(edge);
            }
        }
    }
    io::WritePoseGraph(TemplatePoseGraphName(fragment_id),poseGraph);

}

void FragmentMaker::optimizePoseGraph(size_t fragment_id, Parser config)
{
    using namespace open3d;
    auto max_correspondence_distance = config.getValue<double>("max_depth_diff");
    auto preference_loop_closure = config.getValue<double>("preference_loop_closure_local");
    utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);
    registration::PoseGraph poseGraph;
    io::ReadPoseGraph(TemplatePoseGraphName(fragment_id),poseGraph);
    auto method = registration::GlobalOptimizationLevenbergMarquardt();
    auto criteria = registration::GlobalOptimizationConvergenceCriteria();
    auto option = registration::GlobalOptimizationOption(max_correspondence_distance,0.25,preference_loop_closure,0);

    registration::GlobalOptimization(poseGraph,method,criteria,option);
    io::WritePoseGraph(TemplatePoseGraphName(fragment_id),poseGraph);
    utility::SetVerbosityLevel(utility::VerbosityLevel::Error);

}

void FragmentMaker::makePointCloud(size_t fragment_id,FrameVector frameVector, Parser config)
{
    using namespace open3d;
    registration::PoseGraph poseGraph;
    io::ReadPoseGraph(TemplatePoseGraphName(fragment_id),poseGraph);

    size_t intergrate_n_frame_per_fragment = config.getValue<int>("intergrate_n_frame_per_fragment");
    size_t n_frame_per_fragment = frameVector.size();
    size_t frame_id_base = std::ceil(n_frame_per_fragment/intergrate_n_frame_per_fragment);
    if(frame_id_base == 0)
        frame_id_base = 1;


    double voxel_size = config.getValue<double>("volume_size")/config.getValue<double>("resolution");
    double depth_factor = config.getValue<double>("depth_factor");
    double depth_truncate = config.getValue<double>("depth_truncate");

    integration::ScalableTSDFVolume volume(voxel_size,0.04,open3d::integration::TSDFVolumeColorType::Gray32);


    int width = config.getValue<int>("Camera.width");
    int height = config.getValue<int>("Camera.height");
    double fx = config.getValue<double>("Camera.fx");
    double fy = config.getValue<double>("Camera.fy");
    double cx = config.getValue<double>("Camera.cx");
    double cy = config.getValue<double>("Camera.cy");
    camera::PinholeCameraIntrinsic intrinsic;
    intrinsic.SetIntrinsics(width,height,fx,fy,cx,cy);


    for(size_t i = 0; i < frameVector.size(); i++)
    {
        if(i % frame_id_base == 0)
        {
            std::cout<<"fragment:"<<fragment_id<<" integrating "<<i<<"frame\n";
            geometry::Image depth;
            geometry::Image infraRed;
            auto frame = frameVector[i];
            auto pose = poseGraph.nodes_[i].pose_;
            bool read = false;
            read = io::ReadImage(frame.getDepthImagePath().c_str(), depth);
            if(!read)
                continue;
            read = io::ReadImageFromPNG(frame.getInfraRedImagePath().c_str(),infraRed);
            if(!read)
                continue;
            auto rgbd = geometry::RGBDImage::CreateFromColorAndDepth(
                    infraRed, depth, depth_factor,
                    depth_truncate, true);

            volume.Integrate(*rgbd,intrinsic,pose.inverse());
        }
    }
    auto mesh = volume.ExtractTriangleMesh();
    mesh->ComputeVertexNormals();
    auto pcd = volume.ExtractPointCloud();

    visualization::DrawGeometries({pcd});
    io::WritePointCloud(TemplatePoinCloudName(fragment_id),*pcd);
}

std::shared_ptr<open3d::geometry::PointCloud> FragmentMaker::createPoinCloudFromFrame(Frame frame, Parser config)
{
    using namespace open3d;
    int width = config.getValue<int>("Camera.width");
    int height = config.getValue<int>("Camera.height");
    double fx = config.getValue<double>("Camera.fx");
    double fy = config.getValue<double>("Camera.fy");
    double cx = config.getValue<double>("Camera.cx");
    double cy = config.getValue<double>("Camera.cy");
    camera::PinholeCameraIntrinsic intrinsic;
    intrinsic.SetIntrinsics(width,height,fx,fy,cx,cy);

    double depth_factor = config.getValue<double>("depth_factor");
    double depth_truncate = config.getValue<double>("depth_truncate");

    geometry::Image depth;
    geometry::Image infraRed;
    bool read = false;
    read = io::ReadImage(frame.getDepthImagePath().c_str(), depth);
    if(!read)
        return NULL;
    read = io::ReadImageFromPNG(frame.getInfraRedImagePath().c_str(),infraRed);
    if(!read)
        return NULL;
    auto rgbd = geometry::RGBDImage::CreateFromColorAndDepth(
            infraRed, depth, depth_factor,
            depth_truncate, true);

    return geometry::PointCloud::CreateFromRGBDImage(*rgbd,intrinsic);
}
