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
//    optimizePoseGraph(fragment_id,config);
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

    poseGraph.nodes_.push_back(registration::PoseGraphNode(frameVector[0].getConstTwc()));
    for(size_t s = 0; s < end_id; s++)
    {
        for(size_t t = s + 1; t < end_id; t++)
        {
            //odometry
            if(t == s + 1)
            {
                auto Tctw = frameVector[t].getConstTcw();
                auto Twcs = frameVector[t].getConstTwc();
                auto Tctcs = Tctw * Twcs;
                poseGraph.nodes_.push_back(registration::PoseGraphNode(frameVector[t].getConstTwc()));
                poseGraph.edges_.push_back(registration::PoseGraphEdge(s,t,Tctcs));
            }
            //local loop closure
            else if(s%keyFrame_id_base == 0 &&
                    t%keyFrame_id_base == 0)
            {
                //todo loopclosure detection
                auto Tctw = frameVector[t].getConstTcw();
                auto Twcs = frameVector[t].getConstTwc();
                auto Tctcs = Tctw * Twcs;
                registration::PoseGraphEdge edge(s,t,Tctcs);
                edge.uncertain_= true;
                poseGraph.edges_.push_back(edge);
            }
        }
    }
    io::WritePoseGraph(TemplatePoseGraphName(fragment_id),poseGraph);

}

void FragmentMaker::optimizePoseGraph(size_t fragment_id, Parser config)
{
    using namespace open3d;
    utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);
    registration::PoseGraph poseGraph;
    io::ReadPoseGraph(TemplatePoseGraphName(fragment_id),poseGraph);
    auto method = registration::GlobalOptimizationLevenbergMarquardt();
    auto criteria = registration::GlobalOptimizationConvergenceCriteria();
    auto option = registration::GlobalOptimizationOption();
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


    double voxel_size = config.getValue<double>("voxel_size");
    double depth_factor = config.getValue<double>("depth_factor");
    double depth_truncate = config.getValue<double>("depth_truncate");
    integration::ScalableTSDFVolume volume(voxel_size,10*voxel_size,open3d::integration::TSDFVolumeColorType::Gray32);


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
            geometry::Image depth;
            geometry::Image infraRed;
            auto frame = frameVector[i];
            auto pose = poseGraph.nodes_[i].pose_;
            bool read = false;
            read = io::ReadImage(frame.getDepthImagePath().c_str(), depth);
            if(!read)
                return;
            read = io::ReadImageFromPNG(frame.getInfraRedImagePath().c_str(),infraRed);
            if(!read)
                return;
            auto rgbd = geometry::RGBDImage::CreateFromColorAndDepth(
                    infraRed, depth, depth_factor,
                    depth_truncate, true);

            volume.Integrate(*rgbd,intrinsic,pose.inverse());
        }
    }
    auto pcd = volume.ExtractPointCloud();
//    visualization::DrawGeometriesWithCustomAnimation({pcd}, "Animation", 1920, 1080);
    io::WritePointCloud(TemplatePoinCloudName(fragment_id),*pcd);
}
