//
// Created by jojo on 03.01.20.
//

#include "FragmentsRegister.h"

using namespace Reconstruction;

void FragmentsRegister::registerFragments(std::string config_file, size_t n_fragments)
{
    Parser config;
    config.load(config_file);
    makePoseGraphForScene(config, n_fragments);
}
void FragmentsRegister::makePoseGraphForScene(Parser config, size_t n_fragments)
{
    using namespace open3d;
    registration::PoseGraph poseGraph;
    poseGraph.nodes_.push_back(registration::PoseGraphNode(Eigen::Matrix4d::Identity())); //base node
//    poseGraph = io::ReadPoseGraph()

    std::vector<MatchingResult> matching_results;
    for(size_t s = 0; s < n_fragments; s++)
    {
        for(size_t t = s + 1; t < n_fragments; t++)
        {
            matching_results.push_back(MatchingResult(s,t));
        }
    }
    for(auto& matching_result : matching_results)
    {
        registerPoincloudPair(config, matching_result);
    }
    for(auto matching_result : matching_results)
    {
        if(matching_result.success)
        {
            updatePoseGraph(config,matching_result,poseGraph);
        }
    }
    io::WritePoseGraph("fragments/global.json",poseGraph);
}

void FragmentsRegister::registerPoincloudPair(Parser config, FragmentsRegister::MatchingResult& matching_result)
{
    using namespace open3d;
    auto s = matching_result.s;
    auto t = matching_result.t;
    geometry::PointCloud source_pcd;
    geometry::PointCloud target_pcd;
    geometry::PointCloud source_pcd_down;
    geometry::PointCloud target_pcd_down;
    registration::Feature source_fpfh;
    registration::Feature target_fpfh;

    io::ReadPointCloud(Parser::plyFileName(s),source_pcd);
    io::ReadPointCloud(Parser::plyFileName(t),target_pcd);

    preprocessPointCloud(config, source_pcd, source_pcd_down, source_fpfh);
    preprocessPointCloud(config, target_pcd, target_pcd_down, target_fpfh);
//    computeInitialRegistration();

}

void FragmentsRegister::updatePoseGraph(Parser config, const FragmentsRegister::MatchingResult matching_result, open3d::registration::PoseGraph& poseGraph)
{
    using namespace open3d;
    auto s = matching_result.s;
    auto t = matching_result.t;
    auto Tctcs = matching_result.Tctcs;
    auto Tctw = matching_result.Tctcs*matching_result.Tcsw;
    auto information = matching_result.information;
    //odometry
    if(t == s + 1 )
    {
        registration::PoseGraphNode node(Tctw.inverse());
        registration::PoseGraphEdge edge(s,t,Tctcs,information,false);
        poseGraph.nodes_.push_back(node);
        poseGraph.edges_.push_back(edge);
    }
    //loop closure
    else
    {
        registration::PoseGraphEdge edge(s,t,Tctcs,information,true);
        poseGraph.edges_.push_back(edge);
    }

}

void FragmentsRegister::preprocessPointCloud(Parser config, const open3d::geometry::PointCloud pcd,
                                             open3d::geometry::PointCloud& pcd_down,
                                             open3d::registration::Feature& pcd_fpfh)
{
    using namespace open3d;
    auto voxel_size = config.getValue<double>("voxel_size");
    pcd_down = *pcd.VoxelDownSample(voxel_size);
    pcd_down.EstimateNormals(geometry::KDTreeSearchParamHybrid(voxel_size*2.0,30));

    pcd_fpfh = *registration::ComputeFPFHFeature(pcd_down,geometry::KDTreeSearchParamHybrid(voxel_size*5.0,100));

}

bool FragmentsRegister::computeInitialRegistration(Parser config, size_t s, size_t t,
                                                   open3d::geometry::PointCloud source_pcd_down,
                                                   open3d::geometry::PointCloud target_pcd_down,
                                                   open3d::registration::Feature source_fpfh,
                                                   open3d::registration::Feature target_fpfh,
                                                   Eigen::Matrix4d& Tctcs, Eigen::Matrix6d& information)
{
    using namespace open3d;
    registration::PoseGraph poseGraph_fragment;
    auto voxel_size = config.getValue<double>("vexel_size");
    //odometry
    if(t == s + 1)
    {
        io::ReadPoseGraph(Parser::plyFileName(s),poseGraph_fragment);
        auto Twcs0 = poseGraph_fragment.nodes_.front().pose_;
        auto Twcsn = poseGraph_fragment.nodes_.back().pose_;
        auto Tcsncs0 = Twcsn.inverse()*Twcs0;

        Tctcs = Tcsncs0; //initial value

        auto result = registration::RegistrationColoredICP(source_pcd_down,target_pcd_down,voxel_size,Tctcs,
                registration::ICPConvergenceCriteria(1e-6,1e-6,50));

        Tctcs = result.transformation_;
        information = registration::GetInformationMatrixFromPointClouds(source_pcd_down,
                                                                        target_pcd_down,
                                                                        voxel_size*1.4,
                                                                        Tctcs);
    }
    else
    {
        auto distance_threshold = voxel_size * 1.4;
        auto checker1 = registration::CorrespondenceCheckerBasedOnEdgeLength(0.9);
        auto checker2 = registration::CorrespondenceCheckerBasedOnDistance(distance_threshold);
        registration::RegistrationRANSACBasedOnFeatureMatching(source_pcd_down,
                target_pcd_down,
                source_fpfh,
                target_fpfh,
                distance_threshold,
                registration::TransformationEstimationPointToPoint(false),
                4,
                {checker1,checker2},
                registration::RANSACConvergenceCriteria(4000000,500));
    }
}
