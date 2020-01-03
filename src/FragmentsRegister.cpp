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
    io::ReadPointCloud(Parser::plyFileName(s),source_pcd);
    io::ReadPointCloud(Parser::plyFileName(s),target_pcd);

//    preprocessPointCloud();

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