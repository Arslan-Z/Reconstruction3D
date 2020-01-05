//
// Created by jojo on 05.01.20.
//

#include "GlobalRefiner.h"

using namespace Reconstruction;

void GlobalRefiner::refine(std::string config_file, std::string global_poseGraphName)
{
    Parser config;
    config.load(config_file);
    auto matches = PoseGraphMethods::createMatchesFromPoseGraph(global_poseGraphName);
    for(auto& match : matches)
    {
        registerPointCloudPair(config,match);
    }
    PoseGraphMethods::createPoseGraphFromMatches(matches,"fragments/global_optimized_temp.json");//todo
    PoseGraphMethods::optimizePoseGraphForScene(
            config_file,
            "fragments/global_optimized_temp.json",
            "fragments/global_optimized_refined.json");
}

void GlobalRefiner::registerPointCloudPair(Parser config, Reconstruction::PoseGraphMethods::MatchingResult &match)
{
    using namespace open3d;
    auto s = match.s;
    auto t = match.t;
    auto trans = match.Tctcs;
    auto info = match.information;
    geometry::PointCloud source_pcd;
    geometry::PointCloud target_pcd;
    io::ReadPointCloud(Parser::plyFileName(s),source_pcd);
    io::ReadPointCloud(Parser::plyFileName(t),target_pcd);

    auto voxel_size = config.getValue<double>("voxel_size");
    std::vector<double> voxel_sizes = {voxel_size,voxel_size/2.0,voxel_size/4.0};
    std::vector<size_t> max_iters = {50, 30, 14};

    utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);
    RegistrationMethods::mutiScaleICP(
            config,
            source_pcd,
            target_pcd,
            voxel_sizes,
            max_iters,
            trans,
            info
            );
    match.Tctcs = trans;
    match.information = info;
    match.success = true;
//    RegistrationMethods::draw_registration_result(source_pcd,target_pcd,trans);
}
