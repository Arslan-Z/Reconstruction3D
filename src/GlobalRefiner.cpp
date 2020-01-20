//
// Created by jojo on 05.01.20.
//

#include <Open3D/Registration/FastGlobalRegistration.h>
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


//    utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);
    if(t == s + 1)
    {
//        std::vector<double> voxel_sizes = {voxel_size};
//        std::vector<size_t> max_iters = {50};
//        auto fit = RegistrationMethods::mutiScaleICP(
//                config,
//                source_pcd,
//                target_pcd,
//                voxel_sizes,
//                max_iters,
//                trans,
//                info
//        );
//        match.Tctcs = trans;
        match.information =  registration::GetInformationMatrixFromPointClouds(source_pcd,
                                                                               target_pcd,
                                                                             voxel_size*1.4,
                                                                               match.Tctcs);
        match.success = true;
    }
    else
    {

        geometry::PointCloud source_pcd_down;
        geometry::PointCloud target_pcd_down;
        registration::Feature source_pcd_fpfh;
        registration::Feature target_pcd_fpfh;

        RegistrationMethods::preprocessPointCloud(config,source_pcd,source_pcd_down,source_pcd_fpfh);
        RegistrationMethods::preprocessPointCloud(config,target_pcd,target_pcd_down,target_pcd_fpfh);
        auto success = RegistrationMethods::register_point_cloud_fpfh(
                config,s,t,
                source_pcd_down,
                target_pcd_down,
                source_pcd_fpfh,
                target_pcd_fpfh,
                trans,info);
//        match.Tctcs = trans;
//        match.information = info;
        match.success = success;

        if(success)
        {
            std::cout<<"success"<<std::endl;
            trans = match.Tctcs;
            std::vector<double> voxel_sizes = {voxel_size,voxel_size/2.0,voxel_size/4.0};
            std::vector<size_t> max_iters = {50, 30, 14};
            auto fit = RegistrationMethods::mutiScaleICP(
                    config,
                    source_pcd,
                    target_pcd,
                    voxel_sizes,
                    max_iters,
                    trans,
                    info
            );
            std::cout<<fit<<std::endl;
            if(fit>0.1)
            {
                match.Tctcs = trans;
                match.information = info;
                match.success = true;
            }
            else
            {
                match.success = false;
            }
//            RegistrationMethods::draw_registration_result(source_pcd,target_pcd,trans);
        }
        else
        {

            std::cout<<"fail"<<std::endl;
        }


    }

}
