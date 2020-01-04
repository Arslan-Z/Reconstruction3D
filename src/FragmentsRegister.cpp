//
// Created by jojo on 03.01.20.
//

#include "FragmentsRegister.h"
#include <thread>
using namespace Reconstruction;

void FragmentsRegister::registerFragments(std::string config_file, size_t n_fragments)
{
    Parser config;
    config.load(config_file);
    makePoseGraphForScene(config, n_fragments);
    optimizePoseGraph(config,"fragments/global.json");
}
void FragmentsRegister::makePoseGraphForScene(Parser config, size_t n_fragments)
{
    using namespace open3d;
    bool debug = true;
    bool multi_thread = false;
    if(debug)
    {
        utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);
    }

    std::vector<MatchingResult> matching_results;
    for(size_t s = 0; s < n_fragments; s++)
    {
        for(size_t t = s + 1; t < n_fragments; t++)
        {
            matching_results.push_back(MatchingResult(s,t));
        }
    }
    if(multi_thread)
    {
        std::vector<std::reference_wrapper<std::thread>> threads;
        for(auto& matching_result : matching_results)
        {
//            std::thread thr(FragmentsRegister::registerPoincloudPair,config,matching_result);
//            threads.push_back(thr)
        }
    }
    else
    {
        for(auto& matching_result : matching_results)
        {
            std::cout<<"registrating pair: "<<matching_result.s<<"/"<<matching_result.t<<std::endl;
            registerPoincloudPair(config, matching_result);
        }
    }
    registration::PoseGraph poseGraph;
    poseGraph.nodes_.push_back(registration::PoseGraphNode(Eigen::Matrix4d::Identity())); //base node
    Eigen::Matrix4d current_Tcsw = Eigen::Matrix4d::Identity();

    for(auto matching_result : matching_results)
    {
        if(matching_result.success)
        {
            updatePoseGraph(config, matching_result, current_Tcsw, poseGraph);
        }
    }
    io::WritePoseGraph("fragments/global.json",poseGraph);//todo
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

    bool read = false;
    read = io::ReadPointCloud(Parser::plyFileName(s),source_pcd);
    if(!read)
    {
        std::cout<<"fragment pcd not found. Skip\n";
        return;
    }
    read = io::ReadPointCloud(Parser::plyFileName(t),target_pcd);
    if(!read)
    {
        std::cout<<"fragment pcd not found. Skip\n";
        return;
    }
    if(source_pcd.points_.size() == 0 || target_pcd.points_.size() == 0)
    {
        std::cout<<"fragment invalid. Skip\n";
        return;
    }

    preprocessPointCloud(config, source_pcd, source_pcd_down, source_fpfh);
    preprocessPointCloud(config, target_pcd, target_pcd_down, target_fpfh);
    Eigen::Matrix4d Tctcs;
    Eigen::Matrix6d information;

    bool success = computeInitialRegistration(config,s,t,
            source_pcd_down,target_pcd_down,
            source_fpfh,target_fpfh,
            Tctcs,information);

    matching_result.success = success;
    matching_result.Tctcs = Tctcs;
    matching_result.information = information;
//    if(success)
//    {
//    draw_registration_result(source_pcd_down,target_pcd_down,Tctcs);
//    }
}

void FragmentsRegister::updatePoseGraph(Parser config, const FragmentsRegister::MatchingResult matching_result, Eigen::Matrix4d& Tcsw, open3d::registration::PoseGraph& poseGraph)
{
    using namespace open3d;
    auto s = matching_result.s;
    auto t = matching_result.t;
    auto information = matching_result.information;
    auto Tctcs = matching_result.Tctcs;
    //odometry
    if(t == s + 1 )
    {
        auto Tctw = matching_result.Tctcs * Tcsw;
        Tcsw = Tctw; //update
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
    pcd_down = *pcd.VoxelDownSample(2*voxel_size);
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
    auto voxel_size = config.getValue<double>("voxel_size");
    //odometry
    if(t == s + 1)
    {
        io::ReadPoseGraph(Parser::poseGraphName(s),poseGraph_fragment);
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
        return true;
    }
    else
    {
        auto method = config.getValue<std::string>("global_registration");
        registration::RegistrationResult result;
        if(method == "ransac")
        {
//            registration::PoseGraph global_poseGraph;
//            io::ReadPoseGraph(Parser::globalPoseGraphName(),global_poseGraph);
//            auto Twcs = global_poseGraph.nodes_[s].pose_;
//            auto Twct = global_poseGraph.nodes_[t].pose_;
//            Tctcs = Twct.inverse()*Twcs; //initial value

//            auto result = registration::RegistrationColoredICP(source_pcd_down,target_pcd_down,voxel_size,Tctcs,
//                                                               registration::ICPConvergenceCriteria(1e-6,1e-6,50));
            auto distance_threshold = voxel_size * 1.4;
            auto checker1 = registration::CorrespondenceCheckerBasedOnEdgeLength(0.9);
            auto checker2 = registration::CorrespondenceCheckerBasedOnDistance(distance_threshold);

            result = registration::RegistrationRANSACBasedOnFeatureMatching(source_pcd_down,
                    target_pcd_down,
                    source_fpfh,
                    target_fpfh,
                    distance_threshold,
                    registration::TransformationEstimationPointToPoint(false),
                    4,
                    {checker1,checker2},
                    registration::RANSACConvergenceCriteria(40000000,500));
        }
        else
        {
            result = registration::FastGlobalRegistration(source_pcd_down,
                                                               target_pcd_down,
                                                               source_fpfh,
                                                               target_fpfh,
                                                               registration::FastGlobalRegistrationOption());
        }

        if(result.transformation_.trace() == 4.0)
        {
            Tctcs = Eigen::Matrix4d::Identity();
            information = Eigen::Matrix6d::Identity();
            return false;
        }
        information = registration::GetInformationMatrixFromPointClouds(source_pcd_down,
                                                                        target_pcd_down,
                                                                        voxel_size*1.4,
                                                                        result.transformation_);
        if(information(5,5)/std::min(source_pcd_down.points_.size(),target_pcd_down.points_.size())<0.3)
        {
            Tctcs = Eigen::Matrix4d::Identity();
            information = Eigen::Matrix6d::Identity();
            return false;
        }
        Tctcs = result.transformation_;

    }
}

void FragmentsRegister::draw_registration_result(const open3d::geometry::PointCloud source,
                                                 const open3d::geometry::PointCloud target,
                                                 const Eigen::Matrix4d trans)
{
    std::shared_ptr<open3d::geometry::PointCloud> source_pcd(new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> target_pcd(new open3d::geometry::PointCloud);

    *source_pcd = source;
    *target_pcd = target;
    source_pcd->PaintUniformColor(Eigen::Vector3d(1, 0.706, 0));
    target_pcd->PaintUniformColor(Eigen::Vector3d(0, 0.651, 0.929));
    source_pcd->Transform(trans);
    source_pcd->EstimateNormals();
    target_pcd->EstimateNormals();
    open3d::visualization::DrawGeometries({source_pcd,target_pcd});
}

void FragmentsRegister::optimizePoseGraph(Parser config, std::string poseGraphName)
{
    using namespace open3d;
    utility::SetVerbosityLevel(utility::VerbosityLevel::Debug);
    registration::PoseGraph poseGraph;
    io::ReadPoseGraph(poseGraphName,poseGraph);
    auto method = registration::GlobalOptimizationLevenbergMarquardt();
    auto criteria = registration::GlobalOptimizationConvergenceCriteria();
    auto max_correspondence_distance = config.getValue<double>("voxel_size") * 1.4;
    auto preference_loop_closure = config.getValue<double>("preference_loop_closure");
    auto option = registration::GlobalOptimizationOption(max_correspondence_distance,
            0.25,preference_loop_closure,0);
    registration::GlobalOptimization(poseGraph,method,criteria,option);
    io::WritePoseGraph("fragments/global_optimized.json",poseGraph); //todo
    utility::SetVerbosityLevel(utility::VerbosityLevel::Error);
}
