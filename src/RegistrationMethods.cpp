//
// Created by jojo on 05.01.20.
//

#include "RegistrationMethods.h"
#include "Open3D/Registration/ColoredICP.h"
#include "Open3D/Registration/FastGlobalRegistration.h"

using namespace Reconstruction;

void RegistrationMethods::preprocessPointCloud(Parser config, const open3d::geometry::PointCloud pcd,
                                             open3d::geometry::PointCloud& pcd_down,
                                             open3d::registration::Feature& pcd_fpfh)
{
    using namespace open3d;
    auto voxel_size = config.getValue<double>("voxel_size");
    pcd_down = *pcd.VoxelDownSample(voxel_size);
    pcd_down.EstimateNormals(geometry::KDTreeSearchParamHybrid(voxel_size*2.0,30));

    pcd_fpfh = *registration::ComputeFPFHFeature(pcd_down,geometry::KDTreeSearchParamHybrid(voxel_size*5.0,100));

}

bool RegistrationMethods::register_point_cloud_fpfh(
        Parser config, size_t s, size_t t,
        open3d::geometry::PointCloud source_pcd_down,
        open3d::geometry::PointCloud target_pcd_down,
        open3d::registration::Feature source_fpfh,
        open3d::registration::Feature target_fpfh,
        Eigen::Matrix4d &Tctcs,
        Eigen::Matrix6d &information)
{
    using namespace open3d;
    auto voxel_size = config.getValue<double>("voxel_size");
    auto method = config.getValue<std::string>("global_registration");
    registration::RegistrationResult result;
    if(method == "ransac")
    {
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
        information = Eigen::Matrix6d::Identity();
        return false;
    }
    information = registration::GetInformationMatrixFromPointClouds(source_pcd_down,
                                                                    target_pcd_down,
                                                                    voxel_size*1.4,
                                                                    result.transformation_);
    if(information(5,5)/std::min(source_pcd_down.points_.size(),target_pcd_down.points_.size())<0.3)
    {
        information = Eigen::Matrix6d::Identity();
        return false;
    }
    Tctcs = result.transformation_;
}

double RegistrationMethods::mutiScaleICP(Parser config,
        open3d::geometry::PointCloud source_pcd,
        open3d::geometry::PointCloud target_pcd,
        std::vector<double> voxel_sizes, std::vector<size_t> max_iters,
        Eigen::Matrix4d &Tctcs, Eigen::Matrix6d &information,
        bool visualize)
{

    double fitness = 0.0;
    if(voxel_sizes.size()!=max_iters.size())
        return fitness;//todo
    if(voxel_sizes.empty())
        return fitness;//todo
    using namespace open3d;

    auto current_Tctcs = Tctcs; //initial value
    for(size_t scale = 0; scale < voxel_sizes.size(); scale++)
    {
        auto voxel_size = voxel_sizes[scale];
        auto max_iter = max_iters[scale];

        auto source_down = source_pcd.VoxelDownSample(voxel_size);
        auto target_down = target_pcd.VoxelDownSample(voxel_size);

        source_down->EstimateNormals(geometry::KDTreeSearchParamHybrid(voxel_size*2.0,30));
        target_down->EstimateNormals(geometry::KDTreeSearchParamHybrid(voxel_size*2.0,30));

        auto result = registration::RegistrationColoredICP(
                *source_down,
                *target_down,
                voxel_size,
                current_Tctcs,
                registration::ICPConvergenceCriteria(1e-6,1e-6,max_iter)
                );
        if(fitness<result.fitness_)
        {
            fitness = result.fitness_;
            current_Tctcs = result.transformation_;
        }
        else
        {
            information = registration::GetInformationMatrixFromPointClouds(
                    *source_down,
                    *target_down,
                    voxel_size*1.4,
                    result.transformation_);
            Tctcs = result.transformation_;

            break;
        }

        if(scale == voxel_sizes.size() - 1)
        {
            information = registration::GetInformationMatrixFromPointClouds(
                    *source_down,
                    *target_down,
                    voxel_size*1.4,
                    result.transformation_
            );
            Tctcs = result.transformation_;

        }
        if(visualize)
        {
            RegistrationMethods::draw_registration_result(source_pcd,target_pcd,current_Tctcs);
        }
    }

    return fitness;
}


void RegistrationMethods::draw_registration_result(
        const open3d::geometry::PointCloud source,
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
