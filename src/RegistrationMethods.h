//
// Created by jojo on 05.01.20.
//

#ifndef RECONSTRUCTION3D_REGISTRATIONMETHODS_H
#define RECONSTRUCTION3D_REGISTRATIONMETHODS_H
#include "util/Parser.h"
#include "Open3D/Open3D.h"

namespace Reconstruction
{
    class RegistrationMethods
    {
    public:
        static void preprocessPointCloud(Parser config, const open3d::geometry::PointCloud pcd,
                open3d::geometry::PointCloud& pcd_down,
        open3d::registration::Feature& pcd_fpfh);

        static bool register_point_cloud_fpfh(Parser config, size_t s, size_t t,
                                              open3d::geometry::PointCloud source_pcd_down,
                                              open3d::geometry::PointCloud target_pcd_down,
                                              open3d::registration::Feature source_fpfh,
                                              open3d::registration::Feature target_fpfh,
                                              Eigen::Matrix4d& Tctcs, Eigen::Matrix6d& information);

        static double mutiScaleICP(Parser config,
                                 open3d::geometry::PointCloud source_pcd,
                                 open3d::geometry::PointCloud target_pcd,
                                 std::vector<double> voxel_sizes,
                                 std::vector<size_t> max_iters,
                                 Eigen::Matrix4d& Tctcs, Eigen::Matrix6d& information,
                                 bool visualize=false);

        static void draw_registration_result(const open3d::geometry::PointCloud source, const open3d::geometry::PointCloud target,
                                             const Eigen::Matrix4d trans);

    };
}



#endif //RECONSTRUCTION3D_REGISTRATIONMETHODS_H
