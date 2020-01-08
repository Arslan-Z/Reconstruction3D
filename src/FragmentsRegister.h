//
// Created by jojo on 03.01.20.
//

#ifndef RECONSTRUCTION3D_FRAGMENTSREGISTER_H
#define RECONSTRUCTION3D_FRAGMENTSREGISTER_H

#include <list>
#include "util/Parser.h"
#include "PoseGraphMethods.h"

#include "Open3D/Open3D.h"
#include "Open3D/Registration/GlobalOptimization.h"
#include "Open3D/Registration/ColoredICP.h"
#include "Open3D/Registration/FastGlobalRegistration.h"
//#include "Open3D/Registration/TransformationEstimation.h"

namespace Reconstruction
{

    class FragmentsRegister
    {
    public:
        static void registerFragments(std::string config_file, size_t n_fragments);
        static void optimizePoseGraph(Parser config, std::string poseGraphName);

    private:
        static void makePoseGraphForScene(Parser config, size_t n_fragments, const std::string poseGraphName);
        static void registerPoincloudPair(Parser config, PoseGraphMethods::MatchingResult& matching_result);
        static void preprocessPointCloud(Parser config, const open3d::geometry::PointCloud pcd,
                                         open3d::geometry::PointCloud& pcd_down,
                                         open3d::registration::Feature& pcd_fpfh);
        static bool computeInitialRegistration(const Parser config, const size_t s, const size_t t,
                                               const open3d::geometry::PointCloud source_pcd_down,
                                               const open3d::geometry::PointCloud target_pcd_down,
                                               const open3d::registration::Feature source_fpfh,
                                               const open3d::registration::Feature target_fpfh,
                                               Eigen::Matrix4d& Tctcs, Eigen::Matrix6d& information);
        static void updatePoseGraph(Parser config, const PoseGraphMethods::MatchingResult matching_result, Eigen::Matrix4d& Tcsw, open3d::registration::PoseGraph& poseGraph);
        static void draw_registration_result(const open3d::geometry::PointCloud source, const open3d::geometry::PointCloud target,
                                             const Eigen::Matrix4d trans);
    };
}



#endif //RECONSTRUCTION3D_FRAGMENTSREGISTER_H
