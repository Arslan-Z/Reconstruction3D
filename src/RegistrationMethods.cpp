//
// Created by jojo on 05.01.20.
//

#include "RegistrationMethods.h"
#include "Open3D/Registration/ColoredICP.h"
#include "Open3D/Registration/FastGlobalRegistration.h"

using namespace Reconstruction;

void RegistrationMethods::mutiScaleICP(Parser config,
        size_t s, size_t t,
        open3d::geometry::PointCloud source_pcd,
        open3d::geometry::PointCloud target_pcd,
        std::vector<double> voxel_sizes, std::vector<size_t> max_iters,
        Eigen::Matrix4d &Tctcs, Eigen::Matrix6d &information)
{

    if(voxel_sizes.size()!=max_iters.size())
        return;//todo
    if(voxel_sizes.empty())
        return;//todo
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
        current_Tctcs = result.transformation_;
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
    }

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
