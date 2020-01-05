#ifndef INTEGRATER_H
#define INTEGRATER_H

#include <stdio.h>

#include <iostream>

#include "Frame.h"
#include "util/Parser.h"
#include "Open3D/Open3D.h"
namespace Reconstruction
{
    class Integrater
    {
    public:
        typedef std::shared_ptr<open3d::integration::ScalableTSDFVolume> Volume;
        Integrater();
        static Volume createVolume(const Parser config);
        static void integrate(const Parser config, const std::string poseGraphName,const size_t n_fragments, const FrameVector frameVector);
        static void integrateFragment(const Parser config, Volume volume, const size_t fragment_id, const FrameVector frameVector, const Eigen::Matrix4d Twc0);
        bool integrateFrame(const Frame frame);
        void init(std::string strSettingPath);
        bool saveTSDF(const char* path);
        bool generateMesh(bool visualize = true);
        bool generatePointCloud(bool visualize = true);

        unsigned int id;
    private:
        struct TSDF_Param
        {
            float tsdf_voxel_size;
            float tsdf_size;
            float tsdf_res;
            double depth_factor;
            double depth_truncate;

        };
        TSDF_Param mTSDF_param;
        std::shared_ptr<open3d::integration::ScalableTSDFVolume> mVolume_ptr;
        open3d::camera::PinholeCameraIntrinsic mIntrinsic;

    };

}

#endif // INTEGRATER_H
