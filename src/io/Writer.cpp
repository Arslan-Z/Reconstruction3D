//
// Created by jojo on 04.01.20.
//

#include "Writer.h"
#include "Eigen/Geometry"

#include <stdio.h>
#include <iostream>

using namespace Reconstruction;

void Writer::writeLocalPoseGraphTo_ITE_Format(const open3d::registration::PoseGraph poseGraph_local,
                                              const FrameVector frameVector_global,
                                              const std::string camera_file,
                                              const std::string image_paths_file,
                                              size_t fragment_id,
                                              Eigen::Matrix4d Twc0,
                                              Parser config)
{
    size_t n_frame_per_fragment = config.getValue<int>("n_frame_per_fragment");

    FrameVector local_frameVec;
    local_frameVec.insert(local_frameVec.end(),
                          frameVector_global.begin()+fragment_id*n_frame_per_fragment,
                          std::min(frameVector_global.begin()+(fragment_id+1)*n_frame_per_fragment,frameVector_global.end()));

    FILE* pf_camera_file = fopen(camera_file.c_str(),"a");
    FILE* pf_image_paths_file = fopen(image_paths_file.c_str(),"a");
    if(pf_camera_file != NULL &&
        pf_image_paths_file != NULL)
    {
        for(size_t node_id = 0; node_id < poseGraph_local.nodes_.size(); node_id++)
        {
            Eigen::AngleAxisd angleAxis;
            auto node = poseGraph_local.nodes_[node_id];
            auto frame = local_frameVec[node_id];
            auto Tc0cn = node.pose_;

            auto Twcn = Twc0*Tc0cn;//todo
            std::cout<<Twcn<<std::endl;
            angleAxis.fromRotationMatrix(Twcn.block<3,3>(0,0)); //Twc
            Eigen::Vector3d rot = angleAxis.axis();
            rot *= angleAxis.angle();

            Eigen::Vector3d trans = Twcn.block<3,1>(0,3);
            fprintf(pf_camera_file,"%lf %lf %lf %lf %lf %lf\n",rot[0],rot[1],rot[2],trans[0],trans[1],trans[2]);

            auto rgb_path = frame.getRGBImagePath();
            auto ir_path = frame.getInfraRedImagePath();
            auto depth_path = frame.getDepthImagePath();
            fprintf(pf_image_paths_file,"%s %s %s\n",rgb_path.c_str(),ir_path.c_str(),depth_path.c_str());
        }
        fclose(pf_camera_file);
        fclose(pf_image_paths_file);
    }
    else
    {
//        std::cout<<"cannot open file"<<std::endl;
    }
}

void Writer::writeGlobalPoseGraphTo_ITE_Format(const FrameVector frameVector_global, Parser config)
{
    using namespace open3d;
    registration::PoseGraph poseGraph_global;
    io::ReadPoseGraph(Parser::globalPoseGraphName(),poseGraph_global);

    std::string cam_file = "camera_opt.txt";
    std::string path_file = "img_paths.txt";
    FILE* cam = fopen(cam_file.c_str(),"w");
    FILE* path = fopen(path_file.c_str(),"w");
    fclose(cam);
    fclose(path);
//    for(size_t node_id = 0; node_id < poseGraph_global.nodes_.size(); node_id++)
//    {
//        auto node = poseGraph_global.nodes_[node_id];
//        registration::PoseGraph poseGraph;
//        io::ReadPoseGraph(Parser::poseGraphName(node_id),poseGraph);
//        writeLocalPoseGraphTo_ITE_Format(poseGraph,frameVector_global,
//                                         cam_file,path_file,node_id,
//                                         node.pose_,
//                                         config);
//
//    }
    size_t node_id = 7;
    auto node = poseGraph_global.nodes_[node_id];
    registration::PoseGraph poseGraph;
    io::ReadPoseGraph(Parser::poseGraphName(node_id),poseGraph);
    writeLocalPoseGraphTo_ITE_Format(poseGraph,frameVector_global,
                                     cam_file,path_file,node_id,
                                     node.pose_,
                                     config);

}
