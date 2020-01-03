//
// Created by jojo on 06.12.19.
//

#include "Reader.h"
using namespace Reconstruction::io;


Reconstruction::FrameVector Reader::readITEFrames(const char *cam_file,const char *dataset_path)
{
    FILE* pF_cam;
    FrameVector frameVec;
    pF_cam = fopen(cam_file,"r");
    if(pF_cam == NULL) {
        printf("camera file not found!\n");
        return frameVec;
    }

    int cam_id_offset = -1;
    unsigned int line = 0;
    int id = 0;
    while(!feof(pF_cam))
    {
//FrameId qw qx qy qz px py pz absoluteTime timeIndex
        Frame frame;
        unsigned int frame_id;
        double frame_id_temp,time_id;
        char* cam_id_str = new char[7];
        double t;
        double x,y,z,qx,qy,qz,qw;
        int num_scanned = fscanf(pF_cam,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",&frame_id_temp,&qw,&qx,&qy,&qz,&x,&y,&z,&t,&time_id);
        frame_id = frame_id_temp;

        frame.mGlobalIndex = id;
        id++;

        if(cam_id_offset < 0)
        {
            cam_id_offset = static_cast<int>(frame_id_temp);
            frame_id = cam_id_offset;
        }
        else
            frame_id = cam_id_offset + line;

        line++;

        sprintf(cam_id_str,"%06d",frame_id);
        std::string rgbImg_path = std::string(dataset_path);
        rgbImg_path += "rgb/";
        rgbImg_path += "ColorRealsense";
        rgbImg_path += std::string(cam_id_str);
        rgbImg_path += ".png";
        std::string depthImg_path = std::string(dataset_path);
        depthImg_path += "depth/";
        depthImg_path += "DepthRealsense";
        depthImg_path += std::string(cam_id_str);
        depthImg_path +=    ".png";

        std::string infraRedImg_path = std::string(dataset_path);
        infraRedImg_path += "ir/";
        infraRedImg_path += "InfraredRealsense";
        infraRedImg_path += std::string(cam_id_str);
        infraRedImg_path += ".png";

        frame.setImagePaths(rgbImg_path.c_str(),depthImg_path.c_str(),infraRedImg_path.c_str());
        if(num_scanned != 10)
            break;

//read in Twc -> Tcw
        Eigen::Matrix3d R;
        Eigen::Vector3d trans;
        auto q = Eigen::Quaterniond(qw,qx,qy,qz);
        R = Eigen::AngleAxisd(q).inverse().toRotationMatrix();
        trans = R*Eigen::Vector3d(x,y,z);
        trans *= -1;

        frame.setFromQuaternionAndPoint(q.inverse(),trans);
        frame.setTimeStamp(t);


        frameVec.push_back(frame);
    }

    fclose(pF_cam);

    return frameVec;
}




