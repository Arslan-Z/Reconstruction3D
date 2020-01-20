//
// Created by jojo on 31.12.19.
//
#include "stdio.h"
#include "../io/Reader.h"
#include "../FragmentMaker.h"
#include "../PoseGraphMethods.h"

int main(int argc,char** argv)
{
    auto frameVec = Reconstruction::io::Reader::readITEFrames("../dataset/ITE_Long/cameras.txt",
            "../dataset/ITE_Long/observations.txt",
            "../dataset/ITE_Long/",1);

    std::string config_file = "../dataset/ITE_Long/ITE.yaml";
    std::cout<<"read "<<frameVec.size()<<"frames"<<std::endl;
    Reconstruction::FragmentMaker::makeFragments(frameVec,config_file);
    auto poseGraph = Reconstruction::PoseGraphMethods::createGlobalPoseGraphFromFrames(frameVec,config_file);
    open3d::io::WritePoseGraph("fragments/global_optimized.json",poseGraph);
    return 0;
}
