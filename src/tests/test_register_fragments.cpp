//
// Created by jojo on 03.01.20.
//
#include "stdio.h"
#include "../util/Parser.h"
#include "../io/Reader.h"
#include "../io/Writer.h"
#include "../Integrater.h"
#include "../FragmentMaker.h"
#include "../FragmentsRegister.h"
int main(int argc,char** argv)
{
    auto frameVec = Reconstruction::io::Reader::readITEFrames("../dataset/ITE_Logo/cameras.txt",
                                                              "../dataset/ITE_Logo/");
    std::string config_file = "../dataset/ITE_Logo/ITE.yaml";
    Parser config;
    config.load(config_file);
    size_t n_frame_per_fragment = config.getValue<int>("n_frame_per_fragment");
    size_t n_fragments = std::ceil(frameVec.size()/n_frame_per_fragment) + 1;

    Reconstruction::Writer::writeGlobalPoseGraphTo_ITE_Format(frameVec,config);
    auto frame_opt = Reconstruction::io::Reader::readITEFormat("camera_opt.txt","img_paths.txt");
//    open3d::registration::PoseGraph poseGraph;
//    open3d::io::ReadPoseGraph(Parser::poseGraphName(0),poseGraph);


//    Reconstruction::FragmentsRegister::registerFragments(config_file,n_fragments);
//    Reconstruction::Integrater::integrate(config,"fragments/global_optimized.json",n_fragments,frameVec);
//        Reconstruction::FragmentsRegister::optimizePoseGraph(config,Parser::globalPoseGraphName());
    Reconstruction::Integrater integrater;
    integrater.init(config_file);
    int i = 0;
    for(auto frame : frame_opt)
    {
        if(i%3 != 0)
        {
            i++;
            continue;
        }
//        if(i<100)
//        {
//            i++;
//            continue;
//        }
        if(i>150)
            break;
        printf("%d\n",i);
        i++;
        integrater.integrateFrame(frame);
    }
    integrater.generateMesh();

}
