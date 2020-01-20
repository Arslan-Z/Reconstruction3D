//
// Created by jojo on 05.01.20.
//
#include "../Integrater.h"
#include "../io/Reader.h"
#include "../util/Parser.h"

int main(int argc,char** argv)
{
    using namespace Reconstruction;
    Parser config;
    config.load("../dataset/ITE_Long/ITE.yaml");
    auto frameVec = Reconstruction::io::Reader::readITEFrames("../dataset/ITE_Long/cameras.txt",
                                                              "../dataset/ITE_Long/");
    size_t n_frame_per_fragment = config.getValue<int>("n_frame_per_fragment");
    size_t n_fragments = std::ceil(frameVec.size()/n_frame_per_fragment) + 1;


    auto frame_opt = Reconstruction::io::Reader::readITEFormat("camera_opt.txt","img_paths.txt");
//    auto frameVec = Reconstruction::io::Reader::readITEFrames("../dataset/ITE_Logo/cameras.txt",
//                                                              "../dataset/ITE_Logo/");
    std::string poseGraphName = argv[1];
    Integrater::integrate(config,poseGraphName,11,frameVec);

//    Reconstruction::Integrater integrater;
//    integrater.init("../dataset/ITE_Logo/ITE.yaml");
//    int i = 0;
//    for(auto frame : frame_opt)
//    {
//        if(i%2 != 0)
//        {
//            i++;
//            continue;
//        }
////        if(i<100)
////        {
////            i++;
////            continue;
////        }
////        if(i>150)
////            break;
//        printf("%d\n",i);
//        i++;
//        integrater.integrateFrame(frame);
//    }
//    integrater.generateMesh();
}

