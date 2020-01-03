//
// Created by jojo on 03.01.20.
//
#include "stdio.h"
#include "../util/Parser.h"
#include "../io/Reader.h"
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

    Reconstruction::FragmentsRegister::registerFragments(config_file,n_frame_per_fragment);

//    Reconstruction::Integrater integrater;
//    integrater.init("../dataset/ITE_Logo/ITE.yaml");
//    integrater.id = 0;
////    make_single_fragment(frameVec);
//    int i = 0;
//    for(auto frame : frameVec)
//    {
//        printf("%d\n",i);
//        integrater.integrateFrame(frame);
//        if(i>100)
//            break;
//        else
//            i++;
//    }
//    integrater.generatePointCloud(true);
    return 0;
}
