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
    auto frameVec = Reconstruction::io::Reader::readITEFrames("../dataset/ITE_Long/cameras.txt",
                                                              "../dataset/ITE_Long/");
    std::string config_file = "../dataset/ITE_Long/ITE.yaml";
    Parser config;
    config.load(config_file);
    size_t n_frames_per_fragment = config.getValue<int>("n_frames_per_fragment");
    size_t n_fragments = std::ceil(frameVec.size()/n_frames_per_fragment) + 1;

//    open3d::registration::PoseGraph poseGraph;
//    open3d::io::ReadPoseGraph(Parser::poseGraphName(0),poseGraph);


    Reconstruction::FragmentsRegister::registerFragments(config_file,n_fragments);
//    Reconstruction::Integrater::integrate(config,"fragments/global_optimized.json",n_fragments,frameVec);


}
