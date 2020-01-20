//
// Created by jojo on 05.01.20.
//
#include "stdio.h"
#include "../util/Parser.h"
#include "../GlobalRefiner.h"
#include "../Integrater.h"
#include "../io/Reader.h"
int main(int argc,char** argv)
{
    using namespace Reconstruction;
//    Parser config;
//    config.load("../dataset/ITE_Logo/ITE.yaml");
//    auto frameVec = Reconstruction::io::Reader::readITEFrames("../dataset/ITE_Logo/cameras.txt",
//                                                              "../dataset/ITE_Logo/");
//    size_t n_frame_per_fragment = config.getValue<int>("n_frame_per_fragment");
//    size_t n_fragments = std::ceil(frameVec.size()/n_frame_per_fragment) + 1;
    GlobalRefiner::refine("../dataset/ITE_Long/ITE.yaml",
            "fragments/global_optimized.json");
}