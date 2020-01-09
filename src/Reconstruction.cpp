//
// Created by jojo on 09.01.20.
//
#include "io/Reader.h"
#include "FragmentMaker.h"
#include "FragmentsRegister.h"
#include "GlobalRefiner.h"
#include "Integrater.h"

int main(int argc,char** argv)
{
    auto frameVec = Reconstruction::io::Reader::readITEFrames("../dataset/ITE_Logo/cameras.txt",
                                                              "../dataset/ITE_Logo/");
    std::string config_file = "../dataset/ITE_Logo/ITE.yaml";

    Parser config;
    config.load(config_file);
    size_t n_frames_per_fragment = config.getValue<int>("n_frames_per_fragment");
    size_t n_fragments = std::ceil(frameVec.size()/n_frames_per_fragment) + 1;

    Reconstruction::FragmentMaker::makeFragments(frameVec,config_file);
    Reconstruction::FragmentsRegister::registerFragments(config_file,n_fragments);
    Reconstruction::GlobalRefiner::refine("../dataset/ITE_Logo/ITE.yaml",
                          "fragments/global_optimized.json");
    Reconstruction::Integrater::integrate(config,"fragments/global_optimized_refined.json",11,frameVec);

}

