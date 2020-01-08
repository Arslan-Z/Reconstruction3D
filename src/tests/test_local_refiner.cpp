//
// Created by jojo on 07.01.20.
//
#include "../io/Reader.h"
#include "../util/Parser.h"
#include "../LocalRefiner.h"

int main(int argc,char** argv)
{
    auto frameVec = Reconstruction::io::Reader::readITEFrames("../dataset/ITE_Logo/cameras.txt",
                                                              "../dataset/ITE_Logo/");
    std::string config_file = "../dataset/ITE_Logo/ITE.yaml";
    Parser config;
    config.load(config_file);
    std::vector<Reconstruction::FrameVector> fragments;
    size_t n_frame_per_fragment = 50;
    size_t n_fragments = std::ceil(frameVec.size()/n_frame_per_fragment) + 1;

    size_t frame_id = 0;
    for(size_t i = 0; i < n_fragments; i++)
    {
        Reconstruction::FrameVector fragment;
        if(i != n_fragments-1)
        {
            for(int num = 0; num < n_frame_per_fragment; num++, frame_id++)
            {
                fragment.push_back(frameVec[frame_id]);
            }
        }
        else
        {
            for(; frame_id < frameVec.size(); frame_id++)
            {
                fragment.push_back(frameVec[frame_id]);
            }
        }
        fragments.push_back(fragment);
    }
    for(auto fragment : fragments)
    {
        std::vector<std::reference_wrapper<Reconstruction::Frame>> frameVectorRef(fragment.begin(),fragment.end());

        Reconstruction::LocalRefiner::refineAndCreateMesh(frameVectorRef,config);
    }
}
