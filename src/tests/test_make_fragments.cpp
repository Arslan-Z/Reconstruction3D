//
// Created by jojo on 31.12.19.
//
#include "stdio.h"
#include "../io/Reader.h"
#include "../Integrater.h"
#include "../FragmentMaker.h"
int main(int argc,char** argv)
{
    auto frameVec = Reconstruction::io::Reader::readITEFrames("../dataset/ITE_Logo/cameras.txt",
            "../dataset/ITE_Logo/");
    std::string config_file = "../dataset/ITE_Logo/ITE.yaml";
    Reconstruction::FragmentMaker fragmentMaker;

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
        static size_t i = 0;
        Reconstruction::FragmentMaker::processSingleFragment(i,fragment,config_file);
        i++;
    }
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
