//
// Created by jojo on 31.12.19.
//
#include "stdio.h"
#include "../io/Reader.h"
#include "../FragmentMaker.h"

int main(int argc,char** argv)
{
    auto frameVec = Reconstruction::io::Reader::readITEFrames("../dataset/ITE_Logo/cameras.txt",
            "../dataset/ITE_Logo/");
    std::string config_file = "../dataset/ITE_Logo/ITE.yaml"    ;

    Reconstruction::FragmentMaker::makeFragments(frameVec,config_file);
    return 0;
}
