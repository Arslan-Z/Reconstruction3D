//
// Created by jojo on 05.01.20.
//
#include "stdio.h"
#include "../util/Parser.h"
#include "../GlobalRefiner.h"
int main(int argc,char** argv)
{
    using namespace Reconstruction;
    GlobalRefiner::refine("../dataset/ITE_Logo/ITE.yaml","fragments/global_opt.json");
}