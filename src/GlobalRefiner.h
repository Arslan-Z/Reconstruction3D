//
// Created by jojo on 05.01.20.
//

#ifndef RECONSTRUCTION3D_GLOBALREFINER_H
#define RECONSTRUCTION3D_GLOBALREFINER_H

#include "util/Parser.h"
#include "RegistrationMethods.h"
#include "PoseGraphMethods.h"
#include "Open3D/Open3D.h"

namespace Reconstruction
{
    class GlobalRefiner
    {
    public:
        static void refine(const std::string config_file, const std::string global_poseGraphName);

    private:
        static void registerPointCloudPair(Parser config, PoseGraphMethods::MatchingResult& match);
    };
}




#endif //RECONSTRUCTION3D_GLOBALREFINER_H
