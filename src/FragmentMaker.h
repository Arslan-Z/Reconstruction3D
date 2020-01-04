//
// Created by jojo on 01.01.20.
//

#ifndef RECONSTRUCTION3D_FRAGMENTMAKER_H
#define RECONSTRUCTION3D_FRAGMENTMAKER_H

#include "Frame.h"
#include "util/Parser.h"

#include "Open3D/Open3D.h"
#include "Open3D/Registration/GlobalOptimization.h"
#include "Open3D/Utility/Console.h"
#define TemplatePoseGraphName(x) "fragments/fragment"+std::to_string(x)+".json"
#define TemplatePoinCloudName(x) "fragments/fragment"+std::to_string(x)+".ply"
namespace Reconstruction
{
    class FragmentMaker
    {
    public:
        static void processSingleFragment(size_t fragment_id, FrameVector frameVector, const std::string config_file);

    private:
        static void makePoseGraph(size_t fragment_id, FrameVector frameVector, Parser config);
        static void optimizePoseGraph(size_t fragment_id, Parser config);
        static void makePointCloud(size_t fragment_id,FrameVector frameVector, Parser config);
        static std::shared_ptr<open3d::geometry::PointCloud> createPoinCloudFromFrame(const Frame frame, const Parser config);

    };
}



#endif //RECONSTRUCTION3D_FRAGMENTMAKER_H
