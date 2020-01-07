//
// Created by jojo on 07.01.20.
//

#ifndef RECONSTRUCTION3D_GEOMETRYMETHODS_H
#define RECONSTRUCTION3D_GEOMETRYMETHODS_H

#include "Frame.h"
#include "util/Parser.h"
#include "Open3D/Open3D.h"

namespace Reconstruction
{
    class GeometryMethods
    {
    public:
        static bool createRGBDImageFromFrame(const Frame frame, const Parser config, open3d::geometry::RGBDImage& rgbd);

    };
}



#endif //RECONSTRUCTION3D_GEOMETRYMETHODS_H
