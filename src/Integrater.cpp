#include "Integrater.h"
#include "util/Parser.h"
#include "GeometryMethods.h"

using namespace Reconstruction;

Integrater::Integrater()
{

}
void Integrater::init(std::string strSettingPath)
{
    Parser parser;
    parser.load(strSettingPath);
    int width = parser.getValue<int>("Camera.width");
    int height = parser.getValue<int>("Camera.height");
    double fx = parser.getValue<double>("Camera.fx");
    double fy = parser.getValue<double>("Camera.fy");
    double cx = parser.getValue<double>("Camera.cx");
    double cy = parser.getValue<double>("Camera.cy");
    mIntrinsic.SetIntrinsics(width,height,fx,fy,cx,cy);
    mTSDF_param.tsdf_size = parser.getValue<float>("TSDF.size");

    mTSDF_param.tsdf_res = parser.getValue<float>("TSDF.resolution");
    mTSDF_param.depth_factor = parser.getValue<double>("TSDF.depth_factor");
    mTSDF_param.depth_truncate = parser.getValue<double>("TSDF.depth_truncate");
    mTSDF_param.tsdf_voxel_size = parser.getValue<float>("TSDF.voxel_size");

    mVolume_ptr.reset(
            new open3d::integration::ScalableTSDFVolume
            (mTSDF_param.tsdf_size/mTSDF_param.tsdf_res,
             4*mTSDF_param.tsdf_size/mTSDF_param.tsdf_res,
             open3d::integration::TSDFVolumeColorType::Gray32)
            );



}


bool Integrater::integrateFrame(const Frame frame)
{
    using namespace open3d;
    geometry::Image color;
    geometry::Image depth;
    geometry::Image infraRed;

    bool read = false;
    read = io::ReadImage(frame.getRGBImagePath().c_str(), color);
    if(!read)
        return false;
    read = io::ReadImage(frame.getDepthImagePath().c_str(), depth);
    if(!read)
        return false;
    read = io::ReadImageFromPNG(frame.getInfraRedImagePath().c_str(),infraRed);
    if(!read)
        return false;
    auto rgbd = geometry::RGBDImage::CreateFromColorAndDepth(
            infraRed, depth, mTSDF_param.depth_factor,
            mTSDF_param.depth_truncate, true);


    Eigen::Matrix4d extrinsic = Eigen::Matrix4d::Identity();

    auto pose = frame.getConstTcw();

    extrinsic.topLeftCorner(3,3)<<pose(0,0),pose(0,1),pose(0,2),
            pose(1,0),pose(1,1),pose(1,2),
            pose(2,0),pose(2,1),pose(2,2);
    extrinsic.topRightCorner(3,1)<<pose(0,3),pose(1,3),pose(2,3);

    mVolume_ptr->Integrate(*rgbd,
                           mIntrinsic,
                           extrinsic);
}

bool Integrater::saveTSDF(const char* path)
{
//    auto mesh = mVolume.ExtractTriangleMesh();
//                    io::WriteTriangleMesh("mesh_" + save_index_str + ".ply",
//                                          *mesh);
    return true;
}
bool Integrater::generatePointCloud(bool visualize)
{
    using namespace open3d;
    auto pcd = mVolume_ptr->ExtractPointCloud();
    io::WritePointCloud("pcd/pcd"+std::to_string(id) + ".ply",*pcd,false,true);
    if(visualize)
        visualization::DrawGeometriesWithCustomAnimation(
                {pcd}, "Animation", 1920, 1080);
}
bool Integrater::generateMesh(bool visualize)
{
    using namespace open3d;
    auto mesh = mVolume_ptr->ExtractTriangleMesh();
//    mesh->ComputeVertexNormals();
    io::WriteTriangleMesh("mesh/mesh.ply",*mesh);

    if(visualize)
        visualization::DrawGeometriesWithCustomAnimation(
            {mesh}, "Animation", 1920, 1080);
}

void Integrater::integrate(Parser config, std::string poseGraphName, size_t n_fragments, const FrameVector frameVector)
{
    using namespace open3d;
    auto volume = Reconstruction::Integrater::createVolume(config);
    registration::PoseGraph global_poseGraph;
    io::ReadPoseGraph(poseGraphName,global_poseGraph);

    for(size_t fragment_id = 0; fragment_id < global_poseGraph.nodes_.size(); fragment_id++) //global_poseGraph.nodes_.size()-1
    {
        auto Twc0 = global_poseGraph.nodes_[fragment_id].pose_;
        Reconstruction::Integrater::integrateFragment(config,volume,fragment_id,frameVector,Twc0);//todo
    }
//    auto Tc0w = global_poseGraph.nodes_[11].pose_.inverse();
//    Reconstruction::Integrater::integrateFragment(config,volume,11,frameVector,Tc0w);
    open3d::io::WriteTriangleMesh(Parser::FianalPlyName(),*volume->ExtractTriangleMesh());
    open3d::visualization::DrawGeometries({volume->ExtractTriangleMesh()});
}



Integrater::Volume Integrater::createVolume(Parser config)
{
    using namespace open3d;

    double voxel_size = config.getValue<double>("Integrater.volume_size")/config.getValue<double>("Integrater.resolution");
    double sdf_trunc = config.getValue<double>("Integrater.sdf_trunc");
    bool color = config.getValue<bool>("Integrater.color");
    integration::TSDFVolumeColorType type;
    if(color)
        type = integration::TSDFVolumeColorType::RGB8;
    else
        type = integration::TSDFVolumeColorType::Gray32;

    Volume volume(new integration::ScalableTSDFVolume(voxel_size,sdf_trunc,type));

    return volume;
}

void Integrater::integrateFragment(Parser config, Integrater::Volume volume, const size_t fragment_id,
                                   const FrameVector frameVector, const Eigen::Matrix4d Twc0)
{
    using namespace open3d;


    size_t n_frame_per_fragment = config.getValue<int>("n_frames_per_fragment");

    FrameVector local_frameVec;
    local_frameVec.insert(local_frameVec.end(),
            frameVector.begin()+fragment_id*n_frame_per_fragment,
                          std::min(frameVector.begin()+(fragment_id+1)*n_frame_per_fragment,frameVector.end()));

    registration::PoseGraph fragment_poseGraph;
    io::ReadPoseGraph(Parser::poseGraphName(fragment_id),fragment_poseGraph);

    for(size_t node_id = 0; node_id < fragment_poseGraph.nodes_.size(); node_id++)
    {
        if(node_id%2 != 0) //every two frames todo
            continue;

        auto node = fragment_poseGraph.nodes_[node_id];
        auto frame = local_frameVec[node_id];
        auto Tc0cn = node.pose_;
        auto pose = Twc0 * Tc0cn;
        geometry::RGBDImage rgbd;
        GeometryMethods::createRGBDImageFromFrame(frame,config,rgbd,false);


        int width = config.getValue<int>("Camera.width");
        int height = config.getValue<int>("Camera.height");
        double fx = config.getValue<double>("Camera.fx");
        double fy = config.getValue<double>("Camera.fy");
        double cx = config.getValue<double>("Camera.cx");
        double cy = config.getValue<double>("Camera.cy");

        camera::PinholeCameraIntrinsic intrinsic;
        intrinsic.SetIntrinsics(width,height,fx,fy,cx,cy);

        volume->Integrate(rgbd,intrinsic,pose.inverse());

    }

}
