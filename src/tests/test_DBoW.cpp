//
// Created by jojo on 11.01.20.
//
#include <iostream>
#include <vector>

#include "DBoW2.h"
//#include "../../thrid_party/DBoW2/DBoW2/TemplatedVocabulary.h"
//#include "../../thrid_party/DBoW2/DBoW2/TemplatedDatabase.h"
//#include "../../thrid_party/DBoW2/DBoW2/QueryResults.h"
//#include "../../thrid_party/DBoW2/DBoW2/FORB.h"

#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
typedef DBoW2::TemplatedVocabulary<DBoW2::FORB::TDescriptor, DBoW2::FORB>
        OrbVocabulary;
typedef DBoW2::TemplatedDatabase<DBoW2::FORB::TDescriptor, DBoW2::FORB>
        OrbDatabase;

void loadFeatures(std::vector<std::vector<cv::Mat>>& features);
void changeStructure(const cv::Mat &plain, std::vector<cv::Mat> &out);
void vocCreation(const std::vector<std::vector<cv::Mat > > &features);
void dbCreation(const std::string voc_name);
void loadFeature(std::vector<cv::Mat>& feature,int i);
int main(int argc,char** argv)
{
//    OrbDatabase db("small_db.yml.gz");
//    DBoW2::QueryResults ret;
//    int i = std::stoi(argv[1]) ;
//    std::vector<cv::Mat> feature;
//    loadFeature(feature,i);
//    db.query(feature,ret,4);
//    std::cout<<i<<"."<<ret<<std::endl;

    std::vector<std::vector<cv::Mat>> features;
    loadFeatures(features);
    vocCreation(features);
//    dbCreation("small_voc.yml.gz");
    return 0;
}
void loadFeature(std::vector<cv::Mat>& feature,int i)
{
    cv::Ptr<cv::ORB> orb = cv::ORB::create();

    std::stringstream ss;
    char* cam_id_str = new char[7];
    sprintf(cam_id_str,"%06d",i);
    ss << "../dataset/ITE_Logo/ir/InfraredRealsense" << cam_id_str << ".png";

    cv::Mat image = cv::imread(ss.str(), cv::IMREAD_GRAYSCALE);
    cv::Mat mask;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    orb->detectAndCompute(image, mask, keypoints, descriptors);
    changeStructure(descriptors,feature);
}
void loadFeatures(std::vector<std::vector<cv::Mat>>& features)
{
    features.clear();
//    features.reserve(2);
    cv::Ptr<cv::ORB> orb = cv::ORB::create();

    for(int i = 170; i < 750; ++i)
    {
        std::stringstream ss;
        char* cam_id_str = new char[7];
        sprintf(cam_id_str,"%06d",i);
        ss << "../dataset/ITE_Logo/ir/InfraredRealsense" << cam_id_str << ".png";

        cv::Mat image = cv::imread(ss.str(), cv::IMREAD_GRAYSCALE);
        cv::Mat mask;
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;

        orb->detectAndCompute(image, mask, keypoints, descriptors);
        features.push_back(std::vector<cv::Mat >());
        changeStructure(descriptors, features.back());
    }
}

void changeStructure(const cv::Mat &plain, std::vector<cv::Mat> &out)
{
    out.resize(plain.rows);

    for(int i = 0; i < plain.rows; ++i)
    {
        out[i] = plain.row(i);
    }

}
void vocCreation(const std::vector<std::vector<cv::Mat > > &features)
{
    using namespace std;
    using namespace DBoW2;
    OrbVocabulary voc;
    voc.create(features);
    voc.save("small_voc.yml.gz");

    cout << "Vocabulary information: " << endl
         << voc << endl << endl;

    // lets do something with this vocabulary
    cout << "Matching images against themselves (0 low, 1 high): " << endl;
    BowVector v1, v2;
    for(int i = 0; i < 5; i++)
    {
        voc.transform(features[i], v1);
        for(int j = i; j < 5; j++)
        {
            voc.transform(features[j], v2);

            double score = voc.score(v1, v2);
            cout << "Image " << i << " vs Image " << j << ": " << score << endl;
        }
    }
}

void dbCreation(const std::string voc_name)
{
    OrbVocabulary voc(voc_name);
    OrbDatabase db(voc, false, 0);
    db.save("small_db.yml.gz");
}