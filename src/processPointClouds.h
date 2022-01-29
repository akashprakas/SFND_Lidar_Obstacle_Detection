// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"
#include <unordered_set>


template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
  
};


template<typename PointT>
std::unordered_set<int> Ransac3d(typename  pcl::PointCloud<PointT>::Ptr cloud,
                                 int maxIterations, float distanceTol) {
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    // TODO: Fill in this function
    int size = (int)(cloud->size());
    std::cout << " The size is " << size << std::endl;

    int numberInliers = 0;
    int previousMax = 0;

    float AG, BG, CG, DG;

    // For max iterations
    for (int i = 0; i < maxIterations; ++i) {

        std::unordered_set<int> inliers;
        while (inliers.size() < 3)
            inliers.insert(rand() % size);

        float x1, y1, x2, y2, x3, y3, A, B, C, D, d, z1, z2, z3;

        auto itr = inliers.begin();
        x1 = cloud->points[*itr].x;
        y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
        itr++;
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
        itr++;
        x3 = cloud->points[*itr].x;
        y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;

        float ni, nj, nk;
        ni = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
        nj = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
        nk = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);

        A = ni;
        B = nj;
        C = nk;
        D = -(ni * x1 + nj * y1 + nk * z1);

        for (int i = 0; i < size; ++i) {
            if (inliers.count(i) > 0)
                continue;
            const auto &point = cloud->points;
            const auto &x = point[i].x;
            const auto &y = point[i].y;
            const auto &z = point[i].z;

            auto d = fabs(A * x + B * y + C * z + D) / (sqrt(A * A + B * B + C * C));
            if (d <= distanceTol) {
                inliersResult.insert(i);
            }
        }
        numberInliers = (int)inliersResult.size();
        if (numberInliers > previousMax) {
            // STORE A,B,C;
            AG = A;
            BG = B;
            CG = C;
            DG = D;
            previousMax = numberInliers;
        }
        inliersResult.clear();
    }

    // Measure distance between every point and fitted line
    // If distance is smaller than threshold count it as inlier

    // Return indicies of inliers from fitted line with most inliers
    inliersResult.clear();
    for (int i = 0; i < size; ++i) {
        const auto &point = cloud->points;
        const auto &x = point[i].x;
        const auto &y = point[i].y;
        const auto &z = point[i].z;

        auto d = fabs(AG * x + BG * y + CG * z + DG) /
                 (sqrt(AG * AG + BG * BG + CG * CG));

        // auto d =
        // fabs(AG * x + BG * y + CG + DG) / (sqrt(AG * AG + BG * BG + CG * CG));
        if (d < distanceTol) {
            inliersResult.insert(i);
        }
    }

    return inliersResult;
}




#endif /* PROCESSPOINTCLOUDS_H_ */