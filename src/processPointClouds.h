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



// BRING IN KD TREE HERE

struct Node
{
    std::vector<float> point;
    int id;
    Node* left;
    Node* right;

    Node(std::vector<float> arr, int setId)
            :	point(arr), id(setId), left(NULL), right(NULL)
    {}

    ~Node()
    {
        delete left;
        delete right;
    }
};

struct KdTree
{
    Node* root;

    KdTree()
            : root(nullptr)
    {}

    ~KdTree()
    {
        delete root;
    }

    void insert(std::vector<float> point, int id)
    {
        // TODO: Fill in this function to insert a new point into the tree
        // the function should create a new node and place correctly with in the root
        int depth=0;
        internalInsert(root, depth, point,id);

    }

    void internalInsert(Node * &node,int & depth , std::vector<float> point, int id ){
        if(node == nullptr){
            node = new Node(point,id );
        }
        else {
            auto indexToLook = depth % 3;
            depth = depth + 1;

            auto NodeCheckPoint = node->point[indexToLook];

            if (point[indexToLook] < NodeCheckPoint) {
                internalInsert(node->left, depth, point, id);
            } else {
                internalInsert(node->right, depth, point, id);
            }

        }
    }
// the idea is take [[x1,y1],[x2,y2]] and calculate the
// width = distanceTol
// x1 = xTarget-width, x2 = xTarget + width.
// y1 = yTarget - width, y2 = yTarget + width
    bool isInBox(std::vector<std::vector<float>>& boxCordinates, std::vector<float> point )
    {
        auto x = point[0];
        auto y = point[1];
        auto z = point[2];
        if((x> boxCordinates[0][0] && y > boxCordinates[0][1] && z >boxCordinates[0][2]) && (x < boxCordinates[1][0] && y < boxCordinates[1][1] && z < boxCordinates[1][2])){
            return true;
        }
        else{
            return false;
        }
    }

    // return a list of point ids in the tree that are within distance of target
    void internalSearch(Node*& node ,std::vector<int>& ids, std::vector<std::vector<float>>& boxCordinates,int depth , std::vector<float>& target ,
                        float distanceTol )
    {

        if (node != nullptr) {
            auto IndexToLook = depth%3;
            depth = depth +1;
            auto point = node->point;
            bool shouldInsert = isInBox(boxCordinates, point);
            if (shouldInsert) {
                float distance = sqrt((node->point[0] - target[0])*(node->point[0] - target[0]) +(node->point[1] - target[1])*(node->point[1] - target[1]) +(node->point[2] - target[2])*(node->point[2] - target[2]));
                if (distance < distanceTol){
                    ids.push_back(node->id);}
            }

            if(((target[IndexToLook]) - distanceTol) < node->point[IndexToLook]){
                internalSearch(node->left, ids, boxCordinates, depth, target,distanceTol);
            }
            if(((target[IndexToLook]) + distanceTol) > node->point[IndexToLook]){
                internalSearch(node->right, ids, boxCordinates, depth, target,distanceTol);
            }

        }


    }

    std::vector<int> search(std::vector<float> target, float distanceTol)
    {
        // Initially create the box coordinates for the isBox function to work
        auto x = target[0];
        auto y = target[1];
        auto z = target[2];
        std::vector<std::vector<float>> boxCordinates = {{x - distanceTol,y - distanceTol, z- distanceTol},{x + distanceTol, y+ distanceTol,z + distanceTol}};
        // Basic step
        // Loop over the tree and if the point is within the radius then measure the distane else leave. Return the ids that are within the box.
        std::vector<int> ids;
        int depth = 0;
        internalSearch(root,ids,boxCordinates,depth,target,distanceTol);

        return ids;
    }

};


// BRING IN THE EUCLEDIAN CLUSTERING CODE

inline void Proximity(int pointIndex, std::vector<int>& cluster,KdTree* &tree,float distanceTol,
               std::unordered_set<int>& ProcessedPoints,const std::vector<std::vector<float>>& points)
{
//    LOG("Reaching inside proximity");
    ProcessedPoints.insert(pointIndex);
    cluster.push_back(pointIndex);
    auto nearByPoints = tree->search(points[pointIndex],distanceTol);
    for(auto & nearPoint : nearByPoints )
    {
        if(ProcessedPoints.find(nearPoint) == ProcessedPoints.end())
        {
            Proximity(nearPoint,cluster,tree,distanceTol,ProcessedPoints,points);
        }

    }

}
//
//
inline std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

    // TODO: Fill out this function to return list of indices for each cluster

    std::vector<std::vector<int>> clusters;

    std::unordered_set<int> ProcessedPoints;
    for (int i=0; i<points.size(); i++)
    {

        if(ProcessedPoints.find(i)==ProcessedPoints.end()){
//            LOG("reaching inside the intial loop");
            std::vector<int> cluster;
            Proximity(i,cluster,tree,distanceTol,ProcessedPoints,points);
            clusters.push_back(cluster);
        }
    }

    return clusters;

}


#endif /* PROCESSPOINTCLOUDS_H_ */