// PCL lib Functions for processing point clouds

#include "processPointClouds.h"

// constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

// de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(
        typename pcl::PointCloud<PointT>::Ptr cloud) {
    std::cout << cloud->points.size() << std::endl;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
        typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes,
        Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint) {

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region
    // based filtering
    typename  pcl::PointCloud<PointT>::Ptr cloudFiltered {new pcl::PointCloud<PointT>};
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(filterRes,filterRes,filterRes);
    sor.filter(*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion {new pcl::PointCloud<PointT>};

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5,1.7,-1,1));
    roof.setMax(Eigen::Vector4f {2.6,1.7,-.4,1});
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr  inliers {new pcl::PointIndices};
    for(int point : indices)
            inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
            endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds"
              << std::endl;

    return cloudRegion;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
        typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SeparateClouds(
        pcl::PointIndices::Ptr inliers,
        typename pcl::PointCloud<PointT>::Ptr cloud) {
    // TODO: Create two new point clouds, one cloud with obstacles and other with
    // segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud(
            new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr roadCloud(
            new pcl::PointCloud<PointT>());

    for (int index: inliers->indices) {
        roadCloud->points.push_back(cloud->points[index]);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacleCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
            segResult(obstacleCloud, roadCloud);
    // std::pair<typename pcl::PointCloud<PointT>::Ptr,
    //           typename pcl::PointCloud<PointT>::Ptr>
    //     segResult(cloud, cloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr,
        typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::SegmentPlane(
        typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations,
        float distanceThreshold) {

    auto startTime = std::chrono::steady_clock::now();

   auto PlaneInliers = Ransac3d<PointT>(cloud,maxIterations,distanceThreshold);
   // Now we have the result of index of the points in the plane we need to separate them to obstacle and palne points now
   //For that iterate through the cloud and if the index is not in inliers push to obstacle cloud and otherwise push to plane cloud

   typename  pcl::PointCloud<PointT>::Ptr plane_cloud{new pcl::PointCloud<PointT>};
   typename  pcl::PointCloud<PointT>::Ptr obstacle_cloud{new pcl::PointCloud<PointT>};

   for(int i = 0 ; i < (int)cloud->size();++i)
   {   // If point is  not in list mark as an obstacle otherwise mark it as plane
       if(PlaneInliers.find(i) == PlaneInliers.end()){
           obstacle_cloud->push_back((*cloud)[i]);
       }
       else{
           plane_cloud->push_back((*cloud)[i]);
       }
   }


    std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
            segResult(obstacle_cloud, plane_cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
            endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count()
              << " milliseconds" << std::endl;
//    std::cout << "The size of the cloud is " << cloud->height * cloud->width
//              << "  "
//              << "the size of inliers is " << inliers->indices.size()
//              << std::endl;

    return  segResult;


    /*
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
        std::cout << "could not estimate a plane \n";
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
            endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count()
              << " milliseconds" << std::endl;
    std::cout << "The size of the cloud is " << cloud->height * cloud->width
              << "  "
              << "the size of inliers is " << inliers->indices.size()
              << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr,
            typename pcl::PointCloud<PointT>::Ptr>
            segResult = SeparateClouds(inliers, cloud);
    return segResult;

     */
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr>
ProcessPointClouds<PointT>::Clustering(
        typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance,
        int minSize, int maxSize) {

/*1) First in need to build the tree from the points, so create points from cloud
 * 2) THen pass the tree to the euclidean cluster
 * 3) Then extract the point cloud according to the vector of indices
 */

    auto startTime = std::chrono::steady_clock::now();
    std::vector<std::vector<float>> points;
    KdTree* tree{new KdTree};
    for(int i = 0; (int)i<cloud->size();++i)
    {
        std::vector<float> insertPoint{cloud->points[i].x,cloud->points[i].y,cloud->points[i].z};
//        points.emplace_back(cloud->points[i].x,cloud->points[i].y,cloud->points[i].z);
        points.push_back(insertPoint);
        tree->insert(points[i],i);
    }

    std::vector<std::vector<int>> clusters = euclideanCluster(points, tree, clusterTolerance);
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusterToReturn;
    for(const auto & cluster : clusters){
        if(cluster.size() < minSize || cluster.size() > maxSize){
            continue;
        }
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        for(const auto & index : cluster)
        {
            cloud_cluster->push_back((*cloud)[index]);
        }
//        std::cout<< "The size of the cluster is " << cloud_cluster->size() << std::endl;
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        if (cloud_cluster->size() > minSize && cloud_cluster->size() < maxSize){
        clusterToReturn.push_back(cloud_cluster);}
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
            endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count()
              << " milliseconds and found " << clusters.size() << " clusters"
              << std::endl;

    return  clusterToReturn;

/*
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group
    // detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (auto it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster(new pcl::PointCloud<PointT>);
        for (const auto &idx: it->indices)
            cloud_cluster->push_back((*cloud)[idx]);

        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        clusters.push_back(cloud_cluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
            endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count()
              << " milliseconds and found " << clusters.size() << " clusters"
              << std::endl;

    return clusters;

    */
}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(
        typename pcl::PointCloud<PointT>::Ptr cluster) {

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(
        typename pcl::PointCloud<PointT>::Ptr cloud, std::string file) {
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file
              << std::endl;
}

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr
ProcessPointClouds<PointT>::loadPcd(std::string file) {

    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file
              << std::endl;

    return cloud;
}

template<typename PointT>
std::vector<boost::filesystem::path>
ProcessPointClouds<PointT>::streamPcd(std::string dataPath) {

    std::vector<boost::filesystem::path> paths(
            boost::filesystem::directory_iterator{dataPath},
            boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}