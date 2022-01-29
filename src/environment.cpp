/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "processPointClouds.h"
#include "render/render.h"
#include "sensors/lidar.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene,
                             pcl::visualization::PCLVisualizer::Ptr &viewer) {

  Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
  Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
  Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
  Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

  std::vector<Car> cars;
  cars.push_back(egoCar);
  cars.push_back(car1);
  cars.push_back(car2);
  cars.push_back(car3);

  if (renderScene) {
    renderHighway(viewer);
    egoCar.render(viewer);
    car1.render(viewer);
    car2.render(viewer);
    car3.render(viewer);
  }

  return cars;
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer) {
  // ----------------------------------------------------
  // -----Open 3D viewer and display simple highway -----
  // ----------------------------------------------------
  // RENDER OPTIONS
  bool renderScene = false;
  std::vector<Car> cars = initHighway(renderScene, viewer);
  Lidar *lidar = new Lidar(cars, 0);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = lidar->scan();

  // renderRays( viewer,lidar->position, cloud);

  renderPointCloud(viewer, cloud, "point cloud");

  // TODO:: Create lidar sensor
  ProcessPointClouds<pcl::PointXYZ> processor;
  auto segmentCloud = processor.SegmentPlane(cloud, 100, 0.2);
//  renderPointCloud(viewer, segmentCloud.first, "obstacleCloud", Color(1, 0, 0));
//  renderPointCloud(viewer, segmentCloud.second, "PlaneCloud", Color(0, 1, 0));
  // TODO:: Create point processor
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = processor.Clustering(segmentCloud.first,1.0,10,500);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for( auto cluster : cloudClusters){
        std::cout<< " cluster size ";
        processor.numPoints(cluster);
        Box box = processor.BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        ++clusterId;
    }

}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& Cloud)
{
//    auto* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
//    auto Cloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
//    renderPointCloud(viewer,inputCloud,"inputCloud");
    auto inputCloud = pointProcessorI->FilterCloud(Cloud,.1f,Eigen::Vector4f(-10.0,-5.0,-2.0,10.0),Eigen::Vector4f(30.0,8.0,1.0,1.0));
    auto segmentedCloud = pointProcessorI->SegmentPlane(inputCloud,25,0.3);
//    renderPointCloud(viewer,segmentedCloud.first,"inputCloud");
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters  = pointProcessorI->Clustering(segmentedCloud.first,0.5,100,3000);

//    for (int i =0 ;i< 10; i++){
//        renderPointCloud(viewer,cloudClusters[i],"random");
//    }
    int clusterID = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
    for(auto cluster : cloudClusters)
    {   std::cout<< " cluster size ";
        pointProcessorI->numPoints(cluster);
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer,box,clusterID);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterID),colors[clusterID]);
        ++clusterID;
    }
}

void mycityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* &pointProcessorI)
{
    auto Cloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    auto inputCloud = pointProcessorI->FilterCloud(Cloud,.1f,Eigen::Vector4f(-10.0,-5.0,-2.0,10.0),Eigen::Vector4f(30.0,8.0,1.0,1.0));
    auto segmentedCloud = pointProcessorI->SegmentPlane(inputCloud,100,0.3);
//    renderPointCloud(viewer, segmentedCloud.first, "obstacleCloud", Color(1, 0, 0));
    renderPointCloud(viewer, segmentedCloud.second, "PlaeCloud", Color(1, 0, 0));
}
/*FINAL PROJECT
 *1) Initial things is fitlering ? But we dnt have any function implemented for it yet? ---> CAN BE DONE LAST IF NOT NECESSARY SPEED IS THERE
 *2) The next thing is segmentation , for that we have implemented the 3D ransac, Do that and render and see if its working?
 *3) Here we use  the kd tree to search for clusters, but i think it  was implemeted for 2D case should we extend it to 3D Case?
 *
 *
 */


// setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle,
                pcl::visualization::PCLVisualizer::Ptr &viewer) {

  viewer->setBackgroundColor(0, 0, 0);

  // set camera position and angle
  viewer->initCameraParameters();
  // distance away in meters
  int distance = 10;

  switch (setAngle) {
  case XY:
    viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
    break;
  case TopDown:
    viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
    break;
  case Side:
    viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
    break;
  case FPS:
    viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
  }

  if (setAngle != FPS)
    viewer->addCoordinateSystem(1.0);
}

int main(int argc, char **argv) {
  std::cout << "starting enviroment" << std::endl;

  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  CameraAngle setAngle = XY;
  initCamera(setAngle, viewer);
//  simpleHighway(viewer);
    ProcessPointClouds<pcl::PointXYZI> * pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    mycityBlock(viewer,pointProcessorI);
    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    }


/*
ProcessPointClouds<pcl::PointXYZI> * pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
auto streamIterator = stream.begin();
pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

//    cityBlock(viewer);
    while (!viewer->wasStopped ())
    {

        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce ();
    }

    */
}