/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../processPointClouds.h"
#include "../../render/render.h"
#include <unordered_set>
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <cmath>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  // Add inliers
  float scatter = 0.6;
  for (int i = -5; i < 5; i++) {
    double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    pcl::PointXYZ point;
    point.x = i + scatter * rx;
    point.y = i + scatter * ry;
    point.z = 0;

    cloud->points.push_back(point);
  }
  // Add outliers
  int numOutliers = 10;
  while (numOutliers--) {
    double rx = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    double ry = 2 * (((double)rand() / (RAND_MAX)) - 0.5);
    pcl::PointXYZ point;
    point.x = 5 * rx;
    point.y = 5 * ry;
    point.z = 0;

    cloud->points.push_back(point);
  }
  cloud->width = cloud->points.size();
  cloud->height = 1;

  return cloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D() {
  ProcessPointClouds<pcl::PointXYZ> pointProcessor;
  return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}

pcl::visualization::PCLVisualizer::Ptr initScene() {
  pcl::visualization::PCLVisualizer::Ptr viewer(
      new pcl::visualization::PCLVisualizer("2D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->initCameraParameters();
  viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  viewer->addCoordinateSystem(1.0);
  return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                               int maxIterations, float distanceTol) {
  std::unordered_set<int> inliersResult;
  srand(time(NULL));

  // TODO: Fill in this function
  int size = (int)(cloud->size());

  int numberInliers = 0;
  int previousMax = -1;

  double A, B, C;
  // variables to store the most optimal value;

  // For max iterations
  for (int i = 0; i < maxIterations; ++i) {
    // Randomly sample subset and fit line
    int randNum1 = rand() % (size);
    int randNum2 = rand() % (size);

    auto &point1 = cloud->points[randNum1];
    auto &point2 = cloud->points[randNum2];

    // We need to do ax +by+ c = 0 form, so we can take out the distance later
    auto a = point1.y - point2.y;
    auto b = point2.x - point1.x;
    auto c = point1.x * point2.y - point2.x * point1.y;

    for (int i = 0; i < size; ++i) {
      const auto &point = cloud->points;
      const auto &x = point[i].x;
      const auto &y = point[i].y;

      auto d = fabs(a * x + b * y + c) / (sqrt(a * a + b * b));
      if ((float)d < distanceTol) {
        inliersResult.insert(i);
      }
    }
    numberInliers = inliersResult.size();
    if (numberInliers > previousMax) {
      // STORE A,B,C;
      A = a;
      B = b;
      C = c;
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

    auto d = abs(A * x + B * y + C) / (sqrt(A * A + B * B));
    if ((float)d < distanceTol) {
      inliersResult.insert(i);
    }
  }

  return inliersResult;
}

std::unordered_set<int> Ransac3d(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                 int maxIterations, float distanceTol) {
  std::unordered_set<int> inliersResult;
  srand(time(NULL));

  // TODO: Fill in this function
  int size = (int)(cloud->size());
  std::cout << " The size is " << size << endl;

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
    numberInliers = inliersResult.size();
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

int main() {

  // Create viewer
  pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

  // Create data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

  // TODO: Change the max iteration and distance tolerance arguments for Ransac
  // function
  std::unordered_set<int> inliers = Ransac3d(cloud, 100, 0.2);

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(
      new pcl::PointCloud<pcl::PointXYZ>());

  for (int index = 0; index < cloud->points.size(); index++) {
    pcl::PointXYZ point = cloud->points[index];
    if (inliers.count(index))
      cloudInliers->points.push_back(point);
    else
      cloudOutliers->points.push_back(point);
  }

  // Render 2D point cloud with inliers and outliers
  if (inliers.size()) {
    renderPointCloud(viewer, cloudInliers, "inliers", Color(0, 1, 0));
    renderPointCloud(viewer, cloudOutliers, "outliers", Color(1, 0, 0));
  } else {
    renderPointCloud(viewer, cloud, "data");
  }

  while (!viewer->wasStopped()) {
    viewer->spinOnce();
  }
}
