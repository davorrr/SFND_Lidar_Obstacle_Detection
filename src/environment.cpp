/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include <unordered_set>

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // // ----------------------------------------------------
    // // -----Open 3D viewer and display simple highway -----
    // // ----------------------------------------------------
    
    // // RENDER OPTIONS
    // bool renderScene = false;
    // std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // // TODO:: Create lidar sensor 
    // Lidar* LidarSensor = new Lidar(
    //     cars, 0
    // );

    // pcl::PointCloud<pcl::PointXYZ>::Ptr InputCloud = LidarSensor->scan();

    // // renderRays(viewer, LidarSensor->position, InputCloud);


    // renderPointCloud(viewer, InputCloud, "Point Cloud");



    // // TODO:: Create point processor

    // //ProcessPointClouds<pcl::PointXYZ> pointProcessor;

    // ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl::PointXYZ>();
    
    // std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor->SegmentPlane(InputCloud, 100, 0.5);
    // renderPointCloud(viewer, segmentCloud.first, "ObstacleCloud", Color(1,0,0));
    // renderPointCloud(viewer, segmentCloud.second, "PlaneCloud", Color(0,1,0));

    // // Adding code for rendering clusters

    // std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor->Clustering(segmentCloud.first, 1.0, 3, 30);

    // int clusterId = 0;
    // std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    // for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    // {
    //     std::cout << "Cluster size: ";
    //     pointProcessor->numPoints(cluster);
    //     renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId]);
    //     ++clusterId;
    // }


}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{


    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud (new pcl::PointCloud<pcl::PointXYZI>);
    
    filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.5f, Eigen::Vector4f(-30, -6, -10, 1), Eigen::Vector4f(30, 6, 10, 1));

    // std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlane(filterCloud, 100, 0.3);
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->SegmentPlaneCustomRansac(filterCloud, 100, 0.3);

    //renderPointCloud(viewer, segmentCloud.first, "ObstacleCloud", Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second, "PlaneCloud", Color(0,1,0));

	auto startTime = std::chrono::steady_clock::now();

    std::vector<std::vector<int>> clustersIndices = pointProcessorI->clusteringCustom(segmentCloud.first, 0.7, 4, 160);

    // std::vector<std::vector<int>> clustersIndices = pointProcessorI->clusteringCustomOptimized(segmentCloud.first, 0.8, 5, 100);


   // std::vector<std::vector<int>> clusters = pointProcessorI->euclideanCluster(segmentCloud.first, tree, 3.0);

  	auto endTime = std::chrono::steady_clock::now();
  	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
  	std::cout << "clustering found " << clustersIndices.size() << " and took " << elapsedTime.count() << " milliseconds" << std::endl;

  	// Render clusters
  	int clusterId = 0;
	std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};
  	for(std::vector<int> cluster : clustersIndices)
  	{
  		pcl::PointCloud<pcl::PointXYZI>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZI>());
  		for(int indice: cluster)
            clusterCloud->points.push_back(segmentCloud.first->points[indice]);

  		renderPointCloud(viewer, clusterCloud,"cluster"+std::to_string(clusterId),Color(0,0,1));
  		++clusterId;

        Box box = pointProcessorI->BoundingBox(clusterCloud);
        renderBox(viewer, box, clusterId);
  	}
  	
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    // simpleHighway(viewer);
    //cityBlock(viewer);

    while (!viewer->wasStopped ())
    {

        //clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        //load PCD and run it through obstacle detection pipeline
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();


        viewer->spinOnce ();
    } 
}