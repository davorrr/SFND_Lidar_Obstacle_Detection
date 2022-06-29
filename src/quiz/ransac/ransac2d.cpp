/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));

	while(maxIterations--)
	{
		// Line formula
		// Ax + By + C = 0

		// Distance
		// d = |Ax + Bx +C|/sqrt(sqr(A) + sqr(B))

		// If we have 2 points x1, y1 and x2, y2 the line equiation is
		// (y2 - y1) + (x2 - x1) + (x1*y2 - x2*y1) = 0


		// PROGRAM MODIFED FOR PLANES!
		std::unordered_set<int> inliers;

		//Randomly pick a point. Finds 2 random indexes and inserts them in an unordered set.
		//These 2 indexses are later used to find 2 random points in the cloud.
		//Because it's an unordered set, that prevents 2 same random points to be added to it.
		while(inliers.size() < 3)
			inliers.insert(rand()%(cloud->points.size()));

		float x1, x2, x3, y1, y2, y3, z1, z2, z3;

		auto itr = inliers.begin(); // Returns an iterator, not an actual value. 
									// Actual value is taken by dereferencing an iterator 

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

		//vector<float> v1{x2 - x1, y2 - y2, z2 - z1};
		//vector<float> v2{x3 - x1, y3 - y1, z3 - z1};

		float i = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
		float j = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1);
		float k = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);

		float D = -(i*x1 + j*y1 + k*z1);
		//v1 = <x2 - x1, y2 - y2, z2 - z1>;
		//v2 = <x3 - x1, y3 - y1, z3 - z1>;


		// float a = y1 - y2;
		// float b = x2 - x1;
		// float c = x1*y2 - x2*y1;

		// Iterating through all the points in the cloud
		for(int index = 0; index < cloud->points.size(); index++)
		{
			// If the inlier is already a part of the fitted line we are not going to do anything
			if(inliers.count(index) > 0) // counts a number of elements with value = index. IF bigger then 0 skip the loop
				continue;
			// Taking a random point from the cloud
			pcl::PointXYZ point = cloud->points[index];
			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;

			// Calculating the distance
			float d = fabs(i*x4 + j*y4 + k*z4 + D)/sqrt(i*i + j*j + k*k);

			// IF calculated distance is lower then distance tolerance we insert the index into inliers set
			if(d <= distanceTol)
				inliers.insert(index);
		}
		if(inliers.size() > inliersResult.size())
			inliersResult = inliers;
	}
	
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 100, 0.3);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}