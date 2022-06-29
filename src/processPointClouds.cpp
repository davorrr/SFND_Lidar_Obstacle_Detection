// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);

    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -.4, 1));;
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for(int point : indices)
        inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane

     pcl::SACSegmentation<PointT> seg;
  // Create 2 new point cloud pointers

  // Point Cloud 1 - obstacles
    typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
  // Point Cloud 2 - Road
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());

    // Inliers can be added to the Road point cloud (Plane) looping over the inliers indices and pushing the corresponding inlier
    // point to the
    for(int index : inliers -> indices)
        planeCloud->points.push_back(cloud->points[index]);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstCloud);


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.


    pcl::SACSegmentation<PointT> seg;

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());

    seg.setOptimizeCoefficients(true);

    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if(inliers->indices.size() == 0){
        std::cout << "Could not estimate a planar model for a given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlaneCustomRansac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    // TODO:: Fill in this function to find inliers for the cloud.

    std::unordered_set<int> inliersResult;
	srand(time(NULL));

	while(maxIterations--)
	{

		std::unordered_set<int> inliers;

		while(inliers.size() < 3)
			inliers.insert(rand()%(cloud->points.size()));

		float x1, x2, x3, y1, y2, y3, z1, z2, z3;

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

		float i = (y2 - y1)*(z3 - z1) - (z2 - z1)*(y3 - y1);
		float j = (z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1);
		float k = (x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1);

		float D = -(i*x1 + j*y1 + k*z1);

		// Iterating through all the points in the cloud
		for(int index = 0; index < cloud->points.size(); index++)
		{
			// If the inlier is already a part of the fitted line we are not going to do anything
			if(inliers.count(index) > 0) // counts a number of elements with value = index. IF bigger then 0 skip the loop
				continue;
			// Taking a random point from the cloud
			pcl::PointXYZI point = cloud->points[index];
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

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

	
    //	Separating planes

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateCloudsCustom(inliersResult, cloud);

    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateCloudsCustom(std::unordered_set<int> inliersResult, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    pcl::PointCloud<pcl::PointXYZI>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZI>());
	pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZI>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZI point = cloud->points[index];
		if(inliersResult.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers, cloudInliers);


    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    for(pcl::PointIndices getIndices : clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for(int index : getIndices.indices)
            cloudCluster->points.push_back(cloud->points[index]);

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;
        clusters.push_back(cloudCluster);
        
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}



//void clusterHelper(int indice, const std::vector<std::vector<float>> points, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol)
template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelper(int indice, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& processed, std::shared_ptr<KdTree> tree, float distanceTol)
{

    // Implement max and min size checking in Search function
	processed[indice] = true;
	cluster.push_back(indice);

    std::vector<float> point;
    point.push_back(cloud->points[indice].x);
    point.push_back(cloud->points[indice].y);
    point.push_back(cloud->points[indice].z);


	std::vector<int> nearest = tree->search(point, distanceTol);

	for(int id : nearest)
	{
		if(!processed[id])
			clusterHelper(id, cloud, cluster, processed, tree, distanceTol);
	}
}


// template<typename PointT>
// void ProcessPointClouds<PointT>::clusterHelperOptimized(int indice, typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int>& cluster, std::vector<bool>& processed, KdTree* tree, float distanceTol)
// {

//     // Implement max and min size checking in Search function
// 	processed[indice] = true;
// 	cluster.push_back(indice);

//     std::vector<float> point;
//     point.push_back(cloud->points[indice].x);
//     point.push_back(cloud->points[indice].y);
//     point.push_back(cloud->points[indice].z);


// 	std::vector<int> nearest = tree->searchOptimized(point, distanceTol);


//     for(int id : nearest)
//     {
//         if(!processed[id])
//         {
//             processed[indice] = true;
// 	        cluster.push_back(indice);

//             std::vector<float> point;
//             point.push_back(cloud->points[indice].x);
//             point.push_back(cloud->points[indice].y);
//             point.push_back(cloud->points[indice].z);

//             nearest.push_back(tree->searchOptimized(point, distanceTol));
//         }
//     }
// }





// Below is the original fnction declaration - to be investigated: 1. why is const there?; 2. why is reference there?
//std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr cloud, std::shared_ptr<KdTree> tree, float distanceTol, int minSize, int maxSize)
{


	// TODO: Fill out this function to return list of indices for each cluster

    // Below are the actual points that are used as an argument of the euclideanClustering function
    //std::vector<std::vector<float>> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3}, {7.2,6.1}, {8.0,5.3}, {7.2,7.1}, {0.2,-7.1}, {1.7,-6.9}, {-1.2,-7.2}, {2.2,-8.9} };


	std::vector<std::vector<int>> clusters;

    //To be investigated if vector of vectors points can be replaced with:
    //cloud->points.size()
	std::vector<bool> processed(cloud->points.size(), false);

	int i = 0;
	while(i < cloud->points.size())
	{
		if(processed[i])
		{
			i++;
			continue;
		}

		std::vector<int> cluster;
		clusterHelper(i, cloud, cluster, processed, tree, distanceTol);
        if((cluster.size() >= minSize) && (cluster.size() <= maxSize))
		    clusters.push_back(cluster);
		i++;
	}
 
	return clusters;

}


template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanClusterOptimized(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree* tree, float distanceTol, int minSize, int maxSize)
{


	// TODO: Fill out this function to return list of indices for each cluster

    // Below are the actual points that are used as an argument of the euclideanClustering function
    //std::vector<std::vector<float>> points = { {-6.2,7}, {-6.3,8.4}, {-5.2,7.1}, {-5.7,6.3}, {7.2,6.1}, {8.0,5.3}, {7.2,7.1}, {0.2,-7.1}, {1.7,-6.9}, {-1.2,-7.2}, {2.2,-8.9} };


	std::vector<std::vector<int>> clusters;

    //To be investigated if vector of vectors points can be replaced with:
    //cloud->points.size()
	std::vector<bool> processed(cloud->points.size(), false);

	int i = 0;
	while(i < cloud->points.size())
	{
		if(processed[i])
		{
			i++;
			continue;
		}

		std::vector<int> cluster;
		
      /////ClusterHelper  
        processed[i] = true;
	    cluster.push_back(i);

        std::vector<float> point;
        point.push_back(cloud->points[i].x);
        point.push_back(cloud->points[i].y);
        point.push_back(cloud->points[i].z);


	    std::vector<int> nearest = tree->searchOptimized(point, distanceTol);


        for(int id : nearest)
        {
            if(!processed[id])
            {
                processed[id] = true;
	            cluster.push_back(id);

                std::vector<float> point;
                point.push_back(cloud->points[id].x);
                point.push_back(cloud->points[id].y);
                point.push_back(cloud->points[id].z);

                std::vector<int> nearest2 = tree->searchOptimized(point, distanceTol);
                nearest.insert(nearest.end(), nearest2.begin(), nearest2.end());
            }
        }
          
     //////////////////////////////////   
        if((cluster.size() >= minSize) && (cluster.size() <= maxSize))
		    clusters.push_back(cluster);
		i++;
	}
 
	return clusters;

}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::clusteringCustom(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

	//KdTree* tree = new KdTree;
    std::shared_ptr<KdTree> tree (std::make_shared<KdTree>());
    //int cloud_size = cloud->points.size();
    tree->insertOptimized(cloud);

    // for(int i=0; i<cloud->points.size(); i++)
    // {
    //     std::vector<float> point;
    //     point.push_back(cloud->points[i].x);
    //     point.push_back(cloud->points[i].y);
    //     point.push_back(cloud->points[i].z);
    //     tree->insertOptimized(point, i);
    //     // tree->insert(point, i);
    // }

    std::vector<std::vector<int>> clusters = euclideanCluster(cloud, tree, clusterTolerance, minSize, maxSize);

    return clusters;

}

// template<typename PointT>
// std::vector<std::vector<int>> ProcessPointClouds<PointT>::clusteringCustomOptimized(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
// {

// 	KdTree* tree = new KdTree;
  

//     for(int i=0; i<cloud->points.size(); i++)
//  	//for(int i=cloud->points.size()-1; i>=0; i--)
//     {
//         std::vector<float> point;
//         point.push_back(cloud->points[i].x);
//         point.push_back(cloud->points[i].y);
//         point.push_back(cloud->points[i].z);
//         tree->insertOptimized(point, i);
//     }

//     //tree->insertOptimized(cloud);


//     std::vector<std::vector<int>> clusters = euclideanClusterOptimized(cloud, tree, clusterTolerance, minSize, maxSize);

//     return clusters;

// }


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

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
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}