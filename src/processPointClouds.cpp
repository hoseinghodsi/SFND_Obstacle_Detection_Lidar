// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>

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

	auto startTime = std::chrono::steady_clock::now();

	pcl::VoxelGrid<PointT> vg;
	typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);

	vg.setInputCloud(cloud);
	vg.setLeafSize(filterRes, filterRes, filterRes);
	vg.filter(*cloudFiltered);

	typename pcl::PointCloud<PointT>::Ptr cloudRegion(new pcl::PointCloud<PointT>);
	pcl::CropBox<PointT> region(true);
	region.setMin(minPoint);
	region.setMax(maxPoint);
	region.setInputCloud(cloudFiltered);
	region.filter(*cloudRegion);

	std::vector<int> indices;

	pcl::CropBox<PointT> roof(true);
	roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
	roof.setMax(Eigen::Vector4f(2.6, 1.7, -.4, 1));
	roof.filter(indices);

	pcl::PointIndices::Ptr inliers{ new pcl::PointIndices };
	for (int point : indices)
		inliers->indices.push_back(point);

	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(cloudRegion);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*cloudRegion);
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
	
	typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT> ());
	typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT> ());

	for (int index : inliers->indices)
	{
		planeCloud->points.push_back(cloud->points[index]);
	}
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud(cloud);
	extract.useIndices(inliers);
	extract.setNegative(true);
	extract.filter(*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud, cloud);
    return segResult;
}

template<typename PointT>
float ProcessPointClouds<PointT>::dotProduct(std::vector<float> vec1, std::vector<float> vec2)
{
	float product = 0;
	for (int index = 0; index < vec1.size(); index++)
	{
		product += vec1[index] * vec2[index];
	}

	return product;
}
template<typename PointT>
std::vector<float> ProcessPointClouds<PointT>::crossProduct(std::vector<float> vec1, std::vector<float> vec2)
{
	std::vector<float> normalPlane;
	normalPlane.push_back(vec1[1] * vec2[2] - vec1[2] * vec2[1]);
	normalPlane.push_back(-(vec1[0] * vec2[2] - vec1[2] * vec2[0]));
	normalPlane.push_back(vec1[0] * vec2[1] - vec1[1] * vec2[0]);

	return normalPlane;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
	std::unordered_set<int> inlierPoints;
	
	while (maxIterations--)
	{
		std::unordered_set<int> inliers;
		while (inliers.size() < 3)
			inliers.insert(rand() % (cloud->points.size()));

		float x0, y0, z0;
		float x1, y1, z1;
		float x2, y2, z2;

		auto itr = inliers.begin();
		x0 = cloud->points[*itr].x;
		y0 = cloud->points[*itr].y;
		z0 = cloud->points[*itr].z;
		itr++;
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;

		std::vector<float> vec1;
		vec1.push_back(x1 - x0);
		vec1.push_back(y1 - y0);
		vec1.push_back(z1 - z0);
		std::vector<float> vec2;
		vec2.push_back(x2 - x0);
		vec2.push_back(y2 - y0);
		vec2.push_back(z2 - z0);

		std::vector<float> normalVector = crossProduct(vec1, vec2);

		std::vector<float> planeEquation;
		planeEquation.push_back(normalVector[0]);
		planeEquation.push_back(normalVector[1]);
		planeEquation.push_back(normalVector[2]);
		planeEquation.push_back(-(normalVector[0] * x0 + normalVector[1] * y0 + normalVector[2] * z0));


		for (int index = 0; index < cloud->points.size(); index++)
		{
			if (inliers.count(index) > 0)
				continue;

			pcl::PointXYZ point = cloud->points[index];
			std::vector<float> newPoint = { point.x, point.y, point.z };

			float dist = fabs(dotProduct(normalVector, newPoint) + planeEquation[3]) / sqrt(planeEquation[0] * planeEquation[0] + planeEquation[1] * planeEquation[1] + planeEquation[2] * planeEquation[2]);

			if (dist <= distanceThreshold)
				inliers.insert(index);
		}
		if (inliers.size() > inlierPoints.size())
			inlierPoints = inliers;
	}
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for (int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if (inlierPoints.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}

	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudInliers, cloudOutliers);
	return segResult;

}

//template<typename PointT>
//std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
//{
//    // Time segmentation process
//    auto startTime = std::chrono::steady_clock::now();
//	pcl::PointIndices::Ptr inliers;
//    // TODO:: Fill in this function to find inliers for the cloud.
//
//    auto endTime = std::chrono::steady_clock::now();
//    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
//    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
//
//    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
//    return segResult;
//}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


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