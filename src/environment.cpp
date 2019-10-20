/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

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
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = true;
    std::vector<Car> cars = initHighway(renderScene, viewer);
  
}

// cityBlock method generates all the filtered, segmented, and clustered data for the entire stream
std::pair<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>, pcl::PointCloud<pcl::PointXYZI>::Ptr> cityBlock(ProcessPointClouds<pcl::PointXYZI> pointProcessor, 
																										     pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud)
{
	pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessor.FilterCloud(inputCloud, 0.15, Eigen::Vector4f(-5, -8, -2, 1), Eigen::Vector4f(30, 8, 2, 1));
	std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segCloud = pointProcessor.SegmentPlane(filterCloud, 100, 0.225);

	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor.EuclideanClustering(segCloud.second, 0.35, 25, 1500);

	std::pair<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>, pcl::PointCloud<pcl::PointXYZI>::Ptr> ProcessedCloud(cloudClusters, inputCloud);
	return ProcessedCloud;
}

// cityBlockViewer visualize the culstered data by cityBlock method
void cityBlockViewer(pcl::visualization::PCLVisualizer::Ptr& viewer, 
					 std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> ProcessedCloud,
					 pcl::PointCloud<pcl::PointXYZI>::Ptr FilteredCloud,
					 ProcessPointClouds<pcl::PointXYZI> pointProcessor, std::vector<Color> colors)
{
	int clusterId = 0;
	renderPointCloud(viewer, FilteredCloud, "ObstCloud", Color(0, 1, 0));

	
	for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : ProcessedCloud)
	{
		
			
		renderPointCloud(viewer, cluster, "Frame"+std::to_string(clusterId), colors[clusterId % colors.size()]);
		
		Box box = pointProcessor.BoundingBox(cluster);
		renderBox(viewer, box, clusterId, colors[clusterId % colors.size()]);

		clusterId++;
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

	ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
	
	std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
	auto streamIterator = stream.begin();
	pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);

    
	std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>> StreamClusters;
	std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> FilteredClouds;
	int cnt = 0;
	while (streamIterator != stream.end())
	{
		inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
		std::pair<std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>, pcl::PointCloud<pcl::PointXYZI>::Ptr> Clusters = cityBlock(*pointProcessorI, inputCloudI);
		StreamClusters.push_back(Clusters.first);
		FilteredClouds.push_back(Clusters.second);
		streamIterator++;
		cnt++;
	}
	
	std::cout << cnt << endl;
	std::vector<Color> colors = { Color(1,0,0), Color(0,1,1), Color(0,0,1), Color(1, 1, 0), Color(1, 0, 1), Color(1, 1, 1) };
	
	
	int FrameId = 0;
    while (!viewer->wasStopped ())
    {
		// Clear viewer
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();

		// Stop viewer after all frames are shown once
		if (FrameId == cnt)
			continue;

		cout << "Frame " << FrameId+1 << endl;
		
		cityBlockViewer(viewer, StreamClusters[FrameId], FilteredClouds[FrameId], *pointProcessorI, colors);

		// An additional method to take a screenshot of each stream of point cloud once the clustering is done
		// ScreenShot(viewer, "Frame", FrameId);
		cout << "+++++++++++++++++++++++" << endl;
		
		
		++FrameId;
		viewer->spinOnce();
    } 
	
	while (!viewer->wasStopped())
	{
		viewer->spinOnce();
	}
}