/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <random>

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

std::unordered_set<int> RansacLine(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	// For max iterations 
	while(maxIterations--){
		std::unordered_set<int> inliers;

		// Randomly sample subset and fit line
		while(inliers.size() <  2){
			// inliers.insert(rand()% (cloud->points.size()));
			std::random_device rd;
			std::mt19937 gen(rd());
			std::uniform_int_distribution<> distrib(0, cloud->points.size() - 1);
			inliers.insert(distrib(gen));

		}	

		float x1, y1, x2, y2;

		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		
		// Line equation ax + by + c = 0
		float a  = (y1 - y2);
		float b = (x2 -x1);
		float c = (x1 * y2 - x2 * y1);

		for(int index = 0; index < cloud->points.size(); index ++){
			// avoid inserting same index that is used fitting the plane
			if(inliers.count(index)> 0){
				continue;  
			}

			pcl::PointXYZ point = cloud->points[index];
			float x3 = point.x;
			float y3 = point.y;
		
			// Measure distance between every point and fitted line
			float distance = fabs(a*x3 + b*y3 + c) / sqrt(a*a + b*b);
			// If distance is smaller than threshold count it as inlier
			if(distance <= distanceTol){
				inliers.insert(index);
			}

		}
		// Return indicies of inliers from fitted line with most inliers
		if(inliers.size() > inliersResult.size()){
			inliersResult = inliers;
		}

	}
	
	return inliersResult;

}

std::unordered_set<int> RansacPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	// For max iterations 
	while(maxIterations--){
		std::unordered_set<int> inliers;

		// Randomly sample subset and fit line
		while(inliers.size() <  3){
			// inliers.insert(rand()% (cloud->points.size()));
			std::random_device rd;
			std::mt19937 gen(rd());
			std::uniform_int_distribution<> distrib(0, cloud->points.size() - 1);
			inliers.insert(distrib(gen));

		}	

		float x1, y1, z1, x2, y2, z2, x3, y3, z3;

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

		// Fit plane : Ax + By + Cz + D = 0
		// Use point1 as a reference and define two vectors on the plane v1 and v2
		float v1x = x2 - x1;
		float v1y = y2 - y1;
		float v1z = z2 - z1;

		float v2x = x3 - x1;
		float v2y = y3 - y1;
		float v2z = z3 - z1;

		// Find normal vector to the plane by taking cross product of v1 \times v2:
		float a = v1y * v2z - v1z * v2y;
		float b = v1z * v2x - v1x * v2z;
		float c = v1x * v2y - v1y * v2x;
		float d = -( a*x1 + b*y1 + c*z1 );

		for(int index = 0; index < cloud->points.size(); index ++){
			// avoid inserting same index that is used fitting the plane
			if(inliers.count(index)> 0){
				continue;  
			}

			pcl::PointXYZ point = cloud->points[index];
			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;

			// Measure distance between every point and fitted line
			float distance = fabs(a*x4 + b*y4 + c*z4 + d) / sqrt(a*a + b*b + c*c);
			// If distance is smaller than threshold count it as inlier
			if(distance <= distanceTol){
				inliers.insert(index);
			}

		}
		// Return indicies of inliers from fitted line with most inliers
		if(inliers.size() > inliersResult.size()){
			inliersResult = inliers;
		}

	}
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create 2D data
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	// std::unordered_set<int> inliers = RansacLine(cloud, 10, 1.0);
	
	// Create 3D data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	std::unordered_set<int> inliers = RansacPlane(cloud, 50, 0.3);

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
  	//   viewer->spinOnce ();
	  viewer->spin();
  	}
  	
}
