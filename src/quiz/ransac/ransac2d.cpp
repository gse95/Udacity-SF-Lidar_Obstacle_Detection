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
	int ind1,ind2,ind3;
	float A,B,C,D,num_inliers, max_inliers;
	float s_A,s_B,s_C,s_D, dist, den;
	max_inliers = 0;
	// TODO: Fill in this function
	for(int i = 0;i < maxIterations;i++){
		ind1 = rand()%cloud->points.size();
		ind2 = rand()%cloud->points.size();
		ind3 = rand()%cloud->points.size();
		
		A = (cloud->points[ind2].y - cloud->points[ind1].y)*(cloud->points[ind3].z - cloud->points[ind1].z)-
			(cloud->points[ind3].y - cloud->points[ind1].y)*(cloud->points[ind2].z - cloud->points[ind1].z);
		B = (cloud->points[ind2].z - cloud->points[ind1].z)*(cloud->points[ind3].x - cloud->points[ind1].x)-
			(cloud->points[ind3].z - cloud->points[ind1].z)*(cloud->points[ind2].x - cloud->points[ind1].x);
		C = (cloud->points[ind2].x - cloud->points[ind1].x)*(cloud->points[ind3].y - cloud->points[ind1].y)-
			(cloud->points[ind3].x - cloud->points[ind1].x)*(cloud->points[ind2].y - cloud->points[ind1].y);
		D = -(A*cloud->points[ind1].x + B*cloud->points[ind1].y + C*cloud->points[ind1].z);

		num_inliers = 0;
		den = sqrt(A*A + B*B + C*C);
		for(int j = 0;j < cloud->points.size();j++){
			dist = fabs(A*cloud->points[j].x + B*cloud->points[j].y + C*cloud->points[j].z + D)/den;
			// std::cout << "Abs " << abs(cloud->points[j].y - (slope*cloud->points[j].x) - offset) << "No abs " << cloud->points[j].y - (slope*cloud->points[j].x) - offset<<std::endl;
			if(dist<distanceTol){
				num_inliers++;
			}
		}
		if(num_inliers > max_inliers){
			s_A = A;
			s_B = B;
			s_C = C;
			s_D = D;

			max_inliers = num_inliers;
		}
	}
	std::cout<< "num inliers:  " << max_inliers <<std::endl;
	den = sqrt(s_A*s_A + s_B*s_B + s_C*s_C);

	for(int j = 0;j < cloud->points.size();j++){
		dist = fabs(s_A*cloud->points[j].x + s_B*cloud->points[j].y + s_C*cloud->points[j].z + s_D)/den;

		if(dist<distanceTol){
			inliersResult.insert(j);
		}
	}
	

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
	std::unordered_set<int> inliers = Ransac(cloud, 10, 0.2);

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
