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
	
	// TODO: Fill in this function
	int i, j, index, indexP1, indexP2, indexP3, maxInliers=0, maxInliersIndexP1=0, maxInliersIndexP2=0, inliersCounter=0, aMax, bMax, cMax, dMax;
	float a, b, c, d, abcSqrt;
	// For max iterations 
	for (i = 0; i < maxIterations; ++i) {
		// Randomly sample subset and fit line
		indexP1 = rand() % cloud->points.size();
		// index P2 calc
		do {
			indexP2 = rand() % cloud->points.size();
		} while (indexP1 == indexP2);
		// index P3 calc
		do {
			indexP3 = rand() % cloud->points.size();
		} while (indexP1 == indexP3 || indexP2 == indexP3);
		/* a = (y1-y2).(z3-z1)-(z2-z1).(y3-y1) */
		a = (cloud->points[indexP1].y - cloud->points[indexP2].y)*(cloud->points[indexP3].z - cloud->points[indexP1].z)-
			(cloud->points[indexP2].z - cloud->points[indexP1].z)*(cloud->points[indexP3].y - cloud->points[indexP2].y);
		/* b = (z2-z1).(x3-x1)-(x2-x1).(z3-z1) */
		b = (cloud->points[indexP2].z - cloud->points[indexP1].z)*(cloud->points[indexP3].x - cloud->points[indexP1].x)-
			(cloud->points[indexP2].x - cloud->points[indexP1].x)*(cloud->points[indexP3].z - cloud->points[indexP2].z);
		/* c = (x2-x1).(y3-y1)-(y2-y1).(x3-x1)  */
		c = (cloud->points[indexP2].x - cloud->points[indexP1].x)*(cloud->points[indexP3].y - cloud->points[indexP1].y)-
			(cloud->points[indexP2].y - cloud->points[indexP1].y)*(cloud->points[indexP3].x - cloud->points[indexP2].x);
		/* d = -(A.x1+B.y1+C.z1) */
		d = -(a*cloud->points[indexP1].x+b*cloud->points[indexP1].y+c*cloud->points[indexP1].z);
		// Measure distance between every point and fitted line
		// If distance is smaller than threshold count it as inlier
		abcSqrt = sqrt(pow(a,2)+pow(b,2)+pow(c,2));
		for(j = 0, inliersCounter = 0; j < cloud->points.size(); ++j) {
			d = abs(a*cloud->points[j].x+b*cloud->points[j].y+c*cloud->points[j].z+d)/abcSqrt;
			if(d <= distanceTol) {
				++inliersCounter;
			}
		}
		if(inliersCounter > maxInliers) {
			maxInliers = inliersCounter;
			aMax = a;
			bMax = b;
			cMax = c;
			dMax = d;
		}
	}
	// Return indicies of inliers from fitted line with most inliers
	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier
	abcSqrt = sqrt(pow(aMax,2)+pow(bMax,2)+pow(cMax,2));
	for(j = 0, inliersCounter = 0; j < cloud->points.size(); ++j) {
		d = abs(aMax*cloud->points[j].x+bMax*cloud->points[j].y+cMax*cloud->points[j].z+dMax)/abcSqrt;
		//d = fabs(a*cloud->points[j].x+b*cloud->points[j].y+c)/abSqrt;
		if(d <= distanceTol) {
			inliersResult.insert(j);	
		}
	}
	// Print the coefficients of the fitted plane
    std::cout << "Plane coefficients: A = " << aMax << ", B = " << bMax << ", C = " << cMax << ", D = " << dMax << std::endl;
	// Visualize the point cloud and the fitted plane
    // Create the segmentation object

	return inliersResult;
}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	// 2D
	// pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	// 3D
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);

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
