// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"

//#define VERBOSE_MODE

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
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, 
    float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
        // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Voxel grid point reduction
    typename pcl::PointCloud<PointT>::Ptr cloudVGFiltered{new pcl::PointCloud<PointT>};
    pcl::VoxelGrid<PointT> voxelGridFilter;
    voxelGridFilter.setInputCloud(cloud);
    voxelGridFilter.setLeafSize(filterRes, filterRes, filterRes);
    voxelGridFilter.filter(*cloudVGFiltered);

    // Region of Interest (ROI) based filtering
    typename pcl::PointCloud<PointT>::Ptr cloudBoxFiltered{new pcl::PointCloud<PointT>};
    
    pcl::CropBox<PointT> cropBoxFilter(true);
    cropBoxFilter.setMin(minPoint);
    cropBoxFilter.setMax(maxPoint);
    cropBoxFilter.setInputCloud(cloudVGFiltered);
    cropBoxFilter.filter(*cloudBoxFiltered);

    // Get the indices of rooftop points
    std::vector<int> indices;

    pcl::CropBox<PointT> roofFilter(true);
    roofFilter.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roofFilter.setMax(Eigen::Vector4f(2.6, 1.7, -0.4, 1));
    roofFilter.setInputCloud(cloudBoxFiltered);
    roofFilter.filter(indices);

    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    for (int point: indices)
        inliers->indices.push_back(point);

    // Remove the rooftop indices
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered{new pcl::PointCloud<PointT>};
    pcl::ExtractIndices<PointT> extract;
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.setInputCloud(cloudBoxFiltered);
    extract.filter(*cloudFiltered);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "downsampled original " << cloud->points.size() << " points to " << cloudFiltered->points.size() << std::endl;
    //std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudFiltered;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds
    (pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT>());
    // Extract the inliers
    for(int index : inliers->indices) {
        planeCloud->points.push_back(cloud->points[index]);
    }
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstCloud);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane
    (typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in this function to find inliers for the cloud.
    ProcessPointClouds<PointT> *processPointCloud(new ProcessPointClouds<PointT>());

	std::unordered_set<int> inliersS = processPointCloud->SegmentPlaneRansac(cloud, maxIterations, distanceThreshold);
    std::unordered_set<int>::iterator it;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    for(it = inliersS.begin(); it != inliersS.end(); ++it) {
        inliers->indices.push_back(*it);
    }
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);
    // segment the largest planar component from the input cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if(inliers->indices.size() == 0) {
        std::cout<<"Could not estimate a planar model for the given dataset."<<std::endl;
    }

    // Create the filtering object
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    #if defined VERBOSE_MODE
        std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    #endif
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
std::unordered_set<int> ProcessPointClouds<PointT>::SegmentPlaneRansac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
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
    #if defined VERBOSE_MODE
        std::cout << "Plane coefficients: A = " << aMax << ", B = " << bMax << ", C = " << cMax << ", D = " << dMax << std::endl;
	#endif
    return inliersResult;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, 
        float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    //const int clusterTolerance = 30;
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);
    
    /* TBD euclidean cluster */

    std::vector<pcl::PointIndices> clusterIndices; // = euclideanCluster(cloud, tree, 40);
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); // cm
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (clusterIndices);

    for(pcl::PointIndices getIndices : clusterIndices) {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
        for(int index : getIndices.indices) {
            cloudCluster->points.push_back(cloud->points[index]);
        }
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;
        clusters.push_back(cloudCluster);
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusterIndices.size() << " clusters" << std::endl;

    return clusters;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::clusteringImplementation(const typename pcl::PointCloud<PointT>::Ptr &cloud, float clusterTolerance, int minSize, int maxSize)
{
    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    KdTree *tree = new KdTree;
    std::vector<std::vector<float>> pointsVec(cloud->points.size());
    for (int i = 0; i < cloud->points.size(); i++)
    {
        tree->insert({cloud->points[i].x, cloud->points[i].y, cloud->points[i].z}, i);
        pointsVec[i] = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
    }

    std::vector<std::vector<int>> clusterIndices = euclideanClusterImplementation(pointsVec, tree, clusterTolerance, minSize, maxSize);

    for (std::vector<int> cluster : clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
        for (int index : cluster)
        {
            cloudCluster->points.push_back(cloud->points[index]);
        }
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

template <typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanClusterImplementation(const std::vector<std::vector<float>> &points, KdTree *&tree, float distanceTol, float minSize, float maxSize)
{

    // TODO: Fill out this function to return list of indices for each cluster

    std::vector<std::vector<int>> clusters;
    std::vector<bool> isProcessedVector(points.size(), false);

    for (size_t i = 0; i < points.size(); ++i)
    {
        if (!isProcessedVector[i])
        {
            std::vector<int> cluster;
            updateCluster(points, tree, distanceTol, i, cluster, isProcessedVector);
            if (cluster.size() >= minSize && cluster.size() <= maxSize)
                clusters.push_back(cluster);
        }
    }

    return clusters;
}

template <typename PointT>
void ProcessPointClouds<PointT>::updateCluster(const std::vector<std::vector<float>> &points, KdTree *tree, float distanceTol, int pointId, std::vector<int> &cluster, std::vector<bool> &isProcessedVector) {
    if (isProcessedVector[pointId] == false) {
        isProcessedVector[pointId] = true;
        cluster.push_back(pointId);
        std::vector<int> possiblePointClusterIds = tree->search(points[pointId], distanceTol);

        for (int id : possiblePointClusterIds) {
            if (isProcessedVector[id] == false) {
                updateCluster(points, tree, distanceTol, id, cluster, isProcessedVector);
            }
        }
    }
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