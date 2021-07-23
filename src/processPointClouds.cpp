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

    // voxel grid point reduction and region based filtering
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
    roof.setMax(Eigen::Vector4f (2.6, 1.7, -.4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for(int point : indices)
    {
        inliers->indices.push_back(point);
        //std::cout << "point" <<point <<std::endl;
    }

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
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacSegment(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{

    auto startTime = std::chrono::steady_clock::now();

    std::unordered_set<int> inliersResult;

    srand(time(NULL));

    PointT point1;
    PointT point2;
    PointT point3;

    int idx1;
    int idx2;
    int idx3;

    float a, b, c, d, dis, len;

    for(int i=0; i< maxIterations; i++)
    {
        std::unordered_set<int> tempIndices;

        while(tempIndices.size() < 3)
		{
			tempIndices.insert(rand() % (cloud->points.size()));
		}

        auto itr = tempIndices.begin();

        idx1 = *itr;
        ++itr;
        idx2 = *itr;
        ++itr;
        idx3 = *itr;

        point1 = cloud->points[idx1];
        point2 = cloud->points[idx2];
        point3 = cloud->points[idx3];

        float a = (point2.y-point1.y)*(point3.z-point1.z)-(point2.z-point1.z)*(point3.y-point1.y);
		float b = (point2.z-point1.z)*(point3.x-point1.x)-(point2.x-point1.x)*(point3.z-point1.z);
		float c = (point2.x-point1.x)*(point3.y-point1.y)-(point2.y-point1.y)*(point3.x-point1.x);
		float d = -(a*point1.x + b*point1.y + c*point1.z);

        len = sqrt(a * a + b * b + c * c);

        // Measure distance between every point and fitted line
		for(int index = 0; index < cloud->points.size(); index++)
		{
			if(index != idx1 || index != idx2 || index != idx3)
            {
                dis = fabs(a * cloud->points[index].x + b * cloud->points[index].y + c * cloud->points[index].z + d) / len;            
            }
			

			// If distance is smaller than threshold, count it as inlier
			if(dis < distanceTol)
				tempIndices.insert(index);
		}
		
		if(tempIndices.size() >  inliersResult.size())
		{
			inliersResult.clear();
            inliersResult = tempIndices;
		}
    }

    if(inliersResult.size() == 0)
        std::cout<<"Unable to identify planes for given data.";

    typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

    for(int index = 0; index < cloud->points.size(); index++)
    {
        PointT point = cloud->points[index];

        if(inliersResult.count(index))
        {
            cloudInliers->points.push_back(point);            
        }
        else
        {
            cloudOutliers->points.push_back(point); 
        }
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segmentationResult(cloudOutliers, cloudInliers);
    return segmentationResult;

}

template<typename PointT>
void ProcessPointClouds<PointT>::proximity(std::vector<int>& cluster, const std::vector<std::vector<float> > &points, KdTree * tree, bool * visited, float distanceTol, int ps)
{
    visited[ps] = true;
    cluster.push_back(ps);

    std::vector<int> nearby = tree->search(points[ps], distanceTol);

    for(int i=0; i<nearby.size(); i++)
    {
       if(!visited[nearby[i]])
       {
           proximity(cluster, points, tree, visited, distanceTol, nearby[i]);
       }
    }
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>> &points, KdTree * tree, float distanceTol, int minSize, int maxSize)
{
    std::vector<std::vector<int>> clusters;

    int pointCount = points.size();
    bool * visited = new bool[pointCount];
    for(int i = 0; i < pointCount; i++)
    {
        visited[i] = false;
    }

    for(int i = 0; i < pointCount; i++)
    {
        if(visited[i])
        {
            continue;
        }
        std::vector<int> cluster;
        proximity(cluster, points, tree, visited, distanceTol, i);
        if(cluster.size() >= minSize && cluster.size() <= maxSize)
        {
            clusters.push_back(cluster);
        }
    }
    return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Cluster(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    KdTree * tree = new KdTree;

    std::vector<std::vector<float> > points(cloud->points.size());
    for(int i=0; i<cloud->points.size(); i++)
    {
        points[i].push_back(cloud->points[i].x);
        points[i].push_back(cloud->points[i].y);
        points[i].push_back(cloud->points[i].z);
        tree->insert(points[i], i);
    }

    std::vector<std::vector<int> > clusterIndices(euclideanCluster(points, tree, clusterTolerance, minSize, maxSize));

    int clusterId = 0;
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusterClouds;
    for(std::vector<int> cluster : clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
        for(int indice:cluster)
        {
            clusterCloud->points.push_back(cloud->points[indice]);            
        }
        clusterClouds.push_back(clusterCloud);
        ++clusterId;
    }

    return clusterClouds;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud(new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr planeCloud(new pcl::PointCloud<PointT> ());

    for(int index : inliers->indices)
        planeCloud->points.push_back(cloud->points[index]);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacleCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud, planeCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::PclSegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	
    // Fnd inliers for the cloud.
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if(inliers->indices.size() == 0)
    {
        std::cout<<"Could not estimate planar model for the given dataset"<< std::endl;
    }    

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::PclCluster(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // perform euclidean clustering to group detected obstacles
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);

    for(pcl::PointIndices getIndices: clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);
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