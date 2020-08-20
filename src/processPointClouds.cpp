// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
// #include "quiz/cluster/cluster.cpp"


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
    typename pcl::PointCloud<PointT>::Ptr f_cloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cf_cloud(new pcl::PointCloud<PointT>());

    // typename pcl::PointCloud<PointT>::Ptr cf_cloud(new pcl::PointCloud<PointT>());

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> vox;
    vox.setInputCloud(cloud);
    vox.setLeafSize(filterRes,filterRes,filterRes);
    vox.filter(*f_cloud);

    pcl::CropBox<PointT> cbox;
    cbox.setMin(minPoint);
    cbox.setMax(maxPoint);
    cbox.setInputCloud(f_cloud);
    cbox.filter(*cf_cloud);
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cf_cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr incloud(new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr outcloud(new pcl::PointCloud<PointT> ());
    // int ind;
    // for(int i=0:i<inliers->indices.size();i++){
    //     ind = inliers->indices[i];
    //     incloud->points.pushback(cloud->points[ind]);
    // }
    pcl::ExtractIndices<PointT> extract;

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*incloud);
    extract.setNegative(true);
    extract.filter(*outcloud);
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(incloud, outcloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // std::unordered_set<int> inliersResult;
 	pcl::PointIndices::Ptr inliers (new pcl::PointIndices());

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
			dist = fabs(A*cloud->points[j].x + B*cloud->points[j].y + C*cloud->points[j].z + D);
            dist = dist/den;
			// std::cout << "Abs " << abs(cloud->points[j].y - (slope*cloud->points[j].x) - offset) << "No abs " << cloud->points[j].y - (slope*cloud->points[j].x) - offset<<std::endl;
			if(dist<(distanceThreshold)){
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
		dist = fabs(s_A*cloud->points[j].x + s_B*cloud->points[j].y + s_C*cloud->points[j].z + s_D);
        dist = dist/den;

		if(dist<(distanceThreshold)){
			inliers->indices.push_back(j);
		}
	}


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

// template<typename PointT>
// std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
// {
//     // Time segmentation process
//     auto startTime = std::chrono::steady_clock::now();
// 	pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
//     pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
//     // TODO:: Fill in this function to find inliers for the cloud.
//     std::cout << "1" <<std::endl;
//     pcl::SACSegmentation<PointT> seg;
//     seg.setModelType(pcl::SACMODEL_PLANE);
//     seg.setMethodType(pcl::SAC_RANSAC);
//     seg.setMaxIterations(maxIterations);
//     seg.setDistanceThreshold(distanceThreshold);
//     std::cout << "2" <<std::endl;

//     seg.setInputCloud(cloud);

//     std::cout << "4" <<std::endl;
//     seg.segment(*inliers,*coefficients);

//     std::cout << "3" <<std::endl;

//     auto endTime = std::chrono::steady_clock::now();
//     auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
//     std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

//     std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
//     return segResult;
// }


template<typename PointT>
void ProcessPointClouds<PointT>::proximityCluster(const std::vector<std::vector<float>>& points, int id, KdTree* tree, float distanceTol, std::vector<bool> &f_p, std::vector<int> &cluster)
{
	cluster.push_back(id);
	f_p[id] = true;
	std::vector<int> ids = tree->search(points[id],distanceTol);
	for(int i=0;i<ids.size();i++){
		if(!f_p[ids[i]]){
			proximityCluster(points, ids[i], tree,distanceTol, f_p, cluster);
		}
	}
	
	
}


template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int minSize, int maxSize)
{

	// TODO: Fill out this function to return list of indices for each cluster
	
	std::vector<std::vector<int>> clusters;
	std::vector<bool> f_p(points.size(),false);
	for(int i=0;i<points.size();i++){
		if(!f_p[i]){
			std::vector<int> cluster;
			proximityCluster(points, i, tree, distanceTol, f_p, cluster);
            if(cluster.size() > minSize && cluster.size() < maxSize){
    			clusters.push_back(cluster);
            }
		}
	}
	return clusters;

}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
	KdTree* tree = new KdTree;

    std::vector<std::vector<float>> points;
    for(int i=0;i<cloud->points.size();i++){
        std::vector<float> t_point = {cloud->points[i].x,cloud->points[i].y,cloud->points[i].z};
        points.push_back(t_point);
        tree->insert(t_point,i);
    }

    std::vector<std::vector<int>> cluster_ind = euclideanCluster(points, tree, clusterTolerance, minSize, maxSize);
    
  	for(std::vector<int> c : cluster_ind)
  	{
  		typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
  		for(int indice: c)
  			clusterCloud->points.push_back(cloud->points[indice]);
        clusters.push_back(clusterCloud);
  	}


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

// template<typename PointT>
// std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
// {

//     // Time clustering process
//     auto startTime = std::chrono::steady_clock::now();

//     std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

//     // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
//     typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
//     tree->setInputCloud(cloud);

//     std::vector<pcl::PointIndices> cluster_ind;
//     pcl::EuclideanClusterExtraction<PointT> ec;
//     ec.setClusterTolerance(clusterTolerance);
//     ec.setMinClusterSize(minSize);
//     ec.setMaxClusterSize(maxSize);
//     ec.setSearchMethod(tree);
//     ec.setInputCloud(cloud);
//     ec.extract(cluster_ind);

//     for(int i=0;i<cluster_ind.size();i++){
//         typename pcl::PointCloud<PointT>::Ptr temp_cloud (new pcl::PointCloud<PointT>);
//         for(int j=0;j<cluster_ind[i].indices.size();j++){
//             temp_cloud->points.push_back(cloud->points[cluster_ind[i].indices[j]]);
//         }
//         temp_cloud->width = temp_cloud->points.size();

//         temp_cloud->height = 1;
//         temp_cloud->is_dense = true;
//         clusters.push_back(temp_cloud);
//     }


//     auto endTime = std::chrono::steady_clock::now();
//     auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
//     std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

//     return clusters;
// }


template<typename PointT>
BoxQ ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    BoxQ boxq;
    Eigen::Vector4f pcaMean;
    pcl::compute3DCentroid(*cluster,pcaMean);
    Eigen::Matrix3f Cov;
    pcl::computeCovarianceMatrixNormalized(*cluster, pcaMean, Cov);
    Eigen::Matrix2f CovXY;
    CovXY << Cov(0,0),Cov(0,1),Cov(1,0),Cov(1,1);
    
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2f> eigen_solver(CovXY, Eigen::ComputeEigenvectors);
    Eigen::Matrix2f eig_vector = eigen_solver.eigenvectors();
    Eigen::Matrix3f temp_rot;
    temp_rot << eig_vector(0,0),eig_vector(0,1),0,eig_vector(1,0),eig_vector(1,1),0,0,0,1;
    //
    // boxq.bboxQuaternion = Eigen::Quaternionf(temp_rot);

    boxq.bboxTransform[0] = (maxPoint.x + minPoint.x)/2;
    boxq.bboxTransform[1] = (maxPoint.y + minPoint.y)/2;
    boxq.bboxTransform[2] = (maxPoint.z + minPoint.z)/2;
    boxq.cube_length = maxPoint.x - minPoint.x;
    boxq.cube_width = maxPoint.y - minPoint.y;
    boxq.cube_height = maxPoint.z - minPoint.z;
    
    return boxq;
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