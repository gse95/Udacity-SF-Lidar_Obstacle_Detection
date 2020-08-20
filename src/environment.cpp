/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "track.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

#define ENABLE_TRACKING false

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

void CityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud,ProcessPointClouds<pcl::PointXYZI>* pointProcessor, Tracker &tracker){
    
    inputCloud = pointProcessor->FilterCloud(inputCloud, 0.3, Eigen::Vector4f(-8,-6,-5,1),Eigen::Vector4f(40,7,4,1));
    // renderPointCloud(viewer,inputCloud,"IC");

    std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, typename pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedPC = pointProcessor->SegmentPlane(inputCloud,100,0.17);
    renderPointCloud(viewer,segmentedPC.first,"Plane",Color(1,0,0));
    // renderPointCloud(viewer,segmentedPC.second,"Obs",Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> c_clusters = pointProcessor->Clustering(segmentedPC.second,0.4,10,800);
    int cluster_id = 0;
    std::vector<Color> colors = {Color(1,1,0),Color(0,1,1),Color(1,0,1)};

    std::vector<Measurement> measurements;
    // std::vector<Eigen::Vector4f> measurements =  
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : c_clusters){
        // pointProcessor->numPoints(cluster);
        renderPointCloud(viewer, cluster,"obst"+std::to_string(cluster_id),colors[cluster_id%3]);
        BoxQ box = pointProcessor->BoundingBox(cluster);
        if(ENABLE_TRACKING){
            Measurement meas;
            meas.position = box.bboxTransform;
            meas.length = box.cube_length;
            meas.width = box.cube_width;
            meas.height = box.cube_height;

            meas.assoc = false;
            measurements.push_back(meas);
        }
        renderBox(viewer,box,cluster_id,colors[cluster_id%3],0.4);
        cluster_id++;
    }  
    if(ENABLE_TRACKING){
        tracker.setMeasurements(measurements);
    }
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidar = new Lidar(cars,0.0);
    // TODO:: Create point processor
    pcl::PointCloud<pcl::PointXYZ>::Ptr tempPC = lidar->scan();
    
    // renderRays(viewer, lidar->position, tempPC);
    // renderPointCloud(viewer,tempPC,"Blah",Color(1,1,0));
    

    ProcessPointClouds<pcl::PointXYZ> PPC;

    std::pair<typename pcl::PointCloud<pcl::PointXYZ>::Ptr, typename pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentedPC = PPC.SegmentPlane(tempPC,100,0.1);
    // renderPointCloud(viewer,segmentedPC.first,"Plane",Color(1,0,0));
    // renderPointCloud(viewer,segmentedPC.second,"Obs",Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> c_clusters = PPC.Clustering(segmentedPC.second,1.5,4,60);
    int cluster_id = 0;
    std::vector<Color> colors = {Color(1,1,0),Color(0,1,1),Color(1,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : c_clusters){
        PPC.numPoints(cluster);
        renderPointCloud(viewer, cluster,"obst"+std::to_string(cluster_id),colors[cluster_id]);
        BoxQ box = PPC.BoundingBox(cluster);
        renderBox(viewer,box,cluster_id);
        cluster_id++;
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

    Tracker tracker;
        
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    ProcessPointClouds<pcl::PointXYZI>* pointProcessor = new ProcessPointClouds<pcl::PointXYZI>();
    std::vector<boost::filesystem::path> streamPcd = pointProcessor->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIt = streamPcd.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud;
    // simpleHighway(viewer);
    while (!viewer->wasStopped ())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        inputCloud = pointProcessor->loadPcd((*streamIt).string());
        CityBlock(viewer, inputCloud, pointProcessor, tracker);
        if(ENABLE_TRACKING){
            tracker.associateMeasurements();
            tracker.printTracks();
        }    
        streamIt++;
        if(streamIt == streamPcd.end()){
            // break;
            streamIt = streamPcd.begin();
        }
        // renderPointCloud(viewer,inputCloud,"IC");
        
        
        viewer->spinOnce ();
    } 
}