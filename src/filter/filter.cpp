#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_picking_event.h>
#include <pcl/point_types.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>

void plane_filtering(pcl::PointCloud<pcl::PointXYZL>::Ptr cloud, pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_filtered, std::string path);
void noise_filtering(pcl::PointCloud<pcl::PointXYZL>::Ptr cloud, pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_filtered, std::string path);
void box_filtering(pcl::PointCloud<pcl::PointXYZL>::Ptr cloud, std::string path);

bool firstBoxMade = false;
std::vector<int> firstBoxIndices;

pcl::PointCloud<pcl::PointXYZL>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZL>);

pcl::PCDWriter cloud_writer;

// Visualizer
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("Viewer"));

// xyzrgb cloud used in the box filter
pcl::PointCloud<pcl::PointXYZRGBL>::Ptr rbgCloud(new pcl::PointCloud<pcl::PointXYZRGBL>);

// Used for the box filter
pcl::PointCloud<pcl::PointXYZRGBL>::Ptr selected_points_1(new pcl::PointCloud<pcl::PointXYZRGBL>);
pcl::PointCloud<pcl::PointXYZRGBL>::Ptr selected_points_2(new pcl::PointCloud<pcl::PointXYZRGBL>);
pcl::PointCloud<pcl::PointXYZRGBL>::Ptr intersection(new pcl::PointCloud<pcl::PointXYZRGBL>);

// Used to convert xyzrgb cloud back into xyz
pcl::PointCloud<pcl::PointXYZL>::Ptr resultCloud(new pcl::PointCloud<pcl::PointXYZL>);

// Path to pcd files
std::string path="/home/andreas/Desktop/andreas_master/Indoor-reconstruction-plus-plus/build/data_dir/fragments/fragment_0/";

int main () {
    // Setup
    pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZL>);

    // Read the cloud that should be filtered
    pcl::PCDReader cloud_reader;
    cloud_reader.read(path+std::string("original.pcd"),*cloud);
    

    // Filter methods
    //plane_filtering(cloud, cloud_filtered, path);
    //noise_filtering(cloud, cloud_filtered, path);
    box_filtering(cloud, path);
    
    return (0);
}

void plane_filtering (pcl::PointCloud<pcl::PointXYZL>::Ptr cloud, pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_filtered, std::string path) 
{   
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);

    pcl::SACSegmentation<pcl::PointXYZL> plane_seg;
    plane_seg.setModelType(pcl::SACMODEL_PLANE);
    plane_seg.setMethodType(pcl::SAC_RANSAC);
    plane_seg.setDistanceThreshold(0.0164);
    plane_seg.setInputCloud(cloud);
    plane_seg.segment(*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZL> extract_indices;
    extract_indices.setInputCloud(cloud);
    extract_indices.setIndices(inliers);
    extract_indices.setNegative(true);
    extract_indices.filter(*cloud_filtered);

     cloud_writer.write<pcl::PointXYZL>(path+std::string("plane_filter.pcd"),*cloud_filtered, false);
}

void noise_filtering(pcl::PointCloud<pcl::PointXYZL>::Ptr cloud, pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_filtered, std::string path) 
{
    // Noise filtering
    pcl::StatisticalOutlierRemoval<pcl::PointXYZL> sor;
    sor.setInputCloud (cloud);
    sor.setMeanK (1000);
    sor.setStddevMulThresh (0.4);
    sor.filter (*cloud_filtered);

    cloud_writer.write<pcl::PointXYZL>(path+std::string("noise_filter.pcd"),*cloud_filtered, false);
}

void pp_callback(const pcl:: visualization::AreaPickingEvent& event, void* args) {
    std::vector< int > indices;
    std::vector< int > diff;

    //Colors for the selected points
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBL> blue(selected_points_2, 0, 0, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBL> green(intersection, 0, 255, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBL> red(rbgCloud, 255, 0, 0);

    pcl::ExtractIndices<pcl::PointXYZL> extract;
    
    // Adds selected points to vector
    event.getPointsIndices(indices);

    if (!firstBoxMade) {
        for (int i = 0; i < indices.size(); ++i) {
            selected_points_1->points.push_back(rbgCloud->points.at(indices[i]));
            rbgCloud->points.at(indices[i]).r = 255;
            rbgCloud->points.at(indices[i]).b = 0;
            rbgCloud->points.at(indices[i]).g = 0;
        }

        // Reset the viewer to include the updated cloud
        viewer->removeAllPointClouds();
        viewer->addPointCloud(rbgCloud, blue);

        firstBoxIndices = indices;
        firstBoxMade = true;
    }
    else {

        for (int i = 0; i < indices.size(); ++i) {
            selected_points_2->points.push_back(rbgCloud->points.at(indices[i]));
        }

        // Takes the intersection of 1st box and 2nd box
        std::set_intersection(firstBoxIndices.begin(), firstBoxIndices.end(), indices.begin(), indices.end(), std::inserter(diff, diff.begin()));
        for (int i = 0; i < diff.size(); ++i) {
            intersection->points.push_back(rbgCloud->points.at(diff[i]));
        }

        // Add the two last point clouds
        viewer->addPointCloud(selected_points_2, blue, "Second box");
        viewer->addPointCloud(intersection, green, "Intersection");

        // Convert cloud back into only XYZ cloud
        for (int index=0; index < intersection->points.size(); index++) {
            pcl::PointXYZL point (intersection->points.at(index).x, intersection->points.at(index).y, intersection->points.at(index).z, intersection->points.at(index).label);
            resultCloud->points.push_back(point);
        }
        
        // Extracting the intersection cloud to file
        extract.setInputCloud(resultCloud);
        extract.filter(*resultCloud);
        extract.setNegative(false);
        pcl::PCDWriter cloud_writer;
        cloud_writer.write<pcl::PointXYZL>(path+std::string("left_half.pcd"),*resultCloud, false);
    }
}

void box_filtering(pcl::PointCloud<pcl::PointXYZL>::Ptr cloud, std::string path) {
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBL> blue(rbgCloud, 0, 0, 255);
    // Turns the original cloud into an rbg cloud
    for (int index=0; index < cloud->points.size(); index++) {
        pcl::PointXYZRGBL point (cloud->points.at(index).x, cloud->points.at(index).y, cloud->points.at(index).z, 255, 255, 255, cloud->points.at(index).label);
        rbgCloud->points.push_back(point);
    }

    viewer->addPointCloud(rbgCloud, blue, "fragment");
    viewer->registerAreaPickingCallback(pp_callback, (void*)&rbgCloud);
    viewer->spin();
}