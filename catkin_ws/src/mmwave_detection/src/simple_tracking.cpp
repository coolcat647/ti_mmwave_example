#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <iostream> 
#include <vector>
#include <string>
#include <math.h>
#include <signal.h>
#include <Eigen/Geometry>
// ROS related
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/Trigger.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
// PCL related
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/registration/icp.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
// Custom ROS service

using namespace std;
using namespace pcl;

//  _   _                _           
// | | | |              | |          
// | |_| | ___  __ _  __| | ___ _ __ 
// |  _  |/ _ \/ _` |/ _` |/ _ \ '__|
// | | | |  __/ (_| | (_| |  __/ |   
// \_| |_/\___|\__,_|\__,_|\___|_|   
//

class SimpleTrackingNode{
public:
    SimpleTrackingNode(ros::NodeHandle* nh);
private:
    bool srv_trigger_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    void mmwave_data_cb(const sensor_msgs::PointCloud2ConstPtr& in_cloud_msg);

    // ROS
    ros::NodeHandle nh_;
    ros::Publisher pub_filtered_pc;
    ros::Publisher pub_marker_array_;
    ros::Subscriber sub_mmwave_pc;

    // PCL
    PointCloud<PointXYZ>::Ptr scene_cloud_;
    uint32_t cnt_pc_; // pair of pc
    std::vector<PointCloud<PointXYZ>> source_clouds;
    
    // TF
    tf::TransformListener *tf_listener_;

    // Visulization
    visualization_msgs::MarkerArray marker_array_;

    const uint32_t fixed_shape_ = visualization_msgs::Marker::CUBE;
};


//  _____                          
// /  ___|                         
// \ `--.  ___  _   _ _ __ ___ ___ 
//  `--. \/ _ \| | | | '__/ __/ _ \
// /\__/ / (_) | |_| | | | (_|  __/
// \____/ \___/ \__,_|_|  \___\___|
//

// Constructor
SimpleTrackingNode::SimpleTrackingNode(ros::NodeHandle* nh):nh_(*nh){
    // PointCloud variable init
    scene_cloud_.reset(new PointCloud<PointXYZ>());
    // source_clouds.reset(new std::vector<PointCloud<PointXYZ>>);

    pub_filtered_pc = nh_.advertise<sensor_msgs::PointCloud2>("filtered_pc", 1);
    pub_marker_array_ = nh_.advertise<visualization_msgs::MarkerArray>("detection_markers", 1);

    sub_mmwave_pc = nh_.subscribe("/mmWaveDataHdl/RScan", 1, &SimpleTrackingNode::mmwave_data_cb, this);
    
    // ROS service
    tf_listener_ = new tf::TransformListener();

    // vector<PointCloud<PointXYZ>::Ptr, Eigen::aligned_allocator <PointCloud<PointXYZ>::Ptr>> source_clouds;

    

}


void SimpleTrackingNode::mmwave_data_cb(const sensor_msgs::PointCloud2ConstPtr& in_cloud_msg){
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    // copyPointCloud(*scene_cloud_, *cloud);
    pcl::fromROSMsg(*in_cloud_msg, *cloud);

    source_clouds.push_back(*cloud);
    cnt_pc_++;

    if(source_clouds.size() == 5){
        vector<PointCloud<PointXYZ>>::iterator it;
        for(it = source_clouds.begin(); it != source_clouds.end(); it++)
            *scene_cloud_ += *it;

        // Filter point cloud by specific dimension
        pcl::PointCloud<PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<PointXYZ>);
        pcl::ConditionAnd<PointXYZ>::Ptr range_cond (new pcl::ConditionAnd<PointXYZ> ());
        range_cond->addComparison (pcl::FieldComparison<PointXYZ>::ConstPtr (new
            pcl::FieldComparison<PointXYZ> ("x", pcl::ComparisonOps::GT, 0.1)));
        range_cond->addComparison (pcl::FieldComparison<PointXYZ>::ConstPtr (new
            pcl::FieldComparison<PointXYZ> ("x", pcl::ComparisonOps::LT, 5.0)));
        range_cond->addComparison (pcl::FieldComparison<PointXYZ>::ConstPtr (new
            pcl::FieldComparison<PointXYZ> ("y", pcl::ComparisonOps::GT, -2.0)));
        range_cond->addComparison (pcl::FieldComparison<PointXYZ>::ConstPtr (new
            pcl::FieldComparison<PointXYZ> ("y", pcl::ComparisonOps::LT, 2.0)));
        range_cond->addComparison (pcl::FieldComparison<PointXYZ>::ConstPtr (new
            pcl::FieldComparison<PointXYZ> ("z", pcl::ComparisonOps::GT, -0.1)));
        range_cond->addComparison (pcl::FieldComparison<PointXYZ>::ConstPtr (new
            pcl::FieldComparison<PointXYZ> ("y", pcl::ComparisonOps::LT, 2.0)));
        // Build the filter
        pcl::ConditionalRemoval<PointXYZ> condrem;
        condrem.setCondition (range_cond);
        condrem.setInputCloud (scene_cloud_);
        condrem.setKeepOrganized(true);
        // Apply filter
        condrem.filter(*cloud_filtered);

        // Remove NaN point
        std::vector<int> mapping;
        pcl::removeNaNFromPointCloud(*cloud_filtered, *cloud_filtered, mapping);

        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        // build the filter
        outrem.setInputCloud(cloud_filtered);
        outrem.setRadiusSearch(0.8);
        outrem.setMinNeighborsInRadius(3);
        // apply filter
        outrem.filter(*cloud_filtered);

        // cloud_filtered->header.frame_id = in_cloud_msg->header.frame_id;
        cloud_filtered->header.frame_id = "map";

        pub_filtered_pc.publish(cloud_filtered);


        if(cloud_filtered->points.size() > 1){
            // ROS_WARN_STREAM("There is no point after filtering on mmwave raw data. Skip this callback.");
            // return;
        
            // Creating the KdTree object for the search method of the extraction
            pcl::search::KdTree<PointXYZ>::Ptr tree (new pcl::search::KdTree<PointXYZ>);
            tree->setInputCloud(cloud_filtered);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<PointXYZ> ec;
            ec.setClusterTolerance (0.8);  // unit: m
            ec.setMinClusterSize (2);     
            ec.setMaxClusterSize (50);   
            ec.setSearchMethod (tree);      
            ec.setInputCloud (cloud_filtered);
            ec.extract (cluster_indices);

            int j = 0;
            marker_array_.markers.clear();
            for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it){ 
                pcl::PointCloud<PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<PointXYZ>);
                for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
                    cloud_cluster->points.push_back(cloud_filtered->points[*pit]); //*
                cloud_cluster->width = cloud_cluster->points.size();
                cloud_cluster->height = 1;
                cloud_cluster->is_dense = true;

                Eigen::Vector4f centroid;
                pcl::compute3DCentroid(*cloud_cluster, centroid);

                // Visualization
                visualization_msgs::MarkerPtr marker(new visualization_msgs::Marker);
                marker->header.frame_id = "map";
                // marker->header.frame_id = in_cloud_msg->header.frame_id;
                marker->header.stamp = ros::Time::now();
                marker->type = fixed_shape_;
                marker->id = j;
                marker->action = visualization_msgs::Marker::ADD;
                marker->pose.position.x = centroid[0];
                marker->pose.position.y = centroid[1];
                marker->pose.position.z = centroid[2];
                marker->pose.orientation.x = 0.0;
                marker->pose.orientation.y = 0.0;
                marker->pose.orientation.z = 0.0;
                marker->pose.orientation.w = 1.0;
                // Set the color -- be sure to set alpha to something non-zero!
                marker->color.r = 0.4f;
                marker->color.g = 0.4f;
                marker->color.b = 1.0f;
                marker->color.a = 0.5f;
                // Set the scale of the marker -- 1x1x1 here means 1m on a side
                marker->scale.x = marker->scale.y = marker->scale.z = 0.8;
                marker->lifetime = ros::Duration(0.2);

                marker_array_.markers.push_back(*marker);
                j++;
            }

            // cout << j << ", " << marker_array_.markers.size() <<  endl;
            pub_marker_array_.publish(marker_array_);          
        }
        scene_cloud_.reset(new PointCloud<PointXYZ>());
        source_clouds.erase(source_clouds.begin());
    }
    

}


// bool SimpleTrackingNode::srv_trigger_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
//     sensor_msgs::PointCloud2 pc;
//     pc  = *(ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/depth_registered/points",
//                                                                  ros::Duration(3)));
    
//     // Get request pointcloud then transform to base_link frame
//     sensor_msgs::PointCloud2 in_cloud_msg;
//     if(tf_listener_->waitForTransform("/base_link", "/camera_color_optical_frame",
//                                       ros::Time::now(),
//                                       ros::Duration(5.0)
//                                       ))
//         pcl_ros::transformPointCloud("base_link", pc, in_cloud_msg,  *tf_listener_);
//     else{
//         ROS_ERROR_STREAM("TF transform failed, skip this request.");
//         res.success = false;
//         return false;
//     }

//     pcl::fromROSMsg(in_cloud_msg, *scene_cloud_);
    
//     // Filter point cloud by specific dimension
//     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<PointXYZRGB>);
//     pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr range_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
//     range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new
//         pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::GT, 0.01)));
//     range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new
//         pcl::FieldComparison<pcl::PointXYZRGB> ("z", pcl::ComparisonOps::LT, 0.2)));
//     range_cond->addComparison (pcl::FieldComparison<pcl::PointXYZRGB>::ConstPtr (new
//         pcl::FieldComparison<pcl::PointXYZRGB> ("y", pcl::ComparisonOps::LT, 0.05)));
//     // Build the filter
//     pcl::ConditionalRemoval<pcl::PointXYZRGB> condrem;
//     condrem.setCondition (range_cond);
//     condrem.setInputCloud (scene_cloud_);
//     condrem.setKeepOrganized(true);
//     // Apply filter
//     condrem.filter (*cloud_filtered);
//     //pub_scene_pc.publish(cloud_filtered);


//     // Remove NaN point
//     std::vector<int> mapping;
//     pcl::removeNaNFromPointCloud(*cloud_filtered, *cloud_filtered, mapping);
//     //cout << "size of filtered cloud: " << cloud_filtered->points.size() << endl;


//     if(cloud_filtered->points.size() > 0){
//         // Creating the KdTree object for the search method of the extraction
//         pcl::search::KdTree<PointXYZRGB>::Ptr tree (new pcl::search::KdTree<PointXYZRGB>);
//         tree->setInputCloud(cloud_filtered);

//         std::vector<pcl::PointIndices> cluster_indices;
//         pcl::EuclideanClusterExtraction<PointXYZRGB> ec;
//         ec.setClusterTolerance (0.02);  // 2cm
//         ec.setMinClusterSize (100);     
//         ec.setMaxClusterSize (25000);   
//         ec.setSearchMethod (tree);      
//         ec.setInputCloud (cloud_filtered);
//         ec.extract (cluster_indices);

//         int j = 0;
//         for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
//             pcl::PointCloud<PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<PointXYZRGB>);
//             for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
//                 cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
//             cloud_cluster->width = cloud_cluster->points.size ();
//             cloud_cluster->height = 1;
//             cloud_cluster->is_dense = true;

//             Eigen::Vector4f centroid;
//             pcl::compute3DCentroid(*cloud_cluster, centroid);
//             cout<<centroid[0] << endl << centroid[1] << endl << centroid[2] << endl;

//             geometry_msgs::PoseStamped p;
//             p.header.stamp = ros::Time::now();
//             p.header.frame_id = "base_link";
//             p.pose.position.x = centroid[0];
//             p.pose.position.y = centroid[1];
//             p.pose.position.z = centroid[2];

//             std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
//             // std::stringstream ss;
//             // ss << "cloud_cluster_" << j << ".pcd";
//             // writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*
//             if(j == 0)
//                 pub_cloud1.publish(cloud_cluster);
//             else if (j == 1)
//                 pub_cloud2.publish(cloud_cluster);
//             else if (j == 2)
//                 pub_cloud3.publish(cloud_cluster);

//             j++;

//             // pub_obj_pose.publish(p);
//             // cout << "pose published." << endl;
//             // res.success = true;
//             // return true;
//         }
//     }
//     else{
//         cout << "skipped." << endl;
//         res.success = false;
//         return false;
//     }
//     return true;
// }

int main(int argc, char** argv){
    ros::init(argc, argv, "simple_tracking_node");
    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    SimpleTrackingNode node(&nh);
    ros::spin();
    return 0;
}