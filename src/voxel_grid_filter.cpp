/*
#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>

ros::Publisher pub;

static pcl::PCLPointCloud2 whole_pc;
static pcl::PCLPointCloud2 filtered;

pcl::PCLPointCloud2ConstPtr cloudPtr(&whole_pc);

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Container for original & filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);
    pcl::concatenatePointCloud (filtered, *cloud, whole_pc);

    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    //pcl::ApproximateVoxelGrid<pcl::PointXYZRGBA> sor;
    sor.setInputCloud (cloudPtr);
    sor.setLeafSize (0.1, 0.1, 0.1);
    sor.filter (filtered);

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output;
    pcl_conversions::fromPCL(filtered, output);

    // Publish the data
    pub.publish (output);
    
}
*/
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/octree/octree_search.h>
#include <pcl/filters/passthrough.h>


typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
typedef PointCloud::ConstPtr PointCloudPtr;

PointCloud whole_pc;
PointCloudPtr whole_ptr(&whole_pc);

ros::Publisher pub, pub_dis;

void cloud_cb (const PointCloudPtr& msg)
{
    if (whole_pc.points.size() > 0){
        pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
        //pcl::octree::OctreePointCloudSearch<pcl::PointXYZI> octree (0.1);
        kdtree.setInputCloud (whole_ptr);
        //octree.addPointsFromInputCloud ();
        float radius = 0.1;
        BOOST_FOREACH (const pcl::PointXYZI& pt, msg->points){
            
            std::vector<int> pointIdxRadiusSearch; //to store index of surrounding points 
            std::vector<float> pointRadiusSquaredDistance; // to store distance to surrounding

            if ( kdtree.radiusSearch (pt, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
                for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i){
                    //std::cout << "-- distance -- " << pointRadiusSquaredDistance[i] << std::endl;
                    whole_pc.points[ pointIdxRadiusSearch[i] ].intensity += pt.intensity;
                }
                if(pointIdxRadiusSearch.size () == 0 ){
                    whole_pc.push_back(pt);
                }
            }
            else{
                whole_pc.push_back(pt);
            }
        }
    }
    else{
        whole_pc += (*msg);
    }

    // decrease
    BOOST_FOREACH (pcl::PointXYZI& pt, whole_pc.points){
        if (pt.intensity > 10.0){
            pt.intensity = 10.0;
        }
        else if (pt.intensity <= 10.0){
            pt.intensity -= 0.0025;
        }
        else if (pt.intensity <= 5.0){
            pt.intensity -= 0.005;
        }
        else if (pt.intensity <= 1.0){
            pt.intensity -= 0.01;
        }
        
    }
    
    //PointCloud::Ptr cloud_filtered(new PointCloud);
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud (whole_ptr);
    pass.setFilterFieldName ("intensity");
    pass.setFilterLimits (0.0, 15.0);
    //pass.setFilterLimitsNegative (true);
    pass.filter (whole_pc);

    // Publish the data
    whole_pc.header.frame_id = msg->header.frame_id;
    whole_pc.header.stamp = msg->header.stamp;
    
    pub.publish (whole_pc);

    PointCloud::Ptr filtered(new PointCloud);
    pcl::VoxelGrid<pcl::PointXYZI> sor;
    //pcl::ApproximateVoxelGrid<pcl::PointXYZRGBA> sor;
    sor.setInputCloud (whole_ptr);
    sor.setLeafSize (0.05, 0.05, 0.05);
    sor.filter (*filtered);
    //sor.setDownsampleAllData(true);

    pub_dis.publish (filtered);
    std::cout << "--size: " << (*filtered).points.size() << std::endl;
}

int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "voxel_grid_filter");
    ros::NodeHandle nh;

    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe<PointCloud> ("/humans/visual_attention", 1, cloud_cb);

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<PointCloud> ("/humans/whole_visual_attention", 1);

    // Create a ROS publisher for the output point cloud
    pub_dis = nh.advertise<PointCloud> ("/humans/discrete_visual_attention", 1);

    // Spin
    ros::spin ();
}