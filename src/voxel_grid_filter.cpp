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

#include <human_provider/HumanAttention.h>

#include <unordered_map>

#include <math.h>

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;

typedef sensor_msgs::PointCloud2 PointCloudROS;

//PointCloud whole_pc;
//PointCloudConstPtr whole_ptr(&whole_pc);
ros::Publisher pub, pub_dis;

class ElementAttention{

    private:
    int last_time_seen;
    int time_from_last_seen;
    int time_seen_last_time;
    float max_attention;
    int max_points;

    PointCloudPtr points_ptr; //(new PointCloud);

    void add_pointcloud(PointCloud new_pointcloud){
        *this->points_ptr += new_pointcloud;
    }

    void merge_points(std::vector<PointT, Eigen::aligned_allocator<PointT>> new_points){
        pcl::KdTreeFLANN<PointT> kdtree;
        kdtree.setInputCloud (this->points_ptr);
        float radius = 0.1;

        BOOST_FOREACH (const PointT& pt, new_points){
            std::vector<int> pointIdxRadiusSearch; //to store index of surrounding points 
            std::vector<float> pointRadiusSquaredDistance; // to store distance to surrounding

            if ( kdtree.radiusSearch (pt, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
                for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i){
                    //std::cout << "-- distance -- " << pointRadiusSquaredDistance[i] << std::endl;
                    this->points_ptr->points[pointIdxRadiusSearch[i]].intensity += pt.intensity;
                }
                if(pointIdxRadiusSearch.size () == 0 ){
                    this->points_ptr->push_back(pt);
                }
            }
            else{
                this->points_ptr->push_back(pt);
            }
        }
    }

    bool is_empty(){
        return (this->points_ptr->points.size() == 0);
    }


    public:

    void handle_new_points(PointCloud new_pointcloud){
        if (this->is_empty()){
            this->add_pointcloud(new_pointcloud);
        }
        else {
            this->merge_points(new_pointcloud.points);
        }
    }

    void forget_step(){
        // decrease
        BOOST_FOREACH (PointT& pt, this->points_ptr->points){
            // TODO: add dtime for a good evolution along the time
            // 2.302585 is the perfect factor 
            pt.intensity = 2.35 * log(pt.intensity) + 1.0; // nice curve and f(1)=1, and f(x<1) < x
        }

        //PointCloud::Ptr cloud_filtered(new PointCloud);
        pcl::PassThrough<PointT> pass;
        pass.setInputCloud (this->points_ptr);
        pass.setFilterFieldName ("intensity");
        pass.setFilterLimits (0.0, 1.0);
        pass.filter (*this->points_ptr);
    }

};

class HumanVisualAttention {

    typedef std::unordered_map<std::string, ElementAttention> ElementsMap;

    private:
    ElementsMap elements;
    int last_time_have_seen_something = 0;

    bool contains(std::string element_id){
        return this->elements.find(element_id) != this->elements.end();
    }

    void add_element(std::string element_id){
        this->elements[element_id] = ElementAttention();
    }

    ElementAttention get_element(std::string element_id){
        return this->elements[element_id];
    }

    // Access specifier 
    public: 
    void handle_new_element_attention(std::string element_id, PointCloud new_pointcloud){
        // add as new element if not already known
        if (!this->contains(element_id)){
            this->add_element(element_id);
        }

        // if new points, then update the current state
        if (new_pointcloud.points.size() > 0){
            this->get_element(element_id).handle_new_points(new_pointcloud);
        }
        else{
            this->get_element(element_id).forget_step();
        }
        
    }
    // Data Members 
  
    // Member Functions() 

};

class HumanManager {

    typedef std::unordered_map<std::string, HumanVisualAttention> HumansMap;

    private:
    HumansMap humans;

    bool contains(std::string human_id){
        return this->humans.find(human_id) != this->humans.end();
    }

    void add_human(std::string human_id){
        this->humans[human_id] = HumanVisualAttention();
    }

    HumanVisualAttention get_human(std::string human_id){
        return this->humans[human_id];
    }

    public:

    void handle_new_ros_msg(const human_provider::HumanAttention::ConstPtr& msg){
        std::string human_id = msg->human_id;

        // add as new human if not already known
        if (!this->contains(human_id)){
            this->add_human(human_id);
        }

        for (size_t i = 0; i < msg->attentions.size (); ++i){
            PointCloud pcl_pointcloud;
            pcl::fromROSMsg(msg->attentions[i], pcl_pointcloud);
            this->get_human(human_id).handle_new_element_attention(msg->object_ids[i], pcl_pointcloud);
        }

    }
};


int main (int argc, char** argv)
{
    // Initialize ROS
    ros::init (argc, argv, "voxel_grid_filter");
    ros::NodeHandle nh;

    HumanManager human_manager;

    // Create a ROS subscriber for the input point cloud
    //ros::Subscriber sub = nh.subscribe<PointCloud> ("/humans/visual_attention", 1, cloud_cb);
    ros::Subscriber sub = nh.subscribe<human_provider::HumanAttention> ("/humans/visual_attention", 1, &HumanManager::handle_new_ros_msg, boost::make_shared<HumanManager>(human_manager));

    // Create a ROS publisher for the output point cloud
    pub = nh.advertise<PointCloud> ("/humans/whole_visual_attention", 1);

    // Create a ROS publisher for the output point cloud
    pub_dis = nh.advertise<PointCloud> ("/humans/discrete_visual_attention", 1);

    // Spin
    ros::spin ();
}