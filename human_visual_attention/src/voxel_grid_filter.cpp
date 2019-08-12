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

#include <human_visual_attention/HumanAttention.h>
#include <human_visual_attention/HumanAttentionArray.h>

#include <human_visual_attention/HumanMemory.h>
#include <human_visual_attention/HumanMemoryArray.h>

#include <unordered_map>

#include <math.h>

#include <boost/algorithm/clamp.hpp>

typedef pcl::PointXYZI PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;

typedef sensor_msgs::PointCloud2 PointCloudROS;

ros::Publisher pub_cumulated, pub_memory;

class ElementAttention{

    private:

    PointCloudPtr points_ptr;

    void add_pointcloud(PointCloud new_pointcloud){
        *this->points_ptr += new_pointcloud;
        std::cout << "cloud after add: " << this->points_ptr->size() << std::endl;
    }

    void merge_points(std::vector<PointT, Eigen::aligned_allocator<PointT>> new_points){
        pcl::KdTreeFLANN<PointT> kdtree;
        kdtree.setInputCloud (this->points_ptr);
        float radius = 0.15;

        BOOST_FOREACH (const PointT& pt, new_points){
            std::vector<int> pointIdxRadiusSearch; //to store index of surrounding points 
            std::vector<float> pointRadiusSquaredDistance; // to store distance to surrounding

            if ( kdtree.radiusSearch (pt, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 ){
                for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i){
                    //std::cout << "-- distance -- " << pointRadiusSquaredDistance[i] << std::endl;
                    if (this->points_ptr->points[pointIdxRadiusSearch[i]].intensity < 1.0){
                        //this->points_ptr->points[pointIdxRadiusSearch[i]].intensity += sqrt((pt.intensity*this->points_ptr->points[pointIdxRadiusSearch[i]].intensity));
                        this->points_ptr->points[pointIdxRadiusSearch[i]].intensity += pt.intensity;
                        this->points_ptr->points[pointIdxRadiusSearch[i]].intensity = boost::algorithm::clamp(this->points_ptr->points[pointIdxRadiusSearch[i]].intensity, 0.0, 1.0);
                        
                        if (this->points_ptr->points[pointIdxRadiusSearch[i]].intensity > this->max_attention){
                            this->max_attention = this->points_ptr->points[pointIdxRadiusSearch[i]].intensity;
                        }
                    }
                }
                if(pointIdxRadiusSearch.size () == 0 ){
                    this->points_ptr->push_back(pt);
                    if (pt.intensity > this->max_attention){
                        this->max_attention = pt.intensity;
                    }
                }
            }
            else{
                this->points_ptr->push_back(pt);
                if (pt.intensity > this->max_attention){
                    this->max_attention = pt.intensity;
                }
            }
        }

        if (this->points_ptr->size() > this->max_points){
            this->max_points = this->points_ptr->size();
        }
    }

    bool is_empty(){
        return (this->points_ptr->points.size() == 0);
    }

    public:
    ros::Time last_seen;
    ros::Duration time_from_last_seen;
    ros::Duration last_watchtime;
    bool is_watched;
    float max_attention;
    int max_points;
    
    ElementAttention(){
        this->points_ptr = boost::shared_ptr<PointCloud>(new PointCloud);
        this->time_from_last_seen = ros::Duration(0.0);
        this->last_watchtime = ros::Duration(0.0);
        this->max_attention = 0.0f;
        this->max_points = 0;
        this->is_watched = false;
    }

    void element_seen(PointCloud new_pointcloud, ros::Time stamp){
        // assert new_pointcloud is not empty
        ros::Duration dtime = stamp - this->last_seen;

        this->last_seen = stamp;
        this->time_from_last_seen = ros::Duration(0.0);

        if (this->is_watched){
            this->last_watchtime += dtime;
        }
        else{
            this->is_watched = true;
            this->last_watchtime = dtime;
        }

        if (this->is_empty()){
            this->add_pointcloud(new_pointcloud);
        }
        else {
            this->merge_points(new_pointcloud.points);
        }
    }

    void set_frame_id(std::string frame_id){
        this->points_ptr->header.frame_id = frame_id;
    }

    void element_not_seen(ros::Time stamp){
        ros::Duration dtime = stamp - this->last_seen;
        if (this->is_watched){
            this->last_seen = stamp;
            this->time_from_last_seen = ros::Duration(0.0);
            this->is_watched = false;
        }
        else{
            this->time_from_last_seen += dtime;
        }
        /*
        // decrease
        BOOST_FOREACH (PointT& pt, this->points_ptr->points){
            // TODO: add dtime for a good evolution along the time
            // 2.302585 is the perfect factor 
            pt.intensity = 2.35 * log(pt.intensity) + 1.0; // nice curve and f(1)=1, and f(x<1) < x
        }

        // filter intensity
        
        pcl::PassThrough<PointT> pass;
        pass.setInputCloud (this->points_ptr);
        pass.setFilterFieldName ("intensity");
        pass.setFilterLimits (0.0, 1.0);
        pass.filter (*this->points_ptr);
        */
    }

    int size(){
        return this->points_ptr->size();
    }

    PointCloudPtr get_points(){
        return this->points_ptr;
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

    void add_element(std::string element_id, std::string frame_id){
        this->elements[element_id] = ElementAttention();
        this->elements[element_id].get_points()->header.frame_id = frame_id;
    }

    // Access specifier 
    public: 
    void handle_new_element_attention(std::string element_id, PointCloud new_pointcloud, ros::Time stamp){
        // add as new element if not already known
        if (!this->contains(element_id)){
            this->add_element(element_id, new_pointcloud.header.frame_id);
        }

        // if new points, then update the current state
        if (new_pointcloud.points.size() > 0){
            this->get_element(element_id).element_seen(new_pointcloud, stamp);
        }
        else{
            this->get_element(element_id).element_not_seen(stamp);
        }
    }

    ElementsMap get_elements(){
        return this->elements;
    }

    ElementAttention& get_element(std::string element_id){
        return this->elements[element_id];
    }
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

    HumanVisualAttention& get_human(std::string human_id){
        return this->humans[human_id];
    }

    public:

    void handle_new_ros_msg(const human_visual_attention::HumanAttentionArray::ConstPtr& msg){

        std::cout << "received!" << std::endl;

        human_visual_attention::HumanAttentionArray attention_array;
        attention_array.header = msg->header;

        human_visual_attention::HumanMemoryArray memory_array;
        memory_array.header = msg->header;

        // for each human
        BOOST_FOREACH (const human_visual_attention::HumanAttention& human, msg->humans){
            std::string human_id = human.human_id;
            std::string element_of_attention = human.element_of_attention;

            std::cout << "human " << human_id << " is looking at " << human.element_of_attention << std::endl;

            // add as new human if not already known
            if (!this->contains(human_id)){
                this->add_human(human_id);
            }
            
            human_visual_attention::HumanAttention new_attention;
            new_attention.human_id = human.human_id;
            new_attention.element_of_attention = human.element_of_attention;
            new_attention.point_of_attention = human.point_of_attention;

            human_visual_attention::HumanMemory human_memory;
            human_memory.human_id = human.human_id;

            // for each element
            for (size_t i = 0; i < human.elements.size (); ++i){
                PointCloud pcl_pointcloud;
                pcl::fromROSMsg(human.elements[i], pcl_pointcloud);
                this->get_human(human_id).handle_new_element_attention(human.element_ids[i], pcl_pointcloud, human.elements[i].header.stamp);
            }

            // get pointcloud for each human, and each element
            for (auto element : this->get_human(human_id).get_elements()){
                new_attention.element_ids.push_back(element.first);
                PointCloudPtr pc = element.second.get_points();
                sensor_msgs::PointCloud2 pc2;
                pcl::toROSMsg(*pc, pc2);
                new_attention.elements.push_back(pc2);

                human_visual_attention::ElementInMemory element_msg;
                element_msg.last_time = element.second.last_seen;
                element_msg.time_from_last_time = element.second.time_from_last_seen;
                element_msg.last_watchtime = element.second.last_watchtime;
                element_msg.max_attention_value = element.second.max_attention;
                element_msg.max_attention_count = element.second.max_points;
                human_memory.elements.push_back(element_msg);
            }
            memory_array.humans.push_back(human_memory);
            attention_array.humans.push_back(new_attention);
        }
        pub_memory.publish(memory_array);
        pub_cumulated.publish(attention_array);
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
    //ros::Subscriber sub = nh.subscribe<human_provider::HumanAttention> ("/humans/visual_attention", 10, &HumanManager::handle_new_ros_msg, boost::make_shared<HumanManager>(human_manager));
    ros::Subscriber sub = nh.subscribe<human_visual_attention::HumanAttentionArray> ("/humans/visual/current", 1, &HumanManager::handle_new_ros_msg, &human_manager);

    // Create a ROS publisher for the output point cloud
    pub_cumulated = nh.advertise<human_visual_attention::HumanAttentionArray> ("/humans/visual/cumulated", 1);

    // Create a ROS publisher for the output element data
    pub_memory = nh.advertise<human_visual_attention::HumanMemoryArray> ("/humans/visual/memory", 1);

    // Spin
    ros::spin();
    /*
    ros::Rate r(30); // 30 hz
    while (ros::ok()){
        ros::spinOnce();
        r.sleep();
    }
    */
}