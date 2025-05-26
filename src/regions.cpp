#include <math.h>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/float32.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>


using namespace std;

class CloudRegions : public rclcpp::Node {

    public:

        vector<string> regions; // list of regions
        map<string, double> config; // list of ROS parameters 
        map<string, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr> region_pubs; // publishers for each region
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub; // publisher for markers

        CloudRegions() : Node("regions") {

            // define regions
            this->regions = {
                "front",
                "frontleft",
                "frontright",
                "rear"
            };

            vector<string> params;

            // create parameters and publishers for each region
            for(string region : this->regions) {
                params.push_back(region + ".x_min");
                params.push_back(region + ".x_max");
                params.push_back(region + ".y_min");
                params.push_back(region + ".y_max");
                params.push_back(region + ".z_min");
                params.push_back(region + ".z_max");
                params.push_back(region + ".color.r");
                params.push_back(region + ".color.g");
                params.push_back(region + ".color.b");
                params.push_back(region + ".color.a");

                region_pubs[region] = this->create_publisher<std_msgs::msg::Float32>("region/" + region + "/closest", 1);
            }

            // do ROS parameter stuff
            for(string param : params) {
                this->declare_parameter(param, rclcpp::PARAMETER_DOUBLE);
            }
            this->declare_parameter("pointcloud_topic", rclcpp::PARAMETER_STRING);

            string pointcloud_topic = this->get_parameter("pointcloud_topic").as_string();
            for(string param : params) {
                this->config[param] = this->get_parameter(param).as_double();
            }
            this->params_subscription_ = this->add_on_set_parameters_callback(bind(&CloudRegions::params_cb, this, placeholders::_1));

            // marker publisher
            this->markers_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("region_markers", 1);

            // pointcloud subscription
            this->pointcloud_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(pointcloud_topic, 1, bind(&CloudRegions::pointcloud_cb, this, placeholders::_1));
            RCLCPP_INFO(this->get_logger(), ("Looking for data on " + pointcloud_topic).c_str());


        }

    private:

        /*
            Takes in a PointCloud2 and determines the distance of the closest point within each region.
        */
        void pointcloud_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::fromROSMsg(*msg, *cloud);
            
            for(string region : this->regions) {
                double closest = INFINITY;
                for(auto pt : cloud->points) {
                    if(point_in_region(pt, region)) {
                        double dist = distance(pt);
                        if(dist < closest) {
                            closest = dist;
                        }
                    }
                }
                auto closest_msg = std_msgs::msg::Float32();
                closest_msg.data = closest;
                region_pubs[region]->publish(closest_msg);
            }
            this->publish_markers();
        }
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscription_;

        /*
            Determines whether a point is within the specified region.
        */
        bool point_in_region(const pcl::PointXYZ& pt, const string& region) {
            if(pt.z > this->config[region + ".z_min"] && pt.z < this->config[region + ".z_max"]) {
                if(pt.y > this->config[region + ".y_min"] && pt.z < this->config[region + ".y_max"]) {
                    if(pt.x > this->config[region + ".x_min"] && pt.z < this->config[region + ".x_max"]) {
                        return true;
                    }
                }
            }
            return false;
        }

        /*
            Finds the distance to a point in XYZ space.
        */
        double distance(pcl::PointXYZ pt) {
            return sqrt(pow(pt.x,2) + pow(pt.y,2) + pow(pt.z,2));
        }

        /*
            Publishes markers for each region.
        */
        void publish_markers() {
            auto markers_msg = visualization_msgs::msg::MarkerArray();

            for(string region : this->regions) {
                auto marker = visualization_msgs::msg::Marker();
                marker.type = 1; // cube
                marker.action = 0; // add
                marker.ns = region; // namespace, essentially the marker's unique name
                marker.header.frame_id = "3d_lidar_link";

                auto pose = geometry_msgs::msg::Pose();
                pose.position.y = (this->config[region + ".x_max"] + this->config[region + ".x_min"]) / 2.0;
                pose.position.x = (this->config[region + ".y_max"] + this->config[region + ".y_min"]) / 2.0;
                pose.position.z = (this->config[region + ".z_max"] + this->config[region + ".z_min"]) / 2.0;
                auto scale = geometry_msgs::msg::Vector3();
                scale.y = (this->config[region + ".x_max"] - this->config[region + ".x_min"]);
                scale.x = (this->config[region + ".y_max"] - this->config[region + ".y_min"]);
                scale.z = (this->config[region + ".z_max"] - this->config[region + ".z_min"]);
                auto color = std_msgs::msg::ColorRGBA();
                color.r = this->config[region + ".color.r"];
                color.g = this->config[region + ".color.g"];
                color.b = this->config[region + ".color.b"];
                color.a = this->config[region + ".color.a"];

                marker.pose = pose;
                marker.scale = scale;
                marker.color = color;

                markers_msg.markers.push_back(marker);
            }
            this->markers_pub->publish(markers_msg);
        }

        // parameter callback
        rcl_interfaces::msg::SetParametersResult params_cb(const vector<rclcpp::Parameter> &parameters) {
            for(rclcpp::Parameter param : parameters) {
                this->config[param.get_name()] = param.as_double();
            }
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            result.reason = "success";
            return result;
        }
        rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_subscription_;


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudRegions>());
  rclcpp::shutdown();
  return 0;
}


