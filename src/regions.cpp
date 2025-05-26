#include <math.h>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/conversions.h"

using namespace std;

class CloudRegions : public rclcpp::Node {

    public:

        list<string> regions;
        map<string, double> config;

        CloudRegions() : Node("regions") {

            this->regions = {
                "front",
                "frontright",
                "frontleft",
                "rear"
            };

            list<string> params;

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
            }

            for(string param : params) {
                this->declare_parameter(param, rclcpp::PARAMETER_DOUBLE);
            }
            this->declare_parameter("pointcloud_topic", rclcpp::PARAMETER_STRING);

            string pointcloud_topic = this->get_parameter("pointcloud_topic").as_string();

            for(string param : params) {
                this->config[param] = this->get_parameter(param).as_double();
            }

            this->add_on_set_parameters_callback(bind(&CloudRegions::params_cb, this, placeholders::_1));




        }

    private:

        void pointcloud_cb(const sensor_msgs::msg::PointCloud2::ConstPtr msg) {
            
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


};


