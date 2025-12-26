#include <dirent.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <types.h>
#include <livox_interfaces/msg/custom_msg.hpp>

class DisplayPredictionNode : public rclcpp::Node
{
public:
    DisplayPredictionNode()
    : Node("display_prediction")
    {
        this->declare_parameter<std::string>("dyn_obj/pc_file", "");
        this->declare_parameter<std::string>("dyn_obj/pred_file", "");
        this->declare_parameter<std::string>("dyn_obj/pc_topic", "/velodyne_points");
        this->declare_parameter<std::string>("dyn_obj/frame_id", "camera_init");

        this->get_parameter("dyn_obj/pc_file", pc_folder_);
        this->get_parameter("dyn_obj/pred_file", pred_folder_);
        this->get_parameter("dyn_obj/pc_topic", points_topic_);
        this->get_parameter("dyn_obj/frame_id", frame_id_);

        pred_num_ = CountFiles(pred_folder_);
        minus_num_ = 0;

        pub_pointcloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/mosa/result_view", rclcpp::QoS(10));
        pub_marker_ = this->create_publisher<visualization_msgs::msg::Marker>(
            "/mosa/text_view", rclcpp::QoS(10));
        pub_iou_view_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/mosa/iou_view", rclcpp::QoS(10));
        pub_static_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/mosa/std_points", rclcpp::QoS(10));
        pub_dynamic_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/mosa/dyn_points", rclcpp::QoS(10));

        auto sensor_qos = rclcpp::SensorDataQoS();
        if(points_topic_ == "/livox/lidar")
        {
            sub_livox_ = this->create_subscription<livox_interfaces::msg::CustomMsg>(
                points_topic_, sensor_qos,
                std::bind(&DisplayPredictionNode::AviaPointsCallback, this, std::placeholders::_1));
        }
        else
        {
            sub_pcl_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                points_topic_, sensor_qos,
                std::bind(&DisplayPredictionNode::PointsCallback, this, std::placeholders::_1));
        }
    }

private:
    static int CountFiles(const std::string &folder)
    {
        if(folder.empty())
        {
            return 0;
        }
        DIR* dir = opendir(folder.c_str());
        if(!dir)
        {
            return 0;
        }
        int count = 0;
        struct dirent* ptr;
        while((ptr = readdir(dir)) != nullptr)
        {
            if(ptr->d_name[0] == '.') {continue;}
            count++;
        }
        closedir(dir);
        return count;
    }

    void PointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg_in)
    {
        PointCloudXYZI::Ptr points_in(new PointCloudXYZI());
        pcl::fromROSMsg(*msg_in, *points_in);
        HandleFrame(points_in);
    }

    void AviaPointsCallback(const livox_interfaces::msg::CustomMsg::SharedPtr msg_in)
    {
        PointCloudXYZI::Ptr points_in(new PointCloudXYZI());
        points_in->resize(msg_in->point_num);
        if(msg_in->point_num == 0) return;
        for(uint32_t i = 0; i < msg_in->point_num; i++)
        {
            points_in->points[i].x = msg_in->points[i].x;
            points_in->points[i].y = msg_in->points[i].y;
            points_in->points[i].z = msg_in->points[i].z;
        }
        HandleFrame(points_in);
    }

    void HandleFrame(const PointCloudXYZI::Ptr &points_in)
    {
        if(frames_ < minus_num_)
        {
            sensor_msgs::msg::PointCloud2 pcl_ros_msg;
            pcl::toROSMsg(*points_in, pcl_ros_msg);
            pcl_ros_msg.header.frame_id = frame_id_;
            pcl_ros_msg.header.stamp = now();
            pub_pointcloud_->publish(pcl_ros_msg);
        }
        else
        {
            std::cout << "frame: " << frames_ << std::endl;

            std::string pred_file = pred_folder_;
            std::stringstream sss;
            sss << std::setw(6) << std::setfill('0') << frames_ - minus_num_;
            pred_file += sss.str();
            pred_file.append(".label");

            std::fstream pred_input(pred_file.c_str(), std::ios::in | std::ios::binary);
            if(!pred_input.good())
            {
                std::cerr << "Could not read prediction file: " << pred_file << std::endl;
                exit(EXIT_FAILURE);
            }

            pcl::PointCloud<pcl::PointXYZI>::Ptr points_out(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr iou_out(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr dynamic_out(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr static_out(new pcl::PointCloud<pcl::PointXYZI>);

            int tp = 0, fn = 0, fp = 0, count = 0;
            float iou = 0.0f;
            for (size_t i = 0; i < points_in->points.size(); i++)
            {
                pcl::PointXYZI point;
                point.x = points_in->points[i].x;
                point.y = points_in->points[i].y;
                point.z = points_in->points[i].z;

                int pred_num;
                pred_input.read((char *) &pred_num, sizeof(int));
                pred_num = pred_num & 0xFFFF;

                point.intensity = 0;
                if(pred_num >= 251)
                {
                    point.intensity = 10;
                    iou_out->push_back(point);
                    dynamic_out->push_back(point);
                }
                else
                {
                    point.intensity = 20;
                    iou_out->push_back(point);
                    static_out->push_back(point);
                }

                points_out->push_back(point);
            }

            sensor_msgs::msg::PointCloud2 pcl_ros_msg;
            pcl::toROSMsg(*points_out, pcl_ros_msg);
            pcl_ros_msg.header.frame_id = frame_id_;
            pcl_ros_msg.header.stamp = now();
            pub_pointcloud_->publish(pcl_ros_msg);

            sensor_msgs::msg::PointCloud2 pcl_msg;
            pcl::toROSMsg(*iou_out, pcl_msg);
            pcl_msg.header.frame_id = frame_id_;
            pcl_msg.header.stamp = now();
            pub_iou_view_->publish(pcl_msg);

            sensor_msgs::msg::PointCloud2 dynamic_msg;
            pcl::toROSMsg(*dynamic_out, dynamic_msg);
            dynamic_msg.header.frame_id = frame_id_;
            dynamic_msg.header.stamp = now();
            pub_dynamic_->publish(dynamic_msg);

            sensor_msgs::msg::PointCloud2 static_msg;
            pcl::toROSMsg(*static_out, static_msg);
            static_msg.header.frame_id = frame_id_;
            static_msg.header.stamp = now();
            pub_static_->publish(static_msg);

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = frame_id_;
            marker.header.stamp = now();
            marker.ns = "basic_shapes";
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w = 1.0;
            marker.id = 0;
            marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;

            if (!points_out->points.empty())
            {
                marker.scale.z = 0.2;
                marker.color.b = 0.0;
                marker.color.g = 0.0;
                marker.color.r = 1.0;
                marker.color.a = 1.0;
                geometry_msgs::msg::Pose pose;
                pose.position.x = points_out->points[0].x;
                pose.position.y = points_out->points[0].y;
                pose.position.z = points_out->points[0].z;
                std::ostringstream str;
                str << "tp: " << tp << " fn: " << fn << " fp: " << fp << " count: " << count << " iou: " << iou;
                marker.text = str.str();
                marker.pose = pose;
                pub_marker_->publish(marker);
            }
        }

        frames_++;
    }

    std::string points_topic_ = "/velodyne_points";
    std::string frame_id_ = "camera_init";
    std::string pc_folder_;
    std::string pred_folder_;

    int pred_num_ = 0;
    int frames_ = 0;
    int minus_num_ = 1;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_marker_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_iou_view_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_static_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_dynamic_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_;
    rclcpp::Subscription<livox_interfaces::msg::CustomMsg>::SharedPtr sub_livox_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DisplayPredictionNode>());
    rclcpp::shutdown();
    return 0;
}
