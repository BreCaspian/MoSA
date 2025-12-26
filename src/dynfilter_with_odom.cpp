#include <chrono>
#include <deque>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <types.h>
#include <mosa/DynObjFilter.h>
#include <livox_interfaces/msg/custom_msg.hpp>

class DynFilterOdomNode : public rclcpp::Node
{
public:
    DynFilterOdomNode()
    : Node("dynfilter_odom")
    {
        this->declare_parameter<bool>("dyn_obj/use_odom", true);
        this->declare_parameter<std::vector<double>>("dyn_obj/static_pos", {0.0, 0.0, 0.0});
        this->declare_parameter<std::vector<double>>("dyn_obj/static_quat", {1.0, 0.0, 0.0, 0.0});
        this->declare_parameter<std::string>("dyn_obj/points_topic", "");
        this->declare_parameter<bool>("dyn_obj/use_livox_custom", true);
        this->declare_parameter<std::string>("dyn_obj/odom_topic", "");
        this->declare_parameter<std::string>("dyn_obj/out_file", "");
        this->declare_parameter<std::string>("dyn_obj/out_file_origin", "");
        this->get_parameter("dyn_obj/use_odom", use_odom_);
        this->get_parameter("dyn_obj/points_topic", points_topic_);
        this->get_parameter("dyn_obj/use_livox_custom", use_livox_custom_);
        this->get_parameter("dyn_obj/odom_topic", odom_topic_);
        this->get_parameter("dyn_obj/out_file", out_folder_);
        this->get_parameter("dyn_obj/out_file_origin", out_folder_origin_);
        this->get_parameter("dyn_obj/static_pos", static_pos_);
        this->get_parameter("dyn_obj/static_quat", static_quat_);

        dyn_filter_ = std::make_shared<DynObjFilter>();
        dyn_filter_->init(*this);

        pub_pcl_dyn_extend_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/mosa/frame_out", rclcpp::QoS(10));
        pub_pcl_dyn_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/mosa/point_out", rclcpp::QoS(10));
        pub_pcl_std_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/mosa/std_points", rclcpp::QoS(10));
        pub_aabb_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/mosa/aabb", rclcpp::QoS(10));
        dyn_filter_->Cluster.SetAabbPublisher(pub_aabb_);

        auto sensor_qos = rclcpp::SensorDataQoS();
        const std::string pcl_topic = points_topic_.empty() ? "/livox/lidar" : points_topic_;
        if(use_livox_custom_ && (points_topic_.empty() || points_topic_ == "/livox/lidar"))
        {
            sub_livox_ = this->create_subscription<livox_interfaces::msg::CustomMsg>(
                "/livox/lidar", sensor_qos,
                std::bind(&DynFilterOdomNode::LivoxCallback, this, std::placeholders::_1));
        }
        else
        {
            sub_pcl_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                pcl_topic, sensor_qos,
                std::bind(&DynFilterOdomNode::PointsCallback, this, std::placeholders::_1));
        }
        if(use_odom_)
        {
            sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
                odom_topic_, sensor_qos,
                std::bind(&DynFilterOdomNode::OdomCallback, this, std::placeholders::_1));
        }

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&DynFilterOdomNode::TimerCallback, this));
    }

private:
    void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        const auto &q_msg = msg->pose.pose.orientation;
        Eigen::Quaterniond cur_q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
        cur_rot = cur_q.matrix();
        cur_pos << msg->pose.pose.position.x,
                   msg->pose.pose.position.y,
                   msg->pose.pose.position.z;

        buffer_rots_.push_back(cur_rot);
        buffer_poss_.push_back(cur_pos);
        buffer_times_.push_back(rclcpp::Time(msg->header.stamp).seconds());
    }

    void PointsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
        pcl::fromROSMsg(*msg, *feats_undistort);
        buffer_pcs_.push_back(feats_undistort);
        if(!use_odom_)
        {
            double stamp = GetStampSeconds(msg->header.stamp);
            if(stamp == 0.0)
            {
                stamp = this->now().seconds();
            }
            buffer_pc_times_.push_back(stamp);
        }
    }

    void LivoxCallback(const livox_interfaces::msg::CustomMsg::SharedPtr msg)
    {
        PointCloudXYZI::Ptr points_in(new PointCloudXYZI());
        points_in->resize(msg->point_num);
        for (uint32_t i = 0; i < msg->point_num; i++)
        {
            points_in->points[i].x = msg->points[i].x;
            points_in->points[i].y = msg->points[i].y;
            points_in->points[i].z = msg->points[i].z;
            points_in->points[i].intensity = static_cast<float>(msg->points[i].reflectivity);
            points_in->points[i].curvature = static_cast<float>(msg->points[i].reflectivity);
        }
        buffer_pcs_.push_back(points_in);
        if(!use_odom_)
        {
            double stamp = GetStampSeconds(msg->header.stamp);
            if(stamp == 0.0)
            {
                stamp = this->now().seconds();
            }
            buffer_pc_times_.push_back(stamp);
        }
    }

    void TimerCallback()
    {
        if(buffer_pcs_.empty())
        {
            return;
        }
        if(use_odom_ && (buffer_poss_.empty() || buffer_rots_.empty() || buffer_times_.empty()))
        {
            return;
        }
        if(!use_odom_ && buffer_pc_times_.empty())
        {
            return;
        }

        PointCloudXYZI::Ptr cur_pc = buffer_pcs_.front();
        buffer_pcs_.pop_front();
        M3D cur_rot = Eigen::Matrix3d::Identity();
        V3D cur_pos = Eigen::Vector3d::Zero();
        double cur_time = 0.0;
        if(use_odom_)
        {
            cur_rot = buffer_rots_.front();
            buffer_rots_.pop_front();
            cur_pos = buffer_poss_.front();
            buffer_poss_.pop_front();
            cur_time = buffer_times_.front();
            buffer_times_.pop_front();
        }
        else
        {
            cur_time = buffer_pc_times_.front();
            buffer_pc_times_.pop_front();
            SetStaticPose(cur_rot, cur_pos);
        }

        std::string file_name = out_folder_;
        std::stringstream ss;
        ss << std::setw(6) << std::setfill('0') << cur_frame_;
        file_name += ss.str();
        file_name.append(".label");

        std::string file_name_origin = out_folder_origin_;
        std::stringstream sss;
        sss << std::setw(6) << std::setfill('0') << cur_frame_;
        file_name_origin += sss.str();
        file_name_origin.append(".label");

        if(file_name.length() > 15 || file_name_origin.length() > 15)
        {
            dyn_filter_->set_path(file_name, file_name_origin);
        }

        dyn_filter_->filter(cur_pc, cur_rot, cur_pos, cur_time);
        dyn_filter_->publish_dyn(pub_pcl_dyn_, pub_pcl_dyn_extend_, pub_pcl_std_, cur_time);
        cur_frame_++;
    }

    std::shared_ptr<DynObjFilter> dyn_filter_;
    M3D cur_rot = Eigen::Matrix3d::Identity();
    V3D cur_pos = Eigen::Vector3d::Zero();

    bool use_odom_ = true;
    bool use_livox_custom_ = true;
    std::vector<double> static_pos_;
    std::vector<double> static_quat_;
    std::string points_topic_;
    std::string odom_topic_;
    std::string out_folder_;
    std::string out_folder_origin_;
    int cur_frame_ = 0;

    std::deque<M3D> buffer_rots_;
    std::deque<V3D> buffer_poss_;
    std::deque<double> buffer_times_;
    std::deque<PointCloudXYZI::Ptr> buffer_pcs_;
    std::deque<double> buffer_pc_times_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_dyn_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_dyn_extend_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_std_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_aabb_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_;
    rclcpp::Subscription<livox_interfaces::msg::CustomMsg>::SharedPtr sub_livox_;

    rclcpp::TimerBase::SharedPtr timer_;

    static double GetStampSeconds(const builtin_interfaces::msg::Time &stamp)
    {
        if(stamp.sec == 0 && stamp.nanosec == 0)
        {
            return 0.0;
        }
        return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
    }

    void SetStaticPose(M3D &rot, V3D &pos)
    {
        if(static_pos_.size() == 3)
        {
            pos << static_pos_[0], static_pos_[1], static_pos_[2];
        }
        if(static_quat_.size() == 4)
        {
            Eigen::Quaterniond q(static_quat_[0], static_quat_[1], static_quat_[2], static_quat_[3]);
            rot = q.normalized().toRotationMatrix();
        }
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynFilterOdomNode>());
    rclcpp::shutdown();
    return 0;
}
