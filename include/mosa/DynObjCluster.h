#ifndef DYN_OBJ_CLUS_H
#define DYN_OBJ_CLUS_H

#include <rclcpp/rclcpp.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_msgs/msg/header.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/Eigenvalues>
#include <unordered_map>
#include <unordered_set>
#include <cluster_predict/DBSCAN_kdtree.h>
#include <cluster_predict/EA_disk.h>
#include <cluster_predict/voxel_cluster.h>
#include <tbb/concurrent_vector.h>
#include <tbb/spin_mutex.h>

typedef pcl::PointXYZINormal PointType;
typedef std::vector<pcl::PointCloud<PointType>> VoxelMap;
#define HASH_length     10000

struct bbox_t
{
    std::vector<std::vector<int>>                                   Point_indices;
    std::vector<pcl::PointCloud<PointType>>                         Point_cloud;
    pcl::PointCloud<PointType>::Ptr                                 Points;
    std::vector<geometry_msgs::msg::PoseWithCovarianceStamped>           Center;
    std::vector<pcl::PointCloud<PointType>>                         Ground_points;
    std::vector<pcl::PointCloud<PointType>>                         true_ground;
    std::vector<std::unordered_set<int>>                          Ground_voxels_set;
    std::vector<std::vector<int>>                                 Ground_voxels_vec;
    std::vector<Eigen::Vector3f>                                    OOBB_Min_point;
    std::vector<Eigen::Vector3f>                                    OOBB_Max_point;
    std::vector<Eigen::Matrix3f>                                    OOBB_R;
    std::vector<int>                                                umap_points_num;

    void reset()
    {
      Point_indices.clear();
      Point_cloud.clear();
      Center.clear();
      Ground_points.clear();
      true_ground.clear();
      Ground_voxels_set.clear();
      Ground_voxels_vec.clear();
      OOBB_Min_point.clear();
      OOBB_Max_point.clear();
      OOBB_R.clear();
    }
};

struct AabbDetection
{
    Eigen::Vector3f center{Eigen::Vector3f::Zero()};
    Eigen::Vector3f size{Eigen::Vector3f::Zero()};
    bool valid{false};
};

struct TrackAabb
{
    int id{0};
    Eigen::Vector3f center{Eigen::Vector3f::Zero()};
    Eigen::Vector3f size{Eigen::Vector3f::Zero()};
    int hits{0};
    int age{0};
    int time_since_update{0};
};

class DynObjCluster {
public:
    int nn_points_size = 3;
    float nn_points_radius = 0.6f;
    int min_cluster_size = 8;
    int max_cluster_size = 25000;
    int cluster_extend_pixel = 2;
    int cluster_min_pixel_number = 4;
    float Voxel_revolusion = 0.3f;
    bool track_enable = true;
    int track_min_hits = 3;
    int track_max_age = 5;
    int track_min_points = 30;
    float track_gate = 1.5f;
    float aabb_alpha = 0.6f;
    int time_ind =0;
    double time_total = 0.0, time_total_average =0.0;
    rclcpp::Time cluster_begin;
    int cur_frame = 0;
    float thrustable_thresold = 0.3f;
    std::string out_file = "";
    std::ofstream out;
    bool debug_en{false};
    std_msgs::msg::Header header;
    Eigen::Matrix3d odom_rot;
    Eigen::Vector3d odom_pos;
    std::vector<Point_Cloud>  umap;
    std::vector<Point_Cloud>  umap_ground;
    std::vector<Point_Cloud>  umap_insidebox;
    int GridMapsize;
    int GridMapedgesize_xy;
    int GridMapedgesize_z;
    Eigen::Vector3f xyz_origin;
    Eigen::Vector3f maprange;
    // ros::Publisher pub_pcl_dyn_extend;

    DynObjCluster(){};
    ~DynObjCluster(){};

    // void Init(ros::Publisher &pub_pcl_dyn_extend_in, ros::Publisher &cluster_vis_high_in, ros::Publisher &pub_ground_points_in);
    void Init();
    void SetAabbPublisher(const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &pub);
    void Clusterprocess(std::vector<int> &dyn_tag, pcl::PointCloud<PointType> event_point, const pcl::PointCloud<PointType> &raw_point, const std_msgs::msg::Header &header_in, const Eigen::Matrix3d odom_rot_in, const Eigen::Vector3d odom_pos_in);
    void ClusterAndTrack(std::vector<int> &dyn_tag, pcl::PointCloud<PointType>::Ptr &points_in, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_in_msg, std_msgs::msg::Header header_in,\
                    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_out_msg, rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_vis, rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predict_path, bbox_t &bbox, double delta,\
                    const pcl::PointCloud<PointType> &raw_point);
    void GetClusterResult(pcl::PointCloud<PointType>::Ptr points_in, std::vector<pcl::PointIndices> &cluster_indices);
    void GetClusterResult_voxel(pcl::PointCloud<PointType>::Ptr points_in, std::vector<Point_Cloud> &umap_in, std::vector<std::vector<int>> &voxel_clusters, std::unordered_set<int> &used_map_set);
    void PubClusterResult_voxel(std::vector<int> &dyn_tag, std_msgs::msg::Header current_header, bbox_t &bbox, double delta,\
                      std::vector<std::vector<int>> &voxel_clusters, const pcl::PointCloud<PointType> &raw_point, std::unordered_set<int> &used_map_set);
    bool ground_estimate(const pcl::PointCloud<PointType> &ground_pcl, const Eigen::Vector3f &world_z, Eigen::Vector3f &ground_norm, Eigen::Vector4f &ground_plane, pcl::PointCloud<PointType> &true_ground, std::unordered_set<int> &extend_pixels);
    void ground_remove(const Eigen::Vector4f &ground_plane, pcl::PointCloud<PointType> &cluster_pcl, std::vector<int> &cluster_pcl_ind, std::vector<int> &dyn_tag_zero, pcl::PointCloud<PointType> &true_ground, std::vector<Point_Cloud> &umap);
    void isolate_remove(pcl::PointCloud<PointType> &cluster_pcl, std::vector<int> &cluster_pcl_ind, std::vector<int> &dyn_tag_zero);
    void oobb_estimate(const VoxelMap &vmap, const pcl::PointCloud<PointType> &points, Eigen::Vector3f &min_point_obj,\
                       Eigen::Vector3f &max_point_obj, Eigen::Matrix3f &R, const Eigen::Vector3f ground_norm);
    void event_extend(const Eigen::Matrix3f &R, bool ground_detect, bbox_t &bbox, std::vector<int> &dyn_tag_one, const int &bbox_index);
    bool esti_plane(Eigen::Vector4f &pca_result, const pcl::PointCloud<PointType> &point);
    void XYZExtract(const int &position, Eigen::Vector3f &xyz);
private:
    bbox_t bbox_cache_;
    void UpdateTracksAndPublish(const std::vector<AabbDetection> &dets, const std_msgs::msg::Header &header_in);

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_before_low;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_before_high;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_after_low;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_after_high;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_points;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_project;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_vis_low;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_vis_high;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obs_pos_low;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr obs_pos_high;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predict_path_low;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predict_path_high;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_aabb_;
    std::vector<TrackAabb> tracks_;
    int next_track_id_{0};
    std::unordered_set<int> last_pub_ids_;
};


#endif
