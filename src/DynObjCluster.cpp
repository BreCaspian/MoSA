#include <mosa/DynObjCluster.h>
#include <cluster_predict/EA_disk.h>
#include <algorithm>
#include <chrono>
#include <tbb/concurrent_vector.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/spin_mutex.h>

namespace {
rclcpp::Time Now()
{
    static rclcpp::Clock clock(RCL_SYSTEM_TIME);
    return clock.now();
}
}  // namespace

// void DynObjCluster::Init(ros::Publisher &pub_pcl_dyn_extend_in, ros::Publisher &cluster_vis_high_in, ros::Publisher &pub_ground_points_in)
void DynObjCluster::Init()
{
    // pub_pcl_dyn_extend = pub_pcl_dyn_extend_in;
    // cluster_vis_high = cluster_vis_high_in;
    // pub_ground_points = pub_ground_points_in;
    // xyz_origin << -20., -20., -20.;
    // maprange << 40., 40., 40.;
    xyz_origin << -100., -100., -20.;
    maprange << 200., 200., 40.;
    GridMapedgesize_xy = ceil(maprange(0) / Voxel_revolusion);
    GridMapedgesize_z = ceil(maprange(2) / Voxel_revolusion);
    GridMapsize = GridMapedgesize_xy * GridMapedgesize_xy * GridMapedgesize_z;
    std::cout << "clustering init begin, please wait------------" << GridMapsize << std::endl;
    umap.reserve(GridMapsize);
    umap.resize(GridMapsize);
    umap_ground.reserve(GridMapsize);
    umap_ground.resize(GridMapsize);
    umap_insidebox.reserve(GridMapsize);
    umap_insidebox.resize(GridMapsize);
    std::cout << "clustering init finish------------" << std::endl;
    if(out_file != "") out.open(out_file, std::ios::out  | std::ios::binary);
}

void DynObjCluster::SetAabbPublisher(const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr &pub)
{
    pub_aabb_ = pub;
}

void DynObjCluster::Clusterprocess(std::vector<int> &dyn_tag, pcl::PointCloud<PointType> event_point, const pcl::PointCloud<PointType> &raw_point, const std_msgs::msg::Header &header_in, const Eigen::Matrix3d odom_rot_in, const Eigen::Vector3d odom_pos_in)
{
    cluster_begin = Now();
    header = header_in;
    odom_rot = odom_rot_in;
    odom_pos = odom_pos_in;
    rclcpp::Time t0 = Now();
    float delta_t = 0.1;
    pcl::PointCloud<PointType> extend_points;
    pcl::PointCloud<PointType>::Ptr cloud_clean_ptr(new pcl::PointCloud<PointType>);
    cloud_clean_ptr = event_point.makeShared();
    ClusterAndTrack(dyn_tag, cloud_clean_ptr, pub_pcl_before_high, header, pub_pcl_after_high, cluster_vis_high, predict_path_high, bbox_cache_, delta_t, raw_point);
    rclcpp::Time t3 = Now();
    time_total = (Now() - t0).seconds();
    time_ind++;
    time_total_average = time_total_average * (time_ind - 1) / time_ind + time_total / time_ind;
    cur_frame += 1;
}

void DynObjCluster::ClusterAndTrack(std::vector<int> &dyn_tag, pcl::PointCloud<PointType>::Ptr &points_in, rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_in_msg, std_msgs::msg::Header header_in,\
                    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points_out_msg,
                                    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr cluster_vis, rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predict_path, bbox_t &bbox, double delta,
                                    const pcl::PointCloud<PointType> &raw_point)
{
    sensor_msgs::msg::PointCloud2 pcl4_ros_msg;
    pcl::toROSMsg(*points_in, pcl4_ros_msg);
    pcl4_ros_msg.header.stamp = header_in.stamp;
    pcl4_ros_msg.header.frame_id = header_in.frame_id;
    std::vector<pcl::PointIndices> cluster_indices;
    std::vector<std::vector<int>> voxel_clusters;
    rclcpp::Time t0 = Now();
    std::unordered_set<int> used_map_set;
    GetClusterResult_voxel(points_in, umap, voxel_clusters, used_map_set);
    PubClusterResult_voxel(dyn_tag, header_in, bbox, delta, voxel_clusters, raw_point, used_map_set);
}

void DynObjCluster::GetClusterResult(pcl::PointCloud<PointType>::Ptr points_in, std::vector<pcl::PointIndices> &cluster_indices)
{
    if (points_in->size() < 2)
    {
        return;
    }
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
    tree->setInputCloud(points_in);
    DBSCANKdtreeCluster<PointType> ec;
    ec.setCorePointMinPts(nn_points_size);
    ec.setClusterTolerance(nn_points_radius);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(points_in);
    ec.extract(cluster_indices);
}

void DynObjCluster::GetClusterResult_voxel(pcl::PointCloud<PointType>::Ptr points_in, std::vector<Point_Cloud> &umap_in, std::vector<std::vector<int>> &voxel_clusters, std::unordered_set<int> &used_map_set)
{
    rclcpp::Time t0 = Now();
    if ( (out_file != "") && points_in->size() < 2)
    {   
        out << (Now() - t0).seconds() << " ";
        return;
    }
    VOXEL_CLUSTER cluster;
    cluster.setInputCloud(*points_in);
    cluster.setVoxelResolution(Voxel_revolusion, GridMapedgesize_xy, GridMapedgesize_z, xyz_origin);
    cluster.setExtendRange(cluster_extend_pixel);
    cluster.setMinClusterSize(cluster_min_pixel_number);
    cluster.createVoxelMap(umap_in, used_map_set);
    cluster.extract(voxel_clusters);
    if(out_file != "") out << (Now() - t0).seconds() << " ";
}

void DynObjCluster::PubClusterResult_voxel(std::vector<int> &dyn_tag, std_msgs::msg::Header current_header, bbox_t &bbox, double delta,
                                           std::vector<std::vector<int>> &voxel_clusters, const pcl::PointCloud<PointType> &raw_point, std::unordered_set<int> &used_map_set)
{
    pcl::PointCloud<PointType> cluster_points;
    pcl::PointCloud<PointType> true_ground;
    visualization_msgs::msg::MarkerArray numbers;
    numbers.markers.reserve(200);
    cluster_points.reserve(raw_point.size());
    true_ground.reserve(raw_point.size());
    Eigen::Matrix3f R = odom_rot.cast<float>();
    Eigen::Vector3f world_z = R.col(2);
    int Grid_size_1d = 3;
    int Grid_size = pow(Grid_size_1d, 3);

    rclcpp::Time t0 = Now();
    bbox.Point_cloud.resize(voxel_clusters.size());
    bbox.Point_indices.resize(voxel_clusters.size());
    bbox.Center.resize(voxel_clusters.size());
    bbox.Ground_points.resize(voxel_clusters.size());
    bbox.true_ground.resize(voxel_clusters.size());
    bbox.Ground_voxels_set.resize(voxel_clusters.size());
    bbox.Ground_voxels_vec.resize(voxel_clusters.size());
    bbox.umap_points_num.resize(voxel_clusters.size());

    int bbox_i = 0;
    for (auto it = voxel_clusters.begin(); it != voxel_clusters.end(); it++)
    {
        Eigen::Vector3f xyz;
        XYZExtract(*(it->begin()), xyz);
        float x_min = xyz(0), x_max = xyz(0);
        float y_min = xyz(1), y_max = xyz(1);
        float z_min = xyz(2), z_max = xyz(2);
        int n = 0;
        for (auto pit = it->begin(); pit != it->end(); ++pit)
        {
            int voxel = *pit;
            n = n + umap[voxel].points_num;
            XYZExtract(voxel, xyz);
            if (xyz(0) < x_min)
                x_min = xyz(0);
            if (xyz(1) < y_min)
                y_min = xyz(1);
            if (xyz(2) < z_min)
                z_min = xyz(2);
            if ((xyz(0) + Voxel_revolusion) > x_max)
                x_max = xyz(0) + Voxel_revolusion;
            if ((xyz(1) + Voxel_revolusion) > y_max)
                y_max = xyz(1) + Voxel_revolusion;
            if ((xyz(2) + Voxel_revolusion) > z_max)
                z_max = xyz(2) + Voxel_revolusion;
        }
        float x_size = x_max - x_min;
        float y_size = y_max - y_min;
        float z_size = z_max - z_min;
        if (cluster_min_pixel_number == 1 || (x_size > Voxel_revolusion + 0.001f && y_size > Voxel_revolusion + 0.001f) || (x_size > Voxel_revolusion + 0.001f && z_size > Voxel_revolusion + 0.001f) || (y_size > Voxel_revolusion + 0.001f && z_size > Voxel_revolusion + 0.001f))
        {
            for (auto pit = it->begin(); pit != it->end(); ++pit)
            {
                int voxel = *pit;
                umap[voxel].bbox_index = bbox_i;
            }
            bbox.Point_cloud[bbox_i].clear();
            bbox.Point_indices[bbox_i].clear();
            bbox.Point_cloud[bbox_i].reserve(static_cast<size_t>(n));
            bbox.Point_indices[bbox_i].reserve(static_cast<size_t>(n));
            geometry_msgs::msg::PoseWithCovarianceStamped center;
            center.header = current_header;
            center.pose.pose.position.x = (x_max + x_min) / 2;
            center.pose.pose.position.y = (y_max + y_min) / 2;
            center.pose.pose.position.z = (z_max + z_min) / 2;
            center.pose.pose.orientation.x = 0;
            center.pose.pose.orientation.y = 0;
            center.pose.pose.orientation.z = 0;
            center.pose.pose.orientation.w = 1;
            center.pose.covariance[0 * 6 + 0] = x_size / 2;
            center.pose.covariance[1 * 6 + 1] = y_size / 2;
            center.pose.covariance[2 * 6 + 2] = z_size / 2;
            center.pose.covariance[3 * 6 + 3] = x_size;
            center.pose.covariance[4 * 6 + 4] = y_size;
            center.pose.covariance[5 * 6 + 5] = z_size;
            center.pose.covariance[2 * 6 + 3] = x_max;
            center.pose.covariance[3 * 6 + 4] = y_max;
            center.pose.covariance[4 * 6 + 5] = z_max;
            center.pose.covariance[3 * 6 + 2] = x_min;
            center.pose.covariance[4 * 6 + 3] = y_min;
            center.pose.covariance[5 * 6 + 4] = z_min;
            bbox.Center[bbox_i] = center;
            bbox.Ground_points[bbox_i].clear();
            bbox.true_ground[bbox_i].clear();
            bbox.Ground_voxels_set[bbox_i].clear();
            bbox.Ground_voxels_vec[bbox_i].clear();
            bbox.umap_points_num[bbox_i] = n;
            bbox_i++;
        }
        else
        {
            for (auto v = it->begin(); v != it->end(); ++v)
            {
                umap[*v].reset();
            }
        }
    }
    bbox.Point_cloud.resize(bbox_i);
    bbox.Point_indices.resize(bbox_i);
    bbox.Center.resize(bbox_i);
    bbox.Ground_points.resize(bbox_i);
    bbox.true_ground.resize(bbox_i);
    bbox.Ground_voxels_set.resize(bbox_i);
    bbox.Ground_voxels_vec.resize(bbox_i);
    bbox.umap_points_num.resize(bbox_i);

    rclcpp::Time t1 = Now();
    double hash_newtime = 0.0;
    std::vector<int> index_bbox(bbox.Center.size());
    for (int i = 0; i < bbox.Center.size(); i++)
    {
        index_bbox[i] = i;
    }
    std::vector<std::vector<int>> ground_voxels_candidates(bbox.Center.size());
    std::vector<std::vector<int>> insidebox_voxels_candidates(bbox.Center.size());
    tbb::parallel_for(tbb::blocked_range<size_t>(0, index_bbox.size(), 64), [&](const tbb::blocked_range<size_t> &range) {
        for(size_t idx = range.begin(); idx != range.end(); ++idx)
        {
            int bbox_i = index_bbox[idx];
            std::vector<int> local_ground;
            std::vector<int> local_inside;
            PointType center;
            float x_size = bbox.Center[bbox_i].pose.covariance[3*6+3];
            float y_size = bbox.Center[bbox_i].pose.covariance[4*6+4];
            float z_size = bbox.Center[bbox_i].pose.covariance[5*6+5];
            center.x = bbox.Center[bbox_i].pose.pose.position.x;
            center.y = bbox.Center[bbox_i].pose.pose.position.y;
            center.z = bbox.Center[bbox_i].pose.pose.position.z;
            PointType max;
            max.x = bbox.Center[bbox_i].pose.covariance[2*6+3];
            max.y = bbox.Center[bbox_i].pose.covariance[3*6+4];
            max.z = bbox.Center[bbox_i].pose.covariance[4*6+5];
            PointType min;
            min.x = bbox.Center[bbox_i].pose.covariance[3*6+2];
            min.y = bbox.Center[bbox_i].pose.covariance[4*6+3];
            min.z = bbox.Center[bbox_i].pose.covariance[5*6+4];
            int n_x = std::max(1.0f, 1.0f * x_size) / Voxel_revolusion;
            int n_y = std::max(1.0f, 1.0f * y_size) / Voxel_revolusion;
            int n_z = std::max(1.0f, 1.0f * z_size) / Voxel_revolusion;
            int voxel_center = floor((center.x - xyz_origin(0))/Voxel_revolusion) * GridMapedgesize_xy * GridMapedgesize_z + floor((center.y - xyz_origin(1))/Voxel_revolusion) * GridMapedgesize_z + floor((center.z - xyz_origin(2))/Voxel_revolusion);
            int ii = 0;
            Eigen::Vector3f xyz;
            for (int i = 0; i <= 2 * n_x +1; i++)
            {   
                ii += (i%2 ? 1:-1) * i;
                int jj = 0;
                for (int j = 0; j <= 2 * n_y +1; j++)
                {   
                    jj += (j%2 ? 1:-1) * j;
                    int kk = 0;
                    for (int k = 0; k <= 2 * n_z +1; k++)
                    {    
                        kk += (k%2 ? 1:-1) * k;
                        int voxel = voxel_center + ii * GridMapedgesize_xy * GridMapedgesize_z + jj * GridMapedgesize_z + kk;
                        if(voxel < 0 || voxel > GridMapsize) continue;
                        XYZExtract(voxel, xyz);
                        Eigen::Vector3f voxel_loc(xyz(0) + 0.5f * Voxel_revolusion, xyz(1) + 0.5f * Voxel_revolusion, xyz(2) + 0.5f * Voxel_revolusion);
                        if (umap[voxel].points_num == 0 && !((voxel_loc(0) > min.x && voxel_loc(0) < max.x) && (voxel_loc(1) > min.y && voxel_loc(1) < max.y) && (voxel_loc(2) > min.z && voxel_loc(2) < max.z)))
                        {
                            local_ground.push_back(voxel);
                        }
                        else if (umap[voxel].points_num == 0 && (voxel_loc(0) > min.x && voxel_loc(0) < max.x) && (voxel_loc(1) > min.y && voxel_loc(1) < max.y) && (voxel_loc(2) > min.z && voxel_loc(2) < max.z))
                        {
                            local_inside.push_back(voxel);
                        }
                    }
                }
            }
            ground_voxels_candidates[bbox_i] = std::move(local_ground);
            insidebox_voxels_candidates[bbox_i] = std::move(local_inside);
        }
    });
    const size_t lock_count = 4096;
    std::vector<tbb::spin_mutex> voxel_locks(lock_count);
    tbb::spin_mutex used_map_lock;
    tbb::parallel_for(tbb::blocked_range<size_t>(0, bbox.Center.size(), 64), [&](const tbb::blocked_range<size_t> &range) {
        for(size_t bbox_i = range.begin(); bbox_i != range.end(); ++bbox_i)
        {
            for (int voxel : ground_voxels_candidates[bbox_i])
            {
                tbb::spin_mutex::scoped_lock lock(voxel_locks[voxel % lock_count]);
                if (umap_ground[voxel].bbox_index > -1)
                {
                    continue;
                }
                umap_ground[voxel].bbox_index = static_cast<int>(bbox_i);
                {
                    tbb::spin_mutex::scoped_lock set_lock(used_map_lock);
                    used_map_set.insert(voxel);
                }
                bbox.Ground_voxels_set[bbox_i].emplace(voxel);
                bbox.Ground_voxels_vec[bbox_i].push_back(voxel);
            }
            for (int voxel : insidebox_voxels_candidates[bbox_i])
            {
                tbb::spin_mutex::scoped_lock lock(voxel_locks[voxel % lock_count]);
                if (umap_insidebox[voxel].bbox_index > -1)
                {
                    continue;
                }
                umap_insidebox[voxel].bbox_index = static_cast<int>(bbox_i);
                {
                    tbb::spin_mutex::scoped_lock set_lock(used_map_lock);
                    used_map_set.insert(voxel);
                }
            }
        }
    });


    rclcpp::Time t2 = Now();
    for (int ite = 0; ite < raw_point.size(); ite++)
    {
        if (dyn_tag[ite] == -1)
            continue;
        int voxel = floor((raw_point[ite].x - xyz_origin(0)) / Voxel_revolusion) * GridMapedgesize_xy * GridMapedgesize_z + floor((raw_point[ite].y - xyz_origin(1)) / Voxel_revolusion) * GridMapedgesize_z + floor((raw_point[ite].z - xyz_origin(2)) / Voxel_revolusion);
        if (voxel < 0 || voxel > GridMapsize)
        {
            continue;
        }
        if (umap_ground[voxel].bbox_index > -1)
        {
            bbox.Ground_points[umap_ground[voxel].bbox_index].push_back(raw_point[ite]);
            if (umap_ground[voxel].points_num == 0)
            {
                umap_ground[voxel].cloud.reset(new pcl::PointCloud<PointType>());
                umap_ground[voxel].cloud->reserve(5);
                umap_ground[voxel].cloud_index.reset(new std::vector<int>());
                umap_ground[voxel].cloud_index->reserve(5);
                umap_ground[voxel].cloud->push_back(raw_point[ite]);
                umap_ground[voxel].cloud_index->push_back(ite);
            }
            else
            {
                umap_ground[voxel].cloud->push_back(raw_point[ite]);
                umap_ground[voxel].cloud_index->push_back(ite);
            }
            umap_ground[voxel].points_num++;
            dyn_tag[ite] = 0;
        }
        else if (umap[voxel].points_num > 0 && umap[voxel].bbox_index > -1)
        {
            auto tmp = raw_point[ite];
            tmp.curvature = ite;
            bbox.Point_cloud[umap[voxel].bbox_index].push_back(tmp);
            bbox.Point_indices[umap[voxel].bbox_index].push_back(ite);
            umap[voxel].cloud->push_back(tmp);
            dyn_tag[ite] = 1;
        }
        else if (umap_insidebox[voxel].bbox_index > -1)
        {
            auto tmp = raw_point[ite];
            tmp.curvature = ite;
            bbox.Point_cloud[umap_insidebox[voxel].bbox_index].push_back(tmp);
            bbox.Point_indices[umap_insidebox[voxel].bbox_index].push_back(ite);
            if (umap_insidebox[voxel].points_num == 0)
            {
                umap_insidebox[voxel].cloud.reset(new pcl::PointCloud<PointType>());
                umap_insidebox[voxel].cloud->reserve(5);
                umap_insidebox[voxel].cloud->push_back(tmp);
            }
            else
            {
                umap_insidebox[voxel].cloud->push_back(tmp);
            }
            umap_insidebox[voxel].points_num++;
            dyn_tag[ite] = 1;
        }
        else
        {
            dyn_tag[ite] = 0;
        }
    }
    rclcpp::Time t3 = Now();
    std::vector<double> ground_estimate_total_time(index_bbox.size(), 0.0);
    std::vector<double> region_growth_time(index_bbox.size(), 0.0);
    std::vector<AabbDetection> detections(index_bbox.size());
    tbb::concurrent_vector<PointType> cluster_points_vec;
    tbb::concurrent_vector<PointType> true_ground_vec;
    tbb::concurrent_vector<int> dyn_tag_zero_global;
    tbb::concurrent_vector<int> dyn_tag_one_global;
    tbb::parallel_for(tbb::blocked_range<size_t>(0, index_bbox.size(), 64), [&](const tbb::blocked_range<size_t> &range) {
        for(size_t idx = range.begin(); idx != range.end(); ++idx)
        {
            int k = index_bbox[idx];
            geometry_msgs::msg::PoseWithCovarianceStamped center = bbox.Center[k];
            float x_size = center.pose.covariance[3*6+3];
            float y_size = center.pose.covariance[4*6+4];
            float z_size = center.pose.covariance[5*6+5];
            float x_min = center.pose.covariance[3*6+2];
            float y_min = center.pose.covariance[4*6+3];
            float z_min = center.pose.covariance[5*6+4];
            float x_max = center.pose.covariance[2*6+3];
            float y_max = center.pose.covariance[3*6+4];
            float z_max = center.pose.covariance[4*6+5];

            Eigen::Vector3f ground_norm(0.0, 0.0, 0.0);
            Eigen::Vector4f ground_plane;
            rclcpp::Time t_ge = Now();
            bool ground_detect = ground_estimate(bbox.Ground_points[k], world_z, ground_norm, ground_plane, bbox.true_ground[k], bbox.Ground_voxels_set[k]);
            ground_estimate_total_time[k] = (Now() - t_ge).seconds();
            Eigen::Matrix3f R;
            R.col(0) = ground_norm;
            std::vector<int> dyn_tag_zero_local;
            std::vector<int> dyn_tag_one_local;
            if(ground_detect)
            {   
                rclcpp::Time t_rg = Now();
                event_extend(R, ground_detect, bbox, dyn_tag_one_local, k);
                region_growth_time[k] = (Now() - t_rg).seconds();
                ground_remove(ground_plane, bbox.Point_cloud[k], bbox.Point_indices[k], dyn_tag_zero_local, bbox.true_ground[k], umap);
            }
            isolate_remove(bbox.Point_cloud[k], bbox.Point_indices[k], dyn_tag_zero_local);
            if ((float)bbox.umap_points_num[k] / (float)bbox.Point_cloud[k].size() < thrustable_thresold) // not trustable
            {   
                for (int i = 0; i < bbox.Point_indices[k].size(); i++)
                {
                    dyn_tag_zero_local.push_back(bbox.Point_indices[k][i]);
                }
            }
            else
            {
                for (const auto &pt : bbox.Point_cloud[k].points)
                {
                    cluster_points_vec.push_back(pt);
                }
                for (const auto &pt : bbox.true_ground[k].points)
                {
                    true_ground_vec.push_back(pt);
                }
            }
            if(track_enable && bbox.Point_cloud[k].size() >= static_cast<size_t>(track_min_points))
            {
                Eigen::Vector3f min_pt(1e9f, 1e9f, 1e9f);
                Eigen::Vector3f max_pt(-1e9f, -1e9f, -1e9f);
                for(const auto &pt : bbox.Point_cloud[k].points)
                {
                    min_pt.x() = std::min(min_pt.x(), pt.x);
                    min_pt.y() = std::min(min_pt.y(), pt.y);
                    min_pt.z() = std::min(min_pt.z(), pt.z);
                    max_pt.x() = std::max(max_pt.x(), pt.x);
                    max_pt.y() = std::max(max_pt.y(), pt.y);
                    max_pt.z() = std::max(max_pt.z(), pt.z);
                }
                detections[k].center = 0.5f * (min_pt + max_pt);
                detections[k].size = max_pt - min_pt;
                detections[k].valid = true;
            }
            for (int idx_zero : dyn_tag_zero_local)
            {
                dyn_tag_zero_global.push_back(idx_zero);
            }
            for (int idx_one : dyn_tag_one_local)
            {
                dyn_tag_one_global.push_back(idx_one);
            }
        }
    });
    tbb::parallel_for(tbb::blocked_range<size_t>(0, dyn_tag_one_global.size(), 256), [&](const tbb::blocked_range<size_t> &range) {
        for(size_t idx = range.begin(); idx != range.end(); ++idx)
        {
            dyn_tag[dyn_tag_one_global[idx]] = 1;
        }
    });
    tbb::parallel_for(tbb::blocked_range<size_t>(0, dyn_tag_zero_global.size(), 256), [&](const tbb::blocked_range<size_t> &range) {
        for(size_t idx = range.begin(); idx != range.end(); ++idx)
        {
            dyn_tag[dyn_tag_zero_global[idx]] = 0;
        }
    });
    cluster_points.clear();
    cluster_points.points.resize(cluster_points_vec.size());
    tbb::parallel_for(tbb::blocked_range<size_t>(0, cluster_points_vec.size(), 256), [&](const tbb::blocked_range<size_t> &range) {
        for(size_t idx = range.begin(); idx != range.end(); ++idx)
        {
            cluster_points.points[idx] = cluster_points_vec[idx];
        }
    });
    cluster_points.width = cluster_points.points.size();
    cluster_points.height = 1;
    cluster_points.is_dense = false;
    true_ground.clear();
    true_ground.points.resize(true_ground_vec.size());
    tbb::parallel_for(tbb::blocked_range<size_t>(0, true_ground_vec.size(), 256), [&](const tbb::blocked_range<size_t> &range) {
        for(size_t idx = range.begin(); idx != range.end(); ++idx)
        {
            true_ground.points[idx] = true_ground_vec[idx];
        }
    });
    true_ground.width = true_ground.points.size();
    true_ground.height = 1;
    true_ground.is_dense = false;
    double total_ground_estimate_total_time=0.0;
    double total_region_growth_time=0.0;
    for(int i = 0; i< index_bbox.size(); i++)
    {
        total_ground_estimate_total_time += ground_estimate_total_time[i];
        total_region_growth_time += region_growth_time[i];
    }
    if(out_file != "") out << total_ground_estimate_total_time << " ";
    if(out_file != "") out << total_region_growth_time << " ";

    // cluster_vis_high.publish(numbers);

    if(track_enable && pub_aabb_)
    {
        UpdateTracksAndPublish(detections, current_header);
    }

    rclcpp::Time t5 = Now();
    for (auto ite = used_map_set.begin(); ite != used_map_set.end(); ite++)
    {
        umap[*ite].reset();
        umap_ground[*ite].reset();
        umap_insidebox[*ite].reset();
    }

    double cluster_time = (Now() - cluster_begin).seconds() - total_region_growth_time;
    if(out_file != "") out << cluster_time << std::endl;
}

bool DynObjCluster::ground_estimate(const pcl::PointCloud<PointType> &ground_pcl, const Eigen::Vector3f &world_z, Eigen::Vector3f &ground_norm, Eigen::Vector4f &ground_plane, pcl::PointCloud<PointType> &true_ground, std::unordered_set<int> &extend_pixels)
{
    if (!ground_pcl.size() > 0)
        return false;
    int BNUM = std::max(4, (int)ground_pcl.size() / 100);
    const float thershold = 0.10f;
    const float max_angle_from_body = 30.0f / 57.3f;
    pcl::PointCloud<PointType> split_pcl;
    int max_count = 0;
    Eigen::Vector3f max_normvec(0, 0, 0);
    pcl::PointCloud<PointType> max_points;
    for (int i = 0; i < ground_pcl.size(); i++)
    {
        split_pcl.push_back(ground_pcl[i]);
        if (split_pcl.size() == BNUM)
        {
            Eigen::Vector4f plane;
            if (esti_plane(plane, split_pcl) && plane[3] < thershold)
            {
                Eigen::Vector3f normvec = plane.head(3).normalized();
                if (normvec.cross(world_z).norm() < sin(max_angle_from_body))
                {
                    int count = 0;
                    pcl::PointCloud<PointType> tmp_points;
                    for (int j = 0; j < ground_pcl.size(); j++)
                    {
                        Eigen::Vector3f point;
                        point[0] = ground_pcl[j].x;
                        point[1] = ground_pcl[j].y;
                        point[2] = ground_pcl[j].z;
                        float dis = fabs(point.dot(plane.head(3)) + 1.0f) / plane.head(3).norm();
                        if (dis < thershold)
                        {
                            tmp_points.push_back(ground_pcl[j]);
                            count++;
                        }
                    }
                    if (count > max_count)
                    {
                        max_count = count;
                        max_normvec = normvec;
                        ground_plane = plane;
                        max_points = tmp_points;
                        if (max_count > 0.6f * ground_pcl.size())
                            break;
                    }
                }
            }
            split_pcl.clear();
        }
    }
    if (ground_pcl.size() > 0 && (max_count > 0.2f * ground_pcl.size() || max_count > 500))
    {
        Eigen::Vector4f plane;
        if (esti_plane(plane, max_points) && plane[3] < thershold)
        {
            Eigen::Vector3f normvec = plane.head(3).normalized();
            if (normvec.cross(world_z).norm() < sin(max_angle_from_body))
            {
                max_normvec = normvec;
                ground_plane = plane;
            }
        }
        for (int j = 0; j < ground_pcl.size(); j++)
        {
            Eigen::Vector3f point;
            point[0] = ground_pcl[j].x;
            point[1] = ground_pcl[j].y;
            point[2] = ground_pcl[j].z;
            float dis = fabs(point.dot(ground_plane.head(3)) + 1.0f) / ground_plane.head(3).norm();
            if (dis < thershold)
            {
                true_ground.push_back(ground_pcl[j]);
                int voxel = floor((ground_pcl[j].x - xyz_origin(0)) / Voxel_revolusion) * GridMapedgesize_xy * GridMapedgesize_z + floor((ground_pcl[j].y - xyz_origin(1)) / Voxel_revolusion) * GridMapedgesize_z + floor((ground_pcl[j].z - xyz_origin(2)) / Voxel_revolusion);
                extend_pixels.erase(voxel);
            }
        }
        if (max_normvec[2] < 0)
            max_normvec *= -1;
        ground_norm = max_normvec;
    }
    if (abs(ground_norm.norm() - 1.0f) < 0.1f)
        return true;
    else
        return false;
}

void DynObjCluster::ground_remove(const Eigen::Vector4f &ground_plane, pcl::PointCloud<PointType> &cluster_pcl, std::vector<int> &cluster_pcl_ind, std::vector<int> &dyn_tag_zero, pcl::PointCloud<PointType> &true_ground, std::vector<Point_Cloud> &umap)
{
    const float thershold = 0.10f;
    pcl::PointCloud<PointType> new_clustet_pcl;
    std::vector<int> new_cluster_pcl_ind;
    for (int i = 0; i < cluster_pcl.size(); i++)
    {
        Eigen::Vector3f point;
        point[0] = cluster_pcl[i].x;
        point[1] = cluster_pcl[i].y;
        point[2] = cluster_pcl[i].z;
        float dis = fabs(point.dot(ground_plane.head(3)) + 1.0f) / ground_plane.head(3).norm();
        if (dis > thershold)
        {
            new_clustet_pcl.push_back(cluster_pcl[i]);
            new_cluster_pcl_ind.push_back(cluster_pcl_ind[i]);
        }
        else
        {
            dyn_tag_zero.push_back(cluster_pcl_ind[i]);
            true_ground.push_back(cluster_pcl[i]);
        }
    }
    cluster_pcl = new_clustet_pcl;
    cluster_pcl_ind = new_cluster_pcl_ind;
}

void DynObjCluster::isolate_remove(pcl::PointCloud<PointType> &cluster_pcl, std::vector<int> &cluster_pcl_ind, std::vector<int> &dyn_tag_zero)
{
    if (cluster_pcl.size() < 2)
    {
        return;
    }
    pcl::PointCloud<PointType> new_cluster_pcl;
    std::vector<int> new_cluster_pcl_ind;
    VOXEL_CLUSTER cluster;
    std::unordered_map<int, Point_Cloud::Ptr> umap_cluster;
    std::vector<std::vector<int>> voxel_cluster;
    cluster.setInputCloud(cluster_pcl);
    cluster.setVoxelResolution(Voxel_revolusion, GridMapedgesize_xy, GridMapedgesize_z, xyz_origin);
    cluster.setExtendRange(cluster_extend_pixel);
    cluster.setMinClusterSize(cluster_min_pixel_number);
    cluster.createVoxelMap(umap_cluster);
    cluster.extract(voxel_cluster);
    int max_cluster_ind = 0;
    int max_voxel_num = 0;
    for (int i = 0; i < voxel_cluster.size(); i++)
    {
        if (voxel_cluster[i].size() > max_voxel_num)
        {
            max_cluster_ind = i;
            max_voxel_num = voxel_cluster[i].size();
        }
    }
    std::unordered_set<int> dyn_index;
    for (int i = 0; i < max_voxel_num; i++)
    {
        int voxel = voxel_cluster[max_cluster_ind][i];
        for (int j = 0; j < umap_cluster[voxel]->cloud->size(); j++)
        {
            new_cluster_pcl.push_back(umap_cluster[voxel]->cloud->points[j]);
            new_cluster_pcl_ind.push_back(cluster_pcl_ind[umap_cluster[voxel]->cloud_index->at(j)]);
            dyn_index.insert(cluster_pcl_ind[umap_cluster[voxel]->cloud_index->at(j)]);
        }
    }
    for (int i = 0; i < cluster_pcl_ind.size(); i++)
    {
        if (!dyn_index.count(cluster_pcl_ind[i]))
        {
            dyn_tag_zero.push_back(cluster_pcl_ind[i]);
        }
    }
    std::unordered_map<int, Point_Cloud::Ptr>().swap(umap_cluster);
    cluster_pcl = new_cluster_pcl;
    cluster_pcl_ind = new_cluster_pcl_ind;
}

void DynObjCluster::UpdateTracksAndPublish(const std::vector<AabbDetection> &dets, const std_msgs::msg::Header &header_in)
{
    if(!pub_aabb_)
    {
        return;
    }
    const float gate = std::max(0.1f, track_gate);
    std::vector<int> det_to_track(dets.size(), -1);
    std::vector<bool> track_used(tracks_.size(), false);

    for(size_t i = 0; i < dets.size(); ++i)
    {
        if(!dets[i].valid)
        {
            continue;
        }
        float best_dist = gate;
        int best_idx = -1;
        for(size_t t = 0; t < tracks_.size(); ++t)
        {
            if(track_used[t])
            {
                continue;
            }
            float dist = (dets[i].center - tracks_[t].center).norm();
            if(dist < best_dist)
            {
                best_dist = dist;
                best_idx = static_cast<int>(t);
            }
        }
        if(best_idx >= 0)
        {
            det_to_track[i] = best_idx;
            track_used[best_idx] = true;
        }
    }

    for(auto &track : tracks_)
    {
        track.age++;
        track.time_since_update++;
    }

    for(size_t i = 0; i < dets.size(); ++i)
    {
        if(!dets[i].valid)
        {
            continue;
        }
        int t = det_to_track[i];
        if(t >= 0)
        {
            TrackAabb &track = tracks_[t];
            track.time_since_update = 0;
            track.hits++;
            track.center = aabb_alpha * dets[i].center + (1.0f - aabb_alpha) * track.center;
            track.size = aabb_alpha * dets[i].size + (1.0f - aabb_alpha) * track.size;
        }
        else
        {
            TrackAabb track;
            track.id = next_track_id_++;
            track.center = dets[i].center;
            track.size = dets[i].size;
            track.hits = 1;
            track.age = 1;
            track.time_since_update = 0;
            tracks_.push_back(track);
        }
    }

    tracks_.erase(std::remove_if(tracks_.begin(), tracks_.end(),
                                 [&](const TrackAabb &t){ return t.time_since_update > track_max_age; }),
                  tracks_.end());

    visualization_msgs::msg::MarkerArray markers;
    std::unordered_set<int> active_ids;
    for(const auto &track : tracks_)
    {
        if(track.hits < track_min_hits)
        {
            continue;
        }
        if(track.time_since_update > track_max_age)
        {
            continue;
        }
        visualization_msgs::msg::Marker marker;
        marker.header = header_in;
        marker.ns = "mosa_aabb";
        marker.id = track.id;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = track.center.x();
        marker.pose.position.y = track.center.y();
        marker.pose.position.z = track.center.z();
        marker.pose.orientation.w = 1.0;
        marker.scale.x = std::max(0.01f, track.size.x());
        marker.scale.y = std::max(0.01f, track.size.y());
        marker.scale.z = std::max(0.01f, track.size.z());
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 0.5f;
        markers.markers.push_back(marker);
        active_ids.insert(track.id);
    }

    for(int prev_id : last_pub_ids_)
    {
        if(active_ids.count(prev_id))
        {
            continue;
        }
        visualization_msgs::msg::Marker del_marker;
        del_marker.header = header_in;
        del_marker.ns = "mosa_aabb";
        del_marker.id = prev_id;
        del_marker.action = visualization_msgs::msg::Marker::DELETE;
        markers.markers.push_back(del_marker);
    }
    last_pub_ids_ = std::move(active_ids);
    pub_aabb_->publish(markers);
}

void DynObjCluster::oobb_estimate(const VoxelMap &vmap, const pcl::PointCloud<PointType> &points, Eigen::Vector3f &min_point_obj,
                                  Eigen::Vector3f &max_point_obj, Eigen::Matrix3f &R, const Eigen::Vector3f ground_norm)
{
    int NMATCH = 5;
    int n = 3; // number of rings
    EA_disk disk(n);
    std::vector<std::vector<Eigen::Vector4f>> NormVectorMap(disk.size);
    std::vector<std::vector<int>> PointSizeList(disk.size);
    for (int i = 0; i < vmap.size(); i++)
    {
        if (!vmap[i].empty() && vmap[i].points.size() >= NMATCH)
        {
            Eigen::Vector4f plane;

            if (esti_plane(plane, vmap[i]))
            {
                plane.head(3) = plane.head(3).normalized();
                if (plane[2] < 0)
                {
                    plane.head(3) *= -1;
                }
                Eigen::Vector2f sphere_coor;
                disk.CatesianToSphere(plane.head(3), sphere_coor);
                Eigen::Vector2f disk_coor;
                disk.SphereToDisk(sphere_coor, disk_coor);
                int index = disk.index_find(disk_coor);
                if (index > pow(2 * (n - 1) + 1, 2) + 4 * n)
                {
                    index = index - 4 * n;
                    plane.head(3) *= -1;
                }
                NormVectorMap[index].push_back(plane);
                PointSizeList[index].push_back(vmap[i].size());
            }
        }
    }
    int max_ind = 0, sec_ind = 0;
    float max_award = 0.0, sec_award = 0.0;
    for (int i = 0; i < NormVectorMap.size(); i++)
    {
        if (!NormVectorMap[i].empty())
        {
            float award = 0.0;
            for (int ite = 0; ite < NormVectorMap[i].size(); ite++)
            {
                award += std::sqrt(PointSizeList[i][ite]) / NormVectorMap[i][ite](3);
            }
            if (award > max_award)
            {
                sec_award = max_award;
                sec_ind = max_ind;
                max_award = award;
                max_ind = i;
            }
            else if (award > sec_award)
            {
                sec_award = award;
                sec_ind = i;
            }
        }
    }
    Eigen::Vector3f direction_main(0.0f, 0.0f, 0.0f);
    Eigen::Vector3f direction_aux(0.0f, 0.0f, 0.0f);
    if (max_award > 0)
    {
        for (int ite = 0; ite < NormVectorMap[max_ind].size(); ite++)
        {
            direction_main = direction_main + NormVectorMap[max_ind][ite].head(3) * PointSizeList[max_ind][ite] / NormVectorMap[max_ind][ite](3);
        }
        direction_main.normalize();
    }
    else
        direction_main << 0.0, 0.0, 1.0;
    if (sec_award > 0)
    {
        for (int ite = 0; ite < NormVectorMap[sec_ind].size(); ite++)
        {
            direction_aux = direction_aux + NormVectorMap[sec_ind][ite].head(3) * PointSizeList[sec_ind][ite] / NormVectorMap[sec_ind][ite](3);
        }
        direction_aux.normalize();
    }
    else
        direction_aux << 1.0, 0.0, 0.0;

    if (ground_norm.norm() < 0.1)
    {
        R.col(0) = direction_main;
        R.col(1) = (direction_aux - direction_aux.dot(R.col(0)) * R.col(0)).normalized();
        Eigen::Vector3f world_z(0.0, 0.0, 1.0);
        if (abs(R.col(1).dot(world_z)) > 0.866f)
        {
            R.col(2) = R.col(1);
            R.col(1) = -(R.col(0).cross(R.col(2))).normalized();
        }
        else if (abs(R.col(0).dot(world_z)) > 0.866f)
        {
            R.col(1).swap(R.col(0));
            R.col(2) = R.col(1);
            R.col(1) = -(R.col(0).cross(R.col(2))).normalized();
        }
        else
        {
            R.col(2) = (R.col(0).cross(R.col(1))).normalized();
        }
    }
    else
    {
        R.col(0) = ground_norm;
        if (ground_norm.dot(direction_main) > 0.95)
        {
            direction_main = direction_aux;
        }
        R.col(1) = (direction_main - direction_main.dot(R.col(0)) * R.col(0)).normalized();
        R.col(2) = (R.col(0).cross(R.col(1))).normalized();
    }

    Eigen::Vector3f point_vec(points[0].x, points[0].y, points[0].z);
    Eigen::Vector3f project = (point_vec.transpose() * R).transpose();
    float x_min = project[0], x_max = project[0];
    float y_min = project[1], y_max = project[1];
    float z_min = project[2], z_max = project[2];
    for (int pit = 0; pit < points.size(); pit++)
    {
        point_vec << points[pit].x, points[pit].y, points[pit].z;
        project = (point_vec.transpose() * R).transpose();
        if (project[0] < x_min)
            x_min = project[0];
        if (project[1] < y_min)
            y_min = project[1];
        if (project[2] < z_min)
            z_min = project[2];
        if (project[0] > x_max)
            x_max = project[0];
        if (project[1] > y_max)
            y_max = project[1];
        if (project[2] > z_max)
            z_max = project[2];
    }
    max_point_obj << x_max, y_max, z_max;
    min_point_obj << x_min, y_min, z_min;
}

void DynObjCluster::event_extend(const Eigen::Matrix3f &R, bool ground_detect,
                                 bbox_t &bbox, std::vector<int> &dyn_tag_one, const int &bbox_index)
{
    for (int i = 0; i < bbox.Ground_voxels_vec[bbox_index].size(); i++)
    {
        int voxel_cur = bbox.Ground_voxels_vec[bbox_index][i];
        if (bbox.Ground_voxels_set[bbox_index].count(voxel_cur) && umap_ground[voxel_cur].points_num > 2)
        {
            int x_ind[6] = {1, -1, 0, 0, 0, 0};
            int y_ind[6] = {0, 0, 1, -1, 0, 0};
            int z_ind[6] = {0, 0, 0, 0, 1, -1};
            for (int ind = 0; ind < 6; ind++)
            {
                int voxel_neighbor = voxel_cur + x_ind[ind] * GridMapedgesize_xy * GridMapedgesize_z + y_ind[ind] * GridMapedgesize_z + z_ind[ind];
                if (voxel_neighbor < 0 || voxel_neighbor > GridMapsize)
                    continue;
                if ((umap_insidebox[voxel_neighbor].bbox_index > -1 && umap_insidebox[voxel_neighbor].points_num > 0 && umap_insidebox[voxel_neighbor].bbox_index == bbox_index) || (umap[voxel_neighbor].points_num > 0 && umap[voxel_neighbor].cloud->size() > 0 && umap[voxel_neighbor].bbox_index == bbox_index))
                {
                    pcl::PointCloud<PointType> points = *(umap_ground[voxel_cur].cloud);
                    Eigen::Vector4f plane;
                    if (ground_detect)
                    {
                        if (esti_plane(plane, points) && abs(R.col(0).dot(plane.head(3).normalized())) < 0.8f)
                        {
                            umap[voxel_cur].bbox_index = bbox_index;
                            umap[voxel_cur].cloud = umap_ground[voxel_cur].cloud;
                            bbox.Point_cloud[bbox_index] += points;
                            for (int j = 0; j < umap_ground[voxel_cur].cloud_index->size(); j++)
                            {
                                dyn_tag_one.push_back(umap_ground[voxel_cur].cloud_index->at(j));
                                bbox.Point_indices[bbox_index].push_back(umap_ground[voxel_cur].cloud_index->at(j));
                            }
                            break;
                        }
                    }
                }
            }
        }
    }
}

bool DynObjCluster::esti_plane(Eigen::Vector4f &pca_result, const pcl::PointCloud<PointType> &point)
{
    const float threshold = 0.1;
    int point_size = point.size();
    Eigen::Matrix<float, Eigen::Dynamic, 3> A;
    Eigen::Matrix<float, Eigen::Dynamic, 1> b;
    A.resize(point_size, 3);
    b.resize(point_size, 1);
    b.setOnes();
    b *= -1.0f;
    for (int j = 0; j < point_size; j++)
    {
        A(j, 0) = point[j].x;
        A(j, 1) = point[j].y;
        A(j, 2) = point[j].z;
    }
    Eigen::Vector3f normvec = A.colPivHouseholderQr().solve(b);
    float norm = normvec.norm();
    float average_dis = 0.0;
    for (int j = 0; j < point_size; j++)
    {
        float tmp = fabs(normvec.dot(A.row(j)) + 1.0);
        average_dis += tmp;
        if (tmp > threshold)
        {
            return false;
        }
    }
    average_dis = std::max(0.01f, average_dis / point_size / norm);
    pca_result(0) = normvec(0);
    pca_result(1) = normvec(1);
    pca_result(2) = normvec(2);
    pca_result(3) = average_dis;
    return true;
}

void DynObjCluster::XYZExtract(const int &position, Eigen::Vector3f &xyz)
{
    int left = position;
    xyz(0) = xyz_origin(0) + (float)floor(position / (GridMapedgesize_xy * GridMapedgesize_z)) * Voxel_revolusion;
    left = left - (float)floor(position / (GridMapedgesize_xy * GridMapedgesize_z)) * (GridMapedgesize_xy * GridMapedgesize_z);
    xyz(1) = xyz_origin(1) + (float)floor(left / GridMapedgesize_z) * Voxel_revolusion;
    left = left - (float)floor(left / GridMapedgesize_z) * GridMapedgesize_z;
    xyz(2) = xyz_origin(2) + left * Voxel_revolusion;
}
