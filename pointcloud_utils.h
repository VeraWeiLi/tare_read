#pragma once



//
// Created by caochao on 06/03/20.
//
#pragma once

// PCL
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>

#include <utils/misc_utils.h>

namespace pointcloud_utils_ns
{
    class VerticalSurfaceExtractor;
    template <typename PCLPointType>
    class PointCloudDownsizer;
    template <typename PCLPointType>
    struct PCLCloud;
}  // namespace pointcloud_utils_ns

class pointcloud_utils_ns::VerticalSurfaceExtractor//没想清提取出来的结果到底是啥
{
private:
    double kRadiusThreshold;
    double kZDiffMax;
    double kZDiffMin;
    int kNeighborThreshold;
    pcl::PointCloud<pcl::PointXYZI>::Ptr extractor_cloud_;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr extractor_kdtree_;

public:
    explicit VerticalSurfaceExtractor();
    ~VerticalSurfaceExtractor() = default;
    void SetRadiusThreshold(double radius_threshold)
    {
        kRadiusThreshold = radius_threshold;
    }
    void SetZDiffMax(double z_diff_max)
    {
        kZDiffMax = z_diff_max;
    }
    void SetZDiffMin(double z_diff_min)
    {
        kZDiffMin = z_diff_min;
    }
    void SetNeighborThreshold(int neighbor_threshold)
    {
        kNeighborThreshold = neighbor_threshold;
    }
    template <class PCLPointType>
    void ExtractVerticalSurface(typename pcl::PointCloud<PCLPointType>::Ptr& cloud, double z_max = DBL_MAX,
        double z_min = -DBL_MAX)
    {
        if (cloud->points.empty())
        {
            return;
        }
        pcl::copyPointCloud(*cloud, *extractor_cloud_);
        for (auto& point : extractor_cloud_->points)//对于extractor_cloud_中的points向量的每一个值都执行一遍操作
        {
            point.intensity = point.z;
            point.z = 0.0;//通过将z坐标设为0，实际上是将点云中的点都放到了一个边界面上来操作（相当于投影，空间中各个位置的点投影到底面上）
        }
        extractor_kdtree_->setInputCloud(extractor_cloud_);//kdtree中输入的点云是经过处理后的点云面
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        std::vector<int> neighbor_indices;
        std::vector<float> neighbor_sqdist;
        for (int i = 0; i < extractor_cloud_->points.size(); i++)
        {
            pcl::PointXYZI point = extractor_cloud_->points[i];
            if (point.intensity > z_max || point.intensity < z_min)
                continue;//跳到下一次循环
            extractor_kdtree_->radiusSearch(point, kRadiusThreshold, neighbor_indices, neighbor_sqdist);//创建两个向量 存储搜索到的近邻 一个存点的索引 一个存点的距离平方
            //第i次循环目标点point是点云中的第i个点，邻居点的索引是按照距离从近到远存放，假如离point最近的点的编号为7，距离为2，那么索引向量的第一个元素的值为7，即
            //neighbor_indices[0]=7，neighbor_sqdist[0]=2
            // kRadiusThreshold是平面点云的半径范围，相当于原始点云在xoy平面的投影上要满足的距离范围
            bool is_vertical = false;
            int neighbor_count = 0;
            for (const auto& idx : neighbor_indices)
            {
                double z_diff = std::abs(point.intensity - extractor_cloud_->points[idx].intensity);//z方向的差距为当前目标点的原z坐标（现i坐标）与找到的邻居点的原z坐标之差
                if (z_diff > kZDiffMin && z_diff < kZDiffMax)//原始点云在z方向上应该满足的距离范围，只有同时也满足z方向上的距离条件的点才会被认为是真正的邻居点
                {
                    neighbor_count++;
                    if (neighbor_count >= kNeighborThreshold)//只有满足条件（投到xoy平面上时离很近，z方向也在条件范围内）的邻居点到达足够量的时候才可以认为该点是vertical
                    {
                        is_vertical = true;
                        break;
                    }
                }
            }
            if (is_vertical)//如果第i个点的满足指标的邻居点数量足够多，就会把i值放入索引向量中
            {
                inliers->indices.push_back(i);
            }
        }
        //以下功能是将下标在inliers中的cloud的point提取出来，再存到cloud中去
        pcl::ExtractIndices<PCLPointType> extract;//按照索引提取点
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);//如果设为true,可以提取指定index之外的点云
        extract.filter(*cloud);
    }

    template <class InputPCLPointType, class OutputPCLPointType>
    void ExtractVerticalSurface(typename pcl::PointCloud<InputPCLPointType>::Ptr& cloud_in,
        typename pcl::PointCloud<OutputPCLPointType>::Ptr& cloud_out, double z_max = DBL_MAX,
        double z_min = -DBL_MAX)
    {
        if (cloud_in->points.empty())
        {
            return;
        }
        pcl::copyPointCloud(*cloud_in, *extractor_cloud_);
        for (auto& point : extractor_cloud_->points)
        {
            point.intensity = point.z;
            point.z = 0.0;
        }
        extractor_kdtree_->setInputCloud(extractor_cloud_);
        cloud_out->clear();
        std::vector<int> neighbor_indices;
        std::vector<float> neighbor_sqdist;
        for (int i = 0; i < extractor_cloud_->points.size(); i++)
        {
            pcl::PointXYZI point = extractor_cloud_->points[i];
            if (point.intensity > z_max || point.intensity < z_min)
                continue;
            extractor_kdtree_->radiusSearch(point, kRadiusThreshold, neighbor_indices, neighbor_sqdist);
            bool is_vertical = false;
            int neighbor_count = 0;
            for (const auto& idx : neighbor_indices)
            {
                double z_diff = std::abs(point.intensity - extractor_cloud_->points[idx].intensity);
                if (z_diff > kZDiffMin && z_diff < kZDiffMax)
                {
                    neighbor_count++;
                    if (neighbor_count >= kNeighborThreshold)
                    {
                        is_vertical = true;
                        break;
                    }
                }
            }
            if (is_vertical)
            {
                OutputPCLPointType point_out;
                point_out.x = cloud_in->points[i].x;
                point_out.y = cloud_in->points[i].y;
                point_out.z = cloud_in->points[i].z;
                cloud_out->points.push_back(point_out);
            }
        }
    }
};

template <typename PCLPointType>
class pointcloud_utils_ns::PointCloudDownsizer
{
private:
    pcl::VoxelGrid<PCLPointType> pointcloud_downsize_filter_;

public:
    explicit PointCloudDownsizer()
    {
    }
    ~PointCloudDownsizer() = default;
    void Downsize(typename pcl::PointCloud<PCLPointType>::Ptr& cloud, double leaf_size_x, double leaf_size_y,
        double leaf_size_z)
    {
        if (cloud->points.empty())
        {
            return;
        }
        pointcloud_downsize_filter_.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
        pointcloud_downsize_filter_.setInputCloud(cloud);
        pointcloud_downsize_filter_.filter(*cloud);
    }
};

template <typename PCLPointType>
struct pointcloud_utils_ns::PCLCloud
{
    std::string pub_cloud_topic_;
    std::string frame_id_;
    typename pcl::PointCloud<PCLPointType>::Ptr cloud_;
    ros::Publisher cloud_pub_;
    PCLCloud(ros::NodeHandle* nh, std::string pub_cloud_topic, std::string frame_id)
        : pub_cloud_topic_(pub_cloud_topic), frame_id_(frame_id)
    {
        cloud_ = typename pcl::PointCloud<PCLPointType>::Ptr(new pcl::PointCloud<PCLPointType>);
        cloud_pub_ = nh->advertise<sensor_msgs::PointCloud2>(pub_cloud_topic_, 2);
    }
    PCLCloud(ros::NodeHandle& nh, std::string pub_cloud_topic, std::string frame_id)
        : pub_cloud_topic_(pub_cloud_topic), frame_id_(frame_id)
    {
        cloud_ = typename pcl::PointCloud<PCLPointType>::Ptr(new pcl::PointCloud<PCLPointType>);
        cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(pub_cloud_topic_, 2);
    }
    ~PCLCloud() = default;
    void Publish()
    {
        misc_utils_ns::PublishCloud<pcl::PointCloud<PCLPointType>>(cloud_pub_, *cloud_, frame_id_);//向pub_cloud_topic_发布sensor_msgs::PointCloud2类型的消息，消息内容是
                                                                                                   //从pcl点形式转成msgs形式的*cloud_，其中消息的header.frame_id是frame_id_
    }
    typedef std::shared_ptr<PCLCloud<PCLPointType>> Ptr;
};
