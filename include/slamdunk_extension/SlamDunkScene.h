/* 
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#ifndef SLAMDUNKSCENE_H
#define SLAMDUNKSCENE_H

#include <slamdunk/slam_dunk.h>
#include <string>
#include <map>
#include <vector>

//PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl/features/integral_image_normal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

//Boost
#include <boost/thread/thread.hpp>

namespace slamdunk
{

typedef pcl::PointXYZRGB SlamDunkCloudType;
typedef pcl::Normal SlamDunkNormalType;

/**
     * Pose Entry struct representing a pose status in times 
     */
typedef struct PoseEntry
{
    PoseEntry *next;
    Eigen::Isometry3d pose;
    int index;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PoseEntry()
    {
        next = NULL;
        index = -1;
    }

    void del()
    {
        if (next != NULL)
        {
            next->del();
            delete next;
            next = NULL;
        }
    }

    PoseEntry *getLast()
    {
        if (next != NULL)
        {
            return next->getLast();
        }
        else
        {
            return this;
        }
    }

    void append(PoseEntry *pose)
    {
        this->next = pose;
        this->next->index = this->index + 1;
    }
} PoseEntry;

/**
     * Manages a collection of PoseEntry, keeping first/last and lastUpdated one
     */
typedef struct PoseHistory
{
    PoseEntry *first;
    PoseEntry *last;
    PoseEntry *lastUpdated;
    int size;

    PoseHistory()
    {
        first = last = lastUpdated = NULL;
        size = 0;
    }

    int getSize()
    {
        return size;
    }

    void appendPose(PoseEntry *pose)
    {
        if (first == NULL)
        {
            first = pose;
            first->index = 0;
        }
        else
        {
            first->getLast()->append(pose);
        }
        last = pose;
        size++;
    }

    void appendPose(Eigen::Isometry3d &epose)
    {
        PoseEntry *pose = new PoseEntry();
        pose->pose = epose;
        appendPose(pose);
    }

    bool getPoseByIndex(int index, PoseEntry *&pose)
    {
        pose = first;
        while (pose->index != index || pose == NULL)
        {
            pose = pose->next;
        }
        return pose != NULL;
    }

    bool markAsUpdatedByIndex(int index)
    {
        PoseEntry *pose;
        if (getPoseByIndex(index, pose))
        {
            lastUpdated = pose;
            return true;
        }
        return false;
    }

    bool markLastAsUpdated()
    {
        if (last != NULL)
        {
            lastUpdated = last;
            return true;
        }
        return false;
    }

    PoseEntry *getLastUpdated()
    {
        return lastUpdated;
    }

    void del(PoseEntry *&p)
    {
        if (p != NULL)
        {
            p->del();
            delete p;
            p = NULL;
        }
    }

    bool neverUpdated()
    {
        return getLastUpdated() == NULL;
    }

    bool isFine()
    {
        return getLastUpdated() == last;
    }

    void del()
    {
        del(first);
        del(last);
        del(lastUpdated);
    }

} PoseHistory;

/**
     * Entry used to integrate/deintegrate frame data
     */
typedef struct ConsumingEntry
{
    std::string key;
    Eigen::Isometry3d old_pose;
    Eigen::Isometry3d new_pose;
    bool replacement;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /**
         * New Entry
         */
    ConsumingEntry(std::string key, Eigen::Isometry3d &new_pose)
    {
        this->key = key;
        this->new_pose = new_pose;
        this->replacement = false;
    }

    /**
         * Replace Entry
         */
    ConsumingEntry(std::string key, Eigen::Isometry3d &old_pose, Eigen::Isometry3d &new_pose)
    {
        this->key = key;
        this->old_pose = old_pose;
        this->new_pose = new_pose;
        this->replacement = true;
    }
} ConsumningEntry;

/**
     *
     */
typedef struct CloudSegmentation
{
    std::string name;
    pcl::PointCloud<SlamDunkCloudType>::Ptr input_cloud;
    pcl::PointCloud<SlamDunkCloudType>::Ptr plane_cloud;
    pcl::PointCloud<SlamDunkCloudType>::Ptr rest_cloud;
    pcl::PointIndices plane_indices;
    pcl::PointIndices rest_indices;

    Eigen::Vector3d plane_normal;
    pcl::ModelCoefficients plane_coefficients;
    bool valid;

    CloudSegmentation()
    {
        input_cloud = pcl::PointCloud<SlamDunkCloudType>::Ptr(new pcl::PointCloud<SlamDunkCloudType>);
        plane_cloud = pcl::PointCloud<SlamDunkCloudType>::Ptr(new pcl::PointCloud<SlamDunkCloudType>);
        rest_cloud = pcl::PointCloud<SlamDunkCloudType>::Ptr(new pcl::PointCloud<SlamDunkCloudType>);
        this->valid = false;
    }

    CloudSegmentation(pcl::PointCloud<SlamDunkCloudType>::Ptr &input_cloud)
    {
        this->input_cloud = input_cloud;
        plane_cloud = pcl::PointCloud<SlamDunkCloudType>::Ptr(new pcl::PointCloud<SlamDunkCloudType>);
        rest_cloud = pcl::PointCloud<SlamDunkCloudType>::Ptr(new pcl::PointCloud<SlamDunkCloudType>);
    }

    void computePlaneOrientation()
    {
        plane_normal = Eigen::Vector3d(
            plane_coefficients.values[0],
            plane_coefficients.values[1],
            plane_coefficients.values[2]);
    }

    void computePlaneRF(Eigen::Matrix4d &rf)
    {
        Eigen::Vector3d orth_x;
        Eigen::Vector3d orth_y;
        Eigen::Vector3d first_plane_normal;
        Eigen::Vector4d centroid;

        pcl::compute3DCentroid(*(input_cloud), plane_indices.indices, centroid);
        first_plane_normal = plane_normal;

        orth_x = Eigen::Vector3d(1.0, 1.0, (-first_plane_normal[0] - first_plane_normal[1]) / first_plane_normal[2]);
        orth_x.normalize();
        orth_y = first_plane_normal.cross(orth_x);
        orth_y.normalize();

        rf = Eigen::Matrix4d::Identity();
        rf(0, 2) = first_plane_normal[0];
        rf(1, 2) = first_plane_normal[1];
        rf(2, 2) = first_plane_normal[2];

        rf(0, 3) = centroid[0];
        rf(1, 3) = centroid[1];
        rf(2, 3) = centroid[2];

        rf(0, 0) = orth_x[0];
        rf(1, 0) = orth_x[1];
        rf(2, 0) = orth_x[2];

        rf(0, 1) = orth_y[0];
        rf(1, 1) = orth_y[1];
        rf(2, 1) = orth_y[2];
    }

} CloudSegmentation;

typedef struct CloudFilteringParameters
{
    bool filter_z;
    double z_pass[2];
    bool radius_outlier;
    double radius_outlier_radius;
    int radius_outlier_min_inliers;

    CloudFilteringParameters()
    {
        filter_z = false;
        z_pass[0] = 0.0;
        z_pass[1] = 0.0;
        radius_outlier = false;
        radius_outlier_radius = 0.0;
        radius_outlier_min_inliers = 0;
    }
} CloudFilteringParameters;

/**
     * 
     */
class SlamDunkScene
{
  public:
    SlamDunkScene();
    SlamDunkScene(boost::shared_ptr<slamdunk::SlamDunk> slamdunk);
    virtual ~SlamDunkScene();

    static std::string getKeyframeName(double timestamp);
    static pcl::PointCloud<SlamDunkCloudType>::Ptr createCloudFromFrame(RGBDFrame &frame, double fx = 525., double fy = 525, double cx = 320, double cy = 240, int sample_jumps = 1, CloudFilteringParameters parameters = CloudFilteringParameters());
    static pcl::PointCloud<SlamDunkNormalType>::Ptr computeNormals(pcl::PointCloud<SlamDunkCloudType>::Ptr &cloud);
    std::map<std::string, pcl::PointCloud<SlamDunkCloudType>::Ptr> &getClouds();
    std::map<std::string, RGBDFrame> &getRGBDFrames();
    std::map<std::string, Eigen::Isometry3d, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Isometry3d>>> &getPoses();
    void addCloud(std::string &key, pcl::PointCloud<SlamDunkCloudType>::Ptr &cloud);
    bool existsCloud(std::string &key);
    void addRGBDFrame(std::string &key, RGBDFrame &frame);
    bool existsRGBDFrame(std::string &key);
    void addPose(std::string &key, Eigen::Isometry3d &pose, bool keyframe = false);
    void addPoseToHistory(std::string &key, Eigen::Isometry3d &pose);
    bool existsPose(std::string &key);
    bool existsPoseHistory(std::string &key);
    void getPoseInMapFrame(std::string &key, Eigen::Isometry3d &pose);
    void getGenericPoseInMapFrame(Eigen::Matrix4d &relative_pose, Eigen::Matrix4d &pose);

    //Plane segmentation
    bool planesExtraction(pcl::PointCloud<SlamDunkCloudType>::Ptr &cloud, CloudSegmentation &segmentation);
    void computeSegmentation(std::string &key, double angle_th = 30.0 * M_PI / 180.0, double height_th = 0.3);
    void computeFirstSegmentation(std::string &key);

    //History management
    void setSlamDunkHandle(boost::shared_ptr<slamdunk::SlamDunk> slamdunk);
    void updatePoseHistory(std::string &key, Eigen::Isometry3d &pose, bool update_optimized_frames = true);
    void getAvailableEntries(std::vector<ConsumingEntry> &entries, int max_entries = 10, bool include_newentries_in_counter = false);
    void getAvailableEntries(std::vector<ConsumingEntry> &new_entries, std::vector<ConsumingEntry> &replace_entries, int max_new_entries = 5, int max_replace_entries = 5);

    bool addHierarchy(std::string &key, std::string &key_2);
    bool existHierarchy(std::string &key);

    std::map<std::string, pcl::PointCloud<SlamDunkCloudType>::Ptr> clouds;
    std::map<std::string, RGBDFrame> rgbd_frames;
    std::map<std::string, Eigen::Isometry3d, std::less<std::string>, Eigen::aligned_allocator<std::pair<const std::string, Eigen::Isometry3d>>> poses;
    std::map<std::string, std::vector<std::string>> hierarchy;
    std::map<std::string, bool> keyframes_mask;
    std::map<std::string, PoseHistory *> keyframes_pose_history;
    std::map<std::string, CloudSegmentation> segmentations;
    std::map<std::string, pcl::PointCloud<SlamDunkCloudType>::Ptr> ground_slices;
    std::map<std::string, pcl::PointCloud<SlamDunkCloudType>::Ptr> obstacles;
    std::map<std::string, Eigen::Matrix4d> rfs;

    std::string last_keyframe_key;
    Eigen::Isometry3d last_keyframe_pose;
    Eigen::Isometry3d first_camera_pose;
    Eigen::Isometry3d first_camera_in_map;

    bool isEmpty();
    int size();

    bool relativeDeltaPose(std::string key_parent, std::string key_child, Eigen::Isometry3d &delta_pose);
    void initSegmenter(int min_inliers = 10, int max_iterations = 100, float distance_th = 0.03, bool optimize_coefficient = true);

    //

    void addNewEntryCallaback(std::function<void(std::vector<slamdunk::ConsumingEntry> &)> callback);
    void addOptimezedEntryCallabck(std::function<void(std::vector<slamdunk::ConsumingEntry> &)> callback);
    void spinIntegrationUpdate();

    std::vector<std::function<void(std::vector<slamdunk::ConsumingEntry> &)>> callbacks_new_entry;
    std::vector<std::function<void(std::vector<slamdunk::ConsumingEntry> &)>> callbacks_optimized_entry;

    int max_spin_integration_steps;
    int max_spin_deintegration_steps;

  protected:
    boost::mutex history_lock;
    boost::shared_ptr<slamdunk::SlamDunk> _slamdunk;
    pcl::SACSegmentation<SlamDunkCloudType> _segmenter;
    int _segment_min_inliers;
    std::string first_segmentation_keyframe;
    void initParams();
};

typedef std::map<std::string, PoseHistory *>::iterator PoseHistoryIterator;
}
#endif /* SLAMDUNKSCENE_H */
