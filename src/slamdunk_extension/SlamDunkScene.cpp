/* 
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#include <slamdunk_extension/SlamDunkScene.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>

namespace slamdunk
{

SlamDunkScene::SlamDunkScene()
{
    this->_slamdunk = NULL;
    this->initSegmenter();
    this->initParams();
}

/////////////////////////////////////////////////////////////////////////////////////////////////

SlamDunkScene::SlamDunkScene(boost::shared_ptr<slamdunk::SlamDunk> slamdunk)
{
    this->_slamdunk = slamdunk;
    this->initSegmenter();
    this->initParams();
}

/////////////////////////////////////////////////////////////////////////////////////////////////

SlamDunkScene::~SlamDunkScene()
{
}

/////////////////////////////////////////////////////////////////////////////////////////////////

void SlamDunkScene::initParams()
{
    this->max_spin_integration_steps = 1;
    this->max_spin_deintegration_steps = 1;
}

/////////////////////////////////////////////////////////////////////////////////////////////////

void SlamDunkScene::setSlamDunkHandle(boost::shared_ptr<slamdunk::SlamDunk> slamdunk)
{
    this->_slamdunk = slamdunk;
}

/////////////////////////////////////////////////////////////////////////////////////////////////

std::string SlamDunkScene::getKeyframeName(double timestamp)
{
    std::stringstream ss;
    ss << "c" << std::fixed << std::setprecision(6) << timestamp;
    return ss.str();
}
/////////////////////////////////////////////////////////////////////////////////////////////////

pcl::PointCloud<SlamDunkCloudType>::Ptr SlamDunkScene::createCloudFromFrame(RGBDFrame &frame, double fx, double fy, double cx, double cy, int sample_jumps, CloudFilteringParameters parameters)
{
    sample_jumps = sample_jumps > 1 ? sample_jumps : 1;

    pcl::PointCloud<SlamDunkCloudType>::Ptr cloud(new pcl::PointCloud<SlamDunkCloudType>);
    cloud->width = frame.m_color_image.cols / sample_jumps;
    cloud->height = frame.m_color_image.rows / sample_jumps;
    cv::Mat rgb = frame.m_color_image;
    cv::Mat depth = frame.m_depth_image;
    cv::Vec4b val;
    float d;
    const float bad_point = std::numeric_limits<float>::quiet_NaN();
    for (float y = 0; y < depth.rows; y += sample_jumps)
    {
        for (float x = 0; x < depth.cols; x += sample_jumps)
        {
            d = depth.at<float>(y, x);
            SlamDunkCloudType p;
            p.x = (d / fx) * (x - cx);
            p.y = (d / fy) * (y - cy);
            p.z = d;
            val = rgb.at<cv::Vec4b>(y, x);
            p.r = val[2];
            p.g = val[1];
            p.b = val[0];
            cloud->points.push_back(p);
        }
    }

    if (parameters.filter_z)
    {
        pcl::PointCloud<SlamDunkCloudType>::Ptr cloud_z(new pcl::PointCloud<SlamDunkCloudType>);
        pcl::PassThrough<SlamDunkCloudType> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(parameters.z_pass[0], parameters.z_pass[1]);
        pass.filter(*cloud_z);
        cloud = cloud_z;
    }

    if (parameters.radius_outlier)
    {
        pcl::PointCloud<SlamDunkCloudType>::Ptr cloud_out(new pcl::PointCloud<SlamDunkCloudType>);
        pcl::RadiusOutlierRemoval<SlamDunkCloudType> outrem;
        outrem.setInputCloud(cloud);
        outrem.setRadiusSearch(parameters.radius_outlier_radius);
        outrem.setMinNeighborsInRadius(parameters.radius_outlier_min_inliers);
        outrem.filter(*cloud_out);
        cloud = cloud_out;
    }

    return cloud;
}
/////////////////////////////////////////////////////////////////////////////////////////////////

pcl::PointCloud<SlamDunkNormalType>::Ptr SlamDunkScene::computeNormals(pcl::PointCloud<SlamDunkCloudType>::Ptr &cloud)
{
    pcl::IntegralImageNormalEstimation<SlamDunkCloudType, SlamDunkNormalType> ne;
    ne.setNormalEstimationMethod(ne.AVERAGE_3D_GRADIENT);
    ne.setMaxDepthChangeFactor(0.02f);
    ne.setNormalSmoothingSize(10.0f);
    ne.setInputCloud(cloud);
    pcl::PointCloud<SlamDunkNormalType>::Ptr normals(new pcl::PointCloud<SlamDunkNormalType>);
    ne.compute(*normals);
    return normals;
}
/////////////////////////////////////////////////////////////////////////////////////////////////

bool SlamDunkScene::existsCloud(std::string &key)
{
    return clouds.find(key) != clouds.end();
}
/////////////////////////////////////////////////////////////////////////////////////////////////

bool SlamDunkScene::existsPose(std::string &key)
{
    return poses.find(key) != poses.end();
}
/////////////////////////////////////////////////////////////////////////////////////////////////

bool SlamDunkScene::existsPoseHistory(std::string &key)
{
    return keyframes_pose_history.find(key) != keyframes_pose_history.end();
}
/////////////////////////////////////////////////////////////////////////////////////////////////

bool SlamDunkScene::existsRGBDFrame(std::string &key)
{
    return rgbd_frames.find(key) != rgbd_frames.end();
}
/////////////////////////////////////////////////////////////////////////////////////////////////

void SlamDunkScene::addCloud(std::string &key, pcl::PointCloud<SlamDunkCloudType>::Ptr &cloud)
{
    clouds[key] = cloud;
}
/////////////////////////////////////////////////////////////////////////////////////////////////

void SlamDunkScene::addPose(std::string &key, Eigen::Isometry3d &pose, bool keyframe)
{
    if (poses.size() <= 0)
    {
        first_camera_pose = pose;
    }
    poses[key] = pose;

    if (keyframe)
    {
        last_keyframe_key = key;
        last_keyframe_pose = pose;
        keyframes_mask[key] = true;
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////

void SlamDunkScene::addPoseToHistory(std::string &key, Eigen::Isometry3d &pose)
{
    //pose history
    if (!existsPoseHistory(key))
    {
        keyframes_pose_history[key] = new PoseHistory();
    }
    keyframes_pose_history[key]->appendPose(pose);
}
/////////////////////////////////////////////////////////////////////////////////////////////////

void SlamDunkScene::addRGBDFrame(std::string &key, RGBDFrame &frame)
{
    rgbd_frames[key] = frame;
}
/////////////////////////////////////////////////////////////////////////////////////////////////

bool SlamDunkScene::addHierarchy(std::string &key, std::string &key_2)
{
    if (existHierarchy(key))
    {
        hierarchy[key].push_back(key_2);
    }
    else
    {
        hierarchy[key] = std::vector<std::string>();
        hierarchy[key].push_back(key_2);
    }
}
/////////////////////////////////////////////////////////////////////////////////////////////////

bool SlamDunkScene::existHierarchy(std::string &key)
{
    return hierarchy.find(key) != hierarchy.end();
}
/////////////////////////////////////////////////////////////////////////////////////////////////

bool SlamDunkScene::isEmpty()
{
    return poses.size() <= 0;
}
/////////////////////////////////////////////////////////////////////////////////////////////////

int SlamDunkScene::size()
{
    return poses.size();
}
/////////////////////////////////////////////////////////////////////////////////////////////////

bool SlamDunkScene::relativeDeltaPose(std::string key_parent, std::string key_child, Eigen::Isometry3d &delta_pose)
{
    if (existsPose(key_parent) && existsPose(key_child))
    {
        Eigen::Isometry3d pose_parent = poses[key_parent];
        Eigen::Isometry3d pose_child = poses[key_child];
        pose_parent = pose_parent.inverse();
        delta_pose = pose_parent * pose_child;
        return true;
    }
    return false;
}

/////////////////////////////////////////////////////////////////////////////////////////////////

void SlamDunkScene::updatePoseHistory(std::string &key, Eigen::Isometry3d &pose, bool update_optimized_frames)
{

    //Refactoring
    //        assert(_slamdunk);

    boost::mutex::scoped_lock lock(history_lock);

    //Add keyframe to pose history
    addPoseToHistory(key, pose);

    //Add corrected keyframes to pose history
    if (update_optimized_frames)
    {
        slamdunk::CameraTracker::StampedPoseVector poses;
        poses = _slamdunk->getMovedFrames();
        std::string key;
        for (int i = 0; i < poses.size(); i++)
        {
            key = getKeyframeName(poses[i].first);
            addPoseToHistory(key, poses[i].second);
        }
    }

    lock.unlock();
}

/////////////////////////////////////////////////////////////////////////////////////////////////

void SlamDunkScene::getAvailableEntries(std::vector<ConsumingEntry> &entries, int max_entries, bool include_newentries_in_counter)
{
    //Refactoring
    assert(_slamdunk);

    boost::mutex::scoped_lock lock(history_lock);
    entries.clear();

    int opt_counter = 0;
    for (slamdunk::PoseHistoryIterator it = keyframes_pose_history.begin();
         it != keyframes_pose_history.end(); ++it)
    {
        PoseHistory *history = it->second;
        std::string key = it->first;

        if (history->neverUpdated())
        {

            entries.push_back(ConsumingEntry(key, history->last->pose));
            history->markLastAsUpdated();
            if (include_newentries_in_counter)
            {
                opt_counter++;
            }
        }
        else
        {
            if (history->isFine())
            {
                //Nothing to do
            }
            else
            {
                entries.push_back(ConsumingEntry(key, history->lastUpdated->pose, history->last->pose));
                history->markLastAsUpdated();
                opt_counter++;
            }
        }
        //Max reached
        if (opt_counter >= max_entries)
            break;
    }

    lock.unlock();
}

/////////////////////////////////////////////////////////////////////////////////////////////////

void SlamDunkScene::getAvailableEntries(std::vector<ConsumingEntry> &new_entries, std::vector<ConsumingEntry> &replace_entries, int max_new_entries, int max_replace_entries)
{

    assert(_slamdunk);

    new_entries.clear();
    replace_entries.clear();
    boost::mutex::scoped_lock lock(history_lock);

    std::vector<std::string> keys;
    for (slamdunk::PoseHistoryIterator it = keyframes_pose_history.begin();
         it != keyframes_pose_history.end(); ++it)
    {
        keys.push_back(it->first);
    }

    sort(keys.begin(), keys.end(), std::greater<std::string>());

    int new_counter = 0;
    int replace_counter = 0;
    for (int i = 0; i < keys.size(); i++)
    {
        std::string key = keys[i];
        PoseHistory *history = keyframes_pose_history[key];

        if (history->neverUpdated())
        {
            if (new_counter < max_new_entries)
            {
                new_counter++;
                new_entries.push_back(ConsumingEntry(key, history->last->pose));
                history->markLastAsUpdated();
            }
        }
        else
        {
            if (history->isFine())
            {
                //Nothing to do
            }
            else
            {
                if (replace_counter < max_replace_entries)
                {
                    replace_counter++;
                    replace_entries.push_back(ConsumingEntry(key, history->lastUpdated->pose, history->last->pose));
                    history->markLastAsUpdated();
                }
            }
        }
        if (new_counter >= max_new_entries && replace_counter >= max_replace_entries)
            break;
    }

    lock.unlock();
}

/////////////////////////////////////////////////////////////////////////////////////////////////

void SlamDunkScene::initSegmenter(int min_inliers, int max_iterations, float distance_th, bool optimize_coefficient)
{
    _segmenter.setOptimizeCoefficients(optimize_coefficient);
    _segmenter.setModelType(pcl::SACMODEL_PLANE);
    _segmenter.setMethodType(pcl::SAC_RANSAC);
    _segmenter.setMaxIterations(max_iterations);
    _segmenter.setDistanceThreshold(distance_th);
    _segment_min_inliers = min_inliers;
}

/////////////////////////////////////////////////////////////////////////////////////////////////

bool SlamDunkScene::planesExtraction(pcl::PointCloud<SlamDunkCloudType>::Ptr &cloud, CloudSegmentation &segmentation)
{

    if (cloud->size() < _segment_min_inliers)
    {
        return false;
    }

    segmentation = CloudSegmentation(cloud);

    //Segmentation
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    _segmenter.setInputCloud(cloud);
    _segmenter.segment(*inliers, segmentation.plane_coefficients);

    if (inliers->indices.size() <= _segment_min_inliers)
    {
        return false;
    }

    pcl::PointIndices p_indices;
    pcl::PointIndices r_indices;
    pcl::PointCloud<SlamDunkCloudType>::Ptr p_cloud(new pcl::PointCloud<SlamDunkCloudType>);
    pcl::PointCloud<SlamDunkCloudType>::Ptr r_cloud(new pcl::PointCloud<SlamDunkCloudType>);
    // Extract the inliers of PLANES and for REST
    pcl::ExtractIndices<SlamDunkCloudType> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(p_indices.indices);
    extract.filter(*p_cloud);
    extract.setNegative(true);
    extract.filter(r_indices.indices);
    extract.filter(*r_cloud);

    segmentation.plane_indices = p_indices;
    segmentation.rest_indices = r_indices;
    segmentation.plane_cloud = p_cloud;
    segmentation.rest_cloud = r_cloud;
    segmentation.computePlaneOrientation();

    return true;
}

/////////////////////////////////////////////////////////////////////////////////////////////////

void SlamDunkScene::computeFirstSegmentation(std::string &key)
{
    CloudSegmentation first_segmentation;
    planesExtraction(clouds[key], first_segmentation);
    segmentations[key] = first_segmentation;
    first_segmentation_keyframe = key;

    obstacles[key] = pcl::PointCloud<SlamDunkCloudType>::Ptr(new pcl::PointCloud<SlamDunkCloudType>);
    ground_slices[key] = pcl::PointCloud<SlamDunkCloudType>::Ptr(new pcl::PointCloud<SlamDunkCloudType>);

    (*(obstacles[key])) = (*first_segmentation.rest_cloud);
    (*(ground_slices[key])) = (*first_segmentation.plane_cloud);

    rfs[key] = Eigen::Matrix4d();
    segmentations[key].computePlaneRF(rfs[key]);

    first_camera_in_map.matrix() = rfs[key].inverse();
}

/////////////////////////////////////////////////////////////////////////////////////////////////

void SlamDunkScene::computeSegmentation(std::string &key, double angle_th, double height_th)
{

    printf("@@ computrin segmentation\n");
    if (first_segmentation_keyframe.empty())
    {
        computeFirstSegmentation(key);
        return;
    }

    printf("@@ compute rf\n");
    CloudSegmentation segmentation;
    Eigen::Matrix4d T_CAM0_CAMi = poses[key].matrix();
    pcl::PointCloud<SlamDunkCloudType>::Ptr cloud = clouds[key];
    Eigen::Matrix4d T_CAM0_CT0;
    Eigen::Matrix4d T_CT0_CAM0;
    Eigen::Vector3d z_CT0(0, 0, 1);
    Eigen::Vector3d z_CTi;
    segmentations[first_segmentation_keyframe].computePlaneRF(T_CAM0_CT0);

    T_CT0_CAM0 = T_CAM0_CT0.inverse();

    printf("@@ map obstacles\n");
    //INIT
    obstacles[key] = pcl::PointCloud<SlamDunkCloudType>::Ptr(new pcl::PointCloud<SlamDunkCloudType>);
    ground_slices[key] = pcl::PointCloud<SlamDunkCloudType>::Ptr(new pcl::PointCloud<SlamDunkCloudType>);

    pcl::PointCloud<SlamDunkCloudType>::Ptr cloud_input(new pcl::PointCloud<SlamDunkCloudType>);
    pcl::PointCloud<SlamDunkCloudType>::Ptr cloud_rest(new pcl::PointCloud<SlamDunkCloudType>);
    pcl::PointCloud<SlamDunkCloudType>::Ptr cloud_discarded_planes(new pcl::PointCloud<SlamDunkCloudType>);

    (*cloud_input) = (*cloud);

    Eigen::Matrix4d T_CT0_CTi;
    Eigen::Matrix4d T_CAMi_CTi;
    bool segmentation_status = true;
    CloudSegmentation current_segmentation;
    double current_angle;
    double current_z_distance;
    int iterations = 0;
    while (segmentation_status)
    {

        segmentation_status = planesExtraction(cloud_input, current_segmentation);

        if (!segmentation_status)
        {
            //Segmantion failed due to few inliers

            if (iterations == 0)
            {
                //First fail means no planes
                (*(obstacles[key])) = (*cloud_input);
            }
            else
            {
                //Generic fail means no rest cloud to iterate on
                (*current_segmentation.rest_cloud) += (*cloud_discarded_planes);
                (*(obstacles[key])) = (*current_segmentation.rest_cloud);
            }
            break;
        }
        else
        {

            current_segmentation.computePlaneRF(T_CAMi_CTi);
            T_CT0_CTi = T_CT0_CAM0 * T_CAM0_CAMi;
            T_CT0_CTi = T_CT0_CTi * T_CAMi_CTi;

            z_CTi << T_CT0_CTi(0, 2), T_CT0_CTi(1, 2), T_CT0_CTi(2, 2);
            current_angle = fabs(acos(z_CTi.dot(z_CT0)));
            current_z_distance = fabs(T_CT0_CTi(2, 3));

            if (current_angle <= angle_th && current_z_distance <= height_th)
            {
                //Correct plane identification
                (*current_segmentation.rest_cloud) += (*cloud_discarded_planes);
                (*(ground_slices[key])) = (*current_segmentation.plane_cloud);
                (*(obstacles[key])) = (*current_segmentation.rest_cloud);
                break;
            }
            else
            {
                //No planes candidates, recursive iteration on rest
                (*cloud_discarded_planes) += (*current_segmentation.plane_cloud);
                (*cloud_input) = (*current_segmentation.rest_cloud);
            }
            iterations++;
        }
    }
    segmentations[key] = current_segmentation;
    rfs[key] = Eigen::Matrix4d();
    segmentations[key].computePlaneRF(rfs[key]);
}

/////////////////////////////////////////////////////////////////////////////////////////////////

void SlamDunkScene::getPoseInMapFrame(std::string &key, Eigen::Isometry3d &pose)
{
    Eigen::Matrix4d T_CAM0_MAP;
    Eigen::Matrix4d T_MAP_CAM0 = rfs[key];
    Eigen::Matrix4d T_CAM0_CAMi = poses[key].matrix();
    Eigen::Matrix4d T_MAP_CAMi;
    T_MAP_CAM0 = T_CAM0_MAP.inverse();
    T_MAP_CAMi = T_MAP_CAM0 * T_CAM0_CAMi;
    pose.matrix() = T_MAP_CAMi;
}
/////////////////////////////////////////////////////////////////////////////////////////////////

void SlamDunkScene::getGenericPoseInMapFrame(Eigen::Matrix4d &relative_pose, Eigen::Matrix4d &pose)
{
    Eigen::Matrix4d T_CAM0_MAP;
    Eigen::Matrix4d T_MAP_CAM0;
    Eigen::Matrix4d T_CAM0_CAMi = relative_pose;
    Eigen::Matrix4d T_MAP_CAMi;
    T_MAP_CAM0 = first_camera_in_map.matrix();
    T_MAP_CAMi = T_MAP_CAM0 * T_CAM0_CAMi;
    pose = T_MAP_CAMi;
}

/////////////////////////////////////////////////////////////////////////////////////////////////

void SlamDunkScene::addNewEntryCallaback(std::function<void(std::vector<slamdunk::ConsumingEntry> &)> callback)
{
    callbacks_new_entry.push_back(callback);
}

/////////////////////////////////////////////////////////////////////////////////////////////////

void SlamDunkScene::addOptimezedEntryCallabck(std::function<void(std::vector<slamdunk::ConsumingEntry> &)> callback)
{
    callbacks_optimized_entry.push_back(callback);
}
/////////////////////////////////////////////////////////////////////////////////////////////////

void SlamDunkScene::spinIntegrationUpdate()
{
    std::vector<slamdunk::ConsumingEntry> new_entries;
    std::vector<slamdunk::ConsumingEntry> replace_entries;
    getAvailableEntries(new_entries, replace_entries, max_spin_integration_steps, max_spin_deintegration_steps);
    for (int i = 0; i < callbacks_new_entry.size(); i++)
    {
        callbacks_new_entry[i](new_entries);
    }
    for (int i = 0; i < callbacks_optimized_entry.size(); i++)
    {
        callbacks_optimized_entry[i](replace_entries);
    }
}
}
