/* 
 * Copyright (C) 2017 daniele de gregorio, University of Bologna - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GNU GPLv3 license.
 *
 * please write to: d.degregorio@unibo.it
 */

#include "slamdunk/slam_dunk.h"
#include "slamdunk/pretty_printer.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/timer/timer.hpp>

#include "internal_timers.hpp"

/////////////////////////////////////////////////////////////////////////////////////////////

const slamdunk::FrameData *slamdunk::SlamDunk::getFrameData(int id)
{
    return m_graph_ptr->getVertexData(id);
}

/////////////////////////////////////////////////////////////////////////////////////////////

double slamdunk::SlamDunk::lastWeightedMeanChi2()
{
    return m_last_wmchi2;
}

/////////////////////////////////////////////////////////////////////////////////////////////

unsigned slamdunk::SlamDunk::getNumberOfKeyframes()
{
    if (m_graph_ptr.use_count() > 0)
    {
        return m_graph_ptr->getNoOfVertices();
    }
    else
    {
        return 0;
    }
}

/////////////////////////////////////////////////////////////////////////////////////////////

int slamdunk::SlamDunk::operator()(const RGBDFrame &frame, Eigen::Isometry3d &estimated_pose)
{
    int tracking_state = SlamDunk::TRACKING_FAILED;
    if (!m_graph_ptr) // This is the first frame
    {
        // extract frame data
        FrameData *frame_data = new FrameData();
        m_tracker->extractFrameData(frame, *frame_data);
        // check no of feats
        if (frame_data->m_descriptors.rows < m_min_extracted_features)
        {
            std::cout << SLAM_DUNK_WARNING_STR("Too few feature extracted, first frame not set") << std::endl;
            delete frame_data;
        }
        else
        {
            tracking_state = SlamDunk::KEYFRAME_DETECTED;

            // create a new vertex and set its frame data
            m_graph_ptr.reset(new GraphBackend());
            m_graph_ptr->setVerbose(m_verbose);

            estimated_pose.setIdentity();
            m_graph_ptr->setVertexEstimate(m_processed_frames, estimated_pose, frame_data);

            if (m_try_loop_inference)
                m_dijkstra.reset(new g2o::HyperDijkstra(&(m_graph_ptr->getGraph())));

            m_moved_frames.clear();
            CameraTracker::VertexPoseSet pose_set;
            const g2o::SparseOptimizer &optimizer = m_graph_ptr->getGraph();
            for (g2o::HyperGraph::VertexIDMap::const_iterator it = optimizer.vertices().begin(); it != optimizer.vertices().end(); ++it)
            {
                const CameraTracker::VertexPoseSet::const_iterator psIt = pose_set.insert(static_cast<const g2o::VertexSE3 *>(it->second)).first;
                m_moved_frames.push_back(std::make_pair(static_cast<const FrameData *>((*psIt)->userData())->m_timestamp, (*psIt)->estimate()));
            }

            m_tracker->reset(estimated_pose, *frame_data);
            m_tracker->setFrames(pose_set);
            if (m_doicp)
            {
#ifdef SLAMDUNK_ICP_ENABLED
                samplePointsAndNormals(frame, *frame_data);
#else  // SLAMDUNK_ICP_ENABLED
                std::cerr << SLAM_DUNK_FATAL_STR("ICP not implemented");
                throw std::runtime_error("ICP not implemented");
#endif // SLAMDUNK_ICP_ENABLED
            }
            if (m_deb_func)
                m_deb_func(m_moved_frames);
        }
    }
    else
    {
        if (m_tracker->track(frame))
        {
            tracking_state = SlamDunk::FRAME_TRACKED;

            const TrackingResult &tr = m_tracker->getTrackingResult();
            estimated_pose = tr.m_pose;
            // Keyframe detection
            bool is_keyframe = tr.m_overlapping_score < m_kf_overlapping; // || tr.m_frame_data.m_descriptors.rows < m_min_extracted_features)
            if (!is_keyframe && m_try_loop_inference)                     // check if a loop closure has happened
            {
                const std::set<int> &tracked_keyframes = m_tracker->getLastTrackedKeyframes();
                std::set<int>::const_iterator tkfIt = tracked_keyframes.begin();

                std::vector<g2o::HyperGraph::Vertex *> tracked_vertices(tracked_keyframes.size());
                for (unsigned k = 0; k < tracked_vertices.size(); ++tkfIt, ++k)
                    tracked_vertices[k] = m_graph_ptr->getGraph().vertex(*tkfIt);

                g2o::UniformCostFunction edge_cost;
                for (unsigned k = 0; k < tracked_vertices.size() && !is_keyframe; ++k)
                {
                    g2o::HyperGraph::Vertex *vx = tracked_vertices[k];
                    m_dijkstra->shortestPaths(vx, &edge_cost, /*max distance*/ m_rba_rings);
                    const g2o::HyperGraph::VertexSet &visited = m_dijkstra->visited();

                    for (unsigned k2 = k; k2 < tracked_vertices.size() && !is_keyframe; ++k2)
                        is_keyframe = visited.count(tracked_vertices[k2]) == 0;
                }

                if (is_keyframe)
                    std::cout << "METRIC LOOP DETECTED!" << std::endl;
            }

            if (is_keyframe)
            {
                tracking_state = SlamDunk::KEYFRAME_DETECTED;

                if (m_verbose)
                    std::cout << SLAM_DUNK_INFO_STR("New keyframe detected #") << m_processed_frames
                              << " (" << (m_graph_ptr->getNoOfVertices() + 1) << " kfs)" << std::endl;
                FrameData *frameDataPtr = new FrameData(tr.m_frame_data);
                g2o::VertexSE3 *current_vx = m_graph_ptr->setVertexEstimate(m_processed_frames, estimated_pose, frameDataPtr);
                if (m_try_loop_inference)
                {
                    g2o::HyperDijkstra::AdjacencyMapEntry entry(current_vx, 0, 0, std::numeric_limits<double>::max());
                    m_dijkstra->adjacencyMap().insert(std::make_pair(entry.child(), entry));
                }

                // RBA optimization
                g2o::HyperGraph::VertexSet vSet, fSet;
                if (m_doicp)
                {
#ifndef SLAMDUNK_ICP_ENABLED
                    std::cerr << SLAM_DUNK_FATAL_STR("ICP not implemented");
                    throw std::runtime_error("ICP not implemented");
#else  // SLAMDUNK_ICP_ENABLED
                    samplePointsAndNormals(frame, *frameDataPtr);
                    std::set<int> matchingFrames;
                    m_graph_ptr->addFrameMatchesXYZ(m_processed_frames, tr.m_ff_matches, &matchingFrames);
                    // compute mean feat distance
                    double mean_feat_dist = 0, variance_feat_dist = 0;
                    for (g2o::SparseOptimizer::EdgeSet::iterator eit = current_vx->edges().begin(); eit != current_vx->edges().end(); ++eit)
                    {
                        g2o::SparseOptimizer::Edge *e = static_cast<g2o::SparseOptimizer::Edge *>(*eit);
                        e->computeError();
                        Eigen::Map<Eigen::Vector3d> err(e->errorData());
                        mean_feat_dist += err.norm();
                    }
                    mean_feat_dist /= (double)current_vx->edges().size();
                    for (g2o::SparseOptimizer::EdgeSet::iterator eit = current_vx->edges().begin(); eit != current_vx->edges().end(); ++eit)
                    {
                        g2o::SparseOptimizer::Edge *e = static_cast<g2o::SparseOptimizer::Edge *>(*eit);
                        //e->computeError();
                        Eigen::Map<Eigen::Vector3d> err(e->errorData());
                        const double demean_e = err.norm() - mean_feat_dist;
                        variance_feat_dist += demean_e * demean_e;
                    }
                    variance_feat_dist = std::sqrt(variance_feat_dist / (double)current_vx->edges().size());
                    mean_feat_dist += 2.0 * variance_feat_dist;
                    if (m_verbose)
                        std::cout << SLAM_DUNK_INFO_STR("ICP auto-params: variance=" << variance_feat_dist << " final-max-dist=" << mean_feat_dist) << std::endl;

                    std::vector<std::pair<int, FLANNIndexPtr>> trees(matchingFrames.size());
                    unsigned tIdx = 0;
                    for (std::set<int>::const_iterator it = matchingFrames.begin(); it != matchingFrames.end(); ++it, ++tIdx)
                    {
                        trees[tIdx].first = *it;
                        FrameData *matchingData = m_graph_ptr->getVertexData(*it);
                        trees[tIdx].second.reset(new FLANNIndex(
                            flann::Matrix<float>(matchingData->m_samplesPts.data(), matchingData->m_samplesNormals.size(), 3),
                            flann::KDTreeIndexParams(1)));
                        trees[tIdx].second->buildIndex();
                    }

                    std::list<g2o::SparseOptimizer::Edge *> icp_edges;
                    int innerIt = 0;
                    unsigned outerIt = 0;
                    float lastChi2 = -1, currentChi2 = -1, gain = -1;
                    {
                        SLAM_DUNK_AUTO_CPU_TIMER("Local Optimization");
                        do
                        {
                            lastChi2 = currentChi2;
                            updateICPConstraints(m_processed_frames, (float)(mean_feat_dist * mean_feat_dist), trees, icp_edges);
                            innerIt += m_graph_ptr->relativeOptimization(m_processed_frames, m_rba_rings, 100, &vSet, &fSet);
                            currentChi2 = m_graph_ptr->chi2();
                            gain = lastChi2 / currentChi2 - 1.f;
                            ++outerIt;
                        } while (!((gain >= 0 && gain < 1e-3) || currentChi2 == 0.0 || outerIt == m_icp_maxit)); // check chi2
                    }
                    if (m_verbose)
                        std::cout << SLAM_DUNK_INFO_STR("RBA optimization performed (")
                                  << outerIt << " outer iterations; " << innerIt << " total inner iterations)";

                    m_last_wmchi2 = m_graph_ptr->weightedMeanChi2();
#endif // SLAMDUNK_ICP_ENABLED
                }
                else
                {
                    m_graph_ptr->addFrameMatchesXYZ(m_processed_frames, tr.m_ff_matches, NULL);
                    int total_its;
                    {
                        SLAM_DUNK_AUTO_CPU_TIMER("Local Optimization");
                        total_its = m_graph_ptr->relativeOptimization(m_processed_frames, m_rba_rings, 100, &vSet, &fSet);
                    }
                    if (m_verbose)
                        std::cout << SLAM_DUNK_INFO_STR("RBA optimization performed (")
                                  << total_its << " iterations)";

                    m_last_wmchi2 = m_graph_ptr->weightedMeanChi2();
                }

                // get solution
                m_moved_frames.clear();
                CameraTracker::VertexPoseSet pose_set;
                for (g2o::HyperGraph::VertexSet::const_iterator it = vSet.begin(); it != vSet.end(); ++it)
                {
                    const CameraTracker::VertexPoseSet::const_iterator psIt = pose_set.insert(static_cast<const g2o::VertexSE3 *>(*it)).first;
                    m_moved_frames.push_back(std::make_pair(static_cast<const FrameData *>((*psIt)->userData())->m_timestamp, (*psIt)->estimate()));
                }
                // signal moved frames
                m_tracker->signalMovedFrames(pose_set);
                m_graph_ptr->getVertexEstimate(m_processed_frames, estimated_pose);

                if (m_deb_func)
                {
#ifdef UNIX
                    //system(("echo \"" + graphToDot(m_graph_ptr.get(), m_processed_frames, fSet, vSet, "GlobalGraph") + "\" | circo -Tsvg | display").c_str());
                    system(("echo \"" + graphToDot(m_graph_ptr.get(), m_processed_frames, fSet, vSet, "GlobalGraph") + "\" | circo -Tsvg | display").c_str());
#endif
                    m_deb_func(m_moved_frames);
                }
            }
            else
                m_tracker->updateMap();
        }
        else if (m_verbose)
            std::cout << SLAM_DUNK_WARNING_STR("Tracking failure") << std::endl;
    }

    if (tracking_state != SlamDunk::TRACKING_FAILED)
        ++m_processed_frames;
    return tracking_state;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

void slamdunk::SlamDunk::forceGlobalOptimization()
{
    //  m_graph_ptr->setFixedView(0,true);
    boost::timer::cpu_timer stopwatch;
    const int total_its = m_graph_ptr->optimize(100);
    stopwatch.stop();
    //  m_graph_ptr->setFixedView(0,false);
    m_last_wmchi2 = m_graph_ptr->weightedMeanChi2();

    if (m_verbose)
        std::cout << SLAM_DUNK_INFO_STR("Global optimization performed (")
                  << total_its << " iterations)"
                  << " >>> " << stopwatch.format() << std::endl;

    // get solution
    m_moved_frames.clear();
    CameraTracker::VertexPoseSet pose_set;
    for (g2o::HyperGraph::VertexIDMap::const_iterator it = m_graph_ptr->getGraph().vertices().begin();
         it != m_graph_ptr->getGraph().vertices().end(); ++it)
    {
        const CameraTracker::VertexPoseSet::const_iterator psIt = pose_set.insert(static_cast<const g2o::VertexSE3 *>(it->second)).first;
        m_moved_frames.push_back(std::make_pair(static_cast<const FrameData *>((*psIt)->userData())->m_timestamp, (*psIt)->estimate()));
    }
    // signal moved frames
    m_tracker->signalMovedFrames(pose_set);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////

void slamdunk::SlamDunk::getMappedPoses(StampedPoseVector &poses) const
{
    for (g2o::HyperGraph::VertexIDMap::const_iterator it = m_graph_ptr->getGraph().vertices().begin();
         it != m_graph_ptr->getGraph().vertices().end(); ++it)
    {
        const g2o::VertexSE3 *vx = static_cast<const g2o::VertexSE3 *>(it->second);
        poses.push_back(std::make_pair(static_cast<const FrameData *>(vx->userData())->m_timestamp, vx->estimate()));
    }
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef SLAMDUNK_ICP_ENABLED

namespace
{

enum
{
    SLAM_DUNK_GRID_SIZE = 4,
    SLAM_DUNK_GRID_SIZE_PW2 = SLAM_DUNK_GRID_SIZE * SLAM_DUNK_GRID_SIZE,
    SLAM_DUNK_GRID_SIZE_PW4 = SLAM_DUNK_GRID_SIZE_PW2 * SLAM_DUNK_GRID_SIZE_PW2,
    SLAM_DUNK_MAX_ICP_DENSITY_MULTIPLIER = 5
};
}

void slamdunk::SlamDunk::samplePointsAndNormals(const RGBDFrame &frame, FrameData &frameData) const
{
    Eigen::Matrix3f kcam = Eigen::Matrix3f::Identity();
    kcam(0, 0) = 1.f / m_inverse_kcam(0, 0);
    kcam(1, 1) = 1.f / m_inverse_kcam(1, 1);
    kcam(0, 2) = -1.f * m_inverse_kcam(0, 2) * kcam(0, 0);
    kcam(1, 2) = -1.f * m_inverse_kcam(1, 2) * kcam(1, 1);

    unsigned kpts_hist[SLAM_DUNK_GRID_SIZE_PW2];
    std::memset(kpts_hist, 0, SLAM_DUNK_GRID_SIZE_PW2 * sizeof(unsigned));
    const float coeffx = (float)SLAM_DUNK_GRID_SIZE / (float)frame.m_depth_image.cols;
    const float coeffy = (float)SLAM_DUNK_GRID_SIZE / (float)frame.m_depth_image.rows;
    for (unsigned i = 0; i < frameData.m_keypoints.size(); ++i)
    {
        Eigen::Vector3f pt2d = kcam * frameData.m_keypoints[i];
        pt2d /= pt2d.z();
        assert(pt2d.x() >= 0 && pt2d.x() < frame.m_depth_image.cols);
        assert(pt2d.y() >= 0 && pt2d.y() < frame.m_depth_image.rows);
        const int index = (int)std::floor(pt2d.y() * coeffy) * SLAM_DUNK_GRID_SIZE + (int)std::floor(pt2d.x() * coeffx);
        assert(index >= 0 && index < SLAM_DUNK_GRID_SIZE_PW2);
        kpts_hist[index] += 1;
    }

    std::vector<std::pair<int, int>> samples;
    const float coeffSamples = (float)frameData.m_keypoints.size() * (float)m_icp_samples / (float)(SLAM_DUNK_GRID_SIZE_PW4);
    const float grid_cols = (float)frame.m_depth_image.cols / (float)SLAM_DUNK_GRID_SIZE;
    const float grid_rows = (float)frame.m_depth_image.rows / (float)SLAM_DUNK_GRID_SIZE;
    const unsigned max_pts_per_cell = (unsigned)(m_icp_samples * (SLAM_DUNK_MAX_ICP_DENSITY_MULTIPLIER / (float)(SLAM_DUNK_GRID_SIZE_PW2)));
    for (unsigned i = 0; i < SLAM_DUNK_GRID_SIZE; ++i)
        for (unsigned j = 0; j < SLAM_DUNK_GRID_SIZE; ++j)
        {
            float noOfSamples = max_pts_per_cell;
            if (kpts_hist[i * SLAM_DUNK_GRID_SIZE + j] > 0)
                noOfSamples = std::min(max_pts_per_cell, (unsigned)(coeffSamples / (float)kpts_hist[i * SLAM_DUNK_GRID_SIZE + j]));

            // extract noOfSamples points from the cell (i,j)
            const float sample_grid_length = std::sqrt(noOfSamples);
            const float stepx = grid_cols / sample_grid_length;
            const float stepy = grid_rows / sample_grid_length;
            for (float vf = i * grid_rows + stepy / 2.f; vf < (i + 1) * grid_rows; vf += stepy)
            {
                const int v = (int)std::floor(vf);
                assert(v < frame.m_depth_image.rows);
                for (float uf = j * grid_cols + stepx / 2.f; uf < (j + 1) * grid_cols; uf += stepx)
                {
                    const int u = (int)std::floor(uf);
                    assert(u < frame.m_depth_image.cols);
                    samples.push_back(std::make_pair(v, u));
                }
            }
        }

    if (m_debug)
    {
        std::cout << SLAM_DUNK_INFO_STR("Sampling Grid: " << SLAM_DUNK_GRID_SIZE << "x" << SLAM_DUNK_GRID_SIZE << "; No of Samples: " << samples.size()) << std::endl;
        cv::Mat img;
        if (frame.m_color_image.channels() == 3)
            img = frame.m_color_image.clone();
        else
            cv::cvtColor(frame.m_color_image, img, CV_GRAY2BGR);
        for (unsigned vu = 0; vu < samples.size(); ++vu)
            cv::circle(img, cv::Point(samples[vu].second, samples[vu].first), 2, cv::Scalar(255, 0, 0), -1);
#ifndef NDEBUG
        cv::drawKeypoints(img, frameData.m_kpts2d, img, cv::Scalar(0, 0, 255),
                          cv::DrawMatchesFlags::DRAW_OVER_OUTIMG | cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
#endif
        cv::imshow("debug", img);
        cv::waitKey();
    }

    /***************
     *    1  | 2 | 3
     *   ---------
     *   0/8 | x | 4
     *   ---------
     *    7  | 6 | 5
     ****************/
    std::vector<Eigen::Vector3f> pts;
    for (unsigned vu = 0; vu < samples.size(); ++vu)
    {
        Eigen::Vector3f nvec[9];
        bool nvec_flags[9] = {false, false, false, false, false, false, false, false, false};

        const int v = samples[vu].first;
        const int u = samples[vu].second;
        const float dvalue = frame.m_depth_image[v][u];
        if (dvalue > 0)
        {
            std::pair<Eigen::Vector3f, Eigen::Vector3f> pointNormal;
            pointNormal.first = m_inverse_kcam * Eigen::Vector3f(u * dvalue, v * dvalue, dvalue);
            // normal estimation
            pointNormal.second.setZero();
            float dn;
            if (u - 1 >= 0)
            {
                dn = frame.m_depth_image(v, u - 1);
                if (dn > 0)
                {
                    nvec[0] = m_inverse_kcam * Eigen::Vector3f((u - 1) * dn, v * dn, dn) - pointNormal.first;
                    nvec_flags[0] = true;
                }
                if (v - 1 >= 0)
                {
                    dn = frame.m_depth_image(v - 1, u - 1);
                    if (dn > 0)
                    {
                        nvec[1] = m_inverse_kcam * Eigen::Vector3f((u - 1) * dn, (v - 1) * dn, dn) - pointNormal.first;
                        nvec_flags[1] = true;
                    }
                    dn = frame.m_depth_image(v - 1, u);
                    if (dn > 0)
                    {
                        nvec[2] = m_inverse_kcam * Eigen::Vector3f(u * dn, (v - 1) * dn, dn) - pointNormal.first;
                        nvec_flags[2] = true;
                    }
                }
                if (v + 1 < frame.m_depth_image.rows)
                {
                    dn = frame.m_depth_image(v + 1, u - 1);
                    if (dn > 0)
                    {
                        nvec[7] = m_inverse_kcam * Eigen::Vector3f((u - 1) * dn, (v + 1) * dn, dn) - pointNormal.first;
                        nvec_flags[7] = true;
                    }
                    dn = frame.m_depth_image(v + 1, u);
                    if (dn > 0)
                    {
                        nvec[6] = m_inverse_kcam * Eigen::Vector3f(u * dn, (v + 1) * dn, dn) - pointNormal.first;
                        nvec_flags[6] = true;
                    }
                }
            }
            if (u + 1 < frame.m_depth_image.cols)
            {
                dn = frame.m_depth_image(v, u + 1);
                if (dn > 0)
                {
                    nvec[4] = m_inverse_kcam * Eigen::Vector3f((u + 1) * dn, v * dn, dn) - pointNormal.first;
                    nvec_flags[4] = true;
                }
                if (v - 1 >= 0)
                {
                    dn = frame.m_depth_image(v - 1, u + 1);
                    if (dn > 0)
                    {
                        nvec[3] = m_inverse_kcam * Eigen::Vector3f((u + 1) * dn, (v - 1) * dn, dn) - pointNormal.first;
                        nvec_flags[3] = true;
                    }
                }
                if (v + 1 < frame.m_depth_image.rows)
                {
                    dn = frame.m_depth_image(v + 1, u + 1);
                    if (dn > 0)
                    {
                        nvec[5] = m_inverse_kcam * Eigen::Vector3f((u + 1) * dn, (v + 1) * dn, dn) - pointNormal.first;
                        nvec_flags[5] = true;
                    }
                }
            }
            nvec[8] = nvec[0];
            nvec_flags[8] = nvec_flags[0];

            unsigned count = 0;
            for (unsigned k = 1; k < 9; ++k)
                if (nvec_flags[k] && nvec_flags[k - 1])
                {
                    pointNormal.second += nvec[k].cross(nvec[k - 1]).normalized();
                    ++count;
                }
            if ((pointNormal.second.array() != 0).any())
            {
                pointNormal.second /= count;
                pointNormal.second.normalize();
                pts.push_back(pointNormal.first);
                frameData.m_samplesNormals.push_back(pointNormal.second);
            }
        }
    }

    frameData.m_samplesPts.resize(3 * pts.size());
    for (unsigned k = 0; k < pts.size(); ++k)
        frameData.m_samplesPts.segment<3>(3 * k) = pts[k];
    assert(frameData.m_samplesPts.rows() == 3 * (int)frameData.m_samplesNormals.size());

    if (m_verbose)
        std::cout << SLAM_DUNK_INFO_STR(pts.size() << " sampled points have valid depth and normal") << std::endl;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
namespace
{

enum
{
    SLAM_DUNK_ICP_KNN = 5,
    SLAM_DUNK_ICP_MAX_PT_MATCHES = 3
};

template <int N>
struct IteratorList
{
    typedef std::list<g2o::SparseOptimizer::Edge *>::iterator value_type;
    value_type its[N];
    int size;

    IteratorList() : size(0)
    {
    }

    inline bool valid() const
    {
        return size >= 0;
    }

    inline bool full() const
    {
        return size == N;
    }

    inline void invalidate()
    {
        size = -1;
    }

    inline void push_nocheck(const value_type &v)
    {
        its[size++] = v;
    }
};
}

void slamdunk::SlamDunk::updateICPConstraints(int ref_frame_id, float icp_sqDistance_th,
                                              const std::vector<std::pair<int, FLANNIndexPtr>> &trees,
                                              std::list<g2o::SparseOptimizer::Edge *> &icp_edges)
{
    Eigen::Isometry3d refTr;
    const FrameData &frameData = *(m_graph_ptr->getVertexEstimateAndData(ref_frame_id, refTr));

    for (std::list<g2o::SparseOptimizer::Edge *>::iterator it = icp_edges.begin(); it != icp_edges.end(); ++it)
        m_graph_ptr->removeEdge(*it);
    icp_edges.clear();

    // For each tree
    // - NN search
    // - simple distance and normal rejection
    // - add icp edges to the graph and to the vector
    flann::SearchParams search_params(-1, 0, true);
    search_params.cores = m_cores;

    Eigen::VectorXf transformed_pts;
    transformed_pts.resize(frameData.m_samplesPts.size());
    flann::Matrix<int> indices(new int[SLAM_DUNK_ICP_KNN * frameData.m_samplesNormals.size()], frameData.m_samplesNormals.size(), SLAM_DUNK_ICP_KNN);
    flann::Matrix<float> dists(new float[SLAM_DUNK_ICP_KNN * frameData.m_samplesNormals.size()], frameData.m_samplesNormals.size(), SLAM_DUNK_ICP_KNN);

    for (unsigned k = 0; k < trees.size(); ++k)
    {
        Eigen::Isometry3d matchingTr;
        const FrameData *matchingFrame = m_graph_ptr->getVertexEstimateAndData(trees[k].first, matchingTr);
        assert(matchingFrame != NULL);
        const Eigen::Isometry3f r2m = (matchingTr.inverse() * refTr).cast<float>();

        for (unsigned h = 0; h < frameData.m_samplesNormals.size(); ++h)
            transformed_pts.segment<3>(3 * h) = r2m * frameData.m_samplesPts.segment<3>(3 * h);
        flann::Matrix<float> query_mtx(transformed_pts.data(), frameData.m_samplesNormals.size(), 3);

        std::map<int, IteratorList<SLAM_DUNK_ICP_MAX_PT_MATCHES>> assigned;

        trees[k].second->knnSearch(query_mtx, indices, dists, SLAM_DUNK_ICP_KNN, search_params);
        for (unsigned i = 0; i < query_mtx.rows; ++i)
        {
            const Eigen::Vector3f transformed_normal = r2m.rotation() * frameData.m_samplesNormals[i];
            const int *indexPtr = indices[i];
            const float *distsPtr = dists[i];

            for (int ne = 0; ne < SLAM_DUNK_ICP_KNN; ++ne)
            {
                if (distsPtr[ne] > icp_sqDistance_th)
                    break;
                if (transformed_normal.dot(matchingFrame->m_samplesNormals[indexPtr[ne]]) > m_icp_cosnormal_th)
                {
                    // good match!
                    IteratorList<SLAM_DUNK_ICP_MAX_PT_MATCHES> &itList = assigned[indexPtr[ne]];
                    if (!itList.valid())
                        break;
                    if (itList.full())
                    {
                        for (int sz = 0; sz < itList.size; ++sz)
                        {
                            m_graph_ptr->removeEdge(*(itList.its[sz]));
                            icp_edges.erase(itList.its[sz]);
                            itList.invalidate();
                        }
                        break;
                    }

                    icp_edges.push_back(
                        m_graph_ptr->addFrameMatchXYZ(ref_frame_id, frameData.m_samplesPts.segment<3>(3 * i).cast<double>(),
                                                      frameData.m_samplesNormals[i].cast<double>(),
                                                      trees[k].first, (matchingFrame->m_samplesPts.segment<3>(3 * indexPtr[ne])).cast<double>(),
                                                      (matchingFrame->m_samplesNormals[indexPtr[ne]]).cast<double>()));
                    itList.push_nocheck(--(icp_edges.end()));
                    break;
                }
            }
        }

#ifndef NDEBUG
        if (m_debug)
        {
            std::cout << "ICP matches with vx #" << trees[k].first << std::endl;
            Eigen::Matrix3d kcam = Eigen::Matrix3d::Identity();
            kcam(0, 0) = 1. / m_inverse_kcam(0, 0);
            kcam(1, 1) = 1. / m_inverse_kcam(1, 1);
            kcam(0, 2) = -1. * m_inverse_kcam(0, 2) * kcam(0, 0);
            kcam(1, 2) = -1. * m_inverse_kcam(1, 2) * kcam(1, 1);
            cv::Mat outimg;
            std::vector<cv::KeyPoint> pts1, pts2;
            std::vector<cv::DMatch> icp_matches;
            for (std::list<g2o::SparseOptimizer::Edge *>::const_iterator it = icp_edges.begin(); it != icp_edges.end(); ++it)
            {
                cv::KeyPoint kpt1, kpt2;
                cv::DMatch dm;

                Eigen::Vector3f pt12d, pt22d;
                const EdgeSE3XYZPair *e = static_cast<const EdgeSE3XYZPair *>(*it);
                pt12d = (e->vertices()[0]->id() == ref_frame_id ? kcam * e->measurement().col(0) : kcam * e->measurement().col(1)).cast<float>();
                pt22d = (e->vertices()[1]->id() == ref_frame_id ? kcam * e->measurement().col(0) : kcam * e->measurement().col(1)).cast<float>();

                kpt1.pt.x = pt12d.x() / pt12d.z();
                kpt1.pt.y = pt12d.y() / pt12d.z();
                kpt2.pt.x = pt22d.x() / pt22d.z();
                kpt2.pt.y = pt22d.y() / pt22d.z();
                dm.queryIdx = pts1.size();
                dm.trainIdx = pts2.size();
                pts1.push_back(kpt1);
                pts2.push_back(kpt2);
                icp_matches.push_back(dm);
            }
            drawMatches(frameData.m_image, pts1, matchingFrame->m_image, pts2, icp_matches, outimg);
            cv::imshow("debug", outimg);
            cv::waitKey();
        }
#endif
    }

    if (m_verbose)
        std::cout << SLAM_DUNK_INFO_STR(icp_edges.size() << " icp matches set") << std::endl;
}

#endif // SLAMDUNK_ICP_ENABLED
