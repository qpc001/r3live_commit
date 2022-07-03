/* 
This code is the implementation of our paper "R3LIVE: A Robust, Real-time, RGB-colored, 
LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package".

Author: Jiarong Lin   < ziv.lin.ljr@gmail.com >

If you use any code of this repo in your academic research, please cite at least
one of our papers:
[1] Lin, Jiarong, and Fu Zhang. "R3LIVE: A Robust, Real-time, RGB-colored, 
    LiDAR-Inertial-Visual tightly-coupled state Estimation and mapping package." 
[2] Xu, Wei, et al. "Fast-lio2: Fast direct lidar-inertial odometry."
[3] Lin, Jiarong, et al. "R2LIVE: A Robust, Real-time, LiDAR-Inertial-Visual
     tightly-coupled state Estimator and mapping." 
[4] Xu, Wei, and Fu Zhang. "Fast-lio: A fast, robust lidar-inertial odometry 
    package by tightly-coupled iterated kalman filter."
[5] Cai, Yixi, Wei Xu, and Fu Zhang. "ikd-Tree: An Incremental KD Tree for 
    Robotic Applications."
[6] Lin, Jiarong, and Fu Zhang. "Loam-livox: A fast, robust, high-precision 
    LiDAR odometry and mapping package for LiDARs of small FoV."

For commercial use, please contact me < ziv.lin.ljr@gmail.com > and
Dr. Fu Zhang < fuzhang@hku.hk >.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.
 3. Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from this
    software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 POSSIBILITY OF SUCH DAMAGE.
*/
#pragma once
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>

#include "tools_logger.hpp"
#include "tools_color_printf.hpp"
#include "tools_eigen.hpp"
#include "tools_data_io.hpp"
#include "tools_timer.hpp"
#include "pointcloud_rgbd.hpp"
#include "image_frame.hpp"
#include "opencv2/calib3d.hpp"
#include "../optical_flow/lkpyramid.hpp"
extern std::string data_dump_dir;

class Rgbmap_tracker
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::vector< cv::Mat >     m_img_vec;
    char                       m_temp_char[ 1024 ];
    cv::Mat                    m_old_frame, m_old_gray;
    cv::Mat                    frame_gray, m_current_frame;
    cv::Mat                    m_current_mask;
    unsigned int               m_frame_idx = 0; ///< 光流跟踪过的帧数量
    double                     m_last_frame_time, m_current_frame_time;
    // m_old_ids: 在update_last_tracking_vector_and_ids()函数中更新， 内容为: [0,1,2,3...], 没什么卵用? 因为 m_old_ids[0] = 0, m_old_ids[1] = 1, ...
    // 存放着 m_map_rgb_pts_in_last_frame_pos 容器 对应于 m_rgb_pts_ptr_vec_in_last_frame、m_last_tracked_pts的关联索引
    // 即 m_map_rgb_pts_in_last_frame_pos[m_old_ids[idx]] = < m_rgb_pts_ptr_vec_in_last_frame[m_old_ids[idx]], m_last_tracked_pts[m_old_ids[idx]] >
    std::vector< int >         m_current_ids, m_old_ids;    ///< 存放着与m_last_tracked_pts, m_current_tracked_pts关联的地图点的索引
    int                        if_debug_match_img = 0;
    unsigned int               m_maximum_vio_tracked_pts = 300;
    cv::Mat                    m_ud_map1, m_ud_map2;
    cv::Mat                    m_intrinsic, m_dist_coeffs;
    // m_last_tracked_pts: 成功跟踪的上一帧的像素点, 在update_last_tracking_vector_and_ids()函数中更新，更新的方式就是把m_map_rgb_pts_in_last_frame_pos中的value保存下来
    std::vector< cv::Point2f > m_last_tracked_pts, m_current_tracked_pts;   ///< 跟踪像素点坐标, m_current_tracked_pts: 当前帧成功跟踪的点
    std::vector< cv::Scalar >  m_colors;
    std::vector< void * >      m_rgb_pts_ptr_vec_in_last_frame; ///< 存放上一帧跟踪到的地图点的指针， 每个元素都是[地图点内存地址], 在update_last_tracking_vector_and_ids()函数中更新，更新的方式就是把m_map_rgb_pts_in_last_frame_pos中的key保存下来
    // key = [地图点内存地址]  value = [地图点反投影到图像的像素点坐标]
    std::map< void *, cv::Point2f > m_map_rgb_pts_in_last_frame_pos;    ///< key: 当前帧跟踪到的地图点， value: 该地图点对应上一帧的像素点坐标
    std::map< void *, cv::Point2f > m_map_rgb_pts_in_current_frame_pos; ///< key: 当前帧跟踪到的地图点指针， value: 对应当前帧的像素点坐标

    std::map< int, std::vector< cv::Point2f > > m_map_id_pts_vec;
    std::map< int, std::vector< int > >         m_map_id_pts_frame;
    std::map< int, std::vector< cv::Point2f > > m_map_frame_pts;
    cv::Mat                                   m_debug_track_img;
    eigen_q                                   q_last_estimated_q = eigen_q::Identity();
    vec_3                                     t_last_estimated = vec_3( 0, 0, 0 );
    std::shared_ptr< LK_optical_flow_kernel > m_lk_optical_flow_kernel;
    Rgbmap_tracker();
    ~Rgbmap_tracker(){};

    void set_intrinsic( Eigen::Matrix3d cam_K, Eigen::Matrix< double, 5, 1 > dist, cv::Size imageSize )
    {
        cv::eigen2cv( cam_K, m_intrinsic );
        cv::eigen2cv( dist, m_dist_coeffs );
        initUndistortRectifyMap( m_intrinsic, m_dist_coeffs, cv::Mat(), m_intrinsic, imageSize, CV_16SC2, m_ud_map1, m_ud_map2 );
    }

    /**
     * @brief update_last_tracking_vector_and_ids
     * 遍历m_map_rgb_pts_in_last_frame_pos容器，利用新数据更新如下容器:
     * - m_last_tracked_pts
     * - m_rgb_pts_ptr_vec_in_last_frame
     * - m_colors
     * - m_old_ids
     */
    void update_last_tracking_vector_and_ids()
    {
        int idx = 0;
        // 清空几个容器
        m_last_tracked_pts.clear();
        m_rgb_pts_ptr_vec_in_last_frame.clear();
        m_colors.clear();
        m_old_ids.clear();
        // 遍历m_map_rgb_pts_in_last_frame_pos容器
        for ( auto it = m_map_rgb_pts_in_last_frame_pos.begin(); it != m_map_rgb_pts_in_last_frame_pos.end(); it++ )
        {
            // 将新数据保存到以下几个容器
            m_rgb_pts_ptr_vec_in_last_frame.push_back( it->first );
            m_last_tracked_pts.push_back( it->second );
            m_colors.push_back( ( ( RGB_pts * ) it->first )->m_dbg_color );
            m_old_ids.push_back( idx );
            idx++;
        }
    }

    /**
     * @brief set_track_pts
     * 只有在init()时被调用
     * @param img
     * @param rgb_pts_vec
     * @param pts_proj_img_vec
     */
    void set_track_pts( cv::Mat &img, std::vector< std::shared_ptr< RGB_pts > > &rgb_pts_vec, std::vector< cv::Point2f > &pts_proj_img_vec )
    {
        // 拷贝图像到m_old_frame
        m_old_frame = img.clone();
        // 转成灰度图
        cv::cvtColor( m_old_frame, m_old_gray, cv::COLOR_BGR2GRAY );
        // 保存 [地图点——反投影到图像的像素点坐标]
        m_map_rgb_pts_in_last_frame_pos.clear();
        for ( unsigned int i = 0; i < rgb_pts_vec.size(); i++ )
        {
            // m_map_rgb_pts_in_last_frame_pos[地图点内存地址] = 地图点反投影到图像的像素点坐标
            m_map_rgb_pts_in_last_frame_pos[ ( void * ) rgb_pts_vec[ i ].get() ] = pts_proj_img_vec[ i ];
        }
        //
        update_last_tracking_vector_and_ids();
    }

    /**
     * @brief init
     * 初始化
     * @param img_with_pose 输入图像和pose
     * @param rgb_pts_vec   输入地图点
     * @param pts_proj_img_vec 输入对应的图像上的像素点
     */
    void init( const std::shared_ptr< Image_frame > &img_with_pose, std::vector< std::shared_ptr< RGB_pts > > &rgb_pts_vec, std::vector< cv::Point2f > &pts_proj_img_vec )
    {
        // 将初始化时筛选的地图点——反投影像素坐标保存到m_map_rgb_pts_in_last_frame_pos容器，同时更新其他几个容器
        set_track_pts( img_with_pose->m_img, rgb_pts_vec, pts_proj_img_vec );
        m_current_frame_time = img_with_pose->m_timestamp;
        m_last_frame_time = m_current_frame_time;
        std::vector< uchar > status;
        // 由于这里是初始化，因此在track_image函数中，只运行到swap数据到prev的几个容器中（即初始化图像金字塔、梯度等），用于下一次的光流跟踪
        m_lk_optical_flow_kernel->track_image( img_with_pose->m_img_gray, m_last_tracked_pts, m_current_tracked_pts, status );
    }

    void update_points( std::vector< cv::Point2f > &pts_vec, std::vector< int > &pts_ids )
    {
        for ( unsigned int i = 0; i < pts_vec.size(); i++ )
        {
            m_map_id_pts_vec[ pts_ids[ i ] ].push_back( pts_vec[ i ] );
            m_map_id_pts_frame[ pts_ids[ i ] ].push_back( m_frame_idx );
            m_map_frame_pts[ m_frame_idx ].push_back( pts_vec[ i ] );
        }
    }

    void undistort_image( cv::Mat &img )
    {
        cv::Mat temp_img;
        temp_img = img.clone();
        remap( temp_img, img, m_ud_map1, m_ud_map2, cv::INTER_LINEAR );
    }

    void update_and_append_track_pts( std::shared_ptr< Image_frame > &img_pose, Global_map &map_rgb, double mini_dis = 10.0, int minimum_frame_diff = 3e8 );
    void reject_error_tracking_pts( std::shared_ptr< Image_frame > &img_pose, double dis = 2.0 );

    /**
     * @brief: 根据跟踪的结果，对容器进行裁减
     */
    template < typename T > void reduce_vector( std::vector< T > &v, std::vector< uchar > status )
    {
        int j = 0;
        for ( unsigned int i = 0; i < v.size(); i++ )
            if ( status[ i ] )
                v[ j++ ] = v[ i ];
        v.resize( j );
    }

    void track_img( std::shared_ptr< Image_frame > &img_pose, double dis = 2.0, int if_use_opencv = 1 );
    int get_all_tracked_pts( std::vector< std::vector< cv::Point2f > > *img_pt_vec = nullptr );
    int remove_outlier_using_ransac_pnp( std::shared_ptr< Image_frame > &img_pose, int if_remove_ourlier = 1 );
};
