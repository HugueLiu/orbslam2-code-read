/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include "Viewer.h"
#include "FrameDrawer.h"
#include "Map.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"

#include <mutex>

namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class Map;
class LocalMapping;
class LoopClosing;
class System;

class Tracking
{  

public:
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetViewer(Viewer* pViewer);

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    // TODO: Modify MapPoint::PredictScale to take into account focal lenght
    void ChangeCalibration(const string &strSettingPath);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);


public:

    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };

    // 当前追踪状态
    eTrackingState mState;
    // 上一帧的追踪状态
    eTrackingState mLastProcessedState;

    // Input sensor， 输入类型，双目，RGBD，单目
    int mSensor;

    // Current Frame，当前帧
    Frame mCurrentFrame;
    // 左图的灰度图
    cv::Mat mImGray;

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    // 当前帧相对于参考关键帧的位姿
    list<cv::Mat> mlRelativeFramePoses;
    // 所有的参考关键帧
    list<KeyFrame*> mlpReferences;
    // 每一帧的时间戳
    list<double> mlFrameTimes;
    // 每一帧的状态，是否丢失
    list<bool> mlbLost;

    // True if local mapping is deactivated and we are performing only localization
    // True: 只进行定位， false: 定位与建图，默认为false
    bool mbOnlyTracking;

    void Reset();

protected:

    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Map initialization for stereo and RGB-D， 
    // 双目、RGBD初始化
    void StereoInitialization();

    // Map initialization for monocular
    // 单目初始化
    void MonocularInitialization();
    void CreateInitialMapMonocular();

    // 检查上一帧中地图点是否被标注为替换，若有则真正执行替换操作
    void CheckReplacedInLastFrame();
    // 通过参考关键帧来进行Tracking
    bool TrackReferenceKeyFrame();
    // 更新上一帧位姿, 创建新的MapPoint
    void UpdateLastFrame();
    // 通过运动模型进行Tracking
    bool TrackWithMotionModel();
    // tracking丢失则通过重定位重新获取当前帧位姿
    bool Relocalization();

    // 更新局部地图
    void UpdateLocalMap();
    // 更新局部MapPoint
    void UpdateLocalPoints();
    // 更新局部关键帧
    void UpdateLocalKeyFrames();

    // 局部地图追踪, 更新局部地图
    bool TrackLocalMap();
    // 对当前帧和局部MapPoint做匹配, 在当前帧中增加新的MapPoint
    void SearchLocalPoints();

    // 是否需要新的关键帧
    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    // 定位模式下，是否需要重新做VO
    // 若上一帧中追踪到的MapPoint太少，则值为true
    bool mbVO;

    //Other Thread Pointers
    // 建图线程
    LocalMapping* mpLocalMapper;
    // 回环线程
    LoopClosing* mpLoopClosing;

    // ORB特征提取
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    // 单目ORB特征提取
    ORBextractor* mpIniORBextractor;

    //BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    // Initalization (only for monocular)
    Initializer* mpInitializer;

    //Local Map

    // 参考关键帧
    KeyFrame* mpReferenceKF;
    // 局部地图关键帧
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    // 局部地图点
    std::vector<MapPoint*> mvpLocalMapPoints;
    
    // System
    System* mpSystem;
    
    //Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    //Map
    Map* mpMap;

    //Calibration matrix
    // 内参
    cv::Mat mK;
    // 去畸变参数
    cv::Mat mDistCoef;
    // 基线*fx
    float mbf;

    //New KeyFrame rules (according to fps)
    // 每秒的最小帧数 = 0
    int mMinFrames;
    // 每秒的最大帧数 = fps
    int mMaxFrames;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    // 远近点阈值
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    // RGBD的深度图缩放因子
    float mDepthMapFactor;

    //Current matches in frame
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info

    // 上个关键帧
    KeyFrame* mpLastKeyFrame;
    // 上一帧
    Frame mLastFrame;
    // 上一关键帧ID, 即创建该关键帧的FrameID
    unsigned int mnLastKeyFrameId;
    // 上一个重定位的FrameID
    unsigned int mnLastRelocFrameId;

    //Motion Model, 运动模型, 前一帧的运动
    cv::Mat mVelocity;

    //Color order (true RGB, false BGR, ignored if grayscale)， 颜色顺序
    bool mbRGB;

    // 用与tracking增加的临时MapPoint
    list<MapPoint*> mlpTemporalPoints;
};

} //namespace ORB_SLAM

#endif // TRACKING_H
