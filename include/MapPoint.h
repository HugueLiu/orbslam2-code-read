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

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"

#include<opencv2/core/core.hpp>
#include<mutex>

namespace ORB_SLAM2
{

class KeyFrame;
class Map;
class Frame;


class MapPoint
{
public:
    MapPoint(const cv::Mat &Pos, KeyFrame* pRefKF, Map* pMap);
    MapPoint(const cv::Mat &Pos,  Map* pMap, Frame* pFrame, const int &idxF);

    // 设置该MapPoint的世界坐标
    void SetWorldPos(const cv::Mat &Pos);
    // 获取该MapPoint的世界坐标
    cv::Mat GetWorldPos();

    cv::Mat GetNormal();
    KeyFrame* GetReferenceKeyFrame();

    // 返回所有观察到MapPoint的关键帧及索引
    std::map<KeyFrame*,size_t> GetObservations();
    // 返回观察到该MapPoint的关键帧数量
    int Observations();
    // 增加看到该点的关键帧
    void AddObservation(KeyFrame* pKF,size_t idx);
    // 去除看到该点的关键帧
    void EraseObservation(KeyFrame* pKF);

    // 该MapPoint在该关键帧中的index，不存在则返回-1
    int GetIndexInKeyFrame(KeyFrame* pKF);
    // 该MapPoint是否存在于该关键帧中
    bool IsInKeyFrame(KeyFrame* pKF);

    // 设置该MapPoint为"Bad"，并清除对应关键帧及地图中的该点， 即删除该MapPoint
    void SetBadFlag();
    // 返回该MapPoint是否“Bad”
    bool isBad();

    // 用新的MapPoint代替当前MapPoint，不会立即删除原对象，而是设置mpReplaced变量
    void Replace(MapPoint* pMP);
    // 返回替代该MapPoint的对象
    MapPoint* GetReplaced();

    // 增加visible的帧数
    void IncreaseVisible(int n=1);
    // 增加found的帧数
    void IncreaseFound(int n=1);
    // found/visible
    float GetFoundRatio();
    inline int GetFound(){
        return mnFound;
    }

    // 为该点选择一个与其他描述距离最小的描述子
    void ComputeDistinctiveDescriptors();

    // 返回该点描述子
    cv::Mat GetDescriptor();

    // 更新该点平均方向
    void UpdateNormalAndDepth();

    // 返回最小距离
    float GetMinDistanceInvariance();
    // 返回最大距离
    float GetMaxDistanceInvariance();
    // 预测该距离的MapPoint在该KeyFrame的哪一层
    int PredictScale(const float &currentDist, KeyFrame*pKF);
    // 预测该距离的MapPoint在该Frame的哪一层
    int PredictScale(const float &currentDist, Frame* pF);

public:
    // MapPoint ID
    long unsigned int mnId;
    static long unsigned int nNextId;
    // 该点关联的关键帧ID
    long int mnFirstKFid;
    // 该点关联的Frame ID
    long int mnFirstFrame;
    // 观测到该点的关键帧数量
    int nObs;

    // Variables used by the tracking
    // 投影后的u
    float mTrackProjX;
    // 投影后的v
    float mTrackProjY;
    // 投影后在右图中的u
    float mTrackProjXR;
    // 是否追踪该点
    bool mbTrackInView;
    // 预测该点在该帧中位于哪一层
    int mnTrackScaleLevel;
    // 平均视角与在当前帧中的视角差的余弦
    float mTrackViewCos;
    // 防止重复添加
    long unsigned int mnTrackReferenceForFrame;
    // 观测到该点的最后一帧ID
    long unsigned int mnLastFrameSeen;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;    
    cv::Mat mPosGBA;
    long unsigned int mnBAGlobalForKF;


    static std::mutex mGlobalMutex;

protected:    

    // Position in absolute coordinates
    // 该点的世界坐标
    cv::Mat mWorldPos;

    // Keyframes observing the point and associated index in keyframe
    // 观测到该点的关键帧以及在该关键帧中的索引
    std::map<KeyFrame*,size_t> mObservations;

    // Mean viewing direction
    // 该点在所有看到该点的关键帧中的方向的平均值
    cv::Mat mNormalVector;

    // Best descriptor to fast matching，描述子
    cv::Mat mDescriptor;

    // Reference KeyFrame，参考关键帧
    KeyFrame* mpRefKF;

    // Tracking counters
    // visible指该点可以被某一帧观测到（在视野内），但不一定是MapPoint
    // 可以观测到该点的帧数
    int mnVisible;
    // 包含该点作为MapPoint的帧数
    int mnFound;

    // 指明该对象是否可用？Bad flag (we do not currently erase MapPoint from memory)
    bool mbBad;
    // 代替该对象的变量
    MapPoint* mpReplaced;

    // Scale invariance distances
    float mfMinDistance;
    float mfMaxDistance;

    // 地图
    Map* mpMap;

    std::mutex mMutexPos;
    std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
