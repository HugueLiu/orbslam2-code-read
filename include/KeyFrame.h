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

#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"

#include <mutex>


namespace ORB_SLAM2
{

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;

class KeyFrame
{
public:
    // 使用Frame创建关键帧
    KeyFrame(Frame &F, Map* pMap, KeyFrameDatabase* pKFDB);

    // Pose functions
    // 设置当前关键帧位姿
    void SetPose(const cv::Mat &Tcw);
    // 返回当前关键帧位姿(Tcw)
    cv::Mat GetPose();
    // 返回当前关键帧位姿(Twc)
    cv::Mat GetPoseInverse();
    // 返回相机中心坐标
    cv::Mat GetCameraCenter();
    // 返回基线中心坐标
    cv::Mat GetStereoCenter();
    // 返回旋转矩阵
    cv::Mat GetRotation();
    // 返回平移矩阵
    cv::Mat GetTranslation();

    // Bag of Words Representation
    void ComputeBoW();

    // Covisibility graph functions
    void AddConnection(KeyFrame* pKF, const int &weight);
    void EraseConnection(KeyFrame* pKF);
    void UpdateConnections();
    void UpdateBestCovisibles();
    std::set<KeyFrame *> GetConnectedKeyFrames();
    std::vector<KeyFrame* > GetVectorCovisibleKeyFrames();
    // 返回最多N个关联关键帧
    std::vector<KeyFrame*> GetBestCovisibilityKeyFrames(const int &N);
    std::vector<KeyFrame*> GetCovisiblesByWeight(const int &w);
    int GetWeight(KeyFrame* pKF);

    // Spanning tree functions
    void AddChild(KeyFrame* pKF);
    void EraseChild(KeyFrame* pKF);
    void ChangeParent(KeyFrame* pKF);
    std::set<KeyFrame*> GetChilds();
    KeyFrame* GetParent();
    bool hasChild(KeyFrame* pKF);

    // Loop Edges
    void AddLoopEdge(KeyFrame* pKF);
    std::set<KeyFrame*> GetLoopEdges();

    // MapPoint observation functions
    // 添加MapPoint
    void AddMapPoint(MapPoint* pMP, const size_t &idx);
    // 清除指定索引的MapPoint
    void EraseMapPointMatch(const size_t &idx);
    // 清除指定MapPoint
    void EraseMapPointMatch(MapPoint* pMP);
    // 修改指定索引的MapPoint
    void ReplaceMapPointMatch(const size_t &idx, MapPoint* pMP);
    // 返回所有“好”的MapPoint
    std::set<MapPoint*> GetMapPoints();
    // 返回所有的MapPoint
    std::vector<MapPoint*> GetMapPointMatches();
    // 返回追踪的MapPoint数量，该MapPoint必须至少存在于minObs个关键帧中
    int TrackedMapPoints(const int &minObs);
    // 返回指定索引的MapPoint
    MapPoint* GetMapPoint(const size_t &idx);

    // KeyPoint functions
    std::vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r) const;
    cv::Mat UnprojectStereo(int i);

    // Image
    bool IsInImage(const float &x, const float &y) const;

    // Enable/Disable bad flag changes
    void SetNotErase();
    void SetErase();

    // Set/check bad flag
    void SetBadFlag();
    bool isBad();

    // Compute Scene Depth (q=2 median). Used in monocular.
    float ComputeSceneMedianDepth(const int q);

    static bool weightComp( int a, int b){
        return a>b;
    }

    static bool lId(KeyFrame* pKF1, KeyFrame* pKF2){
        return pKF1->mnId<pKF2->mnId;
    }


    // The following variables are accesed from only 1 thread or never change (no mutex needed).
public:

    // 下一关键帧帧ID
    static long unsigned int nNextId;
    // 当前关键帧ID
    long unsigned int mnId;
    // 当前关键帧的FrameID
    const long unsigned int mnFrameId;

    // 该帧时间戳
    const double mTimeStamp;

    // Grid (to speed up feature matching)
    // 网格列数
    const int mnGridCols;
    // 网格行数
    const int mnGridRows;
    // 每个网格宽度的倒数
    const float mfGridElementWidthInv;
    // 每个网格高度的倒数
    const float mfGridElementHeightInv;

    // Variables used by the tracking
    // 防止局部关键帧的重复添加
    long unsigned int mnTrackReferenceForFrame;
    long unsigned int mnFuseTargetForKF;

    // Variables used by the local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnBAFixedForKF;

    // Variables used by the keyframe database
    long unsigned int mnLoopQuery;
    int mnLoopWords;
    float mLoopScore;
    // 重定位是该关键帧是否第一次检索到
    long unsigned int mnRelocQuery;
    // 该关键帧与查找帧共同的word个数
    int mnRelocWords;
    // 该关键帧与查找帧的匹配分数
    float mRelocScore;

    // Variables used by loop closing
    cv::Mat mTcwGBA;
    cv::Mat mTcwBefGBA;
    long unsigned int mnBAGlobalForKF;

    // Calibration parameters
    // 内参，基线长度，远近点阈值
    const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

    // Number of KeyPoints
    // 关键点个数
    const int N;

    // KeyPoints, stereo coordinate and descriptors (all associated by an index)
    // 该关键帧的特征点
    const std::vector<cv::KeyPoint> mvKeys;
    // 该关键帧矫正后的特征点
    const std::vector<cv::KeyPoint> mvKeysUn;
    // 双目中右侧相机的水平坐标u，单目为负值
    const std::vector<float> mvuRight;
    // 关键点的深度，单目为负值
    const std::vector<float> mvDepth;
    // 描述子
    const cv::Mat mDescriptors;

    //BoW
    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    // Pose relative to parent (this is computed when bad flag is activated)
    cv::Mat mTcp;

    // Scale
    // 金字塔层数
    const int mnScaleLevels;
    // 缩放因子
    const float mfScaleFactor;
    // 缩放因子的对数
    const float mfLogScaleFactor;
    // 每一层的缩放因子
    const std::vector<float> mvScaleFactors;
    // 每一层的缩放因子的平方
    const std::vector<float> mvLevelSigma2;
    // 每一层的缩放因子的平方的倒数
    const std::vector<float> mvInvLevelSigma2;

    // Image bounds and calibration
    // 图像边界
    const int mnMinX;
    const int mnMinY;
    const int mnMaxX;
    const int mnMaxY;
    // 内参
    const cv::Mat mK;


    // The following variables need to be accessed trough a mutex to be thread safe.
protected:

    // SE3 Pose and camera center
    // 相机位姿
    cv::Mat Tcw;
    // 相机位姿的逆
    cv::Mat Twc;
    // 相机中心坐标
    cv::Mat Ow;

    // 两相机基线中心的世界坐标
    cv::Mat Cw; // Stereo middel point. Only for visualization

    // MapPoints associated to keypoints
    std::vector<MapPoint*> mvpMapPoints;

    // BoW
    KeyFrameDatabase* mpKeyFrameDB;
    ORBVocabulary* mpORBvocabulary;

    // Grid over the image to speed up feature matching
    std::vector< std::vector <std::vector<size_t> > > mGrid;

    std::map<KeyFrame*,int> mConnectedKeyFrameWeights;
    std::vector<KeyFrame*> mvpOrderedConnectedKeyFrames;
    std::vector<int> mvOrderedWeights;

    // Spanning Tree and Loop Edges
    bool mbFirstConnection;
    KeyFrame* mpParent;
    std::set<KeyFrame*> mspChildrens;
    std::set<KeyFrame*> mspLoopEdges;

    // Bad flags
    bool mbNotErase;
    bool mbToBeErased;
    bool mbBad;    

    // 基线长度的一半
    float mHalfBaseline; // Only for visualization

    // 地图
    Map* mpMap;

    std::mutex mMutexPose;
    std::mutex mMutexConnections;
    std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // KEYFRAME_H
