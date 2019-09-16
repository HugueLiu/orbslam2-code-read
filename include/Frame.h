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

#ifndef FRAME_H
#define FRAME_H

#include<vector>

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "ORBextractor.h"

#include <opencv2/opencv.hpp>

namespace ORB_SLAM2
{
// 网格行数
#define FRAME_GRID_ROWS 48
// 网格列数
#define FRAME_GRID_COLS 64

class MapPoint;
class KeyFrame;

class Frame
{
public:
    Frame();

    // Copy constructor. 复制构造函数
    Frame(const Frame &frame);

    // Constructor for stereo cameras. 双目构造函数
    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Constructor for RGB-D cameras. RGB-D构造函数
    Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Constructor for Monocular cameras. 单目构造函数
    Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Extract ORB on the image. 0 for left image and 1 for right image.
    // 提取ORB特征点，并计算描述子（图像金字塔）
    void ExtractORB(int flag, const cv::Mat &im);

    // Compute Bag of Words representation.
    // 将描述子转换为mBowVec,mFeatVec
    void ComputeBoW();

    // Set the camera pose.
    // 设置相机位姿
    void SetPose(cv::Mat Tcw);

    // Computes rotation, translation and camera center matrices from the camera pose.
    // 更新相机旋转，平移矩阵及相机的坐标中心
    void UpdatePoseMatrices();

    // Returns the camera center.
    // 返回相机中心坐标(==twc)
    inline cv::Mat GetCameraCenter(){
        return mOw.clone();
    }

    // Returns inverse of rotation
    // 返回旋转矩阵Rcw的逆(Rwc)
    inline cv::Mat GetRotationInverse(){
        return mRwc.clone();
    }

    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    // MapPoint是否在相机的视野内
    bool isInFrustum(MapPoint* pMP, float viewingCosLimit);

    // Compute the cell of a keypoint (return false if outside the grid)
    // 计算关键点在网格中的位置
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    // 返回给定区域的关键点索引
    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

    // Search a match for each keypoint in the left image to a keypoint in the right image.
    // If there is a match, depth is computed and the right coordinate associated to the left keypoint is stored.
    // 计算右图中与左图关键点匹配的点，并计算深度
    void ComputeStereoMatches();

    // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
    void ComputeStereoFromRGBD(const cv::Mat &imDepth);

    // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
    // 将关键点反投影到世界坐标系
    cv::Mat UnprojectStereo(const int &i);

public:
    // Vocabulary used for relocalization.
    // 词典，用于重定位
    ORBVocabulary* mpORBvocabulary;

    // Feature extractor. The right is used only in the stereo case.
    // ORB特征提取器
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;

    // Frame timestamp.
    // 该Frame的时间戳
    double mTimeStamp;

    // Calibration matrix and OpenCV distortion parameters.
    // 相机内参和去畸变参数
    cv::Mat mK;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
    cv::Mat mDistCoef;

    // Stereo baseline multiplied by fx.
    // 基线长度*fx
    float mbf;

    // Stereo baseline in meters.
    // 双目基线长度
    float mb;

    // Threshold close/far points. Close points are inserted from 1 view.
    // Far points are inserted as in the monocular case from 2 views.
    // 远近点阈值
    float mThDepth;

    // Number of KeyPoints.
    // 左图关键点个数
    int N;

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    // In the stereo case, mvKeysUn is redundant as images must be rectified.
    // In the RGB-D case, RGB images can be distorted.
    // 左图、右图关键点，均为在原图中的坐标
    std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
    // 去畸变后的关键点（左图关键点）（实际计算使用）
    std::vector<cv::KeyPoint> mvKeysUn;

    // Corresponding stereo coordinate and depth for each keypoint.
    // "Monocular" keypoints have a negative value.
    // 左图中每个关键点对应的右图关键点的u
    std::vector<float> mvuRight;
    // 左图中每个关键点对应的深度
    std::vector<float> mvDepth;

    // Bag of Words Vector structures.
    // 该帧图像的词袋表示
    DBoW2::BowVector mBowVec;
    // 该帧图像中每个特征点在vocabulary tree上指定层(L-levelsup)上对应的的node的Id(一个node可以对应多个节点)
    DBoW2::FeatureVector mFeatVec;

    // ORB descriptor, each row associated to a keypoint.
    // ORB描述子，每行表示一个点
    cv::Mat mDescriptors, mDescriptorsRight;

    // MapPoints associated to keypoints, NULL pointer if no association.
    // 该帧包含的地图点
    std::vector<MapPoint*> mvpMapPoints;

    // Flag to identify outlier associations.
    std::vector<bool> mvbOutlier;

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    // 每个网格宽度的倒数
    static float mfGridElementWidthInv;
    // 每个网格宽度的倒数
    static float mfGridElementHeightInv;
    // 网格
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    // Camera pose. 相机位姿，变换矩阵（4*4）
    // 相机坐标系到世界坐标系的变换， 世界坐标系点到相机坐标系点的变换
    cv::Mat mTcw;

    // Current and Next Frame id.
    // 上一帧ID
    static long unsigned int nNextId;
    // 当前帧ID
    long unsigned int mnId;

    // Reference Keyframe.
    KeyFrame* mpReferenceKF;

    // Scale pyramid info.
    // 金字塔层数
    int mnScaleLevels;
    // 缩放因子
    float mfScaleFactor;
    // 缩放因子的岁数
    float mfLogScaleFactor;
    // 每一层的缩放因子
    vector<float> mvScaleFactors;
    // 每一层的缩放因子的倒数
    vector<float> mvInvScaleFactors;
    // 每一层的缩放因数的平方
    vector<float> mvLevelSigma2;
    // 每一层的缩放因子的倒数的平方
    vector<float> mvInvLevelSigma2;

    // Undistorted Image Bounds (computed once).
    // 图像边界
    static float mnMinX;
    static float mnMaxX;
    static float mnMinY;
    static float mnMaxY;

    // 是否是第一次计算（第一帧）或改变了内参
    static bool mbInitialComputations;


private:

    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
    // 关键点去畸变，只对RGDB有效
    void UndistortKeyPoints();

    // Computes image bounds for the undistorted image (called in the constructor).
    // 计算图像边界（只在构造函数中计算一次）
    void ComputeImageBounds(const cv::Mat &imLeft);

    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    // 将关键点分配到网格中
    void AssignFeaturesToGrid();

    // Rotation, translation and camera center
    // 相机位姿的旋转矩阵（3*3）
    cv::Mat mRcw;
    // 相机位姿的旋转矩阵的转置（3*3），旋转矩阵是正交矩阵，转置等于逆
    cv::Mat mRwc;
    // 相机位姿的平移矩阵（3*1）
    cv::Mat mtcw;
    // 左相机中心在世界坐标系的位置
    cv::Mat mOw; //==mtwc
};

}// namespace ORB_SLAM

#endif // FRAME_H
