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

#ifndef ORBEXTRACTOR_H
#define ORBEXTRACTOR_H

#include <vector>
#include <list>
#include <opencv/cv.h>


namespace ORB_SLAM2
{

class ExtractorNode
{
public:
    ExtractorNode():bNoMore(false){}

    // 将该节点分为平均分为4个节点，并根据坐标位置将关键点分配到四个节点中
    void DivideNode(ExtractorNode &n1, ExtractorNode &n2, ExtractorNode &n3, ExtractorNode &n4);

    // 关键点
    std::vector<cv::KeyPoint> vKeys;
    // 左上，右上，左下，右下四个点
    cv::Point2i UL, UR, BL, BR;
    // 一个指向自身的迭代器，用于后续的erase操作
    std::list<ExtractorNode>::iterator lit;
    // 该节点是否只有一个关键点
    bool bNoMore;
};

class ORBextractor
{
public:
    
    enum {HARRIS_SCORE=0, FAST_SCORE=1 };

    ORBextractor(int nfeatures, float scaleFactor, int nlevels,
                 int iniThFAST, int minThFAST);

    ~ORBextractor(){}

    // Compute the ORB features and descriptors on an image.
    // ORB are dispersed on the image using an octree.
    // Mask is ignored in the current implementation.
    // 计算ORB特征点及描述子（四叉树）
    void operator()( cv::InputArray image, cv::InputArray mask,
      std::vector<cv::KeyPoint>& keypoints,
      cv::OutputArray descriptors);

    // 返回金字塔层数
    int inline GetLevels(){
        return nlevels;
    }

    // 返回缩放因子
    float inline GetScaleFactor(){
        return scaleFactor;
    }

    // 返回金字塔每层的缩放因子
    std::vector<float> inline GetScaleFactors(){
        return mvScaleFactor;
    }

    // 返回金字塔每层的缩放因子的倒数
    std::vector<float> inline GetInverseScaleFactors(){
        return mvInvScaleFactor;
    }

    // 返回金字塔每层的缩放因子的平方
    std::vector<float> inline GetScaleSigmaSquares(){
        return mvLevelSigma2;
    }

    // 返回金字塔每层的缩放因子的平方的倒数
    std::vector<float> inline GetInverseScaleSigmaSquares(){
        return mvInvLevelSigma2;
    }
    // 图像金字塔
    std::vector<cv::Mat> mvImagePyramid;

protected:

    // 计算图像金字塔
    void ComputePyramid(cv::Mat image);
    // 关键点计算(以四叉树表示)，每个节点值保存一个关键点
    void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);    
    // 将关键点分配到四叉树中
    std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint>& vToDistributeKeys, const int &minX,
                                           const int &maxX, const int &minY, const int &maxY, const int &nFeatures, const int &level);
    // 未使用
    void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> >& allKeypoints);
    // 计算BRIEF描述子时点的选择方式
    std::vector<cv::Point> pattern;

    // 总特征点个数
    int nfeatures;
    // 金字塔缩放因子
    double scaleFactor;
    // 金字塔层数
    int nlevels;
    // FAST特征点检测时的初始阈值，若该阈值提取不到特征点，则使用minThFAST
    int iniThFAST;
    // FAST特征点检测时的最小阈值
    int minThFAST;

    // 金字塔每层的特征点个数
    std::vector<int> mnFeaturesPerLevel;

    // 计算特征点时每个v对应的最大的u
    std::vector<int> umax;

    // 每一层的缩放因子(相对于原始图像)
    std::vector<float> mvScaleFactor;
    // 每一层的缩放因子的倒数
    std::vector<float> mvInvScaleFactor;    
    // 每一层的缩放因子的平方
    std::vector<float> mvLevelSigma2;
    // 每一层的缩放因子的平方的倒数
    std::vector<float> mvInvLevelSigma2;
};

} //namespace ORB_SLAM

#endif

