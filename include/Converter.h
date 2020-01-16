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

#ifndef CONVERTER_H
#define CONVERTER_H

#include<opencv2/core/core.hpp>

#include<Eigen/Dense>
#include"Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include"Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace ORB_SLAM2
{

class Converter
{
public:
    // 把Mat类型的描述子转换成Vector
    // 每一行作为vector的一个元素，为一个描述子
    static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);

    // 将cv::mat表示的变换矩阵转换为g2o::SE3Quat的形式，包括一个四元数表示的旋转和一个平移变量
    static g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);
    // 未实现，sim3转换为se3必然会丢失缩放因子
    static g2o::SE3Quat toSE3Quat(const g2o::Sim3 &gSim3);

    // 将g2o::SE3Quat形式的位姿表示转换为4x4的cv::mat
    static cv::Mat toCvMat(const g2o::SE3Quat &SE3);
    // 将g2o::Sim3形式的位姿表示转换为4x4的cv::mat
    static cv::Mat toCvMat(const g2o::Sim3 &Sim3);
    // 将4x4的eigen矩阵转换为4x4的cv::mat
    static cv::Mat toCvMat(const Eigen::Matrix<double,4,4> &m);
    // 将3x3的eigen矩阵转换为3x3的cv::mat
    static cv::Mat toCvMat(const Eigen::Matrix3d &m);
    // 将3x1的eigen矩阵转换为3x1的cv::mat
    static cv::Mat toCvMat(const Eigen::Matrix<double,3,1> &m);
    // 将R t转换为4x4的cv::mat
    static cv::Mat toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t);

    // 将一个cv::mat类型的点转换为3x1的eigen向量
    static Eigen::Matrix<double,3,1> toVector3d(const cv::Mat &cvVector);
    // 将一个cv::Point3f类型的点转换为3x1的eigen向量
    static Eigen::Matrix<double,3,1> toVector3d(const cv::Point3f &cvPoint);
    // 将3x3的cv::mat转换为3x3的eigen矩阵
    static Eigen::Matrix<double,3,3> toMatrix3d(const cv::Mat &cvMat3);
    // 将cv::mat类型的旋转矩阵转换为vector表示的四元数（x,y,z,w）
    static std::vector<float> toQuaternion(const cv::Mat &M);
};

}// namespace ORB_SLAM

#endif // CONVERTER_H
