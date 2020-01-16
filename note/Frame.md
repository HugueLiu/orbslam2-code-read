# Frame类
图像帧，表示每一帧的图像。包括特征点的提取，特征点去畸变及双目匹配计算深度等过程。每一帧包含了左右图像（单目只有左图）的关键点及描述子、当前帧的时间、位姿、帧ID、参考关键帧、图像金字塔的相关参数、相机参数、网格表示等内容。

## 1 `Frame::Frame()` : Frame初始化（构造函数）
双目与RGBD的构造函数基本相同，单目只是缺少了双目匹配及深度计算的过程

~~~C++{.line-numbers}
// 设置frame ID
mnId=nNextId++; 
// 两个线程提取特征点(5.2)
thread threadLeft(&Frame::ExtractORB,this,0,imLeft);
thread threadRight(&Frame::ExtractORB,this,1,imRight);
// 关键点去畸变
UndistortKeyPoints();
// 计算左右图像的关键点匹配和关键点深度(4.2)
ComputeStereoMatches();
// 计算图像边界（只在第一帧计算）
ComputeImageBounds();
// 将特征点分配到网格中，加速特征点匹配
AssignFeaturesToGrid(); 
~~~

## 2 `Frame::ComputeStereoMatches()` : 计算左右图像的关键点匹配和关键点深度

~~~C++{.line-numbers}
// 1. 将右图中关键点按v坐标排列，方便后续与左图关键点匹配
// 2. 对于左图中每个关键点，计算与其对应的右图关键点（Hamming距离最小）
// 3. 以上一步获得的右图关键点坐标为初始值，在其附近通过滑动窗口搜索与左图关键点窗口距离最小的窗口
// 4. 在上一步获得的窗口坐标附近拟合抛物线，并得到最小值，以此值作为右图的对应值
// 5. 通过disparity计算特征点深度
~~~

## 3 `Frame::AssignFeaturesToGrid()` ：将关键点按照网格分配
将图像的关键点按照像素坐标分配到网格中。

~~~c++{.line-numbers}
// 若某个去畸变后的关键点在图像的有效边界内，则将其分配到网格
const cv::KeyPoint &kp = mvKeysUn[i];
if(PosInGrid(kp,nGridPosX,nGridPosY))
    mGrid[nGridPosX][nGridPosY].push_back(i);
~~~

## 4 `Frame::ExtractORB()` ：提取图像的ORB特征点及描述子

~~~c++{.line-numbers}
// 若flag=0，提取左图，否则为右图
if(flag==0)
    (*mpORBextractorLeft)(im,cv::Mat(),mvKeys,mDescriptors);
else
    (*mpORBextractorRight)(im,cv::Mat(),mvKeysRight,mDescriptorsRight);
~~~

## 5 `Frame::SetPose()` ：设置并更新该帧的相机位姿矩阵

~~~c++{.line-numbers}
mTcw = Tcw.clone();
UpdatePoseMatrices();
~~~

## 6 `Frame::UpdatePoseMatrices()` ：更新该帧的相机位姿矩阵

~~~c++{.line-numbers}
// 相机坐标系到世界坐标系的旋转矩阵（即将世界坐标系中的点转换到相机坐标系中）
mRcw = mTcw.rowRange(0,3).colRange(0,3);
// 世界坐标系到相机坐标系的旋转矩阵
mRwc = mRcw.t();
// 相机坐标系到世界坐标系的平移向量
mtcw = mTcw.rowRange(0,3).col(3);
// 相机中心在世界坐标系的坐标（=twc）
// R'(Rp+t)+t'= p
// t' = -R't
mOw = -mRcw.t()*mtcw;
~~~

## 7 `Frame::isInFrustum()` ：判断给定点地图点是否在指定视角范围内
给定一个地图点，比较该点在该帧的视角（即相机原点到该点的向量）与该点在其余观测到该点的关键帧中的视角方向的平均值，若它们之间的的差值小于给定值，则返回true。

~~~c++{.line-numbers}
// 将指定点投影到像素平面，计算u,v，确保该点在图像边界内
const cv::Mat Pc = mRcw*P+mtcw;
// 计算该点在该帧的视角方向
const cv::Mat PO = P-mOw;
// 该点距离相机中心的距离，确保其位于指定阈值内
const float dist = cv::norm(PO);
// 当前视角方向与平均视角方向的余弦值，需要大于给定阈值
const float viewCos = PO.dot(Pn)/dist;
// 设置该点在该帧（最新帧）投影的参数（u,v,ur,level）
~~~

## 8 `Frame::GetFeaturesInArea()` ：获取指定区域内的所有关键点

~~~c++{.line-numbers}
// 通过网格的方式获取
// 1. 首先获取指定区域的网格坐标
// 2. 获取指定网格内的关键点
~~~

## 9 `Frame::PosInGrid()` ：计算给定点的网格位置

~~~c++{.line-numbers}
// 计算给定点的网格坐标
posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);
// 并返回该点是否在图像范围内
// （考虑了关键点可能未去畸变的情况）
~~~

## 10 `Frame::ComputeBoW()` ：计算该帧的词袋表示

~~~c++{.line-numbers}
// 将描述子转换为词袋的表示形式
mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
~~~

## 11 `Frame::UndistortKeyPoints()` ：关键点去畸变

~~~c++{.line-numbers}
// 若不需要去畸变，则去畸变的关键点只是关键点的直接复制
if(mDistCoef.at<float>(0)==0.0){
    mvKeysUn=mvKeys;
}
// 否则对关键点去畸变
~~~

## 12 `Frame::ComputeImageBounds()` ：计算图像边界

~~~c++{.line-numbers}
// 对于未去过畸变的图像，图像边界由原图像的四个角去畸变后的位置得到，
// 但是这里并不会实际进行去畸变操作
// 对于去过畸变的图像，图像边界即为图像的实际边界
~~~