# ORB-SLAM2代码解读

## 1. 总述

ORB-SLAM2主要由3个模块组成，分别为Tracking, Mapping和Loop closing. 如下图所示：![orbslam2](orbslam2.jpg)
每个模块对应一个线程，另外还有viewer线程用于显示，执行完Loop closing后还会开启一个全局BA线程。

类|文件|描述
--|--|--
Tracking|`Tracking.cc`|追踪线程，运动优化
LocalMapping|`LocalMapping.cc`|局部地图管理
LoopClosing|`LoopClosing.cc`|回环检测
System|`System.cc`|系统入口，管理所有线程
Map|`Map.cc`|地图类
MapPoint|`MapPoint.cc`|地图点类
Frame|`Frame.cc`|帧类
KeyFrame|`KeyFrame.cc`|关键帧类
viewer|`viewer.cc`|可视化
FrameDrawer|`FrameDrawer.cc`|
MapDrawer|`MapDrawer.cc`|
ORBextractor|`ORBextractor.cc`|提取ORB特征点

## 2. stereo代码解读

### 1. `stereo_kitti.cc` : 程序入口

```C++
LoadImages(); // 加载左右图像的路径及其时间戳
ORB_SLAM2::System SLAM(); // 初始化SLAM系统对象(2.1)
for(int ni=0; ni<nImages; ni++){ // 循环读取图像
    SLAM.TrackStereo(imLeft,imRight,tframe); // 把图像传递给SLAM系统对象(2.2)
}
SLAM.Shutdown(); // 停止所有线程
SLAM.SaveTrajectoryKITTI(); // 保存轨迹
```

### 2.1 `System.cc` : SLAM系统的初始化

```C++
mpVocabulary = new ORBVocabulary();
mpVocabulary->loadFromTextFile(strVocFile); //加载ORB词典
mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary); // 创建关键帧数据库
mpMap = new Map(); // 创建地图
mpFrameDrawer = new FrameDrawer(mpMap);
mpMapDrawer = new MapDrawer(mpMap, strSettingsFile); // 创建可视化对象

mpTracker = new Tracking(); // 创建Tracking对象(3.1)

mpLocalMapper = new LocalMapping();
mptLocalMapping = new thread(); // 创建LocalMapping对象并启动线程

mpLoopCloser = new LoopClosing();
mptLoopClosing = new thread(); // 创建LoopClosing对象并启动线程

mpViewer = new Viewer();
mptViewer = new thread(); // 创建Viewer对象并启动线程

// 设置线程之间的指针
```

### 2.2 `System::TrackStereo()` : Tracking线程（主线程）

```C++
// 判断是否更改模式（定位，定位与建图）
// 判断是否重置
cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft,imRight,timestamp); // 输入图像，获得变换位姿(3.2)
```

### 3.1 `Tracking::Tracking()` : Tracking初始化 

```C++
// 加载相机参数（内参、去畸变参数、基线、帧率、ORB参数等）
mpORBextractorLeft = new ORBextractor();
mpORBextractorRight = new ORBextractor(); // 初始化左右图像的ORB特征提取器(5.1)
mThDepth = mbf*(float)fSettings["ThDepth"]/fx; // 设置远近点阈值
```

### 3.2 `Tracking::GrabImageStereo()` : 输入图像处理  

```C++
cvtColor(mImGray,mImGray,CV_RGB2GRAY);
cvtColor(imGrayRight,imGrayRight,CV_RGB2GRAY);  //图像转换为灰度图
mCurrentFrame = Frame(); //构造Frame(4.1)

Track(); // (3.3)
```

### 3.3 `Tracking::Track()` : Tracking线程（主线程）

```C++
StereoInitialization(); // 第一帧初始化(3.4)
mpFrameDrawer->Update(this); // 更新FrameDrawer

if(!mbOnlyTracking){ // 定位与建图模式
    if(mState==OK){
        // 检查上一帧中地图点是否被标注为替换，若有则真正执行替换操作
        CheckReplacedInLastFrame();
        // 若距离近，通过参考关键帧來Tracking(3.5)
        bOK = TrackReferenceKeyFrame(); 
        // 若有运动模型，通过运动模型來Tracking(3.6)
        bOK = TrackWithMotionModel();
    }else{ // 丢失，重新定位
        bOK = Relocalization();
    }
}else{ // 定位模式
    if(mState==LOST){
        bOK = Relocalization(); // 追踪丢失，重新定位
    }else{
        if(!mbVO){
            bOK = TrackWithMotionModel();
            bOK = TrackReferenceKeyFrame();
        }else{
            bOKReloc = Relocalization();
        }
    }
}
```

### 3.4 `Tracking::StereoInitialization()` : 双目、RGBD初始化

```C++
if(mCurrentFrame.N>500)  // 关键点数量必须大于500
    mCurrentFrame.SetPose(); // 设置当前帧为位姿为坐标原点
    KeyFrame* pKFini = new KeyFrame(); // 创建初始关键帧，并将该帧插入到地图中(6.1)
    // 对于深度大于0的关键点
    MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpMap); // 创建MapPoint
    pNewMP->AddObservation(pKFini,i); // 该关键帧可以观测到该点
    pKFini->AddMapPoint(pNewMP,i); // 向该关键帧添加地图点
    pNewMP->ComputeDistinctiveDescriptors(); // 为该点选择一个“最好”的描述子
    pNewMP->UpdateNormalAndDepth(); // 更新该点的平均方向
    mpMap->AddMapPoint(pNewMP); // 向地图中添加点

    mpLocalMapper->InsertKeyFrame(pKFini); // 向局部建图线程中插入一个关键帧
```

### 3.5 `Tracking::TrackReferenceKeyFrame()` : 通过参考关键帧进行Tracking

```C++
// 计算当前帧的词袋表示
mCurrentFrame.ComputeBoW();
// 计算当前帧与参考关键帧的关键点匹配
matcher.SearchByBoW(mpReferenceKF,mCurrentFrame); 
// 将上一帧位姿的位姿作为该帧的初始位姿
mCurrentFrame.SetPose(mLastFrame.mTcw); 
// 位姿优化
Optimizer::PoseOptimization(&mCurrentFrame); 
// 舍弃outlier点
```

### 3.6 `Tracking::TrackWithMotionModel()` : 通过运动模型进行Tracking

```C++
// 更新上一帧位姿
UpdateLastFrame();
// 根据上一帧的运动设置初始值
mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);
// 通过投影匹配前后帧的关键点
matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,);
// 位姿优化
Optimizer::PoseOptimization(&mCurrentFrame);
// 对齐outlier点
```

### 3.7 `Tracking::Relocalization()` : 重定位

```C++
// 计算当前帧的词袋表示
 mCurrentFrame.ComputeBoW();
```

### 4.1 `Frame.cc` : Frame初始化（构造Frame）

```C++
mnId=nNextId++; // 设置frame ID

thread threadLeft(&Frame::ExtractORB,this,0,imLeft);
thread threadRight(&Frame::ExtractORB,this,1,imRight); // 提取特征点(5.2)
UndistortKeyPoints();   // 关键点去畸变， 只适用于RGBD图像
ComputeStereoMatches(); // 计算左右图像的关键点匹配和关键点深度(4.2)
ComputeImageBounds();   // 计算图像边界（只在第一帧计算）
AssignFeaturesToGrid(); // 将特征点分配到网格中，加速特征点匹配
```

### 4.2 `Frame.cc::ComputeStereoMatches()` : 计算左右图像的关键点匹配和关键点深度

```C++
// 1. 将右图中关键点按v坐标排列，方便后续与左图关键点匹配
// 2. 对于左图中每个关键点，计算与其对应的右图关键点（Hamming距离最小）
// 3. 以上一步获得的右图关键点坐标为初始值，在其附近通过滑动窗口搜索与左图关键点窗口距离最小的窗口
// 4. 在上一步获得的窗口坐标附近拟合抛物线，并得到最小值，以此值作为右图的对应值
// 5. 通过disparity计算特征点深度
```

### 5.1 `ORBextractor.cc` : ORBextractor初始化

```C++
mvScaleFactor[i]=mvScaleFactor[i-1]*scaleFactor; // 每层的缩放因子
mvLevelSigma2[i]=mvScaleFactor[i]*mvScaleFactor[i]; // 每层的缩放因子平方
mvInvScaleFactor[i]=1.0f/mvScaleFactor[i];
mvInvLevelSigma2[i]=1.0f/mvLevelSigma2[i];

mnFeaturesPerLevel[level] = cvRound(nDesiredFeaturesPerScale); // 每层预期的特征点个数

// 计算特征点方向准备
umax.resize(HALF_PATCH_SIZE + 1);
// 计算每个v坐标对应的最大u坐标
int v, v0, vmax = cvFloor(HALF_PATCH_SIZE * sqrt(2.f) / 2 + 1); // 11
int vmin = cvCeil(HALF_PATCH_SIZE * sqrt(2.f) / 2); // 11
const double hp2 = HALF_PATCH_SIZE*HALF_PATCH_SIZE;
// 计算前半段
for (v = 0; v <= vmax; ++v)
    umax[v] = cvRound(sqrt(hp2 - v * v));

// 后半段与前半段对称，确保能组成一个圆
for (v = HALF_PATCH_SIZE, v0 = 0; v >= vmin; --v){
    // 前半段的u相同对应为后半段的v相同，而一个v只能对应一个u
    while (umax[v0] == umax[v0 + 1])
        ++v0;
    umax[v] = v0;
    ++v0;
}
```

![orbslam2](umax.png)

### 5.2 `ORBextractor::operator()()` : 检测关键点及描述子

```C++
ComputePyramid(image);  // 构建图像金字塔(5.3)
ComputeKeyPointsOctTree(); // 关键点计算(以四叉树表示)(5.4)
GaussianBlur(); // 对图像进行高斯滤波
computeDescriptors(); // 计算描述子
```

### 5.3 `ORBextractor::ComputePyramid(cv::Mat image)` : 构建图像金字塔

```C++
// 计算每一层图像
for (int level = 0; level < nlevels; ++level)
    mvImagePyramid[level] = temp(Rect(EDGE_THRESHOLD, EDGE_THRESHOLD, sz.width, sz.height));

    // Compute the resized image
    if( level != 0 ){
        resize(mvImagePyramid[level-1], mvImagePyramid[level], sz, 0, 0, INTER_LINEAR);
    }else{
        copyMakeBorder(image, temp, EDGE_THRESHOLD, EDGE_THRESHOLD,
        EDGE_THRESHOLD, EDGE_THRESHOLD, BORDER_REFLECT_101);
    }
```

### 5.4 `ORBextractor::ComputeKeyPointsOctTree()` : 关键点计算(以四叉树表示)

```C++
for (int level = 0; level < nlevels; ++level)
    // 将图像去除边缘后，按照30的边长划分cell，相邻两个cell有6行/列坐标的重叠
    FAST(); // 在每一层,提取每个cell的FAST特征点
    keypoints = DistributeOctTree(); // 将关键点分配到四叉树节点中(5.5)
    computeOrientation(); // 计算每个特征点的方向
```

### 5.5 `ORBextractor::DistributeOctTree()` : 以四叉数的形式表示关键点，按照坐标将关键点划分到不同的节点中，每个节点只包含一个响应最大的关键点

```C++
// 按照图像纵横比计算初始节点
const int nIni = round(static_cast<float>(maxX-minX)/(maxY-minY));
// 把图像关键点按位置分配到初始节点中
vpIniNodes[kp.pt.x/hX]->vKeys.push_back(kp);
// 把每个节点平均分为4个节点，并重新分配关键点，直至节点达到预期个数或
// 所有节点都只含一个关键点，不可再分
lit->DivideNode(n1,n2,n3,n4);
// 所有节点只保留一个响应最大的关键点，得到最终关键点
if(vNodeKeys[k].response>maxResponse)
    pKP = &vNodeKeys[k];
    maxResponse = vNodeKeys[k].response;
```

### 6.1 `KeyFrame::KeyFrame()` : 通过Frame创建关键帧

```C++
// 1. 复制Frame的相关参数到该关键帧
// 2. 设置该关键帧位姿
```
