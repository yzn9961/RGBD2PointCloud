#include <opencv2/opencv.hpp>
//#include<opencv2/highgui/highgui.hpp>
#include <vector>
#include <string>
#include <Eigen/Core>
#include <pangolin/pangolin.h>
#include <unistd.h>

using namespace std;
using namespace Eigen;
using namespace cv;

//string left_file = "../StereoImage/left/L_10.jpg";
//string right_file = "../StereoImage/right/R_10.jpg";
string parameterFile = "./mydepth.yaml";


//畸变校正所需参数
double cx,cy,fx,fy;//2D->3D所需参数
Mat K,D;//畸变校正所需参数
typedef Matrix<double,6,1> Vector6d;
int cols_l=640;
int rows_l=480;
void Getparameter()
{
    cv::FileStorage fsSettings(parameterFile, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
    }

    fsSettings["K"] >> K;
    fsSettings["D"] >> D;

    fsSettings["cx"] >> cx;
    fsSettings["cy"] >> cy;

    fsSettings["fx"] >> fx;
    fsSettings["fy"] >> fy;

    if(K.empty() || D.empty() || fy==0 || fx==0 || cx==0 || cy==0)
    {
        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
    }
}

Mat imgUndistort(Mat origin)
{   Mat dst;
    Mat NewCamMatrix = getOptimalNewCameraMatrix(K,D,origin.size(),1,origin.size(),0);
    undistort(origin,dst,K,D,NewCamMatrix);
    return dst;
}

Mat show_stereoCalib(Mat rectifyImageL,Mat rectifyImageR) //展示双目立体标定结果
{
    Mat canvas;
    //cvtColor(canvas,canvas,COLOR_BGR2GRAY);
    double sf;
    int w, h;
    sf = 600. / MAX(cv::Size(cols_l,rows_l).width, cv::Size(cols_l,rows_l).height);
    w = cvRound(cv::Size(cols_l,rows_l).width * sf);
    h = cvRound(cv::Size(cols_l,rows_l).height * sf);
    canvas.create(h, w * 2, CV_8UC3);
    //左图像画到画布上
    Mat canvasPart = canvas(Rect(w * 0, 0, w, h));                                //得到画布的一部分
    resize(rectifyImageL, canvasPart, canvasPart.size(), 0, 0, INTER_AREA);     //把图像缩放到跟canvasPart一样大小
    //Rect vroiL(cvRound(validROIL.x*sf), cvRound(validROIL.y*sf),                //获得被截取的区域
        //cvRound(validROIL.width*sf), cvRound(validROIL.height*sf));
    //rectangle(canvasPart, vroiL, Scalar(0, 0, 255), 3, 8);                      //画上一个矩形
    //cout << "Painted ImageL" << endl;

    //右图像画到画布上
    canvasPart = canvas(Rect(w, 0, w, h));                                      //获得画布的另一部分
    resize(rectifyImageR, canvasPart, canvasPart.size(), 0, 0, INTER_LINEAR);
    //Rect vroiR(cvRound(validROIR.x * sf), cvRound(validROIR.y*sf),
        //cvRound(validROIR.width * sf), cvRound(validROIR.height * sf));
    //rectangle(canvasPart, vroiR, Scalar(0, 0, 255), 3, 8);
    //cout << "Painted ImageR" << endl;

    //画上对应的线条
    for (int i = 0; i < canvas.rows; i += 16)
        line(canvas, Point(0, i), Point(canvas.cols, i), Scalar(0, 255, 0), 1, 8);
    //imshow("rectified", canvas);
    return canvas;
}
//点云绘制
void showPointCloud(
    const vector<Vector6d, Eigen::aligned_allocator<Vector6d>>&pointcloud);

// 空洞填充
void insertDepth32f(cv::Mat& depth)
{
    const int width = depth.cols;
    const int height = depth.rows;
    float* data = (float*)depth.data;
    cv::Mat integralMap = cv::Mat::zeros(height, width, CV_64F);
    cv::Mat ptsMap = cv::Mat::zeros(height, width, CV_32S);
    double* integral = (double*)integralMap.data;
    int* ptsIntegral = (int*)ptsMap.data;
    memset(integral, 0, sizeof(double) * width * height);
    memset(ptsIntegral, 0, sizeof(int) * width * height);
    for (int i = 0; i < height; ++i)
    {
        int id1 = i * width;
        for (int j = 0; j < width; ++j)
        {
            int id2 = id1 + j;
            if (data[id2] > 1e-3)
            {
                integral[id2] = data[id2];
                ptsIntegral[id2] = 1;
            }
        }
    }
    // 积分区间
    for (int i = 0; i < height; ++i)
    {
        int id1 = i * width;
        for (int j = 1; j < width; ++j)
        {
            int id2 = id1 + j;
            integral[id2] += integral[id2 - 1];
            ptsIntegral[id2] += ptsIntegral[id2 - 1];
        }
    }
    for (int i = 1; i < height; ++i)
    {
        int id1 = i * width;
        for (int j = 0; j < width; ++j)
        {
            int id2 = id1 + j;
            integral[id2] += integral[id2 - width];
            ptsIntegral[id2] += ptsIntegral[id2 - width];
        }
    }
    int wnd;
    double dWnd = 2;
    while (dWnd > 1)
    {
        wnd = int(dWnd);
        dWnd /= 2;
        for (int i = 0; i < height; ++i)
        {
            int id1 = i * width;
            for (int j = 0; j < width; ++j)
            {
                int id2 = id1 + j;
                int left = j - wnd - 1;
                int right = j + wnd;
                int top = i - wnd - 1;
                int bot = i + wnd;
                left = max(0, left);
                right = min(right, width - 1);
                top = max(0, top);
                bot = min(bot, height - 1);
                int dx = right - left;
                int dy = (bot - top) * width;
                int idLeftTop = top * width + left;
                int idRightTop = idLeftTop + dx;
                int idLeftBot = idLeftTop + dy;
                int idRightBot = idLeftBot + dx;
                int ptsCnt = ptsIntegral[idRightBot] + ptsIntegral[idLeftTop] - (ptsIntegral[idLeftBot] + ptsIntegral[idRightTop]);
                double sumGray = integral[idRightBot] + integral[idLeftTop] - (integral[idLeftBot] + integral[idRightTop]);
                if (ptsCnt <= 0)
                {
                    continue;
                }
                data[id2] = float(sumGray / ptsCnt);
            }
        }
        int s = wnd / 2 * 2 + 1;
        if (s > 201)
        {
            s = 201;
        }
        cv::GaussianBlur(depth, depth, cv::Size(s, s), s, s);
    }
}
int main(int argc,char**argv)
{
    if(argc != 3)
    {
        cerr << endl << "ERROR:格式错误！正确用法 ./DepthMap RGB图像路径 深度图像路径" << endl;
        return 1;
    }
    //// 读取相机参数 ////
    //double fx,fy,cx,cy;
    //double b;
    //读取畸变矫正参数
    Getparameter();
    //// 读取图像 ////
    cv::Mat RGB_src = imread(argv[1],IMREAD_UNCHANGED);//读取灰度图像 格式为CV_8UC1: 8深度 无符号0~255 Channel=1
    cv::Mat Depth_src = imread(argv[2],IMREAD_UNCHANGED);//CV_8UC1

    /// 畸变校正 ///
    Mat rectifyRGB,rectifyDepth,rectifyImageL_gray,rectifyImageR_gray;
    //rectifyRGB = imgUndistort(RGB_src);//RGB畸变矫正
    //rectifyDepth = imgUndistort(Depth_src);//深度畸变矫正
    rectifyRGB = RGB_src;
    rectifyDepth = Depth_src;
    Mat d8, dColor;
    Depth_src.convertTo(d8, CV_8U, 255.0 / 2500);
    applyColorMap(d8, dColor, COLORMAP_OCEAN);
    imshow("Depth (colored)", dColor);
    imshow("RGB",rectifyRGB);
    //// 生成点云 ////
    vector<Vector6d, Eigen::aligned_allocator<Vector6d>> pointcloud;
    pointcloud.reserve(1000000);
    for (int v = 0; v < rectifyRGB.rows; v++)
        for (int u = 0; u < rectifyRGB.cols; u++) {
            unsigned int d = rectifyDepth.ptr<unsigned short>(v)[u]; // 深度值
            if (d == 0) continue; // 为0表示没有测量到
            Eigen::Vector3d point;
            point[2] = double(d)/50 ;
            point[0] = (u - cx) * point[2] / fx;
            point[1] = (v - cy) * point[2] / fy;
            Vector6d p;
            p.head<3>() = point;
            p[5] = rectifyRGB.data[v * rectifyRGB.step + u * rectifyRGB.channels()];   // blue
            p[4] = rectifyRGB.data[v * rectifyRGB.step + u * rectifyRGB.channels() + 1]; // green
            p[3] = rectifyRGB.data[v * rectifyRGB.step + u * rectifyRGB.channels() + 2]; // red
            pointcloud.push_back(p);
        }
    cv::waitKey(0);
    //cout << "有效点云数量为："<<pointcloud.size()<<endl;
    showPointCloud(pointcloud);
}

void showPointCloud(const vector<Vector6d, Eigen::aligned_allocator<Vector6d>>&pointcloud) {

    if (pointcloud.empty()) {
        cerr << "Point cloud is empty!" << endl;
        return;
    }

    pangolin::CreateWindowAndBind("Point Cloud Viewer", 1024, 768);//创建窗口并确定尺寸
    glEnable(GL_DEPTH_TEST);//3D可视化时开启，只绘制朝向摄像头一侧的图像
    glEnable(GL_BLEND);//启用颜色混合
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);//颜色混合方式
    //创建一个相机的观察视角
    pangolin::OpenGlRenderState s_cam(
        pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),//相机视角的尺寸，内参，最近和最远可视距离
        pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)//设置相机的外参：相机位置，相机朝向(俯仰，左右)，相机机轴方向(相机平面的旋转)——>（0.0, -1.0, 0.0）代表-y方向
    );

    pangolin::View &d_cam = pangolin::CreateDisplay()//创建视图
        .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f / 768.0f)//视图在视窗中的范围，以及视图的长宽比
        .SetHandler(new pangolin::Handler3D(s_cam));//显示s_cam所拍摄的内容

    while (pangolin::ShouldQuit() == false) {//若不关闭openGL窗口
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);//清空颜色和深度缓存，防止前后帧之间存在干扰

        d_cam.Activate(s_cam);//激活显示并设置状态矩阵
        glClearColor(1.0f, 1.0f, 1.0f, 1.0f);//刷新缓冲区颜色，防止帧间干扰
        glPointSize(2);//所绘点的大小
        /////// 真正的绘图部分 //////////
        glBegin(GL_POINTS);//点设置的开始
        for (auto &p: pointcloud) {//auto根据后面的变量值自行判断变量类型，继承点云
            glColor3d(p[3]/255.0, p[4]/255.0, p[5]/255.0);//RGB三分量相等即是灰度图像
            glVertex3d(p[0], p[1], p[2]);//确定点坐标
        }
        glEnd();//点设置的结束
        ///////////////////////////////
        pangolin::FinishFrame();//开始执行后期渲染，事件处理以及帧交换
        usleep(5000);   // sleep 5 ms
    }
    return;
}



