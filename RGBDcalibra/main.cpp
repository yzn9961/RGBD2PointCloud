#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>

//定义标定板内角点维数
int CHESSBOARD[2] = {6,9};//纵向6个角点，横向9个角点
using namespace std;
using namespace cv;

Rect validROIL, validROIR;
Mat R, T, E, F; //R 旋转矢量 T平移矢量 E本征矩阵 F基础矩阵
Mat Rl, Rr, Pl, Pr, Q;//校正旋转矩阵R，投影矩阵P 重投影矩阵Q (下面有具体的含义解释）
cv::Mat R_cameraMatrix, R_distCoeffs, R_R, R_T;
cv::Mat L_cameraMatrix, L_distCoeffs, L_R, L_T;
Mat R_src,R_img,L_src,L_img,R_dst,L_dst;//图像
double fx;
void outputCameraParam(void)
{
    /*保存数据*/
    /*输出数据*/
    fx = R_cameraMatrix.at<double>(0,0);
    FileStorage fs("mydepth.yaml", FileStorage::WRITE); 
    if (fs.isOpened())
    {
        // 公共参数
        fs<<"CameraType"<<"\"PinHole\"";
        fs<<"fx"<<R_cameraMatrix.at<double>(0,0);//.at()先行后列 (00)
        fs<<"fy"<<R_cameraMatrix.at<double>(1,1);//11
        fs<<"cx"<<R_cameraMatrix.at<double>(0,2);//02
        fs<<"cy"<<R_cameraMatrix.at<double>(1,2);//12
        fs<<"K"<<R_cameraMatrix;
        fs<<"D"<<R_distCoeffs;
        fs.release();
    }
    else
        cout << "Error: 无法保存！\n";
    cout<<"参数已经写入至：mydepth.yaml！"<<endl;
    cout << "快开始下一步工作吧~再见:)" << endl;
}


int main() {
    //////// 标定数据集的导入 //////////
    //单目输入
    vector<String> image_paths;//数据集中图像的路径
    string path = "./data";//图像名称格式
    glob(path, image_paths);// 将所有符合格式的图像路径存储到image_paths中

    ////// 世界坐标系三维坐标输入 //////
    vector<Point3f>objp; // 存储关键点在真实世界中的三维坐标
    for (int i = 0; i < CHESSBOARD[1]; i++)
    {
        for (int j = 0; j < CHESSBOARD[0]; j++)
            objp.push_back(Point3f(i, j, 0));//逐行标记内角点坐标
    }



    vector<Point2f> corners;
    bool found1;
    vector<vector<Point3f>> objPoints;//为整个数据集创建其在真实世界的三维坐标集合。
    vector<vector<Point2f>> imgPoints;//图像上的关键点坐标位置
    ///////// 遍历数据集 && 确定关键点位置 ///////////
    std::cout<<"正在搜索标定点……"<<endl;
    for (int i = 0; i < image_paths.size(); i++)
    {
        R_src = imread(image_paths[i]);
        cvtColor(R_src, R_img, COLOR_BGR2GRAY);
        //寻找关键点位置
        found1 = findChessboardCorners(R_img,
                                    Size(CHESSBOARD[0], CHESSBOARD[1]),corners,
                                    CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);
        if (found1)//在图像上确定出内角点信息
        {
            TermCriteria criteria(2|1, 30, 0.001);//设置最大迭代次数与迭代终止条件
            //使用亚像素精准化关键点坐标位置
            cornerSubPix(R_img, corners, cv::Size(11, 11), cv::Size(-1, -1), criteria);
            //在图像上显示检测到的关键点
            //drawChessboardCorners(R_src, cv::Size(6, 9), corners, found1);
            //存储关键点信息
            objPoints.push_back(objp);
            imgPoints.push_back(corners);
        }
        //cv::imshow("Corner Detected Right", R_src);//展示图像
        //waitKey(0);
    }
    cv::destroyAllWindows();
    cout << "标定点搜索完成！" << endl << endl << endl;

    ////////// 计算相机内外参数 ///////////
    cout << "开始计算镜头内外参，请稍等……" << endl;
    cv::calibrateCamera(objPoints, imgPoints, cv::Size(R_img.rows,R_img.cols), R_cameraMatrix, R_distCoeffs, R_R, R_T); // LEFT.K LEFT.D
    cout << "镜头内外参计算完成！" << endl;

    outputCameraParam();//参数写入
    Mat mapRx, mapRy;
    //Mat new_camera_matrix,frame,dst;
    //frame = imread("./data_new/left/L_1.jpg");
    //initUndistortRectifyMap(L_cameraMatrix, L_distCoeffs, Rl, Pl, imageSize, CV_32FC1, mapLx, mapLy);
    //remap(frame, dst, mapLx, mapLy, INTER_LINEAR);
    //imshow("left undistort",dst);
    //imshow("handwrite",handwrite_Undistort(frame));
    //imshow("stereo calibrate",show_stereoCalib());
    Mat srcR = imread("./data/Capture_1.jpg");
    cv::Size imageSize(cv::Size(srcR.cols, srcR.rows));
    initUndistortRectifyMap(R_cameraMatrix, R_distCoeffs, Rr, R_cameraMatrix, imageSize, CV_32FC1, mapRx, mapRy);
    Mat rectifyImageR;
    remap(srcR, rectifyImageR, mapRx, mapRy, INTER_LINEAR);
    imshow("right undistort",rectifyImageR);
    waitKey(0);
    return 0;
}
