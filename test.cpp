#include<iostream>
#include<opencv2/opencv.hpp>
using namespace std;
using namespace cv;
int hmin = 0 ,smin = 162 ,vmin = 0;
int hmax = 43 ,smax = 255 ,vmax = 255;
int fps = 29;
int frame_number = 0;
int max(int a,int b)
{
    return (a > b ? a : b);
}
int main()
{    
    //创建窗口和拉动条，调整H、S、V三个参数
    //namedWindow("Trackbar",(600,400));
    //createTrackbar("Hue min","Trackbar",&hmin,179);
    //createTrackbar("Hue max","Trackbar",&hmax,179);
    //createTrackbar("Saturation min","Trackbar",&smin,255);
    //createTrackbar("Saturation max","Trackbar",&smax,255);
    //createTrackbar("Value min","Trackbar",&vmin,255);
    //createTrackbar("Value max","Trackbar",&vmax,255);

    VideoCapture video("orange1.mp4");
    Mat frame;
    //定义一个容器，储存圆心坐标
    vector<Point> centres;

    //定义一个容器，计算圆心坐标差值
    vector<Point> delta;

    //定义卡尔曼滤波器
    KalmanFilter kf(4,2,0);
    kf.statePre.at<float>(0) = 685;
    kf.statePre.at<float>(1) = 173;
    kf.statePre.at<float>(2) = 0;   // 初始x速度
    kf.statePre.at<float>(3) = 0;   // 初始y速度
    kf.transitionMatrix = (Mat_<float>(4,4) << 1,0,1,0,
                                               0,1,0,1,
                                               0,0,1,0,
                                               0,0,0,1);
    Mat_<float> measurement(2,1);

    measurement.setTo(Scalar(0));

    //初始化测量矩阵
    kf.measurementMatrix = (Mat_<float>(2,4) << 1,0,0,0,0,1,0,0);

    //初始化过程噪声协方差矩阵
    setIdentity(kf.processNoiseCov,Scalar::all(0.0001));

    //初始化测量噪声协方差矩阵
    setIdentity(kf.measurementNoiseCov,Scalar::all(0.0001));

    //初始化估计误差协方差矩阵
    setIdentity(kf.errorCovPost,Scalar::all(100));

    while(1)
    {
        video >> frame;
        if(frame.empty())
        {
            break;
        }

        //图像预处理
        Mat imgBlur,imgHSV,imgMask;
        Scalar lower(hmin,smin,vmin);
        Scalar uper(hmax,smax,vmax);
        GaussianBlur(frame,imgBlur,Size(3,3),3,0);
        cvtColor(imgBlur,imgHSV,COLOR_BGR2HSV);
        inRange(imgHSV,lower,uper,imgMask);

        //寻找边缘
        vector<vector<Point>> edge;
        vector<Vec4i> h;
        findContours(imgMask,edge,h,RETR_TREE,CHAIN_APPROX_SIMPLE,Point());

        //如果停下来了，清空圆心容器
        /*if(delta.size() > 0 && delta[delta.size()-1].x * delta[delta.size()-1].x + delta[delta.size()-1].y * delta[delta.size()-1].y == 0)
        {
            centres.clear();
        }*/

        //暂存r
        int rr = 0;

        //绘制边缘和最小外接矩形
        for(size_t i = 0; i < edge.size(); i++)
        {
            //绘制轮廓
            //drawContours(frame,edge,i,Scalar(0,255,255),2,8);

            //利用轮廓下的面积寻找我们所需要的“圆形”
            int area = contourArea(edge[i]);
            //cout << area << endl;

            //圆形的面积在1000以上
            if(area > 1000)
            {
                //绘制最小外接矩形
                Rect boundingRect = cv::boundingRect(edge[i]);
                //rectangle(frame, boundingRect, Scalar(0, 255, 0), 2);

                //寻找最小外接圆圆心，即最小外接矩形两条对角线的交点
                Point centre;
                centre.x=boundingRect.x+boundingRect.width/2;
                centre.y=boundingRect.y+boundingRect.height/2;

                //储存圆心坐标
                centres.push_back(centre);

                //打印圆心坐标
                //cout << centre.x << " , " << centre.y << endl;

                //最小外接圆的半径为最小外接矩形长和宽的一半的大者
                int r = max(boundingRect.width/2,boundingRect.height/2);
                rr = r;

                //绘制最小外接圆及其圆心
                cv::circle(frame,centre,r,Scalar(0,0,255),3,8,0);
                cv::circle(frame,centre,5,Scalar(0,255,0),-1,8,0);
            }
        }
        
        //简单调试

        //打印容器中最后一个圆心坐标
        //cout << centres[centres.size()-1].x << " , " << centres[centres.size()-1].y << endl;

        //设置差值
        if(centres.size() > 1)
        {
            Point p = centres[centres.size()-1]-centres[centres.size()-2];
            delta.push_back(p);
        }

        //打印最新差值
        if(delta.size() > 0)
        {
            //cout << delta[delta.size()-1].x << " , " << delta[delta.size()-1].y << endl;
            //cout << delta[delta.size()-1].x * delta[delta.size()-1].x + delta[delta.size()-1].y * delta[delta.size()-1].y << endl;
        }
        
        //创建储存所有预测点的容器
        deque<Point> pre_points;
        //暂存预测点
        Point pre_p(0,0);
    
        for (size_t i = 0; i < centres.size(); i++)
        {
            measurement(0) = centres[i].x;
            measurement(1) = centres[i].y;

            // 预测
            Mat prediction = kf.predict();

            // 更新
            Mat estimated = kf.correct(measurement);

            // 获取预测的圆心坐标
            Point predictedCenter(prediction.at<float>(0), prediction.at<float>(1));

            //暂存预测点
            pre_p = predictedCenter;
        }
        

        // 绘制预测点
        cv::circle(frame, pre_p, 5, Scalar(255, 0, 0), -1, 8, 0);
        cv::circle(frame, pre_p, rr, Scalar(255, 255, 255), 2, 8, 0);

        //帧数更新
        frame_number++;

        //输出图像
        //imshow("video",imgMask);
        cv::imshow("video2",frame);
        cv::waitKey(10000/video.get(5));

    }


    return 0;
}