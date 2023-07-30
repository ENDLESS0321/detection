#include<iostream>
#include<opencv2/opencv.hpp>
using namespace std;
using namespace cv;
int hmin = 0 ,smin = 162 ,vmin = 0;
int hmax = 43 ,smax = 255 ,vmax = 255;
int max(int a,int b)
{
    return (a > b ? a : b);
}
int main()
{    
    //创建窗口和拉动条，调整H、S、V三个参数
    namedWindow("Trackbar",(600,400));
    createTrackbar("Hue min","Trackbar",&hmin,179);
    createTrackbar("Hue max","Trackbar",&hmax,179);
    createTrackbar("Saturation min","Trackbar",&smin,255);
    createTrackbar("Saturation max","Trackbar",&smax,255);
    createTrackbar("Value min","Trackbar",&vmin,255);
    createTrackbar("Value max","Trackbar",&vmax,255);


    VideoCapture video("orange1.mp4");
    Mat frame;
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

        //输出图像
        imshow("video",imgMask);
        imshow("video2",frame);
        waitKey(1000/video.get(5));

    }


    return 0;
}