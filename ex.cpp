#include <stdlib.h>
#include <iostream>
#include <string>
#include <cv.h>
#include <highgui.h>
#include <boost/thread.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/types_c.h>
#include <geometry_msgs/Twist.h>
#include "ros/ros.h"
#define READIMAGE_ONLY
#ifndef READIMAGE_ONLY
#include <geometry_msgs/Twist.h>
#endif

using namespace cv;
using namespace std;
int flag=0;
int counter=0;
int ca;
int coner;
Mat RGB2HSV(Mat input)
{
    int row = input.rows;
    int col = input.cols;

    Mat output;
    output = input.clone();
    for (int i = 0; i < row; ++i)
    {
        for (int j = 0; j < col; ++j)
        {
            Vec3b pix = input.at<Vec3b>(i, j);//012:BGR
            float b = 1.0 * pix[0] / 255;
            float g = 1.0 * pix[1] / 255;
            float r = 1.0 * pix[2] / 255;
            float maxrgb = max(r, max(g, b));
            float minrgb = min(r, min(g, b));
            float diff = maxrgb - minrgb;
            float v = maxrgb;
            float s = (diff / v);
            float h;
            if (maxrgb - minrgb < 1e-5)h = 0;
            else if (maxrgb == r) h = 60 * (g - b) / diff;
            else if (maxrgb == g) h = 60 * (b - r) / diff + 120;
            else if (maxrgb == b) h = 60 * (r - g) / diff + 240;
            if (h < 0)h += 360;
            else if (h > 359)h -= 360;
            output.at<Vec3b>(i, j)[0] = (int)(h * 180 / 360);
            output.at<Vec3b>(i, j)[1] = (int)(s * 255);
            output.at<Vec3b>(i, j)[2] = (int)(v * 255);
        }
    }
    return output;
}
Mat RGB2HSI(Mat input)
{
    int row = input.rows;
    int col = input.cols;

    Mat output;
    output = input.clone();

    for (int i = 0; i < row; ++i)
    {
        for (int j = 0; j < col; ++j)
        {
            int B = input.at<Vec3b>(i, j)[0];
            int G = input.at<Vec3b>(i, j)[1];
            int R = input.at<Vec3b>(i, j)[2];
            int minRGB = min(min(R, G), B);

            int I = (R + G + B) / 3;
            int S = int(1 - 3 * minRGB / (R + G + B)) * 255;
            int H = int(acos((R - G + R - B) / 2 / (sqrt((R - G) * (R - G) + (R - B) * (R - B))))) * 255;

            output.at<Vec3b>(i, j)[0] = H;
            output.at<Vec3b>(i, j)[1] = S;
            output.at<Vec3b>(i, j)[2] = I;
        }
    }
    return output;
}
Mat check_blue(Mat src)
{
    Mat s1=src.clone();
    //Mat s2;
    src=RGB2HSV(src);
    int r=src.rows;
    int c=src.cols;
    for(int i=0;i<r;i++)
    {
      for(int j=0;j<c;j++)
      {
         if(src.at<Vec3b>(i,j)[2]>45&&src.at<Vec3b>(i,j)[1]>42&&src.at<Vec3b>(i,j)[0]>100&&src.at<Vec3b>(i,j)[0]<125)
         {
            for(int k=0;k<3;k++)
            {
               s1.at<Vec3b>(i,j)[k]=255;
            }
         }
         else{
            for(int k=0;k<3;k++)
            {
               s1.at<Vec3b>(i,j)[k]=0;
            }
         }
      }
    }
    imshow("c1",s1);
return s1;
}
int check_yellow(Mat src)
{
    //Mat s2;
    int a=0;
    src=RGB2HSV(src);
    int r=src.rows;
    int c=src.cols;
    for(int i=r/2;i<r;i++)
    {
      for(int j=0;j<c;j++)
      {
         if(src.at<Vec3b>(i,j)[2]>45&&src.at<Vec3b>(i,j)[1]>42&&src.at<Vec3b>(i,j)[0]>11&&src.at<Vec3b>(i,j)[0]<34)
         {
            a++;
         }
      }
    }
return a;
}
int check_red(Mat src)
{
    //Mat s2;
    int a=0;
    src=RGB2HSV(src);
    int r=src.rows;
    int c=src.cols;
    for(int i=r/2;i<r;i++)
    {
      for(int j=0;j<c;j++)
      {
         if(src.at<Vec3b>(i,j)[2]>45&&src.at<Vec3b>(i,j)[1]>42&&src.at<Vec3b>(i,j)[0]<180&&src.at<Vec3b>(i,j)[0]>156)
         {
            a++;
         }
         else if(src.at<Vec3b>(i,j)[2]>45&&src.at<Vec3b>(i,j)[1]>42&&src.at<Vec3b>(i,j)[0]<10&&src.at<Vec3b>(i,j)[0]>0)
         {
              a++;
         }
      }
    }
return a;
}
Mat th_check(Mat src)
{
    Mat src1=src.clone();
    Mat src2=src.clone();
    int r=src.rows;
    int c=src.cols;
    for(int i=0;i<r;i++)
    {
        for(int j=0;j<c;j++)
          {
             if(src.at<Vec3b>(i,j+1)[0]-src.at<Vec3b>(i,j)[0]>200)
                  {
                          src1.at<Vec3b>(i,j)[0]=0;
                          src1.at<Vec3b>(i,j)[1]=0;
                          src1.at<Vec3b>(i,j)[2]=255;
                  }
               else
                 {
                        for(int k=0;k<3;k++)
                        {
                           src1.at<Vec3b>(i,j)[k]=0;
                        }
                  }
           }
    }
    imshow("th",src1);
int d=0;
int h[2];
int mid;
    for(int i=0;i<r;i++)
    {
        for(int j=0;j<c/2;j++)
        {
             if(src1.at<Vec3b>(i,j)[2]==255)
                {
                    d++;
                    if(d==1)
                      h[0]=j;
                    else if(d==2)
                    {
                           if(j-h[0]<=20)
                           d=1;
                           else
                           h[1]=j;
                      }
                 }
         }
         if(d==2)
          {
             mid=(h[1]+h[0])/2;
             src2.at<Vec3b>(i,mid)[0]=0;
             src2.at<Vec3b>(i,mid)[1]=0;
             src2.at<Vec3b>(i,mid)[2]=255;
           }
         d=0;
    }
    for(int i=0;i<r;i++)
    {
        for(int j=c/2;j<c;j++)
        {
             if(src1.at<Vec3b>(i,j)[2]==255)
                {
                    d++;
                    if(d==1)
                      h[0]=j;
                    else if(d==2)
                    {
                           if(j-h[0]<=20)
                           d=1;
                           else
                           h[1]=j;
                      }
                 }
         }
         if(d==2)
          {
             mid=(h[1]+h[0])/2;
             src2.at<Vec3b>(i,mid)[0]=0;
             src2.at<Vec3b>(i,mid)[1]=0;
             src2.at<Vec3b>(i,mid)[2]=255;
           }
         d=0;
    }
   imshow("th1",src2);
    return src2;
}
int stop(Mat src)
{
   int a=0;
  int m=0;
   int c=src.cols;
   int r=src.rows;
   for(int j=0;j<c/2;j++)
   {
      if(src.at<Vec3b>(r-5,j)[1]==255&&src.at<Vec3b>(r-5,j)[2]==255)
      {
          a++;
      }
   }
    //m=((double)(a))/((double)(c));
   // cout<<m<<"\n";
    if(a<300)
    return 1;
    else
    return 0;
}
int check(Mat src)
{
  int co=0;
   int r=src.rows;
   int c=src.cols;
   //r=r/2;
   for(int i=r/2;i<r/2+50;i++)
   {
   for(int j=0;j<c;j++)
   {
      if(src.at<Vec3b>(i,j)[1]==0&&src.at<Vec3b>(i,j)[2]==255)
      {
         co++;
      }
   }
   }
    if(co>=40)
    return 1;
    else
    return 0;
}
int pattern(Mat src)
{
   /*Mat src_gray1,m;
   medianBlur(src,m,3);
   //Canny(src,m,130,300,3);
   cvtColor(m,src_gray1,COLOR_BGR2GRAY);
   vector<Vec3f> p;
   HoughCircles(src_gray1,p,HOUGH_GRADIENT,1,100,200,23,3,25);
   for(size_t i=0;i<p.size();i++)
   {
     Vec3f cc=p[i];
     circle(src,Point(cc[0],cc[1]),cc[2],Scalar(255,0,0),3,CV_AA);
    cout<<cc[2]<<"\n";
   }
   imshow("Hough",src);*/
   int a1,a2;
   a1=check_yellow(src);
   a2=check_red(src);
   //cout<<a1<<"   y    "<<a2<<"\n";
   if(a1>=3000)
   {
     return 1;
   }
   else if(a2>=3000)
   {
     return 2;
   }
   else
   return 0;
}
int runstate(Mat src,Mat src1)
{
  //int p=pattern(src1);
  if(stop(src)==0&&flag==0)
{
       flag=1;
       ca=pattern(src1);
       return 0;   
}
if(flag==1)
{
   counter++;
   cout<<counter<<"\n";
   if(ca==1&&counter<=80)
   {
      return 4;
    }
   else if(ca==2&&counter<=80)
   {
     return 5;
   }
   else
   {
     flag=0;
     return 1;
    }
}
   int pra1=10;
int  pra2=20;
  int r=src.rows;
  int c=src.cols;
  int a=0;
  int b=0;
  int re;
  int i,j;
r=r-pra1;
for(j=0;j<c/2;j++)
{
   if(src.at<Vec3b>(r,j)[0]==0&&src.at<Vec3b>(r,j)[2]==255)
{
   a=j;
   break;
}
}
if(a==0)
{
     for(j=c/2;j<c;j++)
{
   if(src.at<Vec3b>(r,j)[0]==0&&src.at<Vec3b>(r,j)[2]==255)
{
   a=j-336*3;
   break;
}
}
}
else{
a=a-336;
}
//cout<<flag<<"\n";
//cout<<p<<"\n";
//coner=a;
if(a<pra2&&a>-pra2)
re=1;
else if(a<-pra2)
re=2;
else
re=3;
return re;
}

int main(int argc,char**argv)
{
   ROS_WARN("*****START*****");
   ros::init(argc,argv,"ColorMove");//初始化 ROS 节点
   ros::NodeHandle n;
   VideoCapture capture;
   capture.open(1);
   if(!capture.isOpened())
{
   printf("mdksxt");
   return 0;
}
 // #ifndef READIMAGEONLY
 // ros::Publisher pub =n.advertise<geometry_msgs::Twist>("/smoother_cmd_vel",5);
 // #endif
    ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 5);//定义速度发布器
    geometry_msgs::Twist twist;
  Mat src_frame;
  Mat src_frame1;
  Mat dst;
  Mat src1;
  int m;
  while(ros::ok())
{
   capture.read(src_frame);
   if(src_frame.empty())
{
   break;
}
  //cvtColor(src_frame,src_frame,CV_RGB2GRAY);
  imshow("h",src_frame);
  src_frame1=src_frame;
  src_frame=check_blue(src_frame);
  src1=th_check(src_frame);
  //Canny(src_frame,src_gray,150,200,3);
  /*cvtColor(src_frame,dst,CV_GRAY2BGR);
  vector<Vec4f> plines;
  HoughLinesP(src_frame,plines,1,CV_PI/180,10,160,10);
  for(size_t i=0;i<plines.size();i++)
  {
    Vec4f P=plines[i];
    line(dst,Point(P[0],P[1]),Point(P[2],P[3]),Scalar(0,0,255),3,CV_AA);
  }
  imshow("dst",dst);*/
  /*twist.linear.x = 0.05;//线速度
            twist.linear.y = 0;
            twist.linear.z = 0;
            twist.angular.x = 0;
            twist.angular.y = 0;
            twist.angular.z = 0;//角速度
            cmd_pub.publish(twist);*/
  m=runstate(src1,src_frame1);
  if(m==1)
{
    // twist.linear.x = 0.03;//线速度
           twist.linear.x = 0.1;//线速度
            twist.linear.y = 0;
            twist.linear.z = 0;
            twist.angular.x = 0;
            twist.angular.y = 0;
            twist.angular.z = 0;//角速度
            cmd_pub.publish(twist);
}
else if(m==2)
{
  twist.linear.x = 0.14;//线速度
            twist.linear.y = 0;
            twist.linear.z = 0;
            twist.angular.x = 0;
            twist.angular.y = 0;
            //twist.angular.z =0.15;//角速度
            twist.angular.z =0.15;
            cmd_pub.publish(twist);
}
else if(m==3)
{
     //twist.linear.x = 0.03;//线速度
             twist.linear.x = 0.1;//线速度
            twist.linear.y = 0;
            twist.linear.z = 0;
            twist.angular.x = 0;
            twist.angular.y = 0;
            //twist.angular.z = -0.045;//角速度
            twist.angular.z = -0.15;
            //twist.angular.z = -0.13;
            cmd_pub.publish(twist);
}
else if(m==0)
{
   twist.linear.x = 0;//线速度
            twist.linear.y = 0;
            twist.linear.z = 0;
            twist.angular.x = 0;
            twist.angular.y = 0;
            //twist.angular.z = -0.045;//角速度
            twist.angular.z = 0;
            cmd_pub.publish(twist);
}
else if(m==4)
{
  twist.linear.x = 0.03;//线速度
            twist.linear.y = 0;
            twist.linear.z = 0;
            twist.angular.x = 0;
            twist.angular.y = 0;
            twist.angular.z =0.25;//角速度
            cmd_pub.publish(twist);
}
else if(m==5)
{
     //twist.linear.x = 0.03;//线速度
             twist.linear.x = 0.03;//线速度
            twist.linear.y = 0;
            twist.linear.z = 0;
            twist.angular.x = 0;
            twist.angular.y = 0;
            //twist.angular.z = -0.045;//角速度
            twist.angular.z = -0.2;
            cmd_pub.publish(twist);
}
   ros::spinOnce();
  waitKey(5);
}
return 0;
}
