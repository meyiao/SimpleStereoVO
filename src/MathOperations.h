//
//  MathOperations.h
//  dabino
//
//  Created by 谭智丹 on 16/4/27.
//  Copyright © 2016年 谭智丹. All rights reserved.
//

#ifndef MathOperations_h
#define MathOperations_h

#include<opencv2/opencv.hpp>
#include "camera.hpp"

using namespace cv;
using namespace std;

/* rvec2mat : convert a rvec into a rotation matrix R (R_Sn) */
void rvec2mat(Mat& rvec, Mat& R)
{
    // rotation angle
    double rx = rvec.at<double>(0,0);
    double ry = rvec.at<double>(1,0);
    double rz = rvec.at<double>(2,0);
    double theta = sqrt(rx*rx + ry*ry + rz*rz);
    
    // set rotation matrix to identity
    R = (Mat_<double>(3,3)<<1, 0, 0, 0, 1, 0, 0, 0, 1);
    
    // calculate rotaion matrix
    if (theta > 0.0001) {
        rx /= theta;
        ry /= theta;
        rz /= theta;
        
        // compute some quantity to avoid repetition
        double ct = cos(theta);
        double st = sin(theta);
        double ict = 1 - ct;
        double rxy_ict = rx * ry * ict;
        double rxz_ict = rx * rz * ict;
        double ryz_ict = ry * rz * ict;
        double rx_st = rx * st;
        double ry_st = ry * st;
        double rz_st = rz * st;
        
        // calculate the elements of R
        double R00 = ct + rx * rx * ict;
        double R01 = rxy_ict + rz_st;
        double R02 = rxz_ict - ry_st;
        double R10 = rxy_ict - rz_st;
        double R11 = ct + ry * ry * ict;
        double R12 = ryz_ict + rx_st;
        double R20 = rxz_ict + ry_st;
        double R21 = ryz_ict - rx_st;
        double R22 = ct + rz * rz * ict;
        
        R = (Mat_<double>(3,3)<< R00, R10, R20, R01, R11, R21, R02, R12, R22);
        
    }
    
}


/* Transform3D : Using R&T to transform a 3D point seen in one camera frame into another camera frame
 R : R_Cc0
 T : c0->c, described in the c0 frame
 */
void Transform3D(vector<Point3f>& oldPts3d, vector<Point3f>& newPts3d, Mat& R, Mat& T)
{
    // we need to convert R and T to float type
    Mat fR, fT;
    R.convertTo(fR, CV_32F);
    T.convertTo(fT, CV_32F);
    
    for (size_t i=0; i<oldPts3d.size(); i++) {
        Mat oldx(oldPts3d[i]);
        Mat x;
        x = fR*oldx + fT;
        
        newPts3d.push_back(Point3f(x.at<float>(0,0),x.at<float>(1,0),x.at<float>(2,0)));
    }
}


void projectObj2img(vector<Point3f>& pts3d, vector<Point2f>& pts2d, camera::camera cam, vector<bool>& isInImg)
{
    float fx = cam.fx, fy = cam.fy, cx = cam.cx, cy = cam.cy;
    float minu = 3, maxu = cam.imwidth -3;
    float minv = 3, maxv = cam.imheight-3;
    
    for (size_t i=0; i<pts3d.size(); i++) {
        if (pts3d[i].z == 0) {
            isInImg.push_back(false);
            pts2d.push_back(Point2f(pts3d[i].x, pts3d[i].y));
        }
        else {
            float invz = 1/pts3d[i].z;
            float u = fx*invz*pts3d[i].x + cx;
            float v = fy*invz*pts3d[i].y + cy;
            pts2d.push_back(Point2f(u, v));
            
            if (u<minu || u>maxu || v<minv || v>maxv )
                isInImg.push_back(false);
            else
                isInImg.push_back(true);
            
        }
    }
    
}

#endif /* MathOperations_h */
