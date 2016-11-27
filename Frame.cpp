//
//  Frame.cpp
//  dabino
//
//  Created by 谭智丹 on 16/3/8.
//  Copyright © 2016年 谭智丹. All rights reserved.
//

#include "Frame.hpp"

Frame::Frame(Mat &img, int bs, Point2f ppt, float focalt, float baselinet)
{
    
    im_width = img.cols;
    im_height = img.rows;
    block_size = bs;
    pp = ppt;
    focal = focalt;
    baseline = baselinet;
    
}


// Assign corners into grids.
// We devide an image into small square grids. Then we assign the FAST corners into
// these grids. Each element of 'subidx' is a vector, which contains all the indices for
// corners that belong to this grid.
void Frame::GetFeaturesIntoGrid(vector<Point2f> &vPoints, vector<vector<int> > &subidx)
{
    subidx.clear();
    int n_xSub = im_width/block_size;
    int n_ySub = im_height/block_size;
    subidx.resize(n_xSub*n_ySub);
    
    for (int i=0; i<vPoints.size(); i++) {
        int xi = MIN(vPoints[i].x / block_size, n_xSub-1);
        int yi = MIN(vPoints[i].y / block_size, n_ySub-1);
        
        int numeli = xi*n_ySub + yi;
        subidx[numeli].push_back(i);
    }
    
}


// Sort corners contained in the same grid by their scores.
void Frame::SortSubIdx(vector<cv::KeyPoint> &vKeyPoints, vector<vector<int> > &subidx)
{
    for (size_t i=0; i<subidx.size(); i++) {
        vector<float> sub_response;
        vector<int> dsIdx;
        if (subidx[i].size()>1) {
            for (size_t j=0; j<subidx[i].size(); j++) {
                sub_response.push_back(vKeyPoints[subidx[i][j]].response);
            }
            sortIdx(sub_response, dsIdx, CV_SORT_DESCENDING);
            for (size_t j=0; j<dsIdx.size(); j++) {
                dsIdx[j] = subidx[i][dsIdx[j]];
            }
            for (size_t j=0; j<dsIdx.size(); j++) {
                subidx[i][j] = dsIdx[j];
            }
        }
    }
}


// For each grid, only keep the top N FAST corners with highest scores.
void Frame::SelectTopN(vector<Point2f> &iPoints, vector<Point2f> &oPoints, vector<vector<int> > &subidx, int N)
{
    for (size_t i=0; i<subidx.size(); i++) {
        if (!subidx[i].empty()) {
            
            size_t numPick = MIN(subidx[i].size(), N);
            
            for (size_t j=0; j<numPick; j++) {
                oPoints.push_back(iPoints[subidx[i][j]]);
            }
        }
    }
}


// Calculate stereo disparities.
// For stereo matching, I use the KLT optical-flow tracker for simplicity, it perfoms quite satisfaying.
// But it's rather slow, and not so robust in challenging illumilation environment.
// Instead you can use a similarity score matching method (SAD/SSD/NCC/ZNCC).
// Or you can choose to make the KLT tracker one-dimensional (along the stero epipolar line).
void Frame::FindDisparity(vector<Point2f> &pts1, vector<float>& disp ,cv::Mat &img1, cv::Mat &img2, float ythresh)
{
    vector<float> err;
    vector<uchar> status;
    vector<Point2f> pts2;
    calcOpticalFlowPyrLK(img1, img2, pts1, pts2, status, err, Size(31,31));
    
    size_t k=0;
    for (size_t i=0; i<pts1.size(); i++) {
        if (status[i]) {
            if (abs(pts1[i].y-pts2[i].y)<ythresh) {
                float diffx = (pts1[i].x - pts2[i].x);
                if ((diffx>0.5) &&(diffx<150)) {
                    pts1[k] = pts1[i];
                    disp.push_back(diffx);
                    k++;
                }
            }
        }
    }
    
    pts1.resize(k);
    disp.resize(k);
    
}


/*
void Frame::st2rt(vector<float> &stat, cv::Mat &R, cv::Mat &T)
{
    float rx=stat[0], ry=stat[1], rz=stat[2];
    R = (Mat_<float>(3,3)<<1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f);
    T = (Mat_<float>(3,1)<<stat[3], stat[4], stat[5]);
    
    float r = sqrt(rx*rx + ry*ry + rz*rz);
    if (r > 0.00001) {
        rx /= r; ry /= r; rz /= r;
        
        // compute some quantity to avoid repetition
        float ct = cos(r);
        float st = sin(r);
        float ict = 1.0f - ct;
        float rxy_ict = rx * ry * ict;
        float rxz_ict = rx * rz * ict;
        float ryz_ict = ry * rz * ict;
        float rx_st = rx * st;
        float ry_st = ry * st;
        float rz_st = rz * st;
        
        // calculate the elements of R
        float r11, r12, r13, r21, r22, r23, r31, r32, r33;
        r11 = ct + rx * rx * ict;
        r12 = rxy_ict + rz_st;
        r13 = rxz_ict - ry_st;
        r21 = rxy_ict - rz_st;
        r22 = ct + ry * ry * ict;
        r23 = ryz_ict + rx_st;
        r31 = rxz_ict + ry_st;
        r32 = ryz_ict - rx_st;
        r33 = ct + rz * rz * ict;
        
        R = (Mat_<float>(3,3)<<r11, r12, r13, r21, r22, r23, r31, r32, r33);
        
    }
    
}*/