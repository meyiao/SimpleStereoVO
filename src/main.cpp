//
//  main.cpp
//  dabino
//
//  Created by 谭智丹 on 16/3/8.
//  Copyright © 2016年 谭智丹. All rights reserved.
//

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <vector>
#include <iostream>
#include <fstream>

#include "Frame.hpp"
#include "camera.hpp"
#include "MathOperations.h"

using namespace cv;
using namespace std;

//------------Dataset Parameters-----------------------//!!!!!!!!!!!!!CHANGE!!!!!!!!!!!!!!!!!!!
/*----------Kitti Sequence 00, 01, 02--------*/
const float focal = 718.856;
const Point2f pp = Point2f(607.1928, 185.2157);

/*----------Kitti Sequence 03--------*/
//const float focal = 721.5377;
//const Point2f pp = Point2f(609.5593, 172.854);

/*----------Kitti Sequence 04, 05, 06--------*/
//const float focal = 707.0912;
//const Point2f pp = Point2f(601.8873, 183.1104);

const float baseline = 0.53715;

const int total_frames = 4541;    // 00
//const int total_frames = 1101;      // 01
//const int total_frames = 4661;      // 02
//const int total_frames = 801;       // 03
//const int total_frames = 271;      // 04
//const int total_frames = 2761; // 05
//const int total_frames = 1101;   // 06
 
const Mat cam_mat = (Mat_<float>(3,3)<<focal, 0.0f, pp.x, 0.0f, focal, pp.y, 0.0f, 0.0f, 1.0f);
const float y_threshold = 1.0f;

//------------------------------------------------------------//


int main(int argc, const char * argv[]) {
    
    // The dataset file location
    string head_name = "/Users/tanzhidan/Documents/data/kitti/sequences/00/"; //!!!!!!!CHANGE!!!!!!!!!!!!!!!!!!
    
    // Char array used to store the image name
    char tail_name[20];
    
    // Read the first frame
    Mat fisrt_frame = imread(head_name + "image_0/000000.png", 0);//!!!!!!!!!!MAY NEED CHANGE!!!!!!!!!!!!!!!!!
    Frame frame(fisrt_frame, 30, pp, focal, baseline);
    int start_frame = 0;
    
    // Open a txt file to store the results
    ofstream fout("/Users/tanzhidan/Documents/kitti_0.txt"); // !!!!!!!!!!!CHANGE!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    if (!fout) {
        cout<<"File not opened!"<<endl;
        return 1;
    }

    // Create a window for display
    namedWindow("window", 0);
    
    // Create a camera object
    camera cam(fisrt_frame.cols, fisrt_frame.rows, focal, focal, pp.x, pp.y);
    
    // Containers for previous frame's information
    Mat old_img_l;              // image
    vector<Point2f> old_pts;    // corners' locations
    vector<float> old_disp;     // disparities
    
    // Termination criteria used when executing sub-pixel refinement
    TermCriteria crit = TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);
    
    // Pose state vector
    Mat oldRvec = (Mat_<double>(3,1)<<0, 0, 0); // rotation
    Mat oldTvec = (Mat_<double>(3,1)<<0, 0, 0); // translation
    
    
    // Here we go!
    for (int numFrame=start_frame; numFrame<total_frames; numFrame++) {
        
        clock_t ts = clock();
        
        // First of all, get the file name
        sprintf(tail_name, "%06d.png", numFrame);
        cout << endl;
        cout <<"# Frame:         \t"<<numFrame<<endl;
        
        // Load the stereo image pair
        Mat img_l = imread(head_name+"image_0/"+tail_name, 0);
        Mat img_r = imread(head_name+"image_1/"+tail_name, 0);
        if (img_l.empty()) {
            cout<<"Error: can not load the images!"<<endl;
            return -1;
        }
        
        // Detect fast corners
        vector<KeyPoint> keyPts;
        FAST(img_l, keyPts, 20, true, 2); // threshold is 20, do non-maximun suppression, 16-9 method
        vector<Point2f> pts;
        KeyPoint::convert(keyPts, pts);
        cout<<"# detected:      \t"<<pts.size()<<endl;
        
        // Assign features into grids
        vector<vector<int>> subidx;
        frame.GetFeaturesIntoGrid(pts, subidx);
        
        // Sort the subidx
        frame.SortSubIdx(keyPts, subidx);
        int occupied=0;
        for (size_t i=0; i<subidx.size(); i++) {
            if (!subidx[i].empty()) {
                occupied++;
            }
        }
        
        // Pick up top points in each grid
        vector<Point2f> topPts;
        int want_in_each = MIN(1000 / occupied, 7);
        
        frame.SelectTopN(pts, topPts, subidx, want_in_each);
        cout << "Selected:\t" << topPts.size() << endl;
        
        // Sub-pixel refinement (very optional!)
        cornerSubPix(img_l, topPts, Size(5,5), Size(-1,-1), crit);
        
        // Stereo matching
        vector<float> disp;
        frame.FindDisparity(topPts, disp, img_l, img_r, y_threshold);
        
        
        ///////
        if (numFrame > start_frame) {
            
            //---------Predict points' locations in current frame------//
            vector<Point3f> prevPts3d, prediPts3d;
            vector<Point2f> prediPts;
            vector<bool> canBeSeen;
            
            // 3D points described in last camera coordinates
            for (size_t i=0; i<old_pts.size(); i++) {
                float b_by_d = baseline / old_disp[i];
                float Z = focal * b_by_d;
                float X = (old_pts[i].x-pp.x) * b_by_d;
                float Y = (old_pts[i].y-pp.y) * b_by_d;
                
                prevPts3d.push_back(Point3f(X,Y,Z));
            }
            
            // Tranfrom to current-camera-coordinates.
            // To predict current camera pose(relative to previous camera),
            // we assume that it's just the same as (the relative pose between last_camera & last_last_camera).
            Mat oldR;
            rvec2mat(oldRvec, oldR);
            Transform3D(prevPts3d, prediPts3d, oldR, oldTvec);
            
            // Project these points to image plane.
            projectObj2img(prediPts3d, prediPts, cam, canBeSeen);
            
            
            
            
            //-------------Tracker----------------------------------//
            Mat err;
            vector<uchar> f_status;
            vector<Point2f> new_pts;
            
            // Optical flow tracker
            new_pts = prediPts;
            calcOpticalFlowPyrLK(old_img_l, img_l, old_pts, new_pts, f_status, err, Size(7,7));
            
            // Check: If the distance between the tracked and the predicted locations exceeds 15,
            // it may be a wrong association, we just discard it.
            size_t tracked = 0;
            if (numFrame > start_frame+1) {
                for (size_t i=0; i<old_pts.size(); i++) {
                    if (f_status[i]) {
                        if (sqrt(pow(prediPts[i].x-new_pts[i].x, 2.0)+pow(prediPts[i].y-new_pts[i].y, 2))<15) {
                            old_pts[tracked] = old_pts[i];
                            new_pts[tracked] = new_pts[i];
                            old_disp[tracked] = old_disp[i];
                            tracked++;
                        }
                    }
                }
                old_pts.resize(tracked);
                new_pts.resize(tracked);
                old_disp.resize(tracked);
                cout<<"# tracked:       \t"<<tracked<<endl;
            }
            
            
            // Display
            Mat img1_c;
            cvtColor(old_img_l, img1_c, CV_GRAY2BGR);
            for (size_t i=0; i<old_pts.size(); i++) {
                circle(img1_c, old_pts[i], 4, Scalar(0,0,255), -1);
                line(img1_c, old_pts[i], new_pts[i], Scalar(100,200,100), 2);
            }
            resize(img1_c, img1_c, Size(img1_c.cols/1.5, img1_c.rows));
            imshow("window", img1_c);
            
            
            
            //---------------Compute the relative camera pose--------------//
            // 1. Calculate the 3D points
            vector<Point3f> pts3d;
            for (size_t i=0; i<old_pts.size(); i++) {
                float b_by_d = baseline / old_disp[i];
                float Z = focal * b_by_d;
                float X = (old_pts[i].x-pp.x) * b_by_d;
                float Y = (old_pts[i].y-pp.y) * b_by_d;
                pts3d.push_back(Point3f(X,Y,Z));
            }
            
            // 2. Ransac p3p
            Mat rvec, tvec;
            vector<int> inliers;
            solvePnPRansac(pts3d, new_pts, cam_mat, Mat(), rvec, tvec, false, 500, 2.0f, 0.999, inliers, SOLVEPNP_ITERATIVE);
            cout<<"# inliers:       \t"<<inliers.size()<<endl;
            
            // 3. Reject outliers, PnP
            size_t k=0;
            for (size_t i=0; i<inliers.size(); i++) {
                pts3d[k] = pts3d[inliers[i]];
                new_pts[k] = new_pts[inliers[i]];
                k++;
            }
            pts3d.resize(k);
            new_pts.resize(k);
          
            solvePnP(pts3d, new_pts, cam_mat, Mat(), rvec, tvec);
            
            
            
            
            //--------------------------------------------------------------//
            // Check the result, in case it's too bad.
            // We assumes that :
            // 1. The car mainly moves foward
            // 2. Speed < 35 m/s
            // 3. The difference between previous&current translation < 1 m
            if (numFrame > start_frame+2) {
                double tx = tvec.at<double>(0,0);
                double ty = tvec.at<double>(1,0);
                double tz = tvec.at<double>(2,0);
                double oldtx = oldTvec.at<double>(0,0);
                double oldty = oldTvec.at<double>(1,0);
                double oldtz = oldTvec.at<double>(2,0);
                double difft = sqrt(pow(tx-oldtx, 2) + pow(ty-oldty, 2) + pow(tz-oldtz, 2));
                if (fabs(tx)>0.5 || fabs(ty)>0.5 || fabs(tz)>3.5 || difft > 1) {
                    rvec = oldRvec;
                    tvec = oldTvec;
                }
            }
            
            
            // Save the result
            fout<<numFrame<<"\t";
            fout<<rvec.at<double>(0,0)<<"  \t";
            fout<<rvec.at<double>(1,0)<<"  \t";
            fout<<rvec.at<double>(2,0)<<"  \t";
            fout<<tvec.at<double>(0,0)<<"  \t";
            fout<<tvec.at<double>(1,0)<<"  \t";
            fout<<tvec.at<double>(2,0)<<"  \t";
            fout<<new_pts.size()<<endl;
            
            
            // Update oldRvec and oldTvec
            oldRvec = rvec;
            oldTvec = tvec;
            

        }
        
        // Update
        old_img_l = img_l.clone();
        old_pts = topPts;
        old_disp = disp;
        
        clock_t te = clock();
        cout << "time\t" << (double)(te-ts)/CLOCKS_PER_SEC << "\n";
    

    }
    
    fout.close();
    
    return 0;
    
}
