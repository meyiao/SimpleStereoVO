//  Frame.hpp
//
//  Created by 谭谭谭 on 16/3/8.
//  Copyright © 2016年 谭谭谭. All rights reserved.
//

#ifndef Frame_hpp
#define Frame_hpp

#include <stdio.h>
#include <vector>

#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

class Frame
{
public:
    Frame(Mat& img, int bs, Point2f ppt, float focalt, float baselinet);
    
    // Assign detected FAST corners into grids
    void GetFeaturesIntoGrid(vector<Point2f>& vPoints, vector< vector<int> >& subidx);
    
    // For all FAST corners belonging to the same grid, we sort them according to their
    // FAST-response scores.
    void SortSubIdx(vector<KeyPoint>& vKeyPoints, vector<vector<int> >& subidx);
    
    // For each grid, only keep the N FAST corners with highest scores.
    void SelectTopN(vector<Point2f>& iPoints, vector<Point2f>& oPoints, vector<vector<int>> & subidx, int N);
    
    // Calculate stereo disparities.
    void FindDisparity(vector<Point2f>& pts1, vector<float>& disp, Mat& img1, Mat& img2, float ythresh);
    
    // Convert a state vector to rotaion matrix R, and translation vector T.
    // void st2rt(vector<float>& stat, Mat& R, Mat& T);
    
    
private:
    int im_width;       // image width
    int im_height;      // image height
    int block_size;     // grid size
    Point2f pp;         // principle point
    float focal;        // focal length
    float baseline;     // stereo base line
};

#endif /* Frame_hpp */