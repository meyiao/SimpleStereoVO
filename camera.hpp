//
//  camera.hpp
//  dabino
//
//  Created by 谭智丹 on 16/4/27.
//  Copyright © 2016年 谭智丹. All rights reserved.
//

#ifndef camera_hpp
#define camera_hpp

#include <stdio.h>

#include <opencv2/opencv.hpp>

using namespace cv;

class camera
{
public:
    camera(int width, int height, float focalx, float focaly, float ppx, float ppy);
    float imwidth;      // image width
    float imheight;     // image height
    float fx;           // focal length x
    float fy;           // focal length y
    float cx;           // principal point's x coordinate
    float cy;           // principal point's y coordinate
    Mat   camMatrix;    // camera projection matrix
};


#endif /* camera_hpp */
