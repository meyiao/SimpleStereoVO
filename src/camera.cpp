//
//  camera.cpp
//  dabino
//
//  Created by 谭智丹 on 16/4/27.
//  Copyright © 2016年 谭智丹. All rights reserved.
//

#include "camera.hpp"

/* we can use
 camera cam(.., .., ......);
 to set the camera intrinsics
 */

camera::camera(int width, int height, float focalx, float focaly, float ppx, float ppy)
{
    imwidth = width;
    imheight = height;
    fx = focalx;
    fy = focaly;
    cx = ppx;
    cy = ppy;
    camMatrix = (Mat_<float>(3,3)<<focalx, 0.0f, ppx, 0.0f, focaly, ppy, 0.0f, 0.0f, 1.0f);
}
