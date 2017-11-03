//
//  radar_image_process.h
//  DWRFinder
//
//  Created by ZachQin on 2017/3/8.
//  Copyright © 2017年 Zach. All rights reserved.
//

#ifndef radar_image_process_h
#define radar_image_process_h

#include <CoreGraphics/CoreGraphics.h>

#ifdef __cplusplus
extern "C" {
#endif


CGImageRef CreateCGImageFromFile(const char *path);
char *CreateMaskFromCGImage(CGImageRef image, int *width, int *height);

#ifdef __cplusplus
}
#endif
    
#endif /* radar_image_process_h */
