//
//  radar_image_process.c
//  DWRFinder
//
//  Created by ZachQin on 2017/3/8.
//  Copyright © 2017年 Zach. All rights reserved.
//

#include "radar_image_process.h"
#include <ImageIO/ImageIO.h>


char *CreateMaskFromCGImage(CGImageRef image, int *width, int *height) {
    
    int image_width = (int)CGImageGetWidth(image);
    int image_height = (int)CGImageGetHeight(image);
    *width = image_width;
    *height = image_height;
    CGColorSpaceRef colorSpace = CGColorSpaceCreateDeviceRGB();
    char *raw_data = (char *) calloc(image_height * image_width * 4, sizeof(char));
    char *mask_data = (char *)calloc(image_height * image_width, sizeof(char));
    
    size_t bytesPerPixel = 4;
    size_t bytesPerRow = bytesPerPixel * image_width;
    size_t bitsPerComponent = 8;
    CGContextRef context = CGBitmapContextCreate(raw_data, image_width, image_height,
                                                 bitsPerComponent, bytesPerRow, colorSpace,
                                                 kCGImageAlphaPremultipliedLast | kCGBitmapByteOrder32Big);
    CGColorSpaceRelease(colorSpace);
    
    CGContextDrawImage(context, CGRectMake(0, 0, image_width, image_height), image);
    CGContextRelease(context);
    
    // Now your raw_data contains the image data in the RGBA8888 pixel format.
    for (size_t byte_index = 0 ; byte_index < bytesPerRow * image_height ; byte_index += bytesPerPixel)
    {
//        int alpha = raw_data[byteIndex + 3];
        int red   = raw_data[byte_index + 0];
        int green = raw_data[byte_index + 1];
        int blue  = raw_data[byte_index + 2];

        if (red || green || blue) {
            mask_data[byte_index / bytesPerPixel] = 1;
        }
    }
    
    free(raw_data);
    return mask_data;
}
