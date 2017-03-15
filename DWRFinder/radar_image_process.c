//
//  radar_image_process.c
//  DWRFinder
//
//  Created by ZachQin on 2017/3/8.
//  Copyright © 2017年 Zach. All rights reserved.
//

#include "radar_image_process.h"
#include <ImageIO/ImageIO.h>


char *CreateMaskFromCGImage(CGImageRef image, size_t *width, size_t *height) {
    
    size_t imageWidth = CGImageGetWidth(image);
    size_t imageHeight = CGImageGetHeight(image);
    *width = imageWidth;
    *height = imageHeight;
    CGColorSpaceRef colorSpace = CGColorSpaceCreateDeviceRGB();
    char *rawData = (char*) calloc(imageHeight * imageWidth * 4, sizeof(char));
    char *maskData = (char*)calloc(imageHeight * imageWidth, sizeof(char));
    
    size_t bytesPerPixel = 4;
    size_t bytesPerRow = bytesPerPixel * imageWidth;
    size_t bitsPerComponent = 8;
    CGContextRef context = CGBitmapContextCreate(rawData, imageWidth, imageHeight,
                                                 bitsPerComponent, bytesPerRow, colorSpace,
                                                 kCGImageAlphaPremultipliedLast | kCGBitmapByteOrder32Big);
    CGColorSpaceRelease(colorSpace);
    
    CGContextDrawImage(context, CGRectMake(0, 0, imageWidth, imageHeight), image);
    CGContextRelease(context);
    
    // Now your rawData contains the image data in the RGBA8888 pixel format.
    for (size_t byteIndex = 0 ; byteIndex < bytesPerRow * imageHeight ; byteIndex += bytesPerPixel)
    {
//        int alpha = rawData[byteIndex + 3];
        int red   = rawData[byteIndex + 0];
        int green = rawData[byteIndex + 1];
        int blue  = rawData[byteIndex + 2];

        if (red || green || blue) {
            maskData[byteIndex / bytesPerPixel] = 1;
        }
    }
    
    free(rawData);
    return maskData;
}
