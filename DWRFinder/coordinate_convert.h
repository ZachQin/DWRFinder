//
//  coordinate_convert.h
//  DWRFinder
//
//  Created by ZachQin on 2017/3/22.
//  Copyright © 2017年 Zach. All rights reserved.
//

#ifndef coordinate_convert_h
#define coordinate_convert_h

#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

void MercToLonLat(double x, double y, double *lon, double *lat);
void LonLatToMerc(double lon, double lat, double *x, double *y);
    
#ifdef __cplusplus
}
#endif

#endif /* coordinate_convert_h */
