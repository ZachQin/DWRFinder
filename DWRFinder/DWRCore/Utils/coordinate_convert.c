//
//  coordinate_convert.c
//  DWRFinder
//
//  Created by ZachQin on 2017/3/22.
//  Copyright © 2017年 Zach. All rights reserved.
//

#include "coordinate_convert.h"
#include <math.h>

const double kMercParamK0 = 1.0;
const double kMercParamE = 0.081819190842621486;
const double kMercParamFrMeter = 1.0;
const double kMercParamToMeter = 1.0;
const double kMercParamA = 6378137.0;
const double kMercParamX0 = 0.0;
const double kMercParamY0 = 0.0;
const double kMercParamRa = 0.00000015678559428873979;

#define TOL 1.0e-10
#define N_ITER 15

static double pj_tsfn(double phi, double sinphi, double e) {
    sinphi *= e;
    return (tan (.5 * (M_PI_2 - phi)) /
            pow((1. - sinphi) / (1. + sinphi), .5 * e));
}

static double pj_phi2(double ts, double e) {
    double eccnth, Phi, con, dphi;
    int i;
    
    eccnth = .5 * e;
    Phi = M_PI_2 - 2. * atan (ts);
    i = N_ITER;
    do {
        con = e * sin (Phi);
        dphi = M_PI_2 - 2. * atan (ts * pow((1. - con) /
                                            (1. + con), eccnth)) - Phi;
        Phi += dphi;
    } while ( fabs(dphi) > TOL && --i);
    return Phi;
}

void MercToLonLat(double x, double y, double *lon, double *lat) {
    x = (x * kMercParamToMeter - kMercParamX0) * kMercParamRa;
    y = (y * kMercParamToMeter - kMercParamY0) * kMercParamRa;
    *lon = x / kMercParamK0;
    *lat = pj_phi2(exp(-y / kMercParamK0), kMercParamE);
}

void LonLatToMerc(double lon, double lat, double *x, double *y) {
    *x = kMercParamK0 * lon;
    *y = -kMercParamK0 * log(pj_tsfn(lat, sin(lat), kMercParamE));
    *x = kMercParamFrMeter * (*x * kMercParamA + kMercParamX0);
    *y = kMercParamFrMeter * (*y * kMercParamA + kMercParamY0);
}
