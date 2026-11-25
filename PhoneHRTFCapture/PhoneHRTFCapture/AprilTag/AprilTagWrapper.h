//
//  AprilTagWrapper.h
//  PhoneHRTFCapture
//
//  Thin C interface over the AprilRobotics AprilTag library (v3.4.5, BSD-2,
//  see LICENSE.md in this folder), so Swift never touches matd_t / zarray_t.
//
//  Conventions (same as apriltag_pose.c):
//    camera frame:  x right, y down, z forward (OpenCV), metres
//    tag frame:     corner i of `corners` is the tag-frame point
//                   i=0 (-s/2, +s/2, 0), 1 (+s/2, +s/2, 0), 2 (+s/2, -s/2, 0), 3 (-s/2, -s/2, 0)
//                   with s = tag size (outer edge of the black square)
//    pose:          p_camera = R * p_tag + t
//

#ifndef APRILTAG_WRAPPER_H
#define APRILTAG_WRAPPER_H

#include <stdint.h>

typedef struct {
    int id;
    int hamming;
    float decision_margin;
    double corners[8];   // x0, y0, x1, y1, ... image pixels
    double center[2];    // image pixels
    double R[9];         // row-major
    double t[3];         // metres
    double pose_error;   // object-space error from orthogonal iteration
} ATDetection;

typedef struct ATDetector ATDetector;

/// tag36h11 detector. Corrects at most 1 bit to keep false positives low.
ATDetector *at_detector_create(float quad_decimate, int nthreads);
void at_detector_destroy(ATDetector *detector);

/// Detects tags in an 8-bit grayscale image (e.g. the Y plane of ARFrame.capturedImage)
/// and estimates each tag's pose. Returns the number of detections written to `out`.
int at_detect(ATDetector *detector,
              const uint8_t *gray, int width, int height, int stride,
              double tag_size, double fx, double fy, double cx, double cy,
              ATDetection *out, int max_out);

/// Least-squares rigid transform (Kabsch): finds R (row-major), t minimising
/// sum |R * src_i + t - dst_i|^2 over n points (xyz triples). Returns 0 if n < 3.
int at_rigid_fit(const double *src, const double *dst, int n, double R[9], double t[3]);

#endif
