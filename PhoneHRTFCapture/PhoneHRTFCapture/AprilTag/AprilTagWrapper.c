//
//  AprilTagWrapper.c
//  PhoneHRTFCapture
//

#include "AprilTagWrapper.h"

#include <stdlib.h>
#include "apriltag.h"
#include "apriltag_pose.h"
#include "tag36h11.h"
#include "common/matd.h"
#include "common/zarray.h"

struct ATDetector {
    apriltag_detector_t *td;
    apriltag_family_t *tf;
};

ATDetector *at_detector_create(float quad_decimate, int nthreads) {
    ATDetector *d = calloc(1, sizeof *d);
    if (!d) return NULL;
    d->tf = tag36h11_create();
    d->td = apriltag_detector_create();
    apriltag_detector_add_family_bits(d->td, d->tf, 1);
    d->td->quad_decimate = quad_decimate;
    d->td->quad_sigma = 0.0f;
    d->td->nthreads = nthreads;
    d->td->refine_edges = true;      // re-fits edges at full resolution after decimation
    d->td->decode_sharpening = 0.25;
    return d;
}

void at_detector_destroy(ATDetector *d) {
    if (!d) return;
    apriltag_detector_destroy(d->td);
    tag36h11_destroy(d->tf);
    free(d);
}

int at_detect(ATDetector *d,
              const uint8_t *gray, int width, int height, int stride,
              double tag_size, double fx, double fy, double cx, double cy,
              ATDetection *out, int max_out) {
    if (!d || !gray || !out || max_out <= 0) return 0;

    image_u8_t im = { .width = width, .height = height, .stride = stride, .buf = (uint8_t *)gray };
    zarray_t *dets = apriltag_detector_detect(d->td, &im);
    if (!dets) return 0;

    int n = 0;
    for (int i = 0; i < zarray_size(dets) && n < max_out; i++) {
        apriltag_detection_t *det;
        zarray_get(dets, i, &det);

        ATDetection *o = &out[n++];
        o->id = det->id;
        o->hamming = det->hamming;
        o->decision_margin = det->decision_margin;
        for (int k = 0; k < 4; k++) {
            o->corners[2 * k] = det->p[k][0];
            o->corners[2 * k + 1] = det->p[k][1];
        }
        o->center[0] = det->c[0];
        o->center[1] = det->c[1];

        apriltag_detection_info_t info = {
            .det = det, .tagsize = tag_size, .fx = fx, .fy = fy, .cx = cx, .cy = cy
        };
        apriltag_pose_t pose = { 0 };
        o->pose_error = estimate_tag_pose(&info, &pose);
        for (int k = 0; k < 9; k++) o->R[k] = pose.R ? pose.R->data[k] : 0;
        for (int k = 0; k < 3; k++) o->t[k] = pose.t ? pose.t->data[k] : 0;
        if (pose.R) matd_destroy(pose.R);
        if (pose.t) matd_destroy(pose.t);
    }
    apriltag_detections_destroy(dets);
    return n;
}

int at_rigid_fit(const double *src, const double *dst, int n, double R[9], double t[3]) {
    if (n < 3) return 0;

    double cs[3] = { 0 }, cd[3] = { 0 };
    for (int i = 0; i < n; i++)
        for (int k = 0; k < 3; k++) { cs[k] += src[3 * i + k] / n; cd[k] += dst[3 * i + k] / n; }

    // Cross-covariance H = sum (s - cs)(d - cd)^T
    matd_t *H = matd_create(3, 3);
    for (int i = 0; i < n; i++)
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                MATD_EL(H, r, c) += (src[3 * i + r] - cs[r]) * (dst[3 * i + c] - cd[c]);

    // H = U S V^T  ->  R = V U^T (with reflection fix)
    matd_svd_t svd = matd_svd(H);
    double V[9], U[9];
    for (int k = 0; k < 9; k++) { V[k] = svd.V->data[k]; U[k] = svd.U->data[k]; }
    matd_destroy(H); matd_destroy(svd.U); matd_destroy(svd.S); matd_destroy(svd.V);

    for (int pass = 0; pass < 2; pass++) {
        for (int r = 0; r < 3; r++)
            for (int c = 0; c < 3; c++)
                R[3 * r + c] = V[3 * r + 0] * U[3 * c + 0] + V[3 * r + 1] * U[3 * c + 1] + V[3 * r + 2] * U[3 * c + 2];
        double det = R[0] * (R[4] * R[8] - R[5] * R[7])
                   - R[1] * (R[3] * R[8] - R[5] * R[6])
                   + R[2] * (R[3] * R[7] - R[4] * R[6]);
        if (det >= 0) break;
        for (int r = 0; r < 3; r++) V[3 * r + 2] = -V[3 * r + 2];   // flip smallest singular direction
    }

    for (int r = 0; r < 3; r++)
        t[r] = cd[r] - (R[3 * r] * cs[0] + R[3 * r + 1] * cs[1] + R[3 * r + 2] * cs[2]);
    return 1;
}
