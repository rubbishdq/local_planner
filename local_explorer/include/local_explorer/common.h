#ifndef __COMMON_H__
#define __COMMON_H__

//#define __DEBUG__
namespace local_explorer
{
typedef int flag_t;

const bool USE_HEIGHT_DIFF_THRESHOLD = true;

const double SENSOR_RANGE = 5.0;
const float FX = 343.15219323854245;
const float FY = 343.15219323854245;
const float CX = 320.5;
const float CY = 180.5;
const int CAM_RES[2] = {640, 360};

const double MIN_RANGE_FOR_IGNORANCE = 0.15;
const double MIN_RANGE_FOR_EXTENSION = 1.0;
const int EXTENSION_COUNT = 1;
const double EXTENSION_LENGTH_THRESHOLD = 5e-3;
const double EXTENSION_PARAM = 2.0;

const double MAP_DIMENSION[3] = {15.0, 15.0, 8.0};
const double BOARDER_DIMENSION[3] = {10.0, 10.0, 6.0};
const double MAP_RESOLUTION = 0.15;
const double BOARDER_RESOLUTION = 0.3;
const double BOARDER_Z_RANGE[2] = {-0.05, 3.05};
const double HEIGHT_DIFF_THRESHOLD = 0.71;
const double RAND_ERROR = 1e-2;  // avoid computational error of qhull

const double INVERT_PARAM = 1.018;
const double INVERT_CLOUD_VISUALIZE_PARAM = 8.0;
const double ALLOWED_BOARDER_ERROR = 1e-5;

const float MIN_FRONTIER_RIDGE_LENGTH = 1.0;
const float MIN_FRONTIER_CLUSTER_AREA = 1.0;
//const float FRONTIER_CLUSTER_SIZE_LIMIT[3] = {3.0, 3.0, 1.5};
const float FRONTIER_CLUSTER_SIZE_LIMIT[3] = {15.0, 15.0, 6.0};

const int FRONTIER_COLOR_COUNT = 4096;  // used to visualize frontier clusters
const float MARKER_ALPHA = 0.8;

const int KD_TREE_MAX_DEPTH = 10;
const int KD_TREE_MIN_OBJ_COUNT = 50;
const float KD_TREE_IN_THRESHOLD = 0.5;

const double NEW_VIEWPOINT_DIST_THRESHOLD = 1.0;
const double UPDATE_VIEWPOINT_DIST_THRESHOLD = 0.05;
const double UPDATE_VIEWPOINT_ANGLE_THRESHOLD = 30;

const float EPS_F = 2e-4;
} // namespace local_explorer

#endif