#ifndef __COMMON_H__
#define __COMMON_H__

namespace local_explorer
{
const double SENSOR_RANGE = 5.0;
const double MIN_RANGE_FOR_EXTENSION = 0.5;
const double MAP_DIMENSION[3] = {15.0, 15.0, 8.0};
const double BOARDER_DIMENSION[3] = {10.0, 10.0, 3.0};
const double MAP_RESOLUTION = 0.15;
const double BOARDER_RESOLUTION = 0.25;
const double BOARDER_HEIGHT_RANGE[2] = {0.0, 3.0};

const double INVERT_PARAM = 1.018;
const double INVERT_CLOUD_VISUALIZE_PARAM = 8.0;
const double ALLOWED_BOARDER_ERROR = 1e-5;

const float MIN_FRONTIER_RIDGE_LENGTH = 0.8;
const float MIN_FRONTIER_CLUSTER_AREA = 0.2;
} // namespace local_explorer

#endif