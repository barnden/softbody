#pragma once

#include <Eigen/Dense>

#define nodiscard [[nodiscard]]

#ifdef __GNUC_
#    define[[gnu::flatten]]
#else
#    define flatten __attribute__(flatten)
#endif

using Vec3 = Eigen::Vector3d;