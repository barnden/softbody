/*
 * Copyright (c) 2024, Brandon G. Nguyen <brandon@nguyen.vc>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#pragma once

#include <Eigen/Dense>

#define nodiscard [[nodiscard]]

#ifdef __GNUC_
#    define[[gnu::flatten]]
#else
#    define flatten __attribute__((flatten))
#endif

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Vec3i = Eigen::Vector3i;
