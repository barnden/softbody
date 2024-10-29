/*
 * Copyright (c) 2024, Brandon G. Nguyen <brandon@nguyen.vc>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#pragma once

#include <Eigen/Dense>
#include <ranges>

#define nodiscard [[nodiscard]]

#ifdef __GNUC__
#    define flatten [[gnu::flatten]]
#else
#    define flatten __attribute__((flatten))
#endif

#define hot __attribute__((hot))
#define enumerate(v) std::views::zip(std::views::iota(1), v)

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Vec3i = Eigen::Vector3i;
