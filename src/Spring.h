/*
 * Copyright (c) 2024, Brandon G. Nguyen <brandon@nguyen.vc>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#pragma once

#include "utils.h"

struct Particle;
struct State;

struct Spring {
    size_t p0;
    size_t p1;

    double k; // spring constant
    double d; // damper constant
    double L0; // rest length

    Spring(size_t p0, size_t p1, double k = 1., double d = 1.);
    Spring(Particle const& p0, Particle const& p1, double k = 1., double d = 0.);

    nodiscard std::tuple<double, double, Vec3> lerp(Spring const&, State const&) const;
    nodiscard bool collision(Spring const&, State const&, State const&) const;

    nodiscard Vec3 force(State const&) const;
};
