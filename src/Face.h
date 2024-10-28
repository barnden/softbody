/*
 * Copyright (c) 2024, Brandon G. Nguyen <brandon@nguyen.vc>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#pragma once

#include "utils.h"

#include "Particle.h"
#include "State.h"

struct Face {
    size_t v0;
    size_t v1;
    size_t v2;

    Face(size_t v0, size_t v1, size_t v2);
    Face(Particle const& v0, Particle const& v1, Particle const& v2);

    nodiscard Vec3 normal(State const&) const;
    nodiscard double distance_to_plane(Vec3 const&, State const&) const;

    std::pair<double, double> barycentric(Vec2 const&, State const&) const;
    nodiscard bool collision(Particle const&, State const& initial, State const& final) const;
    nodiscard Vec2 project(Vec3 const&, State const&) const;

    nodiscard inline Vec3 force(State const& state) const
    {
        return Vec3::Zero();
    }
};
