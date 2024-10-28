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
    Face(Particle v0, Particle v1, Particle v2);

    nodiscard Vec3 normal() const;
    nodiscard Vec3 normal(State const&) const;

    nodiscard double distance_to_plane(Vec3 const& p) const;
    nodiscard double distance_to_plane(Vec3 const& p, State const&) const;

    std::pair<double, double> barycentric(Vec2 p) const;
    nodiscard bool collision(Particle p) const;

    nodiscard inline Vec3 force() const
    {
        return Vec3::Zero();
    }

    nodiscard Vec2 project(Vec3 p) const;
};
