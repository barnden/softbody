#pragma once

#include "utils.h"

#include "Particle.h"

struct Face {
    size_t v0;
    size_t v1;
    size_t v2;

    Vec3 normal;

    Face(size_t v0, size_t v1, size_t v2);
    Face(Particle v0, Particle v1, Particle v2);

    nodiscard double distance_to_plane(Vec3 p) const;

    std::pair<double, double> barycentric(Vec2 p) const;
    nodiscard bool collision(Particle p) const;

    nodiscard inline Vec3 force() const
    {
        return Vec3::Zero();
    }

    nodiscard Vec2 project(Vec3 p) const;
};
