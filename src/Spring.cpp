/*
 * Copyright (c) 2024, Brandon G. Nguyen <brandon@nguyen.vc>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#include "Spring.h"
#include "Manager.h"

Spring::Spring(size_t p0, size_t p1, double k, double d)
    : p0(p0)
    , p1(p1)
    , k(k)
    , d(d)
{
    L0 = (g_manager->position(p1) - g_manager->position(p0)).norm();
}

Spring::Spring(Particle p0, Particle p1, double k, double d)
    : p0(p0.index)
    , p1(p1.index)
    , k(k)
    , d(d)
{
    L0 = (p1.position() - p0.position()).norm();
}

Vec3 Spring::force() const
{
    Vec3 delta = g_manager->position(p1) - g_manager->position(p0);
    double L = delta.norm();

    delta.normalize();
    double dV = (g_manager->velocity(p1) - g_manager->position(p0)).dot(delta);

    Vec3 Fs = k * (L - L0) * delta;
    Vec3 Fd = d * dV * delta;

    return Fs + Fd;
}

std::pair<double, double> Spring::lerp(Spring& spring) const {
    Vec3 e1 = (g_manager->position(p1) - g_manager->position(p0)).normalized();
    Vec3 e2 = (g_manager->position(spring.p1) - g_manager->position(spring.p0)).normalized();

    Vec3 n = e1.cross(e2);
    Vec3 q = g_manager->position(spring.p0) - g_manager->position(p0);

    double s = q.dot(e2.cross(n)) / (e1.dot(e2.cross(n)));
    double t = -q.dot(e1.cross(n)) / (e2.dot(e1.cross(n)));

    return std::make_pair(s, t);
}

flatten bool Spring::collision(Spring& spring) const {
    auto const [s, t] = lerp(spring);

    if (s < 0. || s > 1. || t < 0. || t > 1.)
        return false;

    return true;
}