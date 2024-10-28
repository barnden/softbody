/*
 * Copyright (c) 2024, Brandon G. Nguyen <brandon@nguyen.vc>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#include "Spring.h"
#include "Manager.h"
#include "Particle.h"
#include "State.h"

Spring::Spring(size_t p0, size_t p1, double k, double d)
    : p0(p0)
    , p1(p1)
    , k(k)
    , d(d)
{
    L0 = (g_manager->position(p1) - g_manager->position(p0)).norm();
}

Spring::Spring(Particle const& p0, Particle const& p1, double k, double d)
    : p0(p0.index)
    , p1(p1.index)
    , k(k)
    , d(d)
{
    L0 = (p1.position() - p0.position()).norm();
}

Vec3 Spring::force(State const& state) const
{
    Vec3 delta = state.data0[p1] - state.data0[p0];
    double L = delta.norm();

    delta.normalize();
    double dV = (state.data1[p1] - state.data1[p0]).dot(delta);

    Vec3 Fs = k * (L - L0) * delta;
    Vec3 Fd = d * dV * delta;

    return Fs + Fd;
}

std::tuple<double, double, Vec3> Spring::lerp(Spring const& spring, State const& state) const
{
    Vec3 e1 = (state.data0[p1] - state.data0[p0]).normalized();
    Vec3 e2 = (state.data0[spring.p1] - state.data0[spring.p0]).normalized();

    Vec3 n = e1.cross(e2);
    Vec3 q = state.data0[spring.p0] - state.data0[p0];

    double s = q.dot(e2.cross(n)) / (e1.dot(e2.cross(n)));
    double t = -q.dot(e1.cross(n)) / (e2.dot(e1.cross(n)));

    Vec3 pa = state.data0[p0] + s * e1;
    Vec3 qa = state.data0[spring.p0] + t * e2;
    Vec3 m = qa - pa;

    return std::make_tuple(s, t, m);
}

flatten bool Spring::collision(Spring const& spring, State const& initial_state, State const& final_state) const
{
    // There must exist some point that overlaps the edges in some projection in initial and final states
    auto const [s1, t1, m1] = lerp(spring, initial_state);
    if (s1 < 0. || s1 > 1. || t1 < 0. || t1 > 1.)
        return false;

    if (m1.norm() < 1e-6)
        return true;

    auto const [s2, t2, m2] = lerp(spring, final_state);
    if (s2 < 0. || s2 > 1. || t2 < 0. || t2 > 1.)
        return false;

    if (m2.norm() < 1e-6)
        return true;

    if (m1.dot(m2) >= 0.)
        return false;

    return true;
}
