/*
 * Copyright (c) 2024, Brandon G. Nguyen <brandon@nguyen.vc>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#include "Face.h"
#include "Manager.h"

Face::Face(size_t v0, size_t v1, size_t v2)
    : v0(v0)
    , v1(v1)
    , v2(v2)
{
}

flatten Face::Face(Particle const& v0, Particle const& v1, Particle const& v2)
    : Face(v0.index, v1.index, v2.index) {};

inline Vec3 Face::normal(State const& state) const
{
    Vec3 p0 = state.data0[v0];
    Vec3 N = (state.data0[v1] - p0).cross(state.data0[v2] - p0);

    return N.normalized();
}

double Face::distance_to_plane(Vec3 const& p, State const& state) const
{
    return (p - state.data0[v0]).dot(normal(state));
}

Vec2 Face::project(Vec3 const& p, State const& state) const
{
    Vec3 N = normal(state);

    // FIXME: There's definitely a way to do this with fewer comparisons
    if (std::abs(N.x()) > std::abs(N.y()) && std::abs(N.x()) > std::abs(N.z()))
        return { p.y(), p.z() };

    if (std::abs(N.y()) > std::abs(N.x()) && std::abs(N.y()) > std::abs(N.z()))
        return { p.x(), p.z() };

    return { p.x(), p.y() };
}

/**
 * Vec2 Face::project(Vec3 p) const
{
    Vec3 const& p0 = g_manager->position(v0);
    Vec3 p_prime = p - ((p - p0).dot(normal()) * normal()) - p0;

    Vec3 u = (g_manager->position(v1) - p0).normalized();
    Vec3 v = u.cross(normal());

    return { p_prime.dot(u), p_prime.dot(v) };
}
 */

template <typename T>
int sgn(T val) { return (T {} < val) - (val < T {}); }

flatten std::pair<double, double> Face::barycentric(Vec2 const& p, State const& state) const
{
    // Project triangle and point of collision into 2D by dropping largest coordinate in normal
    auto p0 = project(state.data0[v0], state);
    auto p1 = project(state.data0[v1], state);
    auto p2 = project(state.data0[v2], state);

    // This is area of the parallelogram, not the triangle; however, the denominator uses (2 * area) so leaving it like this is OK
    double area = (Eigen::Matrix2d() << (p1 - p0).transpose(), (p2 - p0).transpose()).finished().determinant();

    // Transform p into barycentric coordinates w.r.t. the face's 2D projection
    double alpha = (p0.y() * p2.x() - p0.x() * p2.y() + (p2.y() - p0.y()) * p.x() + (p0.x() - p2.x()) * p.y()) / area;
    double beta = (p0.x() * p1.y() - p0.y() * p1.x() + (p0.y() - p1.y()) * p.x() + (p1.x() - p0.x()) * p.y()) / area;

    return std::make_pair(alpha, beta);
}

flatten bool Face::collision(Particle const& particle, State const& state_initial, State const& state_final) const
{
    Vec3 pi = state_initial.data0[particle.index];
    Vec3 pf = state_final.data0[particle.index];

    // Check collision with plane
    auto di = distance_to_plane(pi, state_initial);
    auto df = distance_to_plane(pf, state_final);

    if (sgn(di) == sgn(df))
        return false;

    // Get point of collision with plane
    auto f = di / (di - df);
    auto collision_state = g_manager->integrate(state_initial, f * g_manager->timestep());
    auto pc = project(collision_state.data0[particle.index], collision_state);

    // Check if barycentric coordinates lie in the triangle
    auto const [alpha, beta] = barycentric(pc, collision_state);
    return (alpha > 0.) && (beta > 0.) && ((1. - alpha - beta) > 0.);
}
