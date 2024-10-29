/*
 * Copyright (c) 2024, Brandon G. Nguyen <brandon@nguyen.vc>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#include "Face.h"
#include "Scene.h"
#include "Simulation.h"
#include <iostream>
Face::Face(size_t v0, size_t v1, size_t v2)
    : m_v0(v0)
    , m_v1(v1)
    , m_v2(v2) {};

flatten Face::Face(Particle const& v0, Particle const& v1, Particle const& v2)
    : Face(v0.index, v1.index, v2.index) {};

inline Vec3 Face::normal(State const& state) const
{
    Vec3 p0 = state.data0[m_v0];
    Vec3 N = (state.data0[m_v1] - p0).cross(state.data0[m_v2] - p0);

    return N.normalized();
}

double inline Face::distance_to_plane(Vec3 const& p, State const& state) const
{
    return (p - state.data0[m_v0]).dot(normal(state));
}

// inline Vec2 Face::project(Vec3 const& p, State const& state) const
// {
//     // Project into 2D plane by dropping largest coordinate
//     Vec3 N = normal(state);

//     // FIXME: There's definitely a way to do this with fewer comparisons
//     if (std::abs(N.x()) > std::abs(N.y()) && std::abs(N.x()) > std::abs(N.z()))
//         return { p.y(), p.z() };

//     if (std::abs(N.y()) > std::abs(N.x()) && std::abs(N.y()) > std::abs(N.z()))
//         return { p.x(), p.z() };

//     return { p.x(), p.y() };
// }

inline Vec2 Face::project(Vec3 const& p, State const& state) const
{
    // Project into 2D plane containing the face
    Vec3 n = normal(state);
    Vec3 const& p0 = state.data0[m_v0];
    Vec3 p_prime = p - ((p - p0).dot(n) * n) - p0;

    Vec3 u = (state.data0[m_v1] - p0).normalized();
    Vec3 v = u.cross(n);

    return { p_prime.dot(u), p_prime.dot(v) };
}

inline decltype(auto) barycentric(Vec2 const& p, Vec2 const& p0, Vec2 const& p1, Vec2 const& p2)
{
    // This is area of the parallelogram, not the triangle; however, the denominator uses (2 * area) so leaving it like this is OK
    double area = (Eigen::Matrix2d() << (p1 - p0).transpose(), (p2 - p0).transpose()).finished().determinant();

    // Transform p into barycentric coordinates w.r.t. the face's 2D projection
    double alpha = (p0.y() * p2.x() - p0.x() * p2.y() + (p2.y() - p0.y()) * p.x() + (p0.x() - p2.x()) * p.y()) / area;
    double beta = (p0.x() * p1.y() - p0.y() * p1.x() + (p0.y() - p1.y()) * p.x() + (p1.x() - p0.x()) * p.y()) / area;

    return std::make_pair(alpha, beta);
}

flatten std::pair<double, double> Face::barycentric(Vec2 const& p, State const& state) const
{
    // Project triangle and point of collision into 2D by dropping largest coordinate in normal
    auto const proj = std::bind(&Face::project, this, std::placeholders::_1, state);
    auto p0 = proj(state.data0[m_v0]);
    auto p1 = proj(state.data0[m_v1]);
    auto p2 = proj(state.data0[m_v2]);

    return ::barycentric(p, p0, p1, p2);
}

template <typename T>
int sgn(T val) { return (T {} < val) - (val < T {}); }

/**
 * Detect collision of a (moving) face against a static point
 *
 * Using this with StaticFace always returns the same result
 */
flatten std::optional<CollisionRecord> Face::collision(
    Vec3 const& position,
    State const& state_initial,
    State const& state_final) const
{
    // Check collision with plane
    auto di = distance_to_plane(position, state_initial);
    auto df = distance_to_plane(position, state_final);

    if (sgn(di) == sgn(df))
        return std::nullopt;

    // Get point of collision with plane
    auto f = di / (di - df);
    auto collision_state = g_simulation->integrate(state_initial, f * g_simulation->timestep());
    auto pc = project(position, collision_state);

    // Check if barycentric coordinates lie in the triangle
    auto const [alpha, beta] = barycentric(pc, collision_state);
    auto const gamma = 1. - alpha - beta;
    if ((alpha < 0) || (beta < 0.) || (gamma < 0.))
        return std::nullopt;

    return CollisionRecord {
        .type = CollisionRecord::Type::STATIC,
        .state = collision_state,
        .fractional_timestep = f,
        .indices = { m_v0, m_v1, m_v2 },
        .weights = { alpha, beta, gamma },
        .normal = normal(collision_state),
    };
}

/**
 * Detect collision of a moving particle against a (potentially moving) face
 */
flatten std::optional<CollisionRecord> Face::collision(
    Particle const& particle,
    State const& state_initial,
    State const& state_final) const
{
    Vec3 pi = state_initial.data0[particle.index];
    Vec3 pf = state_final.data0[particle.index];

    // Check collision with plane
    auto di = distance_to_plane(pi, state_initial);
    auto df = distance_to_plane(pf, state_final);

    if (sgn(di) == sgn(df))
        return std::nullopt;

    // Get point of collision with plane
    auto f = di / (di - df);
    auto collision_state = g_simulation->integrate(state_initial, f * g_simulation->timestep());
    auto pc = project(collision_state.data0[particle.index], collision_state);

    // Check if barycentric coordinates lie in the triangle
    auto const [alpha, beta] = barycentric(pc, collision_state);
    auto const gamma = 1. - alpha - beta;
    if ((alpha < 0) || (beta < 0.) || (gamma < 0.))
        return std::nullopt;

    return CollisionRecord {
        .type = CollisionRecord::Type::STATIC,
        .state = collision_state,
        .fractional_timestep = f,
        .indices = { particle.index, -1uz, -1uz },
        .weights = { 1., 0., 0. },
        .normal = normal(collision_state),
    };
}

// flatten decltype(auto) lerp(size_t p0, size_t p1, size_t q0, size_t q1, State const& state)
// {
//     Vec3 e1 = (state.data0[p1] - state.data0[p0]).normalized();
//     Vec3 e2 = (state.data0[q1] - state.data0[q0]).normalized();

//     Vec3 n = e1.cross(e2);
//     Vec3 q = state.data0[q0] - state.data0[p0];

//     double s = q.dot(e2.cross(n)) / (e1.dot(e2.cross(n)));
//     double t = q.dot(n.cross(e1)) / (e1.dot(e1.cross(n)));

//     Vec3 pa = state.data0[p0] + s * e1;
//     Vec3 qa = state.data0[q0] + t * e2;
//     Vec3 m = qa - pa;

//     return std::make_tuple(s, t, m);
// }

StaticFace::StaticFace(size_t v0, size_t v1, size_t v2)
    : Face(v0, v1, v2)
{
    auto const& p0 = g_scene->position(v0);
    m_normal = (g_scene->position(v1) - p0).cross(g_scene->position(v2) - p0);
    m_normal.normalize();
}

flatten double inline StaticFace::distance_to_plane(Vec3 const& p, State const&) const
{
    return (p - g_scene->position(m_v0)).dot(m_normal);
}

flatten std::pair<double, double> StaticFace::barycentric(Vec2 const& p, State const& state) const
{
    // Project triangle and point of collision into 2D by dropping largest coordinate in normal
    auto const proj = std::bind(&StaticFace::project, this, std::placeholders::_1, state);
    auto p0 = proj(g_scene->position(m_v0));
    auto p1 = proj(g_scene->position(m_v1));
    auto p2 = proj(g_scene->position(m_v2));

    return ::barycentric(p, p0, p1, p2);
}

inline Vec2 StaticFace::project(Vec3 const& p, State const& state) const
{
    // Project into 2D plane containing the face
    Vec3 const& p0 = g_scene->position(m_v0);
    Vec3 p_prime = p - ((p - p0).dot(m_normal) * m_normal) - p0;

    Vec3 u = (g_scene->position(m_v1) - p0).normalized();
    Vec3 v = u.cross(m_normal);

    return { p_prime.dot(u), p_prime.dot(v) };
}
