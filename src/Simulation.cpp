/*
 * Copyright (c) 2024, Brandon G. Nguyen <brandon@nguyen.vc>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#include "Simulation.h"
#include "Scene.h"
#include "State.h"
#include <iostream>

hot flatten bool resting(size_t i, State const& state)
{
    // Resting conditions

    // 1. |v| < eps
    if (state.data1[i].squaredNorm() > 1e-5)
        return false;

    // 2. There exists some surface s.t. distance(particle, surface) < eps
    bool within_distance = false;
    Vec3 normal = Vec3::Zero();

    // 2.1. First check against static scene
    for (auto&& face : g_scene->faces()) {
        if (face.distance_to_plane(state.data0[i], state) < 1e-4) {
            Vec2 p = face.project(state.data0[i], state);
            auto const [alpha, beta] = face.barycentric(p, state);
            auto const gamma = 1. - alpha - beta;

            if (alpha >= 0. && beta >= 0. && gamma >= 0.) {
                within_distance = true;
                normal = face.normal();
                break;
            }
        }
    }

    // 2.2. Then check against mesh itself
    if (!within_distance) {
        for (auto&& face : g_simulation->faces()) {
            if (face.distance_to_plane(state.data0[i], state) < 1e-2) {
                Vec2 p = face.project(state.data0[i], state);
                auto const [alpha, beta] = face.barycentric(p, state);
                auto const gamma = 1. - alpha - beta;

                if (alpha >= 0. && beta >= 0. && gamma >= 0.) {
                    within_distance = true;
                    normal = face.normal(state);
                    break;
                }
            }
        }
    }

    if (!within_distance)
        return false;

    // FIXME: Compute total force acting on this particle
    // 3. Force and face normal should be pointing towards each other
    Vec3 F = g_simulation->external_force();
    if (F.dot(normal) >= 0.)
        return false;

    // FIXME: Implement 4. Tangential force is less than frictional force

    // Therefore, we are resting.
    return true;
}

inline bool emplace(std::optional<CollisionRecord> potential_collision, CollisionRecord& collision)
{
    if (!potential_collision.has_value())
        return false;

    auto record = potential_collision.value();

    // Skip anything that is _very_ small, then handle it in the next step
    if (record.fractional_timestep < 1e-2)
        return false;

    if (record.fractional_timestep < collision.fractional_timestep) {
        collision = record;
        return true;
    }

    return false;
}

hot flatten void Simulation::step(double max_time)
{
    for (auto t = 0.; t < max_time; t += m_timestep) {
        float t_r = std::min(m_timestep, max_time - t);
        auto si = State {};

        while (t_r > 1e-3 * max_time) {
            auto sf = integrate(si, t_r);

            CollisionRecord collision { .fractional_timestep = 1. };
            // (1) (vertex-face, static response) Resolve collisions between nonstatic vertices and static faces
            for (auto i = 0uz; i < m_positions.size(); i++) {
                if (resting(i, si)) {
                    sf.data0[i] = si.data0[i];
                    sf.data1[i] = Vec3::Zero();
                    continue;
                }

                Particle particle(i);
                // Detect collisions
                for (auto&& face : g_scene->faces()) {
                    auto potential_collision = face.collision(particle, si, sf);

                    if (!emplace(potential_collision, collision))
                        continue;
                }
            }

            // (2) (vertex-face, static response) Resolve collisions between static vertices and nonstatic faces
            for (auto&& face : m_faces) {
                for (auto&& vertex : g_scene->vertices()) {
                    auto potential_collision = face.collision(vertex, si, sf);

                    if (!emplace(potential_collision, collision))
                        continue;
                }
            }
            // TODO: (3) (vertex-face, dynamic response) Resolve collisions between nonstatic vertices and nonstatic faces
            // TODO: (4) (edge-edge, static response) Resolve collisions between edges of nonstatic and static faces
            // TODO: (5) (edge-edge, dynamic response) Resolve collisions between edges of nonstatic faces

            // Compute collision response
            if (collision.fractional_timestep < 1.) {
                auto sc = collision.state;

                if (collision.fractional_timestep < 1e-3) {
                    sf = sc;
                    break;
                }

                if (collision.type == CollisionRecord::Type::STATIC) {
                    // Compute velocity at collision point
                    Vec3 v_minus = Vec3::Zero();

                    for (auto [w, i] : std::views::zip(collision.weights, collision.indices)) {
                        if (w < 1e-6)
                            continue;

                        v_minus += w * sc.data1[i];
                    }

                    // TODO: Properly compute collision response (restitution/friction/other)
                    Vec3 v_plus = -0.28 * v_minus;

                    Vec3 v_delta_prime = (v_plus - v_minus) / (collision.weights[0] * collision.weights[0] + collision.weights[1] * collision.weights[1] + collision.weights[2] * collision.weights[2]);

                    sc.data1[collision.indices[0]] += collision.weights[0] * v_delta_prime;
                    sc.data1[collision.indices[1]] += collision.weights[1] * v_delta_prime;
                    sc.data1[collision.indices[2]] += collision.weights[2] * v_delta_prime;
                } else {
                    // TODO: Handle collisions with momentum
                }

                sf = sc;
            }

            t_r -= collision.fractional_timestep * m_timestep;
            si = sf;
        }
        adopt(si);
    }
}
