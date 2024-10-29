/*
 * Copyright (c) 2024, Brandon G. Nguyen <brandon@nguyen.vc>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#include "Simulation.h"
#include "Scene.h"
#include "State.h"
#include <iostream>
hot flatten void Simulation::step(double max_time)
{

    for (auto t = 0.; t < max_time; t += m_timestep) {
        float t_r = std::min(m_timestep, max_time - t);
        auto si = State {};

        while (t_r > 1e-3 * max_time) {
            auto sf = integrate(si, t_r);
            CollisionRecord collision { .fractional_timestep = 1. };

            // (1) (vertex-face, static response) Resolve collisions between nonstatic vertices and static faces
            for (auto&& face : g_scene->faces()) {
                for (auto i = 0uz; i < m_positions.size(); i++) {
                    auto potential_collision = face.collision(Particle { i }, si, sf);

                    if (!potential_collision.has_value())
                        continue;

                    if (potential_collision.value().fractional_timestep < collision.fractional_timestep)
                        collision = potential_collision.value();
                }
            }

            // (2) (vertex-face, static response) Resolve collisions between static vertices and nonstatic faces
            for (auto&& face : m_faces) {
                for (auto&& vertex : g_scene->vertices()) {
                    auto potential_collision = face.collision(vertex, si, sf);

                    if (!potential_collision.has_value())
                        continue;

                    if (potential_collision.value().fractional_timestep < collision.fractional_timestep)
                        collision = potential_collision.value();
                }
            }
            {
                // TODO: (3) (vertex-face, dynamic response) Resolve collisions between nonstatic vertices and nonstatic faces
                // for (auto&& face : m_faces) {
                //     for (auto i = 0uz; i < m_positions.size(); i++) {
                //         // Do not check for collision of vertices of current face with itself
                //         if (i == face.v0() || i == face.v1() || i == face.v2())
                //             continue;

                //         auto potential_collision = face.collision(Particle { i }, si, sf);

                //         if (!potential_collision.has_value())
                //             continue;

                //         if (potential_collision.value().fractional_timestep < collision.fractional_timestep)
                //             collision = potential_collision.value();
                //     }
                // }
                // TODO: (4) (edge-edge, static response) Resolve collisions between edges of nonstatic and static faces
                // TODO: (5) (edge-edge, dynamic response) Resolve collisions between edges of nonstatic faces
            }
            // If fractional timestep is not 1, then we've found a collision and must compute its response
            if (collision.fractional_timestep < 1.) {
                auto sc = collision.state;

                if (collision.type == CollisionRecord::Type::STATIC) {
                    // Compute velocity at collision point
                    Vec3 v_minus = Vec3::Zero();

                    for (auto [w, i] : std::views::zip(collision.weights, collision.indices)) {
                        if (w < 1e-6)
                            continue;

                        v_minus += w * sc.data1[i];
                    }

                    // TODO: Properly compute collision response (restitution/friction/other)
                    Vec3 v_plus = -0.8 * v_minus;

                    Vec3 v_delta_prime = (v_plus - v_minus) / (collision.weights[0] * collision.weights[0] + collision.weights[1] * collision.weights[1] + collision.weights[2] * collision.weights[2]);

                    sc.data1[collision.indices[0]] += collision.weights[0] * v_delta_prime;
                    sc.data1[collision.indices[1]] += collision.weights[1] * v_delta_prime;
                    sc.data1[collision.indices[2]] += collision.weights[2] * v_delta_prime;
                } else {
                    // TODO: Handle collisions with momentum
                }

                si = sc;
            }

            t_r -= collision.fractional_timestep * m_timestep;
            si = sf;
        }
        adopt(si);
    }
}
