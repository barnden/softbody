/*
 * Copyright (c) 2024, Brandon G. Nguyen <brandon@nguyen.vc>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#include "test.h"
#include "Scene.h"
#include "Simulation.h"

TestManager manager {};
Simulation* g_simulation;
Scene* g_scene;

#define __stringify_helper(x) #x
#define __stringify(x)        __stringify_helper(x)
#define location              __FILE__ ":" __stringify(__LINE__)

int main()
{
    g_simulation = new Simulation(1. / 240., Simulation::Integrator::RK4);
    create_cube();

    {
        auto face = g_simulation->face(0);

        Vec3 normal = face.normal(State {});

        auto const& v0 = g_simulation->position(face.v0());
        auto const& v1 = g_simulation->position(face.v1());
        auto const& v2 = g_simulation->position(face.v2());

        auto state_initial = State {};
        auto state_final = g_simulation->integrate(state_initial);

        Vec3 midpoint = (v0 + v1 + v2) / 3.;

        manager.add({ "Face::distance_to_plane", location,
                      [&]() -> bool {
                          Vec3 normal = face.normal(state_initial);
                          Vec3 p0 = v0 + normal;
                          Vec3 p1 = v1 + normal;
                          Vec3 p2 = v2 + normal;
                          Vec3 m = midpoint + normal;

                          return (face.distance_to_plane(p0, state_initial) == 1.) && (face.distance_to_plane(p1, state_initial) == 1.) && (face.distance_to_plane(p2, state_initial) == 1.) && (face.distance_to_plane(m, state_initial) == 1.);
                      } });

        auto const test_barycentric = [&](Vec3 p, double exp_alpha, double exp_beta, double exp_gamma, double tol = 1e-6) -> bool {
            auto p0 = face.project(p, state_initial);
            auto [alpha, beta] = face.barycentric(p0, state_initial);
            auto gamma = 1. - alpha - beta;

            return std::abs(alpha - exp_alpha) < tol && std::abs(beta - exp_beta) < tol && std::abs(gamma - exp_gamma) < tol;
        };

        manager.add({ "Face::barycentric (vertex, 0)", location, [&]() { return test_barycentric(v0, 0., 0., 1.); } });
        manager.add({ "Face::barycentric (vertex, 1)", location, [&]() { return test_barycentric(v1, 1., 0., 0.); } });
        manager.add({ "Face::barycentric (vertex, 2)", location, [&]() { return test_barycentric(v2, 0., 1., 0.); } });
        manager.add({ "Face::barycentric (midpoint)", location, [&]() { return test_barycentric(midpoint, 1. / 3., 1. / 3., 1. / 3.); } });

        manager.add({ "Face::collision (midpoint, expected)", location,
                      [&]() {
                          Vec3 position = midpoint + face.normal(state_initial) * g_simulation->timestep();
                          Vec3 velocity = -face.normal(state_initial);

                          auto particle = g_simulation->add_particle(position, velocity);
                          return face.collision(particle, State {}, g_simulation->integrate()).has_value();
                      } });

        manager.add({ "Face::collision (midpoint, no collision)", location,
                      [&]() {
                          Vec3 position = midpoint + (1. + g_simulation->timestep()) * (face.normal(state_initial) * g_simulation->timestep());
                          Vec3 velocity = -face.normal(state_initial);

                          auto particle = g_simulation->add_particle(position, velocity);
                          return !face.collision(particle, State {}, g_simulation->integrate()).has_value();
                      } });

        manager.run();
    }

    {
        g_scene = new Scene();
        auto v0 = g_scene->add_particle({ 0., 0., 0. });
        auto v1 = g_scene->add_particle({ 0., 1., 0. });
        auto v2 = g_scene->add_particle({ 1., 0., 0. });

        g_scene->add_face({ v0, v1, v2 });
        auto face = g_scene->face(0);

        manager.add({ "StaticFace::distance_to_plane", location,
                      [&]() -> bool {
                          Vec3 normal = face.normal({});
                          Vec3 p0 = g_scene->position(v0) + normal;
                          Vec3 p1 = g_scene->position(v1) + normal;
                          Vec3 p2 = g_scene->position(v2) + normal;
                          Vec3 m = (p0 + p1 + p2) / 3.;

                          return (face.distance_to_plane(p0, {}) == 1.) && (face.distance_to_plane(p1, {}) == 1.) && (face.distance_to_plane(p2, {}) == 1.) && (face.distance_to_plane(m, {}) == 1.);
                      } });

        manager.run();
        delete g_scene;
    }

    delete g_simulation;

    {
        g_simulation = new Simulation(1. / 240.);
        g_scene = new Scene();

        {
            auto p0 = g_simulation->add_particle({ 0., 1e-4, g_simulation->timestep() }, { 0., 0., -1. });
        }

        {
            auto p0 = g_scene->add_particle({ 0., 0., 0. });
            auto p1 = g_scene->add_particle({ 0., 1., 0. });
            auto p2 = g_scene->add_particle({ 1., 0., 0. });

            g_scene->add_face({ p0, p1, p2 });
        }

        manager.add({ "StaticFace::collision with particle", location,
                      [&]() -> bool {
                          StaticFace face = g_scene->face(0);

                          auto initial_state = State {};
                          auto final_state = g_simulation->integrate(initial_state);

                          Vec3 normal = face.normal({});

                          return face.collision(Particle { 0 }, initial_state, final_state).has_value();
                      } });

        manager.run();

        delete g_simulation;
        delete g_scene;
    }

    {
        g_simulation = new Simulation(1. / 240.);

        auto p0 = g_simulation->add_particle({ 0., 0., 0. });
        auto p1 = g_simulation->add_particle({ 0., 1., 0. });
        auto p2 = g_simulation->add_particle({ -.5, .5, 0. });
        auto p3 = g_simulation->add_particle({ .5, .5, 0. });
        auto p4 = g_simulation->add_particle({ 0., .5, 0. });

        g_simulation->add_spring({ p0, p1 });
        g_simulation->add_spring({ p2, p3 });
        g_simulation->add_spring({ p4, p3 });

        auto spring1 = g_simulation->spring(0);
        auto spring2 = g_simulation->spring(1);
        auto spring3 = g_simulation->spring(2);

        manager.add({ "Edge::lerp (orthogonal, overlapping XYZ midpoint-midpoint) ", location,
                      [&]() {
                          auto const [s1, t1, m1] = spring1.lerp(spring2, State {});
                          auto const [s2, t2, m2] = spring2.lerp(spring1, State {});

                          (void)m1;
                          (void)m2;

                          auto evaluation = std::abs(s1 - 0.5) < 1e-6 && std::abs(t1 - 0.5) < 1e-6;
                          auto symmetry = (s1 == t2) && (s2 == t1);

                          return evaluation && symmetry;
                      } });

        manager.add({ "Edge::lerp (orthogonal, overlapping XYZ end-midpoint) ", location,
                      [&]() {
                          auto const [s1, t1, m1] = spring1.lerp(spring3, State {});
                          auto const [s2, t2, m2] = spring3.lerp(spring1, State {});

                          (void)m1;
                          (void)m2;

                          auto evaluation = std::abs(s1 - 0.5) < 1e-6 && std::abs(t1) < 1e-6;
                          auto symmetry = (s1 == t2) && (s2 == t1);

                          return evaluation && symmetry;
                      } });

        manager.add({ "Edge::collision (orthogonal, expected, overlapping XYZ midpoint-midpoint) ", location,
                      [&]() {
                          auto initial_state = State {};
                          auto final_state = g_simulation->integrate(initial_state);
                          auto collision = spring1.collision(spring2, initial_state, final_state) && spring2.collision(spring1, initial_state, final_state);

                          return collision;
                      } });

        manager.run();
        delete g_simulation;
    }

    {
        g_simulation = new Simulation(1. / 240.);

        auto p0 = g_simulation->add_particle({ 0., 0., 0. });
        auto p1 = g_simulation->add_particle({ 0., 1., 0. });
        auto p2 = g_simulation->add_particle({ -.5, .5, 1. / 240. }, { 0., 0., -1. });
        auto p3 = g_simulation->add_particle({ .5, .5, 1. / 240. }, { 0., 0., -1. });

        g_simulation->add_spring({ p0, p1 });
        g_simulation->add_spring({ p2, p3 });

        auto spring1 = g_simulation->spring(0);
        auto spring2 = g_simulation->spring(1);

        manager.add({ "Edge::lerp (orthogonal, overlapping XY midpoint-midpoint) ", location,
                      [&]() {
                          auto const [s1, t1, m1] = spring1.lerp(spring2, State {});
                          auto const [s2, t2, m2] = spring2.lerp(spring1, State {});

                          (void)m1;
                          (void)m2;

                          auto evaluation = std::abs(s1 - 0.5) < 1e-6 && std::abs(t1 - 0.5) < 1e-6;
                          auto symmetry = (s1 == t2) && (s2 == t1);

                          return evaluation && symmetry;
                      } });

        manager.add({ "Edge::collision (orthogonal, overlapping XY midpoint-midpoint) ", location,
                      [&]() {
                          auto initial_state = State {};
                          auto final_state = g_simulation->integrate(initial_state);

                          {
                              // Springs are setup so that they do not intersect initially, but after integration they will
                              auto collision = spring1.collision(spring2, initial_state, initial_state) && spring2.collision(spring1, initial_state, initial_state);

                              if (collision)
                                  return false;
                          }

                          auto collision = spring1.collision(spring2, initial_state, final_state) && spring2.collision(spring1, initial_state, final_state);

                          return collision;
                      } });

        manager.run();
        delete g_simulation;
    }

    manager.statistics();
}
