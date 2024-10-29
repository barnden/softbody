/*
 * Copyright (c) 2024, Brandon G. Nguyen <brandon@nguyen.vc>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#pragma once
#include <functional>
#include <vector>

#include "utils.h"

#include "Face.h"
#include "Particle.h"
#include "Spring.h"
#include "State.h"

class Simulation {
    friend State;

    std::vector<Vec3> m_positions;
    std::vector<Vec3> m_velocities;
    std::vector<Spring> m_springs;
    std::vector<Face> m_faces;
    std::function<State(State const&, double)> m_integrator;
    double m_timestep;

public:
    enum class Integrator {
        EULER,
        RK4,
    };

    explicit Simulation(double timestep, Integrator integrator = Integrator::RK4)
        : m_positions({})
        , m_velocities({})
        , m_springs({})
        , m_timestep(timestep)
    {
        switch (integrator) {
        case Integrator::EULER:
            m_integrator = euler;
            break;

        case Integrator::RK4:
            m_integrator = rk4;
            break;
        }
    }

    ~Simulation() = default;

    Particle add_particle(Vec3 position, Vec3 velocity = Vec3::Zero())
    {
        m_positions.push_back(position);
        m_velocities.push_back(velocity);

        return { m_positions.size() - 1 };
    }

    void add_face(Face face)
    {
        m_faces.push_back(face);
    }

    void add_spring(Spring spring)
    {
        m_springs.push_back(spring);
    }

    void adopt(State state)
    {
        m_positions = state.data0;
        m_velocities = state.data1;
    }

    void step(double max_time = 1. / 60.);

    nodiscard inline Vec3 external_force() const
    {
        // e.g. for gravity
        return { 0., -10., 0. };
    }

    flatten State integrate(State const& initial_state, double timestep)
    {
        return m_integrator(initial_state, timestep);
    }

    flatten State integrate(State const& initial_state)
    {
        return m_integrator(initial_state, m_timestep);
    }

    flatten State integrate(double timestep)
    {
        return m_integrator({}, timestep);
    }

    flatten State integrate()
    {
        return m_integrator({}, m_timestep);
    }

    nodiscard inline Vec3& position(size_t i) { return m_positions[i]; }
    nodiscard inline Vec3& velocity(size_t i) { return m_velocities[i]; }

    nodiscard inline Vec3 const& position(size_t i) const { return m_positions[i]; }
    nodiscard inline Vec3 const& velocity(size_t i) const { return m_velocities[i]; }

    nodiscard inline Face const& face(size_t i) const { return m_faces[i]; }
    nodiscard inline Spring const& spring(size_t i) const { return m_springs[i]; }

    nodiscard inline double timestep() const { return m_timestep; }

    nodiscard inline decltype(auto) vertices() const { return m_positions; }
    nodiscard inline decltype(auto) faces() const { return m_faces; }
    nodiscard inline decltype(auto) springs() const { return m_springs; }
};

void create_cube(Vec3 const& center=Vec3::Zero());

extern Simulation* g_simulation;
