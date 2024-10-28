/*
 * Copyright (c) 2024, Brandon G. Nguyen <brandon@nguyen.vc>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#pragma once
#include <vector>

#include "utils.h"

#include "Face.h"
#include "Particle.h"
#include "Spring.h"
#include "State.h"

struct Manager {
    std::vector<Vec3> positions;
    std::vector<Vec3> velocities;
    std::vector<Spring> springs;
    std::vector<Face> faces;
    double h;

    explicit Manager(double timestep)
        : positions({})
        , velocities({})
        , springs({})
        , h(timestep) {};

    ~Manager() = default;

    Particle add_particle(Vec3 position, Vec3 velocity = Vec3::Zero())
    {
        positions.push_back(position);
        velocities.push_back(velocity);

        return { positions.size() - 1 };
    }

    void add_face(Face face)
    {
        faces.push_back(face);
    }

    void add_spring(Spring spring)
    {
        springs.push_back(spring);
    }

    void adopt(State state)
    {
        positions = state.data0;
        velocities = state.data1;
    }

    nodiscard inline Vec3 external_force() const
    {
        return { 0., -1., 0. };
    }

    nodiscard inline Vec3& position(size_t i)
    {
        return positions[i];
    }
    nodiscard inline Vec3& velocity(size_t i) { return velocities[i]; }

    nodiscard inline Vec3 const& position(size_t i) const { return positions[i]; }
    nodiscard inline Vec3 const& velocity(size_t i) const { return velocities[i]; }
    nodiscard inline Face const& face(size_t i) const { return faces[i]; }
};

extern Manager* g_manager;
