/*
 * Copyright (c) 2024, Brandon G. Nguyen <brandon@nguyen.vc>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#pragma once

#include <vector>

#include "Face.h"
#include "State.h"
#include "utils.h"

class Scene {
    std::vector<Vec3> m_positions;
    std::vector<StaticFace> m_faces;

public:
    Scene()
        : m_positions({})
        , m_faces({}) {};

    nodiscard size_t add_particle(Vec3 const& position)
    {
        m_positions.push_back(position);
        return m_positions.size() - 1;
    }

    void add_face(StaticFace const& face)
    {
        m_faces.push_back(face);
    }

    Vec3 const& position(size_t i) const { return m_positions[i]; }
    StaticFace const& face(size_t i) const { return m_faces[i]; }

    decltype(auto) vertices() const { return m_positions; }
    decltype(auto) faces() const { return m_faces; }
};

extern Scene* g_scene;
