/*
 * Copyright (c) 2024, Brandon G. Nguyen <brandon@nguyen.vc>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#pragma once

#include "utils.h"

#include "Particle.h"
#include "Spring.h"
#include "State.h"

class Face {
protected:
    size_t m_v0;
    size_t m_v1;
    size_t m_v2;

public:
    Face(size_t v0, size_t v1, size_t v2);
    Face(Particle const& v0, Particle const& v1, Particle const& v2);

    nodiscard virtual Vec3 normal(State const&) const;
    nodiscard Vec2 project(Vec3 const&, State const&) const;

    nodiscard virtual double distance_to_plane(Vec3 const&, State const&) const;
    nodiscard virtual std::pair<double, double> barycentric(Vec2 const&, State const&) const;
    nodiscard virtual bool collision(Particle const&, State const&, State const&) const;

    // nodiscard std::tuple<double, double, Vec3> lerp(size_t p0, size_t p1, size_t q0, size_t q1, State const& state) const;
    // flatten nodiscard std::tuple<double, double, Vec3> lerp(Particle const& p0, Particle const& p1, Particle const& q0, Particle const& q1, State const& state) const
    // {
    //     return lerp(p0.index, p1.index, q0.index, q1.index, state);
    // }
    // nodiscard bool collision(Spring const&, State const&, State const&) const;
    // nodiscard bool collision(Particle const&, Particle const&, State const&, State const&) const;
    // nodiscard bool collision(Vec3 const&, Vec3 const&, State const&, State const&) const;

    nodiscard inline Vec3 force(State const& state) const
    {
        return Vec3::Zero();
    }

    nodiscard size_t v0() const { return m_v0; }
    nodiscard size_t v1() const { return m_v1; }
    nodiscard size_t v2() const { return m_v2; }
};

class StaticFace : public Face {
    Vec3 m_normal;

public:
    StaticFace(size_t v0, size_t v1, size_t v2);

    nodiscard Vec3 const& normal() const { return m_normal; }
    nodiscard virtual Vec3 normal(State const&) const override { return m_normal; }
    nodiscard virtual double distance_to_plane(Vec3 const& p, State const&) const override;
    nodiscard virtual std::pair<double, double> barycentric(Vec2 const& p, State const&) const override;
};
