/*
 * Copyright (c) 2024, Brandon G. Nguyen <brandon@nguyen.vc>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#pragma once

#include "utils.h"

struct State {
    std::vector<Vec3> data0;
    std::vector<Vec3> data1;

    State();

    nodiscard State derivative() const;

    State& operator+=(State const& rhs)
    {
        for (auto i = 0uz; i < data0.size(); i++) {
            data0[i] += rhs.data0[i];
            data1[i] += rhs.data1[i];
        }

        return *this;
    }

    State operator+(State const& rhs) const
    {
        State result = *this;
        result += rhs;

        return result;
    }

    State& operator*=(double const rhs)
    {
        for (auto i = 0uz; i < data0.size(); i++) {
            data0[i] *= rhs;
            data1[i] *= rhs;
        }

        return *this;
    }

    State operator*(double const rhs)
    {
        State result = *this;
        result *= rhs;

        return result;
    }

    friend State operator*(double const lhs, State rhs)
    {
        State result = rhs;
        result *= lhs;

        return result;
    }
};

State euler(State const& state, double h);
State rk4(State const& state, double h);
