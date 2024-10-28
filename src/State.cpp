/*
 * Copyright (c) 2024, Brandon G. Nguyen <brandon@nguyen.vc>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#include "State.h"
#include "Manager.h"

State::State()
{
    data0 = decltype(data0)(g_manager->positions);
    data1 = decltype(data1)(g_manager->velocities);
}

State State::derivative() const
{
    State derivative = *this;

    // S = <X, V>, S' = <V, A>
    std::swap(derivative.data0, derivative.data1);

    for (auto& data : derivative.data1)
        data = Vec3::Zero();

    for (auto&& spring : g_manager->springs) {
        derivative.data1[spring.p0] += spring.force();
        derivative.data1[spring.p1] -= spring.force();
    }

    return derivative;
}

State euler(State& state, double h)
{
    return state + h * state.derivative();
}

State rk4(State& state, double h)
{
    auto K1 = state.derivative();
    auto K2 = (state + .5 * h * K1).derivative();
    auto K3 = (state + .5 * h * K2).derivative();
    auto K4 = (state + h * K3).derivative();

    return state + (h / 6.) * (K1 + 2. * K2 + 2. * K3 + K4);
}

