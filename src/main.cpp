/*
 * Copyright (c) 2024, Brandon G. Nguyen <brandon@nguyen.vc>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#include <algorithm>
#include <iostream>
#include <memory>
#include <ranges>

#include "Scene.h"
#include "Simulation.h"
#include "utils.h"

#include "Bunny.h"

Simulation* g_simulation;
Scene* g_scene;

std::ostream& operator<<(std::ostream& stream, Vec3 const& v)
{
    stream << "(" << std::trunc(10000. * v.x()) / 10000. << ", " << std::trunc(10000. * v.y()) / 10000. << ", " << std::trunc(10000. * v.z()) / 10000. << ")";
    return stream;
}

int main()
{
    Bunny bunny;
    g_scene = new Scene();

    {
        auto p0 = g_scene->add_particle({ -5., 0., -5. });
        auto p1 = g_scene->add_particle({ +5., 0., -5. });
        auto p2 = g_scene->add_particle({ -5., 0., +5. });
        auto p3 = g_scene->add_particle({ +5., 0., +5. });

        g_scene->add_face({ p2, p1, p0 });
        g_scene->add_face({ p1, p2, p3 });
    }

    g_simulation = new Simulation(1. / 240., Simulation::Integrator::RK4);

    // create_cube({ 0., 1.5, 0. });
    bunny.initialize({0., 1.5, 0.});

    for (auto i = 0; i < 150; i++) {
        std::cout << "=== simulation step " << (i + 1) << " ===\n";
        // for (auto&& [i, v] : enumerate(g_simulation->vertices()))
        //     std::cout << v << '\t' << g_simulation->velocity(i) << '\n';

        g_simulation->step();
        std::cout << std::endl;
    }

    delete g_simulation;
    delete g_scene;
}
