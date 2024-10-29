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

Simulation* g_simulation = new Simulation(1. / 240.);
Scene* g_scene = new Scene();

int main()
{
    create_cube();

    for (auto i = 0; i < 16; i++) {
        std::cout << "=== simulation step " << (i + 1) << " ===\n";
        for (auto&& v : g_simulation->vertices())
            std::cout << "(" << std::trunc(1000. * v.x()) / 1000. << ", " << std::trunc(1000. * v.y()) / 1000. << ", " << std::trunc(1000. * v.z()) / 1000. << ")\n";

        g_simulation->step();
        std::cout << std::endl;
    }

    delete g_simulation;
    delete g_scene;
}
