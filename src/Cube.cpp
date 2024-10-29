/*
 * Copyright (c) 2024, Brandon G. Nguyen <brandon@nguyen.vc>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#include "Simulation.h"

void create_cube(Vec3 const& center)
{
    auto p0 = g_simulation->add_particle(center + Vec3 { 0., 0., 0. });
    auto p1 = g_simulation->add_particle(center + Vec3 { 0., 1., 0. });
    auto p2 = g_simulation->add_particle(center + Vec3 { 1., 0., 0. });
    auto p3 = g_simulation->add_particle(center + Vec3 { 1., 1., 0. });
    auto p4 = g_simulation->add_particle(center + Vec3 { 0., 0., 1. });
    auto p5 = g_simulation->add_particle(center + Vec3 { 0., 1., 1. });
    auto p6 = g_simulation->add_particle(center + Vec3 { 1., 0., 1. });
    auto p7 = g_simulation->add_particle(center + Vec3 { 1., 1., 1. });

    {
        g_simulation->add_face({ p0, p1, p2 });
        g_simulation->add_face({ p3, p1, p2 });

        g_simulation->add_face({ p4, p5, p6 });
        g_simulation->add_face({ p7, p5, p6 });

        g_simulation->add_face({ p0, p2, p4 });
        g_simulation->add_face({ p6, p2, p4 });

        g_simulation->add_face({ p0, p1, p4 });
        g_simulation->add_face({ p5, p1, p4 });

        g_simulation->add_face({ p1, p3, p5 });
        g_simulation->add_face({ p7, p3, p5 });

        g_simulation->add_face({ p2, p3, p6 });
        g_simulation->add_face({ p7, p3, p6 });
    }

    { // Square face edge connectivity
        g_simulation->add_spring({ p0, p1 });
        g_simulation->add_spring({ p0, p2 });
        g_simulation->add_spring({ p1, p3 });
        g_simulation->add_spring({ p2, p3 });

        g_simulation->add_spring({ p4, p5 });
        g_simulation->add_spring({ p4, p6 });
        g_simulation->add_spring({ p7, p5 });
        g_simulation->add_spring({ p7, p6 });
    }

    { // Edges connecting corners, vertically
        g_simulation->add_spring({ p0, p4 });
        g_simulation->add_spring({ p1, p5 });
        g_simulation->add_spring({ p2, p6 });
        g_simulation->add_spring({ p3, p7 });
    }

    { // Cross edges across each face
        g_simulation->add_spring({ p0, p3 });
        g_simulation->add_spring({ p1, p2 });

        g_simulation->add_spring({ p4, p7 });
        g_simulation->add_spring({ p5, p6 });

        g_simulation->add_spring({ p0, p6 });
        g_simulation->add_spring({ p2, p4 });

        g_simulation->add_spring({ p0, p5 });
        g_simulation->add_spring({ p1, p4 });

        g_simulation->add_spring({ p1, p7 });
        g_simulation->add_spring({ p3, p5 });

        g_simulation->add_spring({ p2, p7 });
        g_simulation->add_spring({ p3, p6 });
    }

    { // Internal diagonals
        g_simulation->add_spring({ p0, p7 });
        g_simulation->add_spring({ p1, p6 });
        g_simulation->add_spring({ p3, p4 });
        g_simulation->add_spring({ p2, p5 });
    }
}
