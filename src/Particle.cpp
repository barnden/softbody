/*
 * Copyright (c) 2024, Brandon G. Nguyen <brandon@nguyen.vc>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#include "Particle.h"
#include "Simulation.h"

Vec3& Particle::position() const { return g_simulation->position(index); }
Vec3& Particle::velocity() const { return g_simulation->velocity(index); }
