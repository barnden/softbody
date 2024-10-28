/*
 * Copyright (c) 2024, Brandon G. Nguyen <brandon@nguyen.vc>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
#include "Particle.h"
#include "Manager.h"

Vec3& Particle::position() const { return g_manager->position(index); }
Vec3& Particle::velocity() const { return g_manager->velocity(index); }
