#include "Particle.h"
#include "Manager.h"

Vec3& Particle::position() const { return g_manager->position(index); }
Vec3& Particle::velocity() const { return g_manager->velocity(index); }
