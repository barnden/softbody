#pragma once

#include "utils.h"

#include "Particle.h"

struct Spring {
    size_t p0;
    size_t p1;

    double k; // spring constant
    double d; // damper constant
    double L0; // rest length

    Spring(size_t p0, size_t p1, double k = 1., double d = 1.);
    Spring(Particle p0, Particle p1, double k = 1., double d = 0.);

    nodiscard Vec3 force() const;
};
