#pragma once

#include "utils.h"

struct Particle {
    size_t index;

    nodiscard Vec3& position() const;
    nodiscard Vec3& velocity() const;

    Particle(size_t const index)
        : index(index) {};
};
