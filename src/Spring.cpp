#include "Spring.h"
#include "Manager.h"

Spring::Spring(size_t p0, size_t p1, double k, double d)
    : p0(p0)
    , p1(p1)
    , k(k)
    , d(d)
{
    L0 = (g_manager->position(p1) - g_manager->position(p0)).norm();
}

Spring::Spring(Particle p0, Particle p1, double k, double d)
    : p0(p0.index)
    , p1(p1.index)
    , k(k)
    , d(d)
{
    L0 = (p1.position() - p0.position()).norm();
}

Vec3 Spring::force() const
{
    Vec3 delta = g_manager->position(p1) - g_manager->position(p0);
    double L = delta.norm();

    delta.normalize();
    double dV = (g_manager->velocity(p1) - g_manager->position(p0)).dot(delta);

    Vec3 Fs = k * (L - L0) * delta;
    Vec3 Fd = d * dV * delta;

    return Fs + Fd;
}
