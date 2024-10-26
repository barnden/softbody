#include <algorithm>
#include <iostream>
#include <memory>
#include <ranges>

#include "utils.h"

struct Particle {
    size_t index;

    inline Vec3& position() const;
    inline Vec3& velocity() const;

    Particle(size_t index)
        : index(index) {};
};

struct Spring {
    size_t p0;
    size_t p1;

    double k; // spring constant
    double d; // damper constant
    double L0; // rest length

    Spring(size_t p0, size_t p1, double k = 1., double d = 1.);
    Spring(Particle p0, Particle p1, double k = 1., double d = 0.);

    Vec3 force() const;
};

struct State {
    std::vector<Vec3> data0;
    std::vector<Vec3> data1;

    State();

    State derivative() const;

    State& operator+=(State const& rhs)
    {
        for (auto i = 0uz; i < data0.size(); i++) {
            data0[i] += rhs.data0[i];
            data1[i] += rhs.data1[i];
        }

        return *this;
    }

    State operator+(State const& rhs)
    {
        State result = *this;
        result += rhs;

        return result;
    }

    State& operator*=(double const rhs)
    {
        for (auto i = 0uz; i < data0.size(); i++) {
            data0[i] *= rhs;
            data1[i] *= rhs;
        }

        return *this;
    }

    State operator*(double const rhs)
    {
        State result = *this;
        result *= rhs;

        return result;
    }

    friend State operator*(double const lhs, State rhs)
    {
        State result = rhs;
        result *= lhs;

        return result;
    }
};

struct Manager {
    std::vector<Vec3> positions;
    std::vector<Vec3> velocities;
    std::vector<Spring> springs;

    Manager()
    {
        positions = decltype(positions)();
        velocities = decltype(velocities)();
        springs = decltype(springs)();
    }

    ~Manager() = default;

    Particle add_particle(Vec3 position, Vec3 velocity = Vec3::Zero())
    {
        positions.push_back(position);
        velocities.push_back(velocity);

        return { positions.size() - 1 };
    }

    void add_spring(Spring spring)
    {
        springs.push_back(spring);
    }

    void adopt(State state)
    {
        positions = state.data0;
        velocities = state.data1;
    }

    inline Vec3& position(size_t i) { return positions[i]; }
    inline Vec3& velocity(size_t i) { return velocities[i]; }

    inline Vec3 const& position(size_t i) const { return positions[i]; }
    inline Vec3 const& velocity(size_t i) const { return velocities[i]; }
};

std::shared_ptr<Manager> g_manager;

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

Vec3& Particle::position() const { return g_manager->position(index); }
Vec3& Particle::velocity() const { return g_manager->velocity(index); }

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

decltype(auto) euler(State& state, double h = 1. / 240.)
{
    return state + h * state.derivative();
}

decltype(auto) rk4(State state, double h=1. / 240.)
{
    auto K1 = state.derivative();
    auto K2 = (state + .5 * h * K1).derivative();
    auto K3 = (state + .5 * h * K2).derivative();
    auto K4 = (state + h * K3).derivative();

    return state + (h / 6.) * (K1 + 2. * K2 + 2. * K3 + K4);
}

int main()
{
    g_manager = std::make_shared<Manager>();

    auto p0 = g_manager->add_particle({ 0., 0., 0. });
    auto p1 = g_manager->add_particle({ 0., 1., 0. });
    auto p2 = g_manager->add_particle({ 1., 0., 0. });
    auto p3 = g_manager->add_particle({ 1., 1., 0. });
    auto p4 = g_manager->add_particle({ 0., 0., 1. });
    auto p5 = g_manager->add_particle({ 0., 1., 1. });
    auto p6 = g_manager->add_particle({ 1., 0., 1. });
    auto p7 = g_manager->add_particle({ 1., 1., 1. });

    // Square face edge connectivity
    g_manager->add_spring({p0, p1});
    g_manager->add_spring({p0, p2});
    g_manager->add_spring({p1, p3});
    g_manager->add_spring({p2, p3});

    g_manager->add_spring({p0, p1});
    g_manager->add_spring({p0, p2});
    g_manager->add_spring({p1, p3});
    g_manager->add_spring({p2, p3});

    // Edges connecting corners, vertically
    g_manager->add_spring({p0, p4});
    g_manager->add_spring({p1, p5});
    g_manager->add_spring({p2, p6});
    g_manager->add_spring({p3, p7});

    // Cross edges across each face
    g_manager->add_spring({p0, p3});
    g_manager->add_spring({p1, p2});

    g_manager->add_spring({p4, p7});
    g_manager->add_spring({p5, p6});

    g_manager->add_spring({p0, p6});
    g_manager->add_spring({p2, p4});

    g_manager->add_spring({p0, p5});
    g_manager->add_spring({p1, p4});

    g_manager->add_spring({p1, p7});
    g_manager->add_spring({p3, p5});

    g_manager->add_spring({p2, p7});
    g_manager->add_spring({p3, p6});

    // Internal diagonals
    g_manager->add_spring({p0, p7});
    g_manager->add_spring({p1, p6});
    g_manager->add_spring({p3, p4});
    g_manager->add_spring({p2, p5});
}