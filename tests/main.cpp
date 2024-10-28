#include "Manager.h"
#include "test.h"

TestManager manager {};
Manager* g_manager = new Manager(1. / 240.);

int main()
{
    auto p0 = g_manager->add_particle({ 0., 0., 0. });
    auto p1 = g_manager->add_particle({ 0., 1., 0. });
    auto p2 = g_manager->add_particle({ 1., 0., 0. });
    auto p3 = g_manager->add_particle({ 1., 1., 0. });
    auto p4 = g_manager->add_particle({ 0., 0., 1. });
    auto p5 = g_manager->add_particle({ 0., 1., 1. });
    auto p6 = g_manager->add_particle({ 1., 0., 1. });
    auto p7 = g_manager->add_particle({ 1., 1., 1. });

    {
        g_manager->add_face({ p0, p1, p2 });
        g_manager->add_face({ p3, p1, p2 });

        g_manager->add_face({ p4, p5, p6 });
        g_manager->add_face({ p7, p5, p6 });

        g_manager->add_face({ p0, p2, p4 });
        g_manager->add_face({ p6, p2, p4 });

        g_manager->add_face({ p0, p1, p4 });
        g_manager->add_face({ p5, p1, p4 });

        g_manager->add_face({ p1, p3, p5 });
        g_manager->add_face({ p7, p3, p5 });

        g_manager->add_face({ p2, p3, p6 });
        g_manager->add_face({ p7, p3, p6 });
    }

    { // Square face edge connectivity
        g_manager->add_spring({ p0, p1 });
        g_manager->add_spring({ p0, p2 });
        g_manager->add_spring({ p1, p3 });
        g_manager->add_spring({ p2, p3 });

        g_manager->add_spring({ p4, p5 });
        g_manager->add_spring({ p4, p6 });
        g_manager->add_spring({ p7, p5 });
        g_manager->add_spring({ p7, p6 });
    }

    { // Edges connecting corners, vertically
        g_manager->add_spring({ p0, p4 });
        g_manager->add_spring({ p1, p5 });
        g_manager->add_spring({ p2, p6 });
        g_manager->add_spring({ p3, p7 });
    }

    { // Cross edges across each face
        g_manager->add_spring({ p0, p3 });
        g_manager->add_spring({ p1, p2 });

        g_manager->add_spring({ p4, p7 });
        g_manager->add_spring({ p5, p6 });

        g_manager->add_spring({ p0, p6 });
        g_manager->add_spring({ p2, p4 });

        g_manager->add_spring({ p0, p5 });
        g_manager->add_spring({ p1, p4 });

        g_manager->add_spring({ p1, p7 });
        g_manager->add_spring({ p3, p5 });

        g_manager->add_spring({ p2, p7 });
        g_manager->add_spring({ p3, p6 });
    }

    { // Internal diagonals
        g_manager->add_spring({ p0, p7 });
        g_manager->add_spring({ p1, p6 });
        g_manager->add_spring({ p3, p4 });
        g_manager->add_spring({ p2, p5 });
    }

    {
        auto face = g_manager->face(0);

        auto const& v0 = g_manager->position(face.v0);
        auto const& v1 = g_manager->position(face.v1);
        auto const& v2 = g_manager->position(face.v2);

        Vec3 midpoint = (v0 + v1 + v2) / 3.;

        manager.add({ "Distance to Plane",
                      [&]() -> bool {
                          Vec3 p0 = v0 + face.normal;
                          Vec3 p1 = v1 + face.normal;
                          Vec3 p2 = v2 + face.normal;
                          Vec3 m = midpoint + face.normal;

                          return (face.distance_to_plane(p0) == 1.) && (face.distance_to_plane(p1) == 1.) && (face.distance_to_plane(p2) == 1.) && (face.distance_to_plane(m) == 1.);
                      } });

        auto const test_barycentric = [&](Vec3 p, double exp_alpha, double exp_beta, double exp_gamma, double tol = 1e-6) -> bool {
            auto p0 = face.project(p);
            auto [alpha, beta] = face.barycentric(p0);
            auto gamma = 1. - alpha - beta;

            return std::abs(alpha - exp_alpha) < tol && std::abs(beta - exp_beta) < tol && std::abs(gamma - exp_gamma) < tol;
        };

        manager.add({ "test_barycentric (vertex, 0)", [&]() { return test_barycentric(v0, 0., 0., 1.); } });
        manager.add({ "test_barycentric (vertex, 1)", [&]() { return test_barycentric(v1, 1., 0., 0.); } });
        manager.add({ "test_barycentric (vertex, 2)", [&]() { return test_barycentric(v2, 0., 1., 0.); } });
        manager.add({ "test_barycentric (midpoint)", [&]() { return test_barycentric(midpoint, 1. / 3., 1. / 3., 1. / 3.); } });

        manager.add({ "collision (midpoint, expected)", [&]() {
                         Vec3 position = midpoint + face.normal * g_manager->h;
                         Vec3 velocity = -face.normal;

                         auto particle = g_manager->add_particle(position, velocity);
                         return face.collision(particle);
                     } });

        manager.add({ "collision (midpoint, no collision)", [&]() {
                         Vec3 position = midpoint + (1. + 1e-6) * (face.normal * g_manager->h);
                         Vec3 velocity = -face.normal;

                         auto particle = g_manager->add_particle(position, velocity);
                         return !face.collision(particle);
                     } });
    }

    manager.run();
}