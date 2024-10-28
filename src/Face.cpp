#include "Face.h"
#include "Manager.h"

Face::Face(size_t v0, size_t v1, size_t v2)
    : v0(v0)
    , v1(v1)
    , v2(v2)
{
    Vec3 p0 = g_manager->position(v0);
    normal = (g_manager->position(v1) - p0).cross(g_manager->position(v2) - p0);

    normal.normalize();
}

flatten Face::Face(Particle v0, Particle v1, Particle v2)
    : Face(v0.index, v1.index, v2.index) {};

double Face::distance_to_plane(Vec3 p) const
{
    return (p - g_manager->position(v0)).dot(normal);
}

Vec2 Face::project(Vec3 p) const
{
    // FIXME: (1) Determine which coordinate that should be dropped on face construction so we don't do this check every single time
    // FIXME: (2) There's definitely a way to do this with fewer comparisons; but it won't matter if we do (1)
    if (std::abs(normal.x()) > std::abs(normal.y()) && std::abs(normal.x()) > std::abs(normal.z()))
        return { p.y(), p.z() };

    if (std::abs(normal.y()) > std::abs(normal.x()) && std::abs(normal.y()) > std::abs(normal.z()))
        return { p.x(), p.z() };

    return { p.x(), p.y() };
}

/**
 * Vec2 Face::project(Vec3 p) const
{
    Vec3 const& p0 = g_manager->position(v0);
    Vec3 p_prime = p - ((p - p0).dot(normal) * normal) - p0;

    Vec3 u = (g_manager->position(v1) - p0).normalized();
    Vec3 v = u.cross(normal);

    return { p_prime.dot(u), p_prime.dot(v) };
}
 */

template <typename T>
int sgn(T val) { return (T {} < val) - (val < T {}); }

flatten std::pair<double, double> Face::barycentric(Vec2 p) const
{
    // Project triangle and point of collision into 2D by dropping largest coordinate in normal
    auto p0 = project(g_manager->position(v0));
    auto p1 = project(g_manager->position(v1));
    auto p2 = project(g_manager->position(v2));

    // This is area of the parallelogram, not the triangle; however, the denominator uses (2 * area) so leaving it like this is OK
    double area = (Eigen::Matrix2d() << (p1 - p0).transpose(), (p2 - p0).transpose()).finished().determinant();

    // Transform p into barycentric coordinates w.r.t. the face's 2D projection
    double alpha = (p0.y() * p2.x() - p0.x() * p2.y() + (p2.y() - p0.y()) * p.x() + (p0.x() - p2.x()) * p.y()) / area;
    double beta = (p0.x() * p1.y() - p0.y() * p1.x() + (p0.y() - p1.y()) * p.x() + (p1.x() - p0.x()) * p.y()) / area;

    return std::make_pair(alpha, beta);
}

flatten bool Face::collision(Particle particle) const
{
    Vec3 pi = particle.position();
    Vec3 pf = particle.position() + particle.velocity() * g_manager->h;

    // Check collision with plane
    auto dn = distance_to_plane(pi);
    auto dnp1 = distance_to_plane(pf);

    if (sgn(dn) == sgn(dnp1))
        return false;

    // Get point of collision with plane
    auto f = dn / (dn - dnp1);
    auto pc = project(particle.position() + particle.velocity() * g_manager->h * f);

    // Check if barycentric coordinates lie in the triangle
    auto [alpha, beta] = barycentric(pc);
    return (alpha > 0.) && (beta > 0.) && ((1. - alpha - beta) > 0.);
}
