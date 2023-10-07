#pragma once

#include <array>
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

namespace Geometry {

static const double kEpsilon = 0.000001;

//=============================================================================

struct Point {
    double x = NAN, y = NAN, z = NAN;

    Point() = default;
    Point(double x, double y, double z) : x(x), y(y), z(z) {}

    bool is_valid() const {
        return !std::isnan(x) && !std::isnan(y) && !std::isnan(z);
    }
    bool operator==(const Point &rhs) const {
        return (fabs(x - rhs.x) < kEpsilon) && (fabs(y - rhs.y) < kEpsilon) &&
               (fabs(z - rhs.z) < kEpsilon);
    }
};

//=============================================================================

struct Vector : public Point {
    Vector() = default;
    Vector(double x, double y, double z) : Point(x, y, z) {}
    Vector(const Point &a) : Point(a) {}

    double module2() const { return x * x + y * y + z * z; }
    double module() const { return std::sqrt(module2()); }

    Vector cross(const Vector &rhs) const {
        return {y * rhs.z - rhs.y * z, rhs.x * z - x * rhs.z,
                x * rhs.y - rhs.x * y};
    }

    double operator*(const Vector &rhs) const {
        return x * rhs.x + y * rhs.y + z * rhs.z;
    }
    Vector operator*(double value) const {
        return {x * value, y * value, z * value};
    }
    Vector &operator-=(const Vector &rhs) {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }
    Vector &operator+=(const Vector &rhs) {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }
};

std::ostream &operator<<(std::ostream &out, const Point &point) {
    return out << "{" << point.x << ", " << point.y << ", " << point.z << "}";
}

Vector operator-(Vector lhs, const Vector &rhs) {
    lhs -= rhs;
    return lhs;
}

Vector operator+(Vector lhs, const Vector &rhs) {
    lhs += rhs;
    return lhs;
}

//=============================================================================
// Line in parametrical form
// x = x_0 + a*t
// y = y_0 + b*t
// z = z_0 + c*t
//=============================================================================

struct Line {
    Vector direct;
    Vector begin;

    // TODO add 3D
    Line(const Point &first, const Point &second)
        : begin(first),
          direct(second.x - first.x, second.y - first.y, second.z - first.z) {}

    bool is_valid() const { return begin.is_valid() && direct.is_valid(); }

    Vector get_intersect(const Line &other) const {
        Vector normal_vec = direct.cross(other.direct);

        double t = (direct.cross(other.begin - begin) * normal_vec) /
                   normal_vec.module2();

        if (!std::isnan(t)) {
            return other.begin + direct * t;
        }
        return {};
    }
};

std::ostream &operator<<(std::ostream &out, const Line &line) {
    // x = x_0 + a * t
    const char *signa = (line.direct.x >= 0.0) ? "+" : "";
    const char *signb = (line.direct.y >= 0.0) ? "+" : "";
    const char *signc = (line.direct.z >= 0.0) ? "+" : "";
    out << "x = " << line.begin.x << signa << line.direct.x << "*t\n";
    out << "y = " << line.begin.y << signb << line.direct.y << "*t\n";
    out << "z = " << line.begin.z << signc << line.direct.z << "*t\n";

    return out;
}

//=============================================================================

struct LineSegment {
    Vector begin;
    Vector end;

    LineSegment(const Vector &begin, const Vector &direct)
        : begin(begin), end(begin + direct) {}

    // only works if line segments in one plane
    Vector get_one_plane_intersect(const LineSegment &other) const {
        double numerator =
            (other.end.y - end.y) * (other.end.x - other.begin.x) -
            (other.end.x - end.x) * (other.end.y - other.begin.y);

        double denominator = (begin.y - end.y) * (end.x - begin.y) -
                             (begin.x - end.x) * (other.end.y - other.begin.y);

        double u = numerator / denominator;

        if (u < 1.0 && u > 0.0) {
            double x = u * (begin.x - end.x) + end.x;
            double y = u * (begin.y - end.y) + end.y;
            double z = u * (begin.z - end.z) + end.z;
            return {x, y, z};
        }

        return {};
    }

    bool is_intersect(const LineSegment &other) const {
        return get_one_plane_intersect(other).is_valid();
    }
};

//=============================================================================

struct Triangle {
    std::array<Vector, 3> vertexes;

    Triangle(const Vector &a, const Vector &b, const Vector &c)
        : vertexes{a, b, c} {}

    double get_ray_distance(const Vector &ray_origin,
                                const Vector &ray_vector) const {
        Vector e1 = vertexes[1] - vertexes[0];
        Vector e2 = vertexes[2] - vertexes[0];
        Vector h = ray_vector.cross(e2);
        double det = e1 * h;

        // Луч параллелен плоскости
        if (std::fabs(det) < kEpsilon) {
            Vector normal = e1.cross(e2);
            double distance =
                normal * (vertexes[0] - ray_origin) / normal.module();
            if (std::fabs(distance) < kEpsilon) {
                // Луч и треугольник в одной плоскости
                Vector e3 = vertexes[2] - vertexes[1];

                Vector point;
                LineSegment ray_segment(ray_origin, ray_vector);

                LineSegment side1(vertexes[0], e1);
                point = side1.get_one_plane_intersect(ray_segment);
                if (point.is_valid()) {
                    return (point - ray_origin).module();
                }

                LineSegment side2(vertexes[1], e3);
                point = side2.get_one_plane_intersect(ray_segment);
                if (point.is_valid()) {
                    return (point - ray_origin).module();
                }

                LineSegment side3(vertexes[2], e2);
                point = side3.get_one_plane_intersect(ray_segment);
                if (point.is_valid()) {
                    return (point - ray_origin).module();
                }
            }
            return NAN;
        }

        double f = 1 / det;
        Vector s = ray_origin - vertexes[0];
        double u = (s * h) * f;
        if (u < 0.0 || u > 1.0) {
            return NAN;
        }

        Vector q = s.cross(e1);
        double v = f * (ray_vector * q);
        if (v < 0.0 || u + v > 1.0) {
            return NAN;
        }
        double t = f * (e2 * q);
        return (ray_vector * t).module();
    }

    bool is_inside(const Vector &dot, const Vector &normal) const {
        if ((vertexes[1] - vertexes[0]).cross(dot - vertexes[0]) * normal <
            kEpsilon)
            return false;

        if ((dot - vertexes[0]).cross(vertexes[2] - vertexes[0]) * normal <
            kEpsilon)
            return false;

        if ((vertexes[1] - dot).cross(vertexes[2] - dot) * normal < kEpsilon)
            return false;
        return true;
    }

    Vector get_normal() const {
        Vector e1 = vertexes[1] - vertexes[0];
        Vector e2 = vertexes[2] - vertexes[0];
        return e1.cross(e2);
    }

    bool is_intersect(const Triangle &other) const {
        int index[] = {0, 1, 2, 0};

        double distance = 0;

        for (int i = 0; i < 3; ++i) {
            Vector direct =
                other.vertexes[index[i + 1]] - other.vertexes[index[i]];

            distance = get_ray_distance(other.vertexes[index[i]], direct);

            if (!std::isnan(distance) && distance < direct.module()) {
                return true;
            }
        }
        Vector normal = get_normal();

        bool inside = true;
        for (const Vector &vertex : other.vertexes) {
            inside *= is_inside(vertex, normal);
        }
        return inside;
    }
};

bool is_intersect(const Triangle &first, const Triangle &second) {
    return first.is_intersect(second) || second.is_intersect(first);
}

} // namespace Geometry
