#include "types.h"
#include "mpc.h"
#include <algorithm>

// 2D scalar cross product of vectors a and b.
inline float cross2D(const Vec3& a, const Vec3& b) {
    return a.x * b.y - a.y * b.x;
}

// Signed turn direction for points a -> b -> c.
inline float orientation2D(const Vec3& a, const Vec3& b, const Vec3& c) {
    return cross2D(b - a, c - a);
}

inline Vec3 calculateCenterOfMass(int num_components, const Component* components) {
    Vec3 com = Vec3 {0.0f, 0.0f, 0.0f};
    float total_mass = 0.0f;
    for (int i = 0; i < num_components; i++) {
        com += components[i].position * components[i].mass;
        total_mass += components[i].mass;
    }
    if (total_mass > 0.0f) {
        com = com / total_mass;
    }

    return com;
}

// Assumes p, a, and b have already been projected to z = 0.
inline float distanceToSegment(const Vec3& p, const Vec3& a, const Vec3& b) {
    Vec3 v = a - b;
    Vec3 w = p - b;

    float len_squared = v.x * v.x + v.y * v.y;

    if (len_squared < 1e-8f) {
        float dx = p.x - b.x;
        float dy = p.y - b.y;
        return dis(dx, dy);
    } else {
        float t = (w.x * v.x + w.y * v.y) / len_squared;
        t = std::max(0.0f, std::min(1.0f, t));

        Vec3 q = b + v * t;

        float dx = p.x - q.x;
        float dy = p.y - q.y;

        return dis(dx, dy);
    }
}

// Build the convex hull of four projected contact points
// Returns the hull vertices in counterclockwise order
// monotone chain algorithm
inline int calculateConvexHull(Array<Vec3, 4> points, Array<Vec3, 4>& hull) {
    // flatten points into ground-plane frame
    for (auto& p : points) {
        p.z = 0.0f;
    }

    // sort points from L->R by x
    std::sort(
        points.begin(),
        points.end(),
        [](const Vec3& a, const Vec3& b) {
            if (a.x != b.x) return a.x < b.x;
            return a.y < b.y;
        }
    );

    Array<Vec3, 8> temp{};
    int k = 0;

    // construct lower hull
    for (const auto& p : points) {
        while (k >= 2 && orientation2D(temp[k - 2], temp[k - 1], p) <= k_epsilon) {
            --k;
        }
        temp[k++] = p;
    }

    // construct upper hull
    const int upper_start = k + 1;

    for (int i = 2; i >= 0; --i) {
        const Vec3& p = points[i];

        while (k >= upper_start && orientation2D(temp[k - 2], temp[k - 1], p) <= k_epsilon) {
            --k;
        }

        temp[k++] = p;
    }

    // First point is repeated at the end.
    if (k > 1) --k;

    if (k > 4) return 0;

    for (int i = 0; i < k; ++i) {
        hull[i] = temp[i];
    }

    return k;
}

// monotonic chain algorithm
inline bool isPointInConvexPolygon(const Vec3& point, const Array<Vec3, 4>& polygon, int count) {
    // invalid polygon
    if (count < 3) return false;

    bool has_positive = false;
    bool has_negative = false;

    // walk along each edge, point on the same side of every edge
    for (int i = 0; i < count; ++i) {
        const Vec3& a = polygon[i];
        const Vec3& b = polygon[(i + 1) % count];

        float cross = orientation2D(a, b, point);

        if (cross > k_epsilon) has_positive = true;
        if (cross < -k_epsilon) has_negative = true;

        if (has_positive && has_negative) return false;
    }

    return true;
}

inline float calculateStabilityCost(const FootholdMpc::State& state, const Vec3& com, const Vec3& candidate_foothold, float target_margin) {
    // model the support configuration after the swinging leg touches down = 3 feet in stance + candidate touchdown
    Array<Vec3, 4> contacts;
    int index = 0;

    for (int i = 0; i < 4; i++) {
        if (i == state.swing_leg) continue;

        if (index >= 3) {
            return k_float_max;
        }

        contacts[index] = state.current_foot_positions_bf[i];
        contacts[index].z = 0.0f;
        index++;
    }

    if (index != 3) {
        return k_float_max;
    }

    // last contact = candidate
    contacts[3] = candidate_foothold;
    contacts[3].z = 0.0f;

    // project the CoM into the same plane
    Vec3 projected_com = com;
    projected_com.z = 0.0f;

    // build support polygon
    Array<Vec3, 4> hull;
    int hull_size = calculateConvexHull(contacts, hull);

    // bad support polygon = 1 foot inside triangle of other 3
    if (hull_size < 3) {
        return k_float_max;
    }

    // find dis to nearest edge
    float min_edge_distance = k_float_max;
    for (int i = 0; i < hull_size; ++i) {
        const Vec3& a = hull[i];
        const Vec3& b = hull[(i + 1) % hull_size]; // wrap last vertex to first

        float distance = distanceToSegment(projected_com, a, b);
        min_edge_distance = std::min(min_edge_distance, distance);
    }

    // inside polygon
    if (isPointInConvexPolygon(projected_com, hull, hull_size)) {
        // penalize being inside but closer than the desired margin
        float margin_deficit = std::max(0.0f, target_margin - min_edge_distance);

        return margin_deficit * margin_deficit;
    } else {
        // outside polygon = distance-to-boundary penalty plus desired margin.
        float outside_distance = min_edge_distance;
        float penalty = target_margin + outside_distance;

        return penalty * penalty;
    }
}