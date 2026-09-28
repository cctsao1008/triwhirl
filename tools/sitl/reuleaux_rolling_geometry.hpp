#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace triwhirl_sitl {

constexpr double kGeometryPi = 3.14159265358979323846;

struct Vec2 {
  double x = 0.0;
  double y = 0.0;
};

inline Vec2 add(const Vec2 a, const Vec2 b) {
  return Vec2{a.x + b.x, a.y + b.y};
}

inline Vec2 sub(const Vec2 a, const Vec2 b) {
  return Vec2{a.x - b.x, a.y - b.y};
}

inline Vec2 mul(const Vec2 a, const double scale) {
  return Vec2{a.x * scale, a.y * scale};
}

inline double dot(const Vec2 a, const Vec2 b) {
  return a.x * b.x + a.y * b.y;
}

inline double normSquared(const Vec2 a) { return dot(a, a); }

inline Vec2 rotate(const Vec2 p, const double angle_rad) {
  const double c = std::cos(angle_rad);
  const double s = std::sin(angle_rad);
  return Vec2{c * p.x - s * p.y, s * p.x + c * p.y};
}

inline std::array<Vec2, 3> reuleauxVertices(const double width_m) {
  const double circumradius_m = width_m / std::sqrt(3.0);
  return std::array<Vec2, 3>{
      Vec2{0.0, -circumradius_m},
      Vec2{0.5 * width_m, 0.5 * circumradius_m},
      Vec2{-0.5 * width_m, 0.5 * circumradius_m},
  };
}

struct ReuleauxContact {
  Vec2 body_point_m{};
  Vec2 world_relative_m{};
  double center_height_m = 0.0;
  double support_derivative_m_per_rad = 0.0;
  double contact_radius_sq_m2 = 0.0;
  double contact_radius_sq_derivative_m2_per_rad = 0.0;
  bool on_arc = false;
  int feature_index = -1;
};

inline bool pointInsideAllGeneratingDisks(
    const Vec2 point, const std::array<Vec2, 3>& centers,
    const double width_m) {
  const double radius_sq = width_m * width_m;
  const double tolerance = std::max(1.0e-14, radius_sq * 1.0e-10);
  for (const Vec2 center : centers) {
    if (normSquared(sub(point, center)) > radius_sq + tolerance) {
      return false;
    }
  }
  return true;
}

// Exact support/contact geometry for an ideal Reuleaux triangle of constant
// width.  The body-fixed origin is the geometric center; theta=0 places one
// vertex directly below the center (the local upright configuration).
//
// The ground is horizontal at world y=0.  For each orientation we minimize the
// world vertical coordinate over the Reuleaux boundary.  Candidate extrema are
// the three vertices plus the outward extreme of each generating circular arc.
// This handles both smooth arc contact and finite vertex normal cones without a
// sampled boundary approximation.
inline ReuleauxContact reuleauxGroundContact(const double theta_rad,
                                             const double width_m) {
  const auto vertices = reuleauxVertices(width_m);

  // World +Y expressed in the body frame: R(-theta) * [0, 1].
  const Vec2 up_body{std::sin(theta_rad), std::cos(theta_rad)};

  double best_score = std::numeric_limits<double>::infinity();
  Vec2 best_point{};
  bool best_on_arc = false;
  int best_index = -1;
  const double score_tolerance = std::max(1.0e-14, width_m * 1.0e-12);

  const auto consider = [&](const Vec2 point, const bool on_arc,
                            const int index, double& score,
                            Vec2& best, bool& best_arc, int& best_feature) {
    const double candidate_score = dot(up_body, point);
    if (candidate_score < score - score_tolerance) {
      score = candidate_score;
      best = point;
      best_arc = on_arc;
      best_feature = index;
    }
  };

  // Vertices are valid support points over the corner normal cones.
  for (int i = 0; i < 3; ++i) {
    consider(vertices[static_cast<std::size_t>(i)], false, i, best_score,
             best_point, best_on_arc, best_index);
  }

  // On a smooth circular arc, the lowest point is center - width * up_body.
  for (int i = 0; i < 3; ++i) {
    const Vec2 candidate =
        sub(vertices[static_cast<std::size_t>(i)], mul(up_body, width_m));
    if (pointInsideAllGeneratingDisks(candidate, vertices, width_m)) {
      consider(candidate, true, i, best_score, best_point, best_on_arc,
               best_index);
    }
  }

  const Vec2 world_relative = rotate(best_point, theta_rad);
  ReuleauxContact result{};
  result.body_point_m = best_point;
  result.world_relative_m = world_relative;
  result.center_height_m = -world_relative.y;

  // Envelope theorem for the support height h(theta):
  //   h = -r_y, dh/dtheta = -r_x.
  result.support_derivative_m_per_rad = -world_relative.x;
  result.contact_radius_sq_m2 = normSquared(best_point);
  result.on_arc = best_on_arc;
  result.feature_index = best_index;

  if (best_on_arc) {
    // p(theta) = center - width * [sin(theta), cos(theta)] on an active arc.
    // d|p|^2/dtheta = 2 p . dp/dtheta.
    const Vec2 up_body_derivative{std::cos(theta_rad), -std::sin(theta_rad)};
    const Vec2 point_derivative = mul(up_body_derivative, -width_m);
    result.contact_radius_sq_derivative_m2_per_rad =
        2.0 * dot(best_point, point_derivative);
  }
  return result;
}

struct RollingPose {
  double center_x_m = 0.0;
  double center_y_m = 0.0;
  double contact_x_m = 0.0;
  double contact_y_m = 0.0;
  Vec2 contact_body_m{};
  Vec2 contact_world_relative_m{};
};

inline RollingPose reuleauxRollingPose(const double theta_rad,
                                       const double center_x_m,
                                       const double width_m) {
  const ReuleauxContact contact = reuleauxGroundContact(theta_rad, width_m);
  RollingPose pose{};
  pose.center_x_m = center_x_m;
  pose.center_y_m = contact.center_height_m;
  pose.contact_x_m = center_x_m + contact.world_relative_m.x;
  pose.contact_y_m = pose.center_y_m + contact.world_relative_m.y;
  pose.contact_body_m = contact.body_point_m;
  pose.contact_world_relative_m = contact.world_relative_m;
  return pose;
}

// No-slip horizontal center velocity.  From v_contact = 0:
//   v_center = -omega x r_contact,
// therefore xdot_center = r_y * theta_dot.
inline double rollingCenterVelocityX(const double theta_rad,
                                     const double theta_rate_rad_s,
                                     const double width_m) {
  const ReuleauxContact contact = reuleauxGroundContact(theta_rad, width_m);
  return contact.world_relative_m.y * theta_rate_rad_s;
}

}  // namespace triwhirl_sitl
