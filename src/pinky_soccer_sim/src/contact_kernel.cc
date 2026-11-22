// Identical effective sphere/foam contact law used by both simulator adapters.
#include <algorithm>
#include <cmath>
#include <array>
#include <vector>

namespace
{
double dampingRatio(double restitution, double stiffness, double mass, double dt)
{
  if (restitution <= 0) return 1.0;  // critically damped wheel support
  // Invert the discrete unilateral spring response once per preset/time step.
  // This includes force release and semi-implicit integration, not a bilateral
  // spring's half-period approximation. Half-step penetration averages impact phase.
  const std::array<double, 4> key = {restitution, stiffness, mass, dt};
  thread_local std::vector<std::pair<std::array<double, 4>, double>> cache;
  for (const auto &item : cache) if (item.first == key) return item.second;
  // Keep the explicit damping update in its monotonic/stable region.
  double low = 0, high = std::min(2.0, .4/(dt*std::sqrt(stiffness/mass)));
  const double target = std::max(.001, restitution);
  for (int i = 0; i < 28; ++i)
  {
    const double z = (low + high)/2;
    const double damping = 2*z*std::sqrt(stiffness*mass);
    double depth = .5*dt, velocity = 1;
    for (int step = 0; step < 10000; ++step)
    {
      const double force = std::max(0.0, stiffness*depth + damping*velocity);
      if (force == 0 && velocity < 0) break;
      velocity -= dt*force/mass;
      depth += dt*velocity;
    }
    if (-velocity > target) low = z;
    else high = z;
  }
  const double result = (low + high)/2;
  cache.push_back({key, result});
  return result;
}
}

extern "C" void pinky_contact(
    const double *p, const double *v, const double *w, const double *c,
    double radius, double mass, double dt, double *out)
{
  // c = [mu, restitution, spring N/m, effective mass kg, rolling decel, gravity]
  std::fill(out, out + 6, 0.0);
  const double depth = radius - p[2];
  if (depth <= 0 || dt <= 0 || radius <= 0 || mass <= 0) return;
  const double zeta = dampingRatio(c[1], c[2], c[3], dt);
  const double damping = 2*zeta*std::sqrt(c[2]*c[3]);
  const double normal = std::max(0.0, c[2]*depth - damping*v[2]);
  out[2] = normal;
  // Velocity of the bottom surface, including sphere rotation.
  const double sx = v[0] - radius*w[1];
  const double sy = v[1] + radius*w[0];
  const double speed = std::hypot(sx, sy);
  if (speed > 1e-12)
  {
    // Bound explicit friction by the solid sphere's tangential effective mass.
    const double force = std::min(c[0]*normal, (2.0/7.0)*mass*speed/dt);
    out[0] = -force*sx/speed;
    out[1] = -force*sy/speed;
    out[3] = radius*out[1];
    out[4] = -radius*out[0];
  }
  const double spin = std::hypot(w[0], w[1]);
  if (spin > 1e-12)
  {
    // Pure rolling: a = torque / (1.4*m*r). Scale by instantaneous normal load.
    const double torque = std::min(1.4*radius*c[4]*normal/c[5],
                                  .4*mass*radius*radius*spin/dt);
    out[3] -= torque*w[0]/spin;
    out[4] -= torque*w[1]/spin;
  }
}

extern "C" void pinky_box_contact(
    const double *p, const double *v, const double *w, const double *c,
    double radius, double mass, double dt, const double *center,
    const double *half, const double *rotation, double *out)
{
  std::fill(out, out + 6, 0.0);
  double local[3] = {}, nearest[3], delta[3], normal[3] = {};
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 3; ++j) local[i] += rotation[3*j+i]*(p[j]-center[j]);
    nearest[i] = std::clamp(local[i], -half[i], half[i]);
    delta[i] = local[i]-nearest[i];
  }
  double distance = std::sqrt(delta[0]*delta[0]+delta[1]*delta[1]+delta[2]*delta[2]);
  if (distance >= radius) return;
  if (distance < 1e-12)
  {
    int axis = 0;
    for (int i = 1; i < 3; ++i)
      if (half[i]-std::abs(local[i]) < half[axis]-std::abs(local[axis])) axis = i;
    delta[0] = delta[1] = delta[2] = 0;
    delta[axis] = local[axis] < 0 ? -1 : 1;
    distance = -(half[axis]-std::abs(local[axis]));
  }
  else for (double &value : delta) value /= distance;
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) normal[i] += rotation[3*i+j]*delta[j];
  // Orthonormal contact frame [tangent, normal cross tangent, normal].
  double t[3] = {-normal[1], normal[0], 0};
  double length = std::hypot(t[0], t[1]);
  if (length < 1e-10) {t[0] = 1; t[1] = 0; length = 1;}
  for (double &value : t) value /= length;
  double basis[9] = {t[0], t[1], t[2],
    normal[1]*t[2]-normal[2]*t[1], normal[2]*t[0]-normal[0]*t[2],
    normal[0]*t[1]-normal[1]*t[0], normal[0], normal[1], normal[2]};
  double position[3] = {0, 0, distance}, linear[3] = {}, angular[3] = {}, force[6];
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
    {linear[i] += basis[3*i+j]*v[j]; angular[i] += basis[3*i+j]*w[j];}
  pinky_contact(position, linear, angular, c, radius, mass, dt, force);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j)
    {out[i] += basis[3*j+i]*force[j]; out[i+3] += basis[3*j+i]*force[j+3];}
}
