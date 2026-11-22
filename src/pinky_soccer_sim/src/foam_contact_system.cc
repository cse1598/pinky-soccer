// Planar compliant contacts. Native object/object collision remains enabled.
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/LinearVelocityCmd.hh>
#include <gz/sim/components/AngularVelocityCmd.hh>
#include <sdf/Element.hh>
#include <array>
#include <chrono>
#include <string>
#include <vector>

extern "C" void pinky_contact(const double *, const double *, const double *,
                             const double *, double, double, double, double *);
extern "C" void pinky_box_contact(const double *, const double *, const double *,
    const double *, double, double, double, const double *, const double *, const double *, double *);

namespace pinky_soccer_sim
{
class FoamContact : public gz::sim::System,
                    public gz::sim::ISystemConfigure,
                    public gz::sim::ISystemPreUpdate
{
  struct Contact
  {
    std::string model, link;
    gz::math::Vector3d offset;
    double radius, mass;
    std::array<double, 6> params;
    bool initialize = false;
    bool enabled = false, clearCommand = false;
    gz::math::Vector3d initialVelocity, initialAngular;
  };
  std::vector<Contact> contacts;
  struct Box
  {
    std::array<double, 3> center, half;
    std::array<double, 9> rotation;
  };
  std::vector<Box> boxes;
  std::array<double, 6> wallParams;

  public: void Configure(const gz::sim::Entity &,
      const std::shared_ptr<const sdf::Element> &sdf,
      gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    if (!sdf->HasElement("contact")) return;
    auto e = sdf->FindElement("contact");
    while (e)
    {
      Contact c;
      c.model = e->Get<std::string>("model");
      c.link = e->Get<std::string>("link");
      c.offset = e->Get<gz::math::Vector3d>("offset");
      c.radius = e->Get<double>("radius");
      c.mass = e->Get<double>("mass");
      if (e->HasElement("initial_velocity"))
      {
        c.initialize = true;
        c.initialVelocity = e->Get<gz::math::Vector3d>("initial_velocity");
        c.initialAngular = e->Get<gz::math::Vector3d>("initial_angular");
      }
      const std::array<std::string, 6> names =
        {"mu", "restitution", "stiffness", "effective_mass", "rolling_decel", "gravity"};
      for (size_t i = 0; i < names.size(); ++i) c.params[i] = e->Get<double>(names[i]);
      contacts.push_back(c);
      e = e->GetNextElement("contact");
    }
    if (sdf->HasElement("wall_parameters"))
    {
      auto params = sdf->FindElement("wall_parameters");
      const std::array<std::string, 6> names =
        {"mu", "restitution", "stiffness", "effective_mass", "rolling_decel", "gravity"};
      for (size_t i = 0; i < names.size(); ++i) wallParams[i] = params->Get<double>(names[i]);
    }
    auto box = sdf->FindElement("box");
    while (box)
    {
      Box b;
      auto pose = box->Get<gz::math::Pose3d>("pose");
      auto size = box->Get<gz::math::Vector3d>("size");
      for (int j = 0; j < 3; ++j)
      {
        b.center[j] = pose.Pos()[j];
        b.half[j] = size[j]/2;
        gz::math::Vector3d axis;
        axis[j] = 1;
        auto column = pose.Rot().RotateVector(axis);
        for (int i = 0; i < 3; ++i) b.rotation[3*i+j] = column[i];
      }
      boxes.push_back(b);
      box = box->GetNextElement("box");
    }
  }

  public: void PreUpdate(const gz::sim::UpdateInfo &info,
                        gz::sim::EntityComponentManager &ecm) override
  {
    if (info.paused || info.dt.count() <= 0) return;
    const double dt = std::chrono::duration<double>(info.dt).count();
    for (auto &c : contacts)
    {
      auto model = ecm.EntityByComponents(gz::sim::components::Name(c.model));
      auto entity = ecm.EntityByComponents(gz::sim::components::Link(),
        gz::sim::components::Name(c.link), gz::sim::components::ParentEntity(model));
      if (entity == gz::sim::kNullEntity) continue;  // robot may spawn after world
      gz::sim::Link link(entity);
      if (!c.enabled) {link.EnableVelocityChecks(ecm); c.enabled = true;}
      if (c.clearCommand)
      {
        ecm.RemoveComponent<gz::sim::components::LinearVelocityCmd>(entity);
        ecm.RemoveComponent<gz::sim::components::AngularVelocityCmd>(entity);
        c.clearCommand = false;
      }
      if (c.initialize)
      {
        link.SetLinearVelocity(ecm, c.initialVelocity);
        link.SetAngularVelocity(ecm, c.initialAngular);
        c.initialize = false;
        c.clearCommand = true;
      }
      const auto velocity = link.WorldLinearVelocity(ecm);
      const auto angular = link.WorldAngularVelocity(ecm);
      const auto inertial = ecm.Component<gz::sim::components::Inertial>(entity);
      if (!velocity || !angular || !inertial) continue;
      const auto pose = gz::sim::worldPose(entity, ecm);
      const auto offset = pose.Rot().RotateVector(c.offset);
      const auto point = pose.Pos() + offset;
      const auto v = *velocity + angular->Cross(offset);
      double p[3] = {point.X(), point.Y(), point.Z()};
      double linear[3] = {v.X(), v.Y(), v.Z()};
      double w[3] = {angular->X(), angular->Y(), angular->Z()};
      double output[6];
      pinky_contact(p, linear, w, c.params.data(), c.radius, c.mass, dt, output);
      if (c.model == "soccer_ball")
      {
        for (const auto &box : boxes)
        {
          double extra[6];
          pinky_box_contact(p, linear, w, wallParams.data(), c.radius, c.mass, dt,
                           box.center.data(), box.half.data(), box.rotation.data(), extra);
          for (int i = 0; i < 6; ++i) output[i] += extra[i];
        }
      }
      gz::math::Vector3d force(output[0], output[1], output[2]);
      gz::math::Vector3d torque(output[3], output[4], output[5]);
      const auto com = pose.Pos() + pose.Rot().RotateVector(inertial->Data().Pose().Pos());
      torque += (point - com).Cross(force);
      link.AddWorldWrench(ecm, force, torque);
    }
  }
};
}
GZ_ADD_PLUGIN(pinky_soccer_sim::FoamContact, gz::sim::System,
              pinky_soccer_sim::FoamContact::ISystemConfigure,
              pinky_soccer_sim::FoamContact::ISystemPreUpdate)
