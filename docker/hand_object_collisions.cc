// Gazebo Classic contact masks: https://get.gazebosim.org/tutorials?tut=collide_bitmask
// Only the grasp object and rh_ collision geometries receive disjoint masks.
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/physics/physics.hh>
#include <functional>
#include <string>

namespace gazebo
{
class HandObjectCollisions : public WorldPlugin
{
  public: void Load(physics::WorldPtr world, sdf::ElementPtr sdf) override
  {
    this->world = world;
    this->objectName = sdf->Get<std::string>("object_name", "rect_object").first;
    this->update = event::Events::ConnectWorldUpdateBegin(
        std::bind(&HandObjectCollisions::Filter, this));
    gzmsg << "[shlite] Hand-object physical contacts disabled for "
          << this->objectName << "\n";
  }

  private: void Filter()
  {
    // Run before physics, including after models are spawned or reset. Keeping
    // the geometry preserves the object's contacts with the arm and the floor.
    for (const auto &model : this->world->Models())
      for (const auto &link : model->GetLinks())
        for (const auto &collision : link->GetCollisions())
        {
          const bool object = model->GetName() == this->objectName;
          // Gazebo may lump fixed hand links into an arm link; in that case
          // rh_ survives in the collision name rather than the link name.
          const bool hand = link->GetName().compare(0, 3, "rh_") == 0 ||
                            collision->GetName().find("rh_") != std::string::npos;
          auto surface = collision->GetSurface();
          if (surface && surface->collideBitmask != 0)
          {
            if (object)
              surface->collideBitmask = 0x01;
            else if (hand)
              surface->collideBitmask = 0x02;
          }
        }
  }

  private: physics::WorldPtr world;
  private: event::ConnectionPtr update;
  private: std::string objectName;
};
GZ_REGISTER_WORLD_PLUGIN(HandObjectCollisions)
}
