/*
 * Copyright 2026 RoboLotus
 * Licensed under the Apache License, Version 2.0.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Visual.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/plugin/Register.hh>

namespace hazard_guard_simulation
{
class HeatTransferSystem final :
  public ignition::gazebo::System,
  public ignition::gazebo::ISystemConfigure,
  public ignition::gazebo::ISystemPreUpdate
{
private:
  struct Layer
  {
    std::string modelName;
    double distance{0.0};
    double sourceTemperature{333.15};
    double decayLength{0.18};
    ignition::math::Pose3d visiblePose;
    ignition::gazebo::Entity entity{ignition::gazebo::kNullEntity};
    std::vector<ignition::gazebo::Entity> thermalEntities;
    bool visible{false};
  };

public:
  void Configure(
    const ignition::gazebo::Entity &,
    const std::shared_ptr<const sdf::Element> & sdf,
    ignition::gazebo::EntityComponentManager &,
    ignition::gazebo::EventManager &) override
  {
    this->ambientTemperature =
      sdf->Get<double>("ambient_temperature", this->ambientTemperature).first;
    this->effectiveDiffusivity = std::max(
      1e-6,
      sdf->Get<double>(
        "effective_diffusivity", this->effectiveDiffusivity).first);
    this->responseTime = std::max(
      0.05, sdf->Get<double>("response_time", this->responseTime).first);
    this->updatePeriod = std::max(
      0.05, sdf->Get<double>("update_period", this->updatePeriod).first);
    this->minimumVisibleRise = std::max(
      0.0,
      sdf->Get<double>(
        "minimum_visible_rise", this->minimumVisibleRise).first);

    auto sdfClone = sdf->Clone();
    if (!sdfClone->HasElement("layer"))
      return;

    for (auto elem = sdfClone->GetElement("layer"); elem;
      elem = elem->GetNextElement("layer"))
    {
      Layer layer;
      layer.modelName = elem->Get<std::string>("model");
      layer.distance = std::max(0.0, elem->Get<double>("distance", 0.0).first);
      layer.sourceTemperature = elem->Get<double>(
        "source_temperature", layer.sourceTemperature).first;
      layer.decayLength = std::max(
        1e-4, elem->Get<double>("decay_length", layer.decayLength).first);
      layer.visiblePose = elem->Get<ignition::math::Pose3d>("pose");
      this->layers.push_back(layer);
    }
  }

  void PreUpdate(
    const ignition::gazebo::UpdateInfo & info,
    ignition::gazebo::EntityComponentManager & ecm) override
  {
    if (info.paused || this->layers.empty())
      return;

    const double now = std::chrono::duration<double>(info.simTime).count();
    if (now < this->lastTime)
    {
      for (auto & layer : this->layers)
        this->ResetLayer(layer, ecm);
      this->lastUpdateTime = -1.0;
    }
    this->lastTime = now;

    if (this->lastUpdateTime >= 0.0 &&
      now - this->lastUpdateTime < this->updatePeriod)
    {
      return;
    }
    this->lastUpdateTime = now;

    for (auto & layer : this->layers)
    {
      this->ResolveLayer(layer, ecm);
      if (layer.entity == ignition::gazebo::kNullEntity ||
        layer.thermalEntities.empty())
      {
        continue;
      }

      const double temperature = this->LayerTemperature(layer, now);
      if (!layer.visible &&
        temperature >= this->ambientTemperature + this->minimumVisibleRise)
      {
        this->SetLayerPose(layer, layer.visiblePose, ecm);
        layer.visible = true;
      }
    }
  }

private:
  double LayerTemperature(const Layer & layer, const double now) const
  {
    // Low-cost constant-source diffusion approximation. Heat arrival follows
    // r^2 / (4 * alpha), while the steady rise decays with distance. The
    // effective alpha intentionally folds natural convection into conduction.
    const double arrivalTime =
      layer.distance * layer.distance / (4.0 * this->effectiveDiffusivity);
    if (now <= arrivalTime)
      return this->ambientTemperature;

    const double sourceRise = std::max(
      0.0, layer.sourceTemperature - this->ambientTemperature);
    const double steadyRise =
      sourceRise * std::exp(-layer.distance / layer.decayLength);
    const double localResponseTime = this->responseTime + 0.35 * arrivalTime;
    const double elapsed = now - arrivalTime;
    return this->ambientTemperature +
      steadyRise * (1.0 - std::exp(-elapsed / localResponseTime));
  }

  void ResolveLayer(
    Layer & layer,
    ignition::gazebo::EntityComponentManager & ecm)
  {
    if (layer.entity == ignition::gazebo::kNullEntity)
    {
      layer.entity = ecm.EntityByComponents(
        ignition::gazebo::components::Name(layer.modelName),
        ignition::gazebo::components::Model());
    }
    if (layer.entity == ignition::gazebo::kNullEntity ||
      !layer.thermalEntities.empty())
    {
      return;
    }

    for (const auto entity : ecm.Descendants(layer.entity))
    {
      if (ecm.Component<ignition::gazebo::components::Visual>(entity))
      {
        layer.thermalEntities.push_back(entity);
      }
    }
  }

  void SetLayerPose(
    Layer & layer,
    const ignition::math::Pose3d & pose,
    ignition::gazebo::EntityComponentManager & ecm)
  {
    auto poseComponent =
      ecm.Component<ignition::gazebo::components::Pose>(layer.entity);
    if (!poseComponent)
      return;
    *poseComponent = ignition::gazebo::components::Pose(pose);
    ecm.SetChanged(
      layer.entity,
      ignition::gazebo::components::Pose::typeId,
      ignition::gazebo::ComponentState::OneTimeChange);
  }

  void ResetLayer(
    Layer & layer,
    ignition::gazebo::EntityComponentManager & ecm)
  {
    if (layer.entity != ignition::gazebo::kNullEntity)
    {
      this->SetLayerPose(
        layer, ignition::math::Pose3d(0, 0, -10, 0, 0, 0), ecm);
    }
    layer.visible = false;
  }

  double ambientTemperature{293.15};
  double effectiveDiffusivity{0.0018};
  double responseTime{3.5};
  double updatePeriod{0.5};
  double minimumVisibleRise{0.35};
  double lastTime{0.0};
  double lastUpdateTime{-1.0};
  std::vector<Layer> layers;
};
}  // namespace hazard_guard_simulation

IGNITION_ADD_PLUGIN(
  hazard_guard_simulation::HeatTransferSystem,
  ignition::gazebo::System,
  hazard_guard_simulation::HeatTransferSystem::ISystemConfigure,
  hazard_guard_simulation::HeatTransferSystem::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(
  hazard_guard_simulation::HeatTransferSystem,
  "hazard_guard_simulation::HeatTransferSystem")
