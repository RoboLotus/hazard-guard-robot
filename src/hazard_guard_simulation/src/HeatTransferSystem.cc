/*
 * Copyright 2026 RoboLotus
 * Licensed under the Apache License, Version 2.0.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <string>
#include <vector>

#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Temperature.hh>
#include <ignition/gazebo/components/Visual.hh>
#include <ignition/math/Temperature.hh>
#include <ignition/msgs/double.pb.h>
#include <ignition/math/Pose3.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

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
    bool alwaysVisible{false};
    bool dynamicTemperature{false};
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
      layer.alwaysVisible = elem->Get<bool>(
        "always_visible", false).first;
      layer.visible = layer.alwaysVisible;
      if (elem->HasElement("temperature_topic"))
      {
        const auto topic = elem->Get<std::string>("temperature_topic");
        layer.dynamicTemperature = !topic.empty();
        if (this->temperatureTopic.empty())
          this->temperatureTopic = topic;
      }
      this->layers.push_back(layer);
    }

    if (!this->temperatureTopic.empty())
    {
      this->transportNode.Subscribe(
        this->temperatureTopic,
        &HeatTransferSystem::OnTemperature,
        this);
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
      this->SetLayerTemperature(layer, temperature, ecm);
      if (!layer.visible &&
        temperature >= this->ambientTemperature + this->minimumVisibleRise)
      {
        this->SetLayerPose(layer, layer.visiblePose, ecm);
        layer.visible = true;
      }
      else if (layer.dynamicTemperature && !layer.alwaysVisible &&
        layer.visible &&
        temperature < this->ambientTemperature + this->minimumVisibleRise)
      {
        this->SetLayerPose(
          layer, ignition::math::Pose3d(0, 0, -10, 0, 0, 0), ecm);
        layer.visible = false;
      }
    }
  }

private:
  void OnTemperature(const ignition::msgs::Double & message)
  {
    if (!std::isfinite(message.data()) || message.data() < 0.0)
      return;
    std::lock_guard<std::mutex> lock(this->temperatureMutex);
    this->dynamicSourceTemperature = message.data();
    this->hasDynamicSourceTemperature = true;
  }

  double SourceTemperature(const Layer & layer) const
  {
    if (!layer.dynamicTemperature)
      return layer.sourceTemperature;
    std::lock_guard<std::mutex> lock(this->temperatureMutex);
    return this->hasDynamicSourceTemperature ?
      this->dynamicSourceTemperature : layer.sourceTemperature;
  }

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
      0.0, this->SourceTemperature(layer) - this->ambientTemperature);
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

  void SetLayerTemperature(
    Layer & layer,
    const double temperature,
    ignition::gazebo::EntityComponentManager & ecm)
  {
    for (const auto entity : layer.thermalEntities)
    {
      const ignition::math::Temperature value(temperature);
      auto component =
        ecm.Component<ignition::gazebo::components::Temperature>(entity);
      if (component)
      {
        *component = ignition::gazebo::components::Temperature(value);
        ecm.SetChanged(
          entity,
          ignition::gazebo::components::Temperature::typeId,
          ignition::gazebo::ComponentState::OneTimeChange);
      }
      else
      {
        ecm.CreateComponent(
          entity,
          ignition::gazebo::components::Temperature(value));
      }
    }
  }

  void ResetLayer(
    Layer & layer,
    ignition::gazebo::EntityComponentManager & ecm)
  {
    if (layer.entity != ignition::gazebo::kNullEntity)
    {
      this->SetLayerPose(
        layer,
        layer.alwaysVisible ? layer.visiblePose :
        ignition::math::Pose3d(0, 0, -10, 0, 0, 0),
        ecm);
    }
    layer.visible = layer.alwaysVisible;
  }

  double ambientTemperature{293.15};
  double effectiveDiffusivity{0.0018};
  double responseTime{3.5};
  double updatePeriod{0.5};
  double minimumVisibleRise{0.35};
  double lastTime{0.0};
  double lastUpdateTime{-1.0};
  ignition::transport::Node transportNode;
  std::string temperatureTopic;
  mutable std::mutex temperatureMutex;
  double dynamicSourceTemperature{293.15};
  bool hasDynamicSourceTemperature{false};
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
