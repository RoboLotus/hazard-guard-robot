/*
 * Copyright 2026 RoboLotus
 * Licensed under the Apache License, Version 2.0.
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <random>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector2.hh>
#include <ignition/plugin/Register.hh>

namespace hazard_guard_simulation
{
class RandomWalkSystem final :
  public ignition::gazebo::System,
  public ignition::gazebo::ISystemConfigure,
  public ignition::gazebo::ISystemPreUpdate
{
public:
  void Configure(
    const ignition::gazebo::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    ignition::gazebo::EntityComponentManager &,
    ignition::gazebo::EventManager &) override
  {
    this->entity = entity;
    this->speed = sdf->Get<double>("speed", this->speed).first;
    this->arrivalTolerance =
      sdf->Get<double>("arrival_tolerance", this->arrivalTolerance).first;
    this->minimumSpeed =
      sdf->Get<double>("minimum_speed", this->speed).first;
    this->maximumSpeed =
      sdf->Get<double>("maximum_speed", this->speed).first;
    if (this->minimumSpeed > this->maximumSpeed)
      std::swap(this->minimumSpeed, this->maximumSpeed);
    this->minimumPause = std::max(
      0.0, sdf->Get<double>("minimum_pause", this->minimumPause).first);
    this->maximumPause = std::max(
      this->minimumPause,
      sdf->Get<double>("maximum_pause", this->maximumPause).first);
    this->backtrackProbability = std::clamp(
      sdf->Get<double>(
        "backtrack_probability", this->backtrackProbability).first,
      0.0, 1.0);
    this->trailPeriod =
      sdf->Get<double>("trail_period", this->trailPeriod).first;
    this->trailZ = sdf->Get<double>("trail_z", this->trailZ).first;
    this->startNode = sdf->Get<int>("start_node", 0).first;
    const auto seed = sdf->Get<unsigned int>("seed", 0u).first;
    this->random.seed(seed == 0u ? std::random_device{}() : seed);
    auto sdfClone = sdf->Clone();

    if (sdfClone->HasElement("waypoint"))
    {
      for (auto elem = sdfClone->GetElement("waypoint"); elem;
        elem = elem->GetNextElement("waypoint"))
      {
        this->waypoints.push_back(elem->Get<ignition::math::Vector2d>());
      }
    }

    std::vector<std::pair<int, int>> edges;
    if (sdfClone->HasElement("edge"))
    {
      for (auto elem = sdfClone->GetElement("edge"); elem;
        elem = elem->GetNextElement("edge"))
      {
        std::istringstream input(elem->Get<std::string>());
        int from = -1;
        int to = -1;
        if (input >> from >> to)
          edges.emplace_back(from, to);
      }
    }

    this->neighbors.resize(this->waypoints.size());
    for (const auto & edge : edges)
    {
      if (edge.first < 0 || edge.second < 0 ||
        static_cast<std::size_t>(edge.first) >= this->waypoints.size() ||
        static_cast<std::size_t>(edge.second) >= this->waypoints.size())
      {
        continue;
      }
      this->neighbors[edge.first].push_back(edge.second);
      this->neighbors[edge.second].push_back(edge.first);
    }

    if (edges.empty() && this->waypoints.size() > 1)
    {
      for (std::size_t index = 0; index < this->waypoints.size(); ++index)
      {
        const auto next = static_cast<int>((index + 1) % this->waypoints.size());
        this->neighbors[index].push_back(next);
        this->neighbors[next].push_back(static_cast<int>(index));
      }
    }

    if (sdfClone->HasElement("trail_model"))
    {
      for (auto elem = sdfClone->GetElement("trail_model"); elem;
        elem = elem->GetNextElement("trail_model"))
      {
        this->trailNames.push_back(elem->Get<std::string>());
      }
    }
    this->trailEntities.assign(
      this->trailNames.size(), ignition::gazebo::kNullEntity);

    if (this->waypoints.empty())
      return;

    this->startNode = std::clamp(
      this->startNode, 0, static_cast<int>(this->waypoints.size()) - 1);
    this->currentNode = this->startNode;
    this->targetNode = this->ChooseNextNode();
    this->ChooseSegmentSpeed();
  }

  void PreUpdate(
    const ignition::gazebo::UpdateInfo & info,
    ignition::gazebo::EntityComponentManager & ecm) override
  {
    if (info.paused || this->waypoints.empty())
      return;

    const double now = std::chrono::duration<double>(info.simTime).count();
    if (this->lastUpdateTime < 0.0 || now < this->lastUpdateTime)
    {
      this->lastUpdateTime = now;
      this->nextTrailSample = now;
      return;
    }

    const double dt = std::min(0.1, now - this->lastUpdateTime);
    this->lastUpdateTime = now;
    if (dt <= 0.0 || now < this->pauseUntil)
      return;

    auto poseComponent =
      ecm.Component<ignition::gazebo::components::Pose>(this->entity);
    if (!poseComponent)
      return;

    auto pose = poseComponent->Data();
    const auto target = this->waypoints[this->targetNode];
    ignition::math::Vector2d position(pose.Pos().X(), pose.Pos().Y());
    auto delta = target - position;
    double distance = delta.Length();

    if (distance <= this->arrivalTolerance)
    {
      position = target;
      this->previousNode = this->currentNode;
      this->currentNode = this->targetNode;
      this->targetNode = this->ChooseNextNode();
      this->ChooseSegmentSpeed();
      if (this->maximumPause > 0.0)
      {
        std::uniform_real_distribution<double> pause(
          this->minimumPause, this->maximumPause);
        this->pauseUntil = now + pause(this->random);
      }
      delta = this->waypoints[this->targetNode] - position;
      distance = delta.Length();
    }

    if (distance > 1e-6)
    {
      const double step = std::min(this->segmentSpeed * dt, distance);
      delta.Normalize();
      position += delta * step;
      pose.Pos().X(position.X());
      pose.Pos().Y(position.Y());
      pose.Rot() = ignition::math::Quaterniond(
        0.0, 0.0, std::atan2(delta.Y(), delta.X()));
      *poseComponent = ignition::gazebo::components::Pose(pose);
      ecm.SetChanged(
        this->entity,
        ignition::gazebo::components::Pose::typeId,
        ignition::gazebo::ComponentState::OneTimeChange);
    }

    this->ResolveTrailEntities(ecm);
    if (now >= this->nextTrailSample)
    {
      this->trailHistory.insert(this->trailHistory.begin(), position);
      if (this->trailHistory.size() > this->trailNames.size())
        this->trailHistory.resize(this->trailNames.size());
      this->nextTrailSample = now + this->trailPeriod;
    }
    this->UpdateTrail(ecm);
  }

private:
  int ChooseNextNode()
  {
    if (this->currentNode < 0 ||
      static_cast<std::size_t>(this->currentNode) >= this->neighbors.size())
    {
      return this->startNode;
    }
    auto candidates = this->neighbors[this->currentNode];
    if (candidates.empty())
      return this->currentNode;
    if (candidates.size() > 1 && this->previousNode >= 0)
    {
      std::bernoulli_distribution backtrack(this->backtrackProbability);
      if (!backtrack(this->random))
      {
        candidates.erase(
          std::remove(candidates.begin(), candidates.end(), this->previousNode),
          candidates.end());
      }
    }
    std::uniform_int_distribution<std::size_t> choice(0, candidates.size() - 1);
    return candidates[choice(this->random)];
  }

  void ChooseSegmentSpeed()
  {
    std::uniform_real_distribution<double> speed(
      this->minimumSpeed, this->maximumSpeed);
    this->segmentSpeed = speed(this->random);
  }

  void ResolveTrailEntities(ignition::gazebo::EntityComponentManager & ecm)
  {
    for (std::size_t index = 0; index < this->trailNames.size(); ++index)
    {
      if (this->trailEntities[index] != ignition::gazebo::kNullEntity)
        continue;
      this->trailEntities[index] = ecm.EntityByComponents(
        ignition::gazebo::components::Name(this->trailNames[index]),
        ignition::gazebo::components::Model());
    }
  }

  void UpdateTrail(ignition::gazebo::EntityComponentManager & ecm)
  {
    for (std::size_t index = 0; index < this->trailEntities.size(); ++index)
    {
      const auto entity = this->trailEntities[index];
      if (entity == ignition::gazebo::kNullEntity)
        continue;
      auto poseComponent =
        ecm.Component<ignition::gazebo::components::Pose>(entity);
      if (!poseComponent)
        continue;

      ignition::math::Pose3d pose(0.0, 0.0, -10.0, 0.0, 0.0, 0.0);
      if (index < this->trailHistory.size())
      {
        pose.Pos().X(this->trailHistory[index].X());
        pose.Pos().Y(this->trailHistory[index].Y());
        pose.Pos().Z(this->trailZ);
      }
      *poseComponent = ignition::gazebo::components::Pose(pose);
      ecm.SetChanged(
        entity,
        ignition::gazebo::components::Pose::typeId,
        ignition::gazebo::ComponentState::OneTimeChange);
    }
  }

  ignition::gazebo::Entity entity{ignition::gazebo::kNullEntity};
  double speed{0.18};
  double minimumSpeed{0.18};
  double maximumSpeed{0.18};
  double segmentSpeed{0.18};
  double minimumPause{0.0};
  double maximumPause{0.0};
  double backtrackProbability{0.18};
  double arrivalTolerance{0.035};
  double trailPeriod{2.0};
  double trailZ{0.003};
  double lastUpdateTime{-1.0};
  double nextTrailSample{0.0};
  double pauseUntil{0.0};
  int startNode{0};
  int currentNode{0};
  int previousNode{-1};
  int targetNode{0};
  std::mt19937 random;
  std::vector<ignition::math::Vector2d> waypoints;
  std::vector<std::vector<int>> neighbors;
  std::vector<std::string> trailNames;
  std::vector<ignition::gazebo::Entity> trailEntities;
  std::vector<ignition::math::Vector2d> trailHistory;
};
}  // namespace hazard_guard_simulation

IGNITION_ADD_PLUGIN(
  hazard_guard_simulation::RandomWalkSystem,
  ignition::gazebo::System,
  hazard_guard_simulation::RandomWalkSystem::ISystemConfigure,
  hazard_guard_simulation::RandomWalkSystem::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(
  hazard_guard_simulation::RandomWalkSystem,
  "hazard_guard_simulation::RandomWalkSystem")
