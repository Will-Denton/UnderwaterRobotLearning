# UnderwaterRobotLearning

Unity ML-Agents project for training a thruster-level PPO controller on **Oogway**, an **8-thruster autonomous underwater vehicle (AUV)** (Duke Robotics Club). The trained policy learns a simple but useful primitive: **move forward to a goal** placed at a random distance while discouraging excessive actuation.

## Project Summary
- **Algorithm:** Proximal Policy Optimization (PPO) via Unity ML-Agents  
- **Actions:** 8 continuous thruster commands in `[-1, 1]`  
- **Observations (10D):**
  - Target position in local frame (3)
  - Linear velocity in local frame (3)
  - Angular velocity in local frame (3)
  - Uprightness (`upDot`) (1)
- **Task:** Goal is spawned along +X at a random distance (8–15 m). Episode succeeds when within 1 m.

## Repository Layout
- `Assets/` — Unity scene(s), scripts (agent, thrusters, physics), prefabs
- `Packages/` — Unity package manifest
- `ProjectSettings/` — Unity project settings
- `UserSettings/` — local editor settings (may be ignored in many repos)
- `oogway_ppo.yaml` — ML-Agents PPO training configuration

## Underwater Physics (Unity)
A lightweight underwater model is implemented in `BuoyancyAndWaterPhysics`:
- Buoyancy force applied at a center of buoyancy
- Quadratic drag per local axis using current-relative velocity
- Rotational damping torque per local axis
- Optional ambient current velocity

This model is designed for fast iteration and RL training throughput rather than high-fidelity hydrodynamics.

## Training
1. Install Unity ML-Agents (Python + Unity package) and ensure the ML-Agents CLI is available.
2. Open the project in Unity and select the training scene.
3. From a terminal, run (example):
   ```bash
   mlagents-learn oogway_ppo.yaml --run-id FinalRun --env <path-to-built-env>
