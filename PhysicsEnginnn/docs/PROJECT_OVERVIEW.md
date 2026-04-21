# PhysicsEnginnn — Project Overview and Development Pipeline

This document explains what has been built so far, how the code is organized, and how data flows through the simulation each frame.

---

## 1) What the project currently is

`PhysicsEnginnn` is a **2D rigid body simulation prototype** with:

- Core math primitives (`Vec2`, transforms)
- Rigid body state + integration
- Shape definitions (`CircleShape`, `PolygonShape`)
- Gravity force system (`ForceGenerator` + `ForceRegistry`)
- World stepping with fixed timestep accumulation
- Optional SDL2 debug renderer for visualizing bodies
- A demo app (`apps/main.cpp`) that spawns test scenes

At this stage, it is mainly a **physics integration and rendering framework**. Collision detection and resolution are not yet implemented in the active simulation loop.

---

## 2) Repository structure and role of each part

## Root CMake

- `CMakeLists.txt`
  - Sets minimum CMake `3.10`
  - Declares project `PhysicsEnginnn`
  - Adds subdirectory `PhysicsEnginnn/`

## Engine CMake

- `PhysicsEnginnn/CMakeLists.txt`
  - Uses C++20
  - Builds static library: `physics_engine`
  - Builds executable app: `PhysicsEnginnn`
  - Optional SDL2 path controlled by `PHYSICSENGINNN_ENABLE_SDL2`
  - Defines compile flag `PHYSICSENGINNN_HAS_SDL2=1` when SDL2 is found

## Public headers (`include/PhysicsEnginnn`)

- `math/Vec2.h`
  - 2D vector type and arithmetic
  - dot/cross utilities
  - normalization and length APIs

- `math/Transform.h`
  - Rotation helper and transform application

- `dynamics/RigidBody.h`
  - Runtime body state (position, velocities, mass/inertia)
  - Force and impulse application APIs
  - Integration methods

- `collision/Shapes.h`
  - `CircleShape`, `PolygonShape`
  - Box factory, normals, world-space conversion, inertia approximation

- `forces/ForceGenerator.h`
  - Abstract force generator interface
  - `GravityForceGenerator`

- `forces/ForceRegistry.h`
  - Connects bodies to generators
  - Applies all forces per simulation step

- `core/World.h`
  - Owns all bodies and their shapes
  - Fixed-step update entrypoint (`Step`)
  - Gravity management

- `rendering/DebugRenderer.h`
  - SDL2 debug drawing API for circles/polygons

> Note: `collision/collision.h` and `dynamics/collisionresolver.h` currently exist but are empty placeholders.

## Source files (`src`)

- `math/Vec2.cpp` / `math/Transform.cpp`
  - Core math implementations

- `dynamics/RigidBody.cpp`
  - Mass/inertia setup
  - Force/impulse integration
  - Linear/angular velocity clamps for stability

- `collision/Shapes.cpp`
  - Shape validation
  - CCW correction
  - edge normal generation
  - inertia computation helpers

- `forces/ForceRegistry.cpp`
  - Registration and per-step force updates

- `core/World.cpp`
  - Fixed timestep accumulator
  - body creation + shape association via `std::variant`
  - gravity registry rebuild logic

- `rendering/DebugRenderer.cpp`
  - SDL init/shutdown and world drawing

## App entrypoint

- `apps/main.cpp`
  - Creates world
  - Adds static boundaries
  - Spawns demo dynamic bodies
  - Runs render/update loop (when SDL2 is enabled)

---

## 3) Runtime pipeline (what happens every frame)

This is the most important flow to understand.

1. **App loop** (`main.cpp`)
   - Measure `frameTime` using `std::chrono`
   - Clamp to `maxFrameTime` to avoid huge time jumps
   - Call `world.Step(clampedFrameTime)`

2. **Variable-to-fixed conversion** (`World::Step`)
   - Add incoming `dt` to accumulator
   - While accumulator >= fixed step (default `1/60`):
     - Call `StepFixed(fixedDt)`
     - subtract fixed step from accumulator

3. **Force phase** (`World::StepFixed`)
   - `m_forceRegistry.UpdateForces(dt)`
   - Gravity generator applies `gravity * mass` to each non-static body

4. **Integration phase** (`World::StepFixed` + `RigidBody`)
   - `IntegrateForces(dt)` updates linear/angular velocity from accumulators
   - `IntegrateVelocity(dt)`:
     - clamps velocities for stability
     - updates position and rotation
     - clears force/torque accumulators

5. **Render phase** (`DebugRenderer`)
   - Iterate world bodies
   - If shape is circle -> draw polyline approximation
   - If shape is polygon -> draw edges

### Current pipeline summary

`input time -> fixed-step accumulator -> force application -> integration -> draw`

No contact solving pass exists yet in this loop.

---

## 4) Body + shape data model

Each world entity is represented by `World::BodyEntry`:

- `dynamics::RigidBody body`
- `std::variant<collision::CircleShape, collision::PolygonShape> shape`

Why this is useful:

- Dynamics state stays generic (mass, velocity, etc.)
- Geometry can vary by shape type
- Rendering/logic can branch using `std::get_if` / `std::get`

---

## 5) Static vs dynamic objects

A body is treated as static when inverse mass is zero.

- In current code, setting `mass = 0` makes body static
- Static bodies:
  - do not accumulate forces
  - do not integrate movement

Used in `main.cpp` for floor/ceiling/walls.

---

## 6) SDL2 integration behavior

SDL2 rendering is optional and controlled by compile-time flags:

- Header include is guarded with `#if PHYSICSENGINNN_HAS_SDL2`
- Renderer usage in `main.cpp` is also guarded

If SDL2 is unavailable, app now prints a message and exits cleanly instead of failing with unresolved renderer types.

---

## 7) What the demo scene is testing right now

The app scene in `main.cpp` includes:

- Arena boundaries (4 static boxes)
- Moving boxes and circles with mixed restitution/friction
- Vertical box stack for stability checks
- Mixed mass-ratio interactions
- Sliding/spin interactions

This gives a broad visual check for integration behavior and force effects.

---

## 8) Current capabilities vs missing pieces

## Implemented

- Rigid body state and motion integration
- Gravity system and registration model
- Shape representation and inertia estimation
- Fixed timestep world stepping
- Optional debug visualization

## Not yet implemented (important)

- Broadphase collision detection
- Narrowphase contact generation
- Collision impulse solver (normal + friction impulses)
- Position correction / penetration stabilization
- Sleeping / island solving
- Constraints/joints

The project contains placeholders (`collision.h`, `collisionresolver.h`) that likely indicate intended next work.

---

## 9) How to build and run (current expected flow)

1. Configure with CMake (Ninja generator is supported in this workspace)
2. Build target `PhysicsEnginnn`
3. Run executable

If SDL2 is found and enabled:

- window opens
- simulation loop renders continuously

If SDL2 is not found:

- app prints that SDL2 support is disabled

---

## 10) Development pipeline recommendation (next milestones)

A clear sequence to continue the engine:

1. **Collision detection foundation**
   - Circle-circle overlap
   - Circle-polygon support
   - Polygon-polygon SAT

2. **Contact manifold structure**
   - contact points
   - normal
   - penetration depth

3. **Impulse solver pass in `World::StepFixed`**
   - integrate forces
   - detect contacts
   - solve velocity constraints (iterations)
   - integrate velocity
   - positional correction

4. **Stabilization improvements**
   - restitution thresholding
   - friction model tuning
   - warm starting

5. **Testing + debug tooling**
   - deterministic test scenes
   - simple regression checks for key scenarios

---

## 11) Quick mental model for the project

- `math`: low-level vector and transform ops
- `dynamics`: body physics state update rules
- `forces`: how accelerations are applied
- `collision`: shape geometry (currently mostly geometric utilities)
- `core`: orchestrates simulation ticking
- `rendering`: visualization only
- `apps/main.cpp`: sandbox scenario and app loop

If you keep this layered view in mind, understanding and extending the project becomes much easier.

---

## 12) Practical extension points

When adding features, these are the natural insertion points:

- Add new force type: implement `ForceGenerator`, register in `World`
- Add new shape: extend shape variant + renderer + inertia helper
- Add collisions: implement collision module and call it from `World::StepFixed`
- Add gameplay controls: modify `apps/main.cpp` event handling

---

## 13) Status snapshot

Project status right now can be described as:

> **A functioning 2D rigid-body integration sandbox with optional SDL2 debug rendering and a clean foundation for adding collision detection/response next.**
