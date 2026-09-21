# Contributing to Space Station OS

Thank you for your interest in contributing to Space Station OS. This guide covers everything you need to go from clone to merged PR.

## Table of contents

- [Getting started](#getting-started)
- [Development setup](#development-setup)
- [Branching and commits](#branching-and-commits)
- [Coding standards](#coding-standards)
- [Testing](#testing)
- [Pull request process](#pull-request-process)
- [CI/CD pipeline](#cicd-pipeline)
- [Architecture overview](#architecture-overview)
- [Package naming conventions](#package-naming-conventions)
- [Issue conventions](#issue-conventions)
- [Where to get help](#where-to-get-help)

---

## Getting started

1. **Fork** the repository on GitHub.
2. **Clone** your fork locally.
3. **Build and run** the demos to make sure your environment works.
4. **Pick an issue** from the [project board](https://github.com/orgs/space-station-os/projects/2/views/1) or open a new one.
5. **Create a branch**, make your changes, open a PR.

If this is your first contribution, issues labeled `good first issue` are a great starting point.

---

## Development setup

SSOS supports two development environments:

- **Pixi** — recommended for reproducible development without a system ROS 2 installation
- **Native ROS 2 Jazzy** — supported for system integration and development against a host ROS installation

Docker is maintained as an advanced environment and is not the primary
development path for v0.9.

### Recommended: Pixi

Pixi provides the SSOS ROS 2 Jazzy environment through RoboStack and uses the
committed `pixi.lock` to reproduce the development environment.

Install Pixi if it is not already available:

```bash
curl -fsSL https://pixi.sh/install.sh | sh
export PATH="$HOME/.pixi/bin:$PATH"
```

Clone your fork:

```bash
git clone https://github.com/<your-fork>/space_station_os.git
cd space_station_os
```

Install the committed environment, build, and test:

```bash
pixi install --locked
pixi run build
pixi run test
```

Launch the full SSOS stack:

```bash
pixi run station
```

Normal development setup must use `pixi install --locked`. If you intentionally
change dependencies in `pixi.toml`, follow the lock-file update procedure in
[PIXI.md](PIXI.md).

### Native ROS 2 Jazzy

Use this path when developing or integrating SSOS against a system ROS 2
installation.

Prerequisites:

- Ubuntu 24.04
- ROS 2 Jazzy Desktop
- ROS development tools

Install ROS 2 Jazzy using the official
[ROS 2 Jazzy installation guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
before continuing.

Create a workspace and clone your fork:

```bash
mkdir -p ~/ssos_ws/src
cd ~/ssos_ws/src

git clone https://github.com/<your-fork>/space_station_os.git
cd ~/ssos_ws
```

Install dependencies:

```bash
source /opt/ros/jazzy/setup.bash

sudo rosdep init   # Skip if rosdep has already been initialized.
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

Build and test:

```bash
source /opt/ros/jazzy/setup.bash

colcon build --symlink-install
source install/setup.bash

colcon test
colcon test-result --verbose
```

Launch SSOS:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ssos_ws/install/setup.bash

ros2 launch space_station space_station.launch.py
```

Source both ROS 2 Jazzy and the workspace overlay in each new terminal before
running ROS 2 commands.

### Docker (advanced)

Docker is maintained as an advanced, separately handled environment rather than
the recommended development setup for v0.9.

See the root [README](../README.md) for the current Docker workflow.

### Verify your setup

For Pixi:

```bash
pixi run build
pixi run test
pixi run station
```

For native ROS 2 Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ssos_ws/install/setup.bash

colcon test
colcon test-result --verbose
ros2 launch space_station space_station.launch.py
```

---

## Branching and commits

### Branch naming

Use the following prefixes:

| Prefix | Use case | Example |
|--------|----------|---------|
| `feature/` | New functionality | `feature/a1-system-interface-types` |
| `bugfix/` | Bug fixes | `bugfix/eclss-o2-threshold` |
| `hotfix/` | Urgent fixes for main | `hotfix/190-eclss` |
| `ci/` | CI/CD changes | `ci/restructure-workflows` |
| `docs/` | Documentation only | `docs/update-gnc-readme` |
| `release/` | Release preparation | `release/v0.9.0` |

### Commit messages

Use clear, descriptive commit messages. Prefix with the scope when possible:

```
feat(interfaces): add SystemState.msg and FaultEvent.msg
fix(eclss): correct O2 threshold bounds check
test(gnc): add orbit estimation unit test
ci: separate build/test from Docker publish
docs: update README for Jazzy
refactor(thermal): migrate to behaviortree_cpp v4
```

The format is `type(scope): description` where type is one of: `feat`, `fix`, `test`, `ci`, `docs`, `refactor`, `chore`.

---

## Coding standards

### C++

- Follow the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html).
- Use `ament_cppcheck` and `ament_cpplint` for automated checks.
- Separate header (`.hpp`) and implementation (`.cpp`) files.
- Place headers in `include/<package_name>/`.
- Use ROS 2 parameter declarations for tunable values — no hardcoded constants for thresholds, rates, or physical parameters.

### Python

- Follow [PEP 8](https://peps.python.org/pep-0008/).
- Use `ament_flake8` for automated checks.
- Type hints are encouraged for public function signatures.

### General

- Every function should have a brief description of its purpose, inputs, and outputs.
- Inline comments for complex logic only — clear code is better than commented code.
- Avoid hardcoded values. Use ROS 2 parameters loaded from YAML config files under the package's `config/` directory.
- Keep modules small and focused for reuse.

### ROS 2 specifics

- New nodes should inherit from `rclcpp_lifecycle::LifecycleNode` (C++) or use managed node patterns (Python) where appropriate.
- All interface definitions (`.msg`, `.srv`, `.action`) go in `space_station_interfaces/`.
- Use `use_sim_time` parameter for any node that interacts with simulation time.

---

## Testing

All contributions should include tests where applicable.

### Unit tests

- **C++:** Use Google Test (`ament_cmake_gtest`).
- **Python:** Use `pytest` (`ament_cmake_pytest`).

### Integration tests

- Use `launch_testing` for tests that require multiple nodes running together.
- Focus on verifying topic connectivity, message flow, and service responses.

### Running tests locally

```bash
cd ~/ssos_ws
colcon build
colcon test
colcon test-result --verbose   # see what failed
```

To test a specific package:

```bash
colcon test --packages-select space_station_eclss
colcon test-result --verbose
```

### Test expectations

- New message/service types: verify they build and can be shown with `ros2 interface show`.
- New nodes: at minimum, a test that the node starts, publishes expected topics, and shuts down cleanly.
- Bug fixes: include a regression test that would have caught the bug.

---

## Pull request process

1. **One PR per concern.** Don't mix a bug fix with a feature addition.
2. **Reference the issue** in your PR description: `Closes #123` or `Related to #123`.
3. **Describe what changed and why.** Include a "What this does NOT change" section for architectural PRs.
4. **Ensure CI passes.** The CI workflow must be green before review.
5. **Keep PRs reviewable.** Under 400 lines of diff is ideal. If larger, explain why it can't be split.
6. **Respond to review feedback** promptly. Resolve conversations when addressed.

### PR title format

```
[scope] Brief description
```

Examples:
```
[A1.1] Add system-level message and service types to space_station_interfaces
[CI] Restructure GitHub Actions: separate CI from Docker publishing
[hotfix] Fix ECLSS O2 recovery threshold
```

---

## CI/CD pipeline

The repository uses three complementary GitHub Actions workflows:

| Workflow | File | Purpose |
|----------|------|---------|
| **Native ROS 2 CI** | `ci.yml` | Build and validation against ROS 2 Jazzy on Ubuntu 24.04 |
| **Pixi CI** | `pixi.yml` | Reproduce the committed locked Pixi environment, build, test, and verify that `pixi.lock` remains unchanged |
| **Docker Publish** | `docker-publish.yml` | Build and publish the container image from `main` and version tags |

Both Native ROS 2 CI and Pixi CI run for pull requests and are used as
repository integration checks.

### Native ROS 2 CI

The native workflow builds the repository in a ROS 2 Jazzy environment on
Ubuntu 24.04.

It currently:

1. installs the native ROS 2 and system dependencies
2. builds the repository with `colcon build`
3. runs `colcon test`
4. reports the test results

The native test execution is currently non-blocking within the workflow, while
build failures still fail the job.

### Pixi CI

The Pixi workflow validates the reproducible development environment from the
committed `pixi.lock`.

It runs:

```bash
pixi run build
pixi run test
```

and then verifies that the CI run did not modify `pixi.lock`.

A mismatch between `pixi.toml` and `pixi.lock`, a Pixi build failure, or a Pixi
test failure causes this workflow to fail.

### Docker publishing

Docker publishing is separate from pull-request validation. The publish
workflow runs from `main` and version tags and publishes the generated image to
GHCR.

Before requesting review, make sure the required CI checks for your pull
request are green.

---

## Architecture overview

SSOS is a ROS 2-based platform for space station subsystem simulation and control. The current architecture (v0.8.x) is being consolidated into v0.9 through three structural epics:

| Epic | Name | Purpose |
|------|------|---------|
| **A** | Structural separation layer | Explicit boundary between system logic and simulation logic |
| **B** | Architectural definition | Subsystem contracts, global state model, failure propagation |
| **C** | Structured simulation framework | World model, initial conditions, fault taxonomy |

The key packages are:

| Package | Subsystem | Language |
|---------|-----------|----------|
| `space_station_eclss` | Environmental Control & Life Support | C++/Python |
| `space_station_gnc` | Guidance, Navigation & Control | C++ |
| `space_station_eps` | Electrical Power System | C++ |
| `space_station_thermal_control` | Thermal Control | C++ |
| `space_station_communication` | CCSDS Communications | C++/Python |
| `space_station_interfaces` | ROS 2 message/service definitions | — |
| `space_station` | Launch files, orchestrator | Python |
| `space_station_mission_control` | OpenMCT telemetry bridge | JavaScript |

New packages being introduced in v0.9:

| Package | Purpose |
|---------|---------|
| `ssos_core` | System manager, global state model, fault bus |
| `ssos_sim` | Simulation controller, world model |
| `ssos_scenarios` | YAML scenario configs, fault definitions |

For detailed architecture, see the [SSOS Documentation](https://space-station-os.github.io/) and the issues tagged with `Epic A`, `Epic B`, `Epic C`.

---

## Package naming conventions

- **Existing packages** use the `space_station_` prefix. These retain their names until refactored.
- **New packages** introduced in the v0.9 consolidation use the `ssos_` prefix.
- When in doubt, follow the existing pattern for the area you're working in.

---

## Issue conventions

### Issue title format

| Type | Format | Example |
|------|--------|---------|
| Epic | `[Epic] X. Name` | `[Epic] A. Structural Separation Layer` |
| Sub-task | `X#. Description` | `A1.1 Add system-level interface types` |
| Bug | `[subsystem] Bug description` | `[GNC] Orbit estimator diverges on eclipse transition` |
| Hotfix | `[Hotfix] Description` | `[Hotfix] Build issue for space_station package` |
| Documentation | `[Documentation] Description` | `[Documentation] Update GNC docs` |
| Cleanup | `[version] Description` | `[v0.8.7] Code quality for space_station_gnc` |

### Labels

| Label | Meaning |
|-------|---------|
| `bug` | Something isn't working |
| `enhancement` | New feature or request |
| `cleanup` | Code quality improvement or refactor |
| `Epic A` / `Epic B` / `Epic C` | Structural consolidation work |
| `good first issue` | Suitable for new contributors |
| `interfaces` | Changes to `space_station_interfaces` |
| `ci/cd` | CI/CD pipeline changes |

---

## Where to get help

- **Discussions:** [GitHub Discussions](https://github.com/space-station-os/space_station_os/discussions) for questions, ideas, and general conversation.
- **Issues:** [GitHub Issues](https://github.com/space-station-os/space_station_os/issues) for bug reports and feature requests.
- **Project board:** [SSOS Project Board](https://github.com/orgs/space-station-os/projects/2/views/1) to see what's in progress.
- **Documentation:** [space-station-os.github.io](https://space-station-os.github.io/) for setup guides, coding rules, and architecture docs.
- **LinkedIn:** [Space Station OS](https://www.linkedin.com/company/space-station-os/) for announcements.
- **YouTube:** [Space Station OS Channel](https://www.youtube.com/@SpaceStationOS) for demo walkthroughs.

---

## License

By contributing to Space Station OS, you agree that your contributions will be licensed under the [Apache 2.0 License](LICENSE).