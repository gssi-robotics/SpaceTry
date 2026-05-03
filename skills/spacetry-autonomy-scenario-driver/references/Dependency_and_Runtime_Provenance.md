# Dependency And Runtime Provenance

Use this note when you need to understand which code is actually running inside the SpaceTry container and why Docker image freshness is separate from `/ws/src` sync freshness.

## Provenance Chain

The imported dependency path is:

1. `deps/spacetry.repos`
2. `docker/Dockerfile` runs `vcs import /ws/src < /ws/deps/spacetry.repos`
3. the imported repository is copied to `/opt/spacetry_deps/space_ros_demos`
4. `/etc/profile.d/spacetry_deps_link.sh` exposes that cached copy back into `/ws/src/space_ros_demos` at runtime
5. `COLCON_IGNORE` files prune packages that should not participate in the build

This means scenario-driver agents must reason about two distinct provenance paths:

- **Image-time provenance** for imported dependencies such as `space_ros_demos`
- **Runtime `/ws/src` provenance** for repo-local packages that are copied into the running container with `docker cp`

For scenario execution, treat those as two different freshness contracts:

- **Baseline image freshness** covers `docker/`, `deps/`, and non-scenario packages under `src/`. Changes there require rebuilding `spacetry:dev`.
- **Runtime package sync freshness** covers repo-local runtime packages such as `src/spacetry_scenario_*`. Changes there require copying each updated package into `/ws/src` and rebuilding it inside the running container.

## What The Dockerfile Does

`docker/Dockerfile` establishes the dependency flow in several steps:

- copies the repository into `/ws`
- imports the pinned dependency repositories listed in `deps/spacetry.repos` into `/ws/src`
- copies `/ws/src/space_ros_demos` into `/opt/spacetry_deps/space_ros_demos`
- removes the workspace copy so a later bind-mounted `/ws` does not hide the cached dependency copy
- installs `/etc/profile.d/spacetry_deps_link.sh`, which recreates `/ws/src/space_ros_demos` as a symlink to `/opt/spacetry_deps/space_ros_demos` in login shells

The important consequence is that a package may appear under `/ws/src` at runtime even though it did not come from the host repository at container startup.

## `COLCON_IGNORE` Filtering

The Dockerfile also writes `COLCON_IGNORE` files into the cached dependency tree.

That filtering is deliberate and affects which imported packages are build-visible:

- duplicate or unwanted packages are ignored
- only the intended subset of imported demos remains active in `colcon build`
- runtime inspection of `/ws/src/space_ros_demos` alone is not enough; you must also understand which subdirectories are ignored

## Why Staleness Happens

Several stale-state failure modes are possible:

- the local image is old, so imported dependency code and image-copied repository files are old
- the local image tag was rebuilt, but the running container was not recreated and is still using the older image id
- the image is current, but the running container does not yet have the newest repo-local runtime package copied into `/ws/src`
- the container source tree under `/ws/src/<package>` is current, but the installed package under `/ws/install` was not rebuilt after that source refresh
- the host repository changed after the image build, but the experiment reused the existing image anyway
- the host package was updated, but the container copy under `/ws/src/<package>` was not refreshed before build or launch

These are different problems and need different checks.

## Operational Guidance

Use `AGENTS.md` for the authoritative Docker execution order. This note explains why those steps matter; it does not replace the repo-wide operational sequence.

- Use `scripts/scenario_preflight.sh` to record the current skill-tree checksum by default and to check Docker daemon access, baseline image freshness, container-versus-image identity, optional skill checksum or commit pinning, Docker auth health, and host-versus-container runtime package sync before a `full_run` that should count as the main trusted result.
- Treat imported dependencies from `deps/spacetry.repos` as image-owned inputs. If they must change, rebuild the image.
- Treat repo-local runtime packages under `src/` as runtime-copied inputs. If they change, refresh `/ws/src/<package>` inside the running container before build or launch.
- After refreshing `/ws/src/<package>`, rebuild every affected runtime package so `/ws/install` is not older than the copied source tree.
- Do not treat a running container as sufficient proof of freshness. The container must also match the current local `spacetry:dev` image id.
- When a run behaves unexpectedly, inspect both the image provenance and the `/ws/src` sync state before assuming the scenario logic itself is wrong.
