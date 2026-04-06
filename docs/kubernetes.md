# Kubernetes Deployment Guide

Kubernetes is the default orchestration backend for the Swarm Digital Twin.
The system is packaged as a Helm chart that deploys a StatefulSet with one pod
per drone, a Zenoh router for cross-pod messaging, and supporting services.

## Architecture

```
                   +-------------------+
                   |   zenoh-router    |  Deployment (1 replica)
                   |   ClusterIP:7447  |
                   +--------+----------+
                            |
              +-------------+-------------+
              |                           |
   +----------+----------+    +-----------+---------+
   | drone pod 0         |    | drone pod 1         |
   |  sitl               |    |  sitl               |
   |  swarm-node ------->|    |  swarm-node ------->|
   |  perception         |    |  perception         |
   |  zenoh-bridge       |    |  zenoh-bridge       |
   +---------------------+    +---------------------+
   StatefulSet (N replicas, shared localhost per pod)
```

Each drone pod contains 4 containers sharing `localhost`:

| Container | Image | Role |
|---|---|---|
| `sitl` | `beret/ardupilot-sitl` | ArduPilot SITL simulator |
| `swarm-node` | `beret/swarm_companion` | Rust swarm control (Raft consensus, boids, formation) |
| `perception` | `beret/swarm_companion` | Python vision pipeline |
| `zenoh-bridge` | `eclipse/zenoh-bridge-ros2dds` | ROS 2 DDS to Zenoh bridge |

An init container derives the drone ID from the pod ordinal (pod-0 = drone 1)
and copies the correct Zenoh config and mission file into a shared volume.

## Prerequisites

- A Kubernetes cluster (k3s, minikube, kind, EKS, GKE, AKS)
- `kubectl` configured and pointed at your cluster
- `helm` 3.x installed
- Docker images available (see [Images](#images) below)

Verify connectivity:

```bash
kubectl get nodes
helm version
```

## Quick Start

```bash
# Deploy a 2-drone swarm
helm install swarm ./helm/swarm-digital-twin \
  --set drones=2 \
  -n swarm --create-namespace

# Watch pods come up (4/4 Ready = all containers running)
kubectl get pods -n swarm -w

# Check swarm node logs
kubectl logs swarm-swarm-digital-twin-0 -n swarm -c swarm-node --tail=20

# Run integration tests
pytest tests/ -v

# Tear down
helm uninstall swarm -n swarm
```

## Images

The chart uses two custom images published on
[Docker Hub](https://hub.docker.com/u/beret) (default):

| Image | Description |
|---|---|
| [`beret/ardupilot-sitl:latest`](https://hub.docker.com/r/beret/ardupilot-sitl) | ArduPilot Copter-4.5.7 SITL (amd64 only) |
| [`beret/swarm_companion:latest`](https://hub.docker.com/r/beret/swarm_companion) | ROS 2 Humble + Rust + perception stack (CUDA 12.2 base) |

Third-party images pulled automatically:

| Image | Description |
|---|---|
| `eclipse/zenoh-bridge-ros2dds:latest` | Zenoh ROS 2 DDS bridge |
| `eclipse/zenoh:latest` | Zenoh router |
| `busybox:1.36` | Init container |

### Setting up Docker Hub

Docker Hub is the default registry. A free account allows unlimited
public repositories.

1. **Create an account** at https://hub.docker.com/signup.

2. **Log in from the command line:**
   ```bash
   docker login
   ```
   Enter your Docker Hub username and password (or a Personal Access Token).
   Credentials are stored in `~/.docker/config.json`.

3. **Verify:**
   ```bash
   docker info | grep Username
   # Should show: Username: your-username
   ```

Repositories are created automatically on first push. No manual setup needed.

### Building and pushing images

After code changes, rebuild and push to Docker Hub:

```bash
# Build and push both images (default: beret/)
./scripts/push_images.sh

# Push with a version tag
./scripts/push_images.sh --tag v0.2.0

# Only rebuild one image
./scripts/push_images.sh --sitl
./scripts/push_images.sh --companion

# Preview commands without executing
./scripts/push_images.sh --dry-run
```

If you already have the images built locally (e.g. from `docker compose build`):

```bash
docker tag ardupilot-sitl:latest beret/ardupilot-sitl:latest
docker tag swarm_companion:latest beret/swarm_companion:latest
docker push beret/ardupilot-sitl:latest
docker push beret/swarm_companion:latest
```

### Using GitHub Container Registry (ghcr.io)

ghcr.io is an alternative to Docker Hub, tied to your GitHub account. Every
GitHub user can push images to `ghcr.io/<username>/` with a Personal Access
Token. This is useful if you want images linked to your GitHub repository.

**1. Create a GitHub Personal Access Token (classic):**

Go to https://github.com/settings/tokens/new and create a token with
these scopes:

| Scope | Purpose |
|---|---|
| `write:packages` | Push images |
| `read:packages` | Pull images (needed for private packages) |
| `delete:packages` | Optional, delete old tags |

Save the token — you will not be able to see it again.

**2. Log in to ghcr.io:**

```bash
echo "YOUR_GITHUB_PAT" | docker login ghcr.io -u YOUR_GITHUB_USERNAME --password-stdin
```

For example:

```bash
echo "ghp_xxxxxxxxxxxx" | docker login ghcr.io -u berecik --password-stdin
```

**3. Push images to ghcr.io:**

```bash
./scripts/push_images.sh --registry ghcr.io/berecik
```

Or manually:

```bash
docker tag ardupilot-sitl:latest ghcr.io/berecik/ardupilot-sitl:latest
docker tag swarm_companion:latest ghcr.io/berecik/swarm_companion:latest
docker push ghcr.io/berecik/ardupilot-sitl:latest
docker push ghcr.io/berecik/swarm_companion:latest
```

**4. Make packages public** (recommended for open-source projects):

After the first push, ghcr.io packages default to **private**. To make them
public so anyone can pull without authentication:

- Go to https://github.com/berecik/swarm_digital_twin/pkgs/container/ardupilot-sitl
- Click "Package settings" (bottom of the right sidebar)
- Under "Danger Zone", click "Change visibility" and select "Public"
- Repeat for `swarm_companion`

**5. Deploy with ghcr.io images:**

```bash
helm install swarm ./helm/swarm-digital-twin \
  --set images.registry=ghcr.io/berecik \
  -n swarm --create-namespace
```

Or set `SWARM_REGISTRY` when using `run_scenario.sh`:

```bash
SWARM_REGISTRY=ghcr.io/berecik ./run_scenario.sh --swarm 2
```

### Using other registries

```bash
# AWS ECR
aws ecr get-login-password | docker login --username AWS --password-stdin 123456789.dkr.ecr.us-east-1.amazonaws.com
./scripts/push_images.sh --registry 123456789.dkr.ecr.us-east-1.amazonaws.com

# Google Artifact Registry
gcloud auth configure-docker us-docker.pkg.dev
./scripts/push_images.sh --registry us-docker.pkg.dev/my-project/swarm
```

For private registries, create an image pull secret:

```bash
kubectl create secret docker-registry regcred \
  --docker-server=your-registry.example.com \
  --docker-username=your-user \
  --docker-password=your-token \
  -n swarm

helm install swarm ./helm/swarm-digital-twin \
  --set imagePullSecrets[0].name=regcred \
  -n swarm --create-namespace
```

### Loading images into k3s

k3s uses containerd, not Docker. If your k3s node has the images in its
local Docker daemon but k3s can't see them, run **on the k3s node**:

```bash
sudo ./scripts/k3s_import_images.sh
```

This saves each image from Docker and imports it into k3s containerd.
After import, set `pullPolicy: Never` in values or use `values-local.yaml`
to prevent k3s from trying to pull from a registry.

## Helm Values

### Default values (`values.yaml`)

```yaml
drones: 6                              # Number of drone pods
images:
  registry: beret                      # Docker Hub namespace (override with --set images.registry=...)
  sitl:
    name: ardupilot-sitl               # → beret/ardupilot-sitl:latest
    tag: latest
  companion:
    name: swarm_companion              # → beret/swarm_companion:latest
    tag: latest
sitl:
  lat: "-0.508333"                     # Quito, Ecuador
  lng: "-78.141667"
  alt: "4500"
  vehicle: copter
```

To switch all custom images to a different registry:

```bash
# ghcr.io
helm install swarm ./helm/swarm-digital-twin --set images.registry=ghcr.io/berecik -n swarm --create-namespace

# Your own Docker Hub namespace
helm install swarm ./helm/swarm-digital-twin --set images.registry=your-username -n swarm --create-namespace

# Local images (no registry)
helm install swarm ./helm/swarm-digital-twin --set images.registry="" -n swarm --create-namespace
```

### Values profiles

| File | Use case |
|---|---|
| `values.yaml` | Default: 6 drones, Docker Hub images, NodePort services |
| `values-local.yaml` | Minikube/kind: 2 drones, local images (`pullPolicy: Never`) |
| `values-cloud.yaml` | EKS/GKE/AKS: `pullPolicy: Always`, ClusterIP services |

```bash
# Local development with minikube
helm install swarm ./helm/swarm-digital-twin \
  -f ./helm/swarm-digital-twin/values-local.yaml \
  -n swarm --create-namespace

# Cloud deployment
helm install swarm ./helm/swarm-digital-twin \
  -f ./helm/swarm-digital-twin/values-cloud.yaml \
  -n swarm --create-namespace

# Override drone count
helm install swarm ./helm/swarm-digital-twin \
  --set drones=3 \
  -n swarm --create-namespace
```

### Deploying a custom mission

```bash
# Generate a formation mission config
python simulation/sitl_waypoints.py formation \
  --n 4 --radius 8 --altitude 20 --output /tmp/mission.json

# Deploy with the mission baked into the ConfigMap
helm upgrade swarm ./helm/swarm-digital-twin \
  --set drones=4 \
  --set-file missionConfig=/tmp/mission.json \
  -n swarm
```

## Operations

### Status

```bash
# Pod status
kubectl get pods -n swarm

# Per-container status
kubectl get pod swarm-swarm-digital-twin-0 -n swarm \
  -o jsonpath='{range .status.containerStatuses[*]}{.name}{"\t"}{.ready}{"\n"}{end}'

# Or use run_scenario.sh
./run_scenario.sh --status
```

### Logs

```bash
# Swarm node logs
kubectl logs swarm-swarm-digital-twin-0 -n swarm -c swarm-node

# SITL logs
kubectl logs swarm-swarm-digital-twin-0 -n swarm -c sitl

# Perception logs
kubectl logs swarm-swarm-digital-twin-0 -n swarm -c perception

# Zenoh bridge logs
kubectl logs swarm-swarm-digital-twin-0 -n swarm -c zenoh-bridge

# Follow logs
kubectl logs swarm-swarm-digital-twin-0 -n swarm -c swarm-node -f
```

### Upgrade

```bash
# Change drone count
helm upgrade swarm ./helm/swarm-digital-twin --set drones=4 -n swarm

# After upgrade, restart pods to pick up new ConfigMaps
kubectl delete pods -n swarm -l app.kubernetes.io/name=swarm-digital-twin
```

### Teardown

```bash
helm uninstall swarm -n swarm

# Or via run_scenario.sh
./run_scenario.sh --down
```

### Persistent cargo build cache

The chart creates a PersistentVolumeClaim (`cargo-cache`) per drone pod to
cache Rust build artifacts. On first deploy, `cargo build` takes several
minutes. Subsequent restarts reuse the cache and start in ~10 seconds.

To clear the cache:

```bash
kubectl delete pvc -n swarm -l app.kubernetes.io/name=swarm-digital-twin
```

## Testing on Kubernetes

All integration tests default to the K8s backend:

```bash
# Run all integration tests (uses current kubectl context)
pytest tests/ -v

# Run a single test file
pytest tests/test_integration_sitl.py -v

# Override to Docker backend
pytest tests/ -v --backend=docker

# Run active-flight tests (requires mission orchestrator)
pytest tests/ -v --run-flight
```

### Test summary (49 tests)

| File | Tests | Description |
|---|---|---|
| `test_integration_sitl.py` | 7 | SITL process, port, coordinates |
| `test_integration_swarm_node.py` | 8 | Rust binary, Zenoh, control loop |
| `test_integration_perception.py` | 7 | Detector process, ROS 2, CV |
| `test_integration_zenoh.py` | 11 | Bridge process, config, DDS domain |
| `test_integration_swarm_formation.py` | 16 | Formation config, startup, flight* |

*3 flight state tests require `--run-flight` and an active mission orchestrator.

## run_scenario.sh

The `run_scenario.sh` script defaults to the K8s backend. All existing
commands work transparently:

```bash
./run_scenario.sh --swarm 2          # Deploy 2-drone swarm via Helm
./run_scenario.sh --status           # Show pod health table
./run_scenario.sh --down             # helm uninstall
./run_scenario.sh --test             # Rust tests + integration tests

# Force Docker Compose backend
./run_scenario.sh --swarm 2 --backend=docker
```

## Services

The chart creates these Kubernetes services:

| Service | Type | Purpose |
|---|---|---|
| `swarm-swarm-digital-twin-headless` | ClusterIP (None) | StatefulSet DNS for pod discovery |
| `swarm-swarm-digital-twin-zenoh-router` | ClusterIP | Zenoh message router (port 7447) |
| `swarm-swarm-digital-twin-mavlink-N` | NodePort | Per-drone MAVLink TCP access (ports 30760, 30770, ...) |

### Connecting QGroundControl

With NodePort services, connect QGC to `<node-ip>:30760` for drone 1,
`<node-ip>:30770` for drone 2, etc.

## Troubleshooting

### Pods stuck in ImagePullBackOff

Images not available in the registry or on the node:

```bash
kubectl describe pod swarm-swarm-digital-twin-0 -n swarm | grep -A3 "Failed"
```

Fix: push images to Docker Hub or import into k3s (see [Images](#images)).

### Swarm node in CrashLoopBackOff

Usually caused by Rust compilation running out of memory. The default memory
limit is 4Gi. Check the OOM kill:

```bash
kubectl describe pod swarm-swarm-digital-twin-0 -n swarm | grep OOM
```

Fix: increase the memory limit in `values.yaml` or clear the cargo cache PVC
and restart.

### Startup probe failures

First-time compilation takes several minutes. The startup probe allows up to
30 minutes (180 checks x 10s). If your node is slower, increase
`failureThreshold` in `statefulset.yaml`.

### Zenoh connectivity issues

Check that the zenoh-router pod is running and the bridge can connect:

```bash
kubectl logs swarm-swarm-digital-twin-0 -n swarm -c zenoh-bridge | head -20
```

The bridge should log `Successfully started plugin ros2dds`.
