# K8s Simulation Namespace & Lifecycle

Detailed instructions for Phase 1 of the [ROADMAP](../ROADMAP.md).

## Goal

Standardize how simulation runs are created, isolated, and cleaned up in
Kubernetes so that repeated runs don't leak resources or collide.

## Steps

### 1. Create a dedicated namespace

```bash
kubectl create namespace swarm-sim
kubectl label namespace swarm-sim purpose=simulation
```

Apply resource quotas to prevent runaway pods:

```yaml
# k8s/sim-quota.yaml
apiVersion: v1
kind: ResourceQuota
metadata:
  name: sim-quota
  namespace: swarm-sim
spec:
  hard:
    pods: "50"
    requests.cpu: "16"
    requests.memory: "32Gi"
    limits.cpu: "32"
    limits.memory: "64Gi"
```

```bash
kubectl apply -f k8s/sim-quota.yaml
```

### 2. Helm playground profile

Create `helm/swarm-digital-twin/values-playground.yaml`:

```yaml
namespace: swarm-sim
droneCount: 6
sitl:
  image: beret/ardupilot-sitl:latest
  resources:
    requests: { cpu: "500m", memory: "512Mi" }
    limits: { cpu: "1", memory: "1Gi" }
gazebo:
  enabled: true
  world: antisana  # or "empty" / "terrain_grid"
telemetry:
  forward: true
  targetPort: 14550
cleanup:
  autoDelete: true
  ttlSeconds: 3600
```

Deploy with:

```bash
helm install sim-run helm/swarm-digital-twin/ \
  -f helm/swarm-digital-twin/values-playground.yaml \
  -n swarm-sim
```

### 3. Lifecycle management

**Start a run:**
```bash
./run_scenario.sh --backend=k8s --single-live
```

**Check status:**
```bash
./run_scenario.sh --status
kubectl get pods -n swarm-sim -o wide
```

**Clean up:**
```bash
helm uninstall sim-run -n swarm-sim
kubectl delete namespace swarm-sim
```

### 4. Automated cleanup

Add a Kubernetes CronJob or TTL controller to delete completed simulation
pods after the configured `ttlSeconds`.

## Acceptance Criteria

- [ ] Namespace creation is idempotent (re-running the create command is safe)
- [ ] Resource quotas prevent more than 50 pods
- [ ] Helm install + uninstall leaves no orphaned resources
- [ ] `./run_scenario.sh --down` tears down all pods in the namespace
