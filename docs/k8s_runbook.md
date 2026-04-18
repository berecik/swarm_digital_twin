# Kubernetes Simulation Runbook

One-command workflows for playground sessions. For full setup details
see [`docs/kubernetes.md`](kubernetes.md) and
[`docs/QUICKSTART.md`](QUICKSTART.md).

---

## Quick Reference

| Action | Command |
|:---|:---|
| **Deploy playground (6 drones)** | `helm install sim helm/swarm-digital-twin/ -f helm/swarm-digital-twin/values-playground.yaml -n swarm-sim --create-namespace` |
| **Deploy local (2 drones, minikube)** | `helm install sim helm/swarm-digital-twin/ -f helm/swarm-digital-twin/values-local.yaml -n swarm --create-namespace` |
| **Check status** | `./run_scenario.sh --status` |
| **Run single drone + live viewer** | `./run_scenario.sh --single-live` |
| **Run swarm mission** | `./run_scenario.sh --swarm 6 --backend=k8s` |
| **View pods** | `kubectl get pods -n swarm-sim -o wide` |
| **View logs (drone 1 SITL)** | `kubectl logs -n swarm-sim swarm-swarm-digital-twin-0 -c sitl` |
| **View logs (drone 1 swarm node)** | `kubectl logs -n swarm-sim swarm-swarm-digital-twin-0 -c swarm-node` |
| **Run topology test** | `helm test sim -n swarm-sim` |
| **Tear down** | `helm uninstall sim -n swarm-sim` |
| **Delete namespace** | `kubectl delete namespace swarm-sim` |
| **Full cleanup** | `./run_scenario.sh --down` |

---

## Detailed Workflows

### 1. Launch a Playground Session

```bash
# Create namespace + deploy with resource quotas
helm install sim helm/swarm-digital-twin/ \
  -f helm/swarm-digital-twin/values-playground.yaml \
  -n swarm-sim --create-namespace

# Wait for all pods (timeout 5 min)
kubectl rollout status statefulset/swarm-swarm-digital-twin \
  -n swarm-sim --timeout=300s

# Verify health
./run_scenario.sh --status
```

### 2. Run a Mission with Live View

```bash
# Start the live viewer listening on UDP 14550
./run_scenario.sh --viz-live &

# In another terminal, run the SITL mission
./run_scenario.sh --single-live --backend=k8s
```

The live viewer opens at `http://127.0.0.1:8765/live`. MAVLink telemetry
flows from the SITL pod via NodePort to the local MAVLinkLiveSource.

### 3. Capture Flight Logs

```bash
# Copy SITL logs from drone 1
kubectl cp swarm-sim/swarm-swarm-digital-twin-0:/sitl/logs/. logs/k8s_run/ -c sitl

# Copy from all drones
for i in $(seq 0 5); do
  mkdir -p logs/k8s_run/drone_$((i+1))
  kubectl cp swarm-sim/swarm-swarm-digital-twin-$i:/sitl/logs/. \
    logs/k8s_run/drone_$((i+1))/ -c sitl
done
```

### 4. Run Helm Tests

```bash
# Validate pod health + service topology
helm test sim -n swarm-sim

# View test results
kubectl logs -n swarm-sim swarm-swarm-digital-twin-test
kubectl logs -n swarm-sim swarm-swarm-digital-twin-test-topology
```

### 5. Upgrade (Change Drone Count)

```bash
# Scale to 3 drones
helm upgrade sim helm/swarm-digital-twin/ \
  -f helm/swarm-digital-twin/values-playground.yaml \
  --set drones=3 -n swarm-sim
```

### 6. Reset (Clean State)

```bash
# Tear down and redeploy
helm uninstall sim -n swarm-sim
kubectl delete pvc -n swarm-sim --all  # Clear cargo build caches
helm install sim helm/swarm-digital-twin/ \
  -f helm/swarm-digital-twin/values-playground.yaml \
  -n swarm-sim
```

### 7. Full Teardown

```bash
helm uninstall sim -n swarm-sim
kubectl delete namespace swarm-sim
# Or via run_scenario.sh:
./run_scenario.sh --down
```

---

## Troubleshooting

| Symptom | Diagnosis | Fix |
|:---|:---|:---|
| Pod stuck in `Pending` | Check resource quota: `kubectl describe quota -n swarm-sim` | Reduce `drones` or increase quota |
| Pod in `ImagePullBackOff` | Wrong registry: `kubectl describe pod ... -n swarm-sim` | Set `--set images.registry=...` |
| SITL `CrashLoopBackOff` | Check SITL logs: `kubectl logs ... -c sitl` | Verify `nodeSelector` matches arch |
| swarm-node startup slow | Cargo compiling from scratch | Wait for startup probe (30 min max); PVC caches subsequent runs |
| Zenoh bridge won't connect | Peer pod not ready: check pod-0 status | Ensure pod-0 (seed) is running first |
| No telemetry in live view | MAVLink NodePort not forwarded | Check: `kubectl get svc -n swarm-sim` for port mapping |

---

## Namespace Convention

| Namespace | Purpose | Profile |
|:---|:---|:---|
| `swarm` | Default (production-like) | `values.yaml` |
| `swarm-sim` | Playground (resource-limited) | `values-playground.yaml` |
| `swarm-dev` | Local development | `values-local.yaml` |
