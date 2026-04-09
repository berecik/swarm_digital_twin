# Quick Start Guide: Swarm Digital Twin

This guide provides simple, step-by-step instructions for setting up the **Swarm Digital Twin** environment on different operating systems using `minikube` (Kubernetes) or Docker Compose.

---

## 📋 Prerequisites (Common)

Before starting, ensure you have the following installed:
- **Docker** (Desktop on MacOS, Engine on Linux)
- **Python 3.10+**
- **Git**

---

## 🚀 1. OS-Specific Setup (minikube)

Choose your operating system to install `minikube`, `kubectl`, and `helm`.

### 🍎 macOS
```bash
# Install tools
brew install minikube kubectl helm

# Start minikube
minikube start --cpus 4 --memory 8192 --disk-size 40g
```

### 🐧 Ubuntu / Debian
```bash
# Install kubectl
sudo apt-get update && sudo apt-get install -y apt-transport-https ca-certificates curl
curl -fsSL https://pkgs.k8s.io/core:/stable:/v1.29/deb/Release.key | sudo gpg --dearmor -o /etc/apt/keyrings/kubernetes-apt-keyring.gpg
echo 'deb [signed-by=/etc/apt/keyrings/kubernetes-apt-keyring.gpg] https://pkgs.k8s.io/core:/stable:/v1.29/deb/ /' | sudo tee /etc/apt/sources.list.d/kubernetes.list
sudo apt-get update && sudo apt-get install -y kubectl

# Install minikube
curl -LO https://storage.googleapis.com/minikube/releases/latest/minikube-linux-amd64
sudo install minikube-linux-amd64 /usr/local/bin/minikube

# Start minikube using Docker driver
minikube start --driver=docker --cpus 4 --memory 8192
```

### 🐧 Arch Linux
```bash
# Install tools
sudo pacman -S minikube kubectl helm

# Start minikube (KVM2 or Docker recommended)
minikube start --driver=docker --cpus 4 --memory 8192
```

### 🐧 Fedora
```bash
# Install kubectl
sudo dnf install -y kubernetes-client

# Install minikube
curl -LO https://storage.googleapis.com/minikube/releases/latest/minikube-linux-amd64
sudo install minikube-linux-amd64 /usr/local/bin/minikube

# Start minikube
minikube start --driver=docker --cpus 4 --memory 8192
```

---

## 🛠 2. Scenario A: Local Kubernetes (2 Drones)

This is the recommended way to test the full stack locally without an external registry.

1.  **Point shell to minikube's Docker daemon:**
    ```bash
    eval $(minikube docker-env)
    ```

2.  **Build images locally:**
    ```bash
    docker build -t ardupilot-sitl:latest -f Dockerfile.sitl .
    docker build -t swarm_companion:latest .
    ```

3.  **Deploy the swarm:**
    ```bash
    # This script auto-detects minikube and uses local images
    ./run_scenario.sh --swarm 2 --backend=k8s
    ```

4.  **Verify status:**
    ```bash
    ./run_scenario.sh --status
    ```

---

## 🛠 3. Scenario B: Docker Compose (6 Drones)

The simplest way to run a full 6-drone swarm if you don't want to use Kubernetes.

1.  **Build images:**
    ```bash
    docker compose build
    ```

2.  **Run the mission:**
    ```bash
    ./run_scenario.sh --swarm 6 --backend=docker
    ```

---

## 📊 4. Verification & Visualization

Once the swarm is running (in either scenario):

-   **Open Visualization:** The `run_scenario.sh` script usually opens the 3D visualizer automatically. If not, run:
    ```bash
    ./run_scenario.sh --viz-only
    ```
-   **Run Integration Tests:**
    ```bash
    ./run_scenario.sh --test
    ```
-   **Tear Down:**
    ```bash
    ./run_scenario.sh --down
    ```

---

## 🔍 Next Steps
- Read [docs/kubernetes.md](kubernetes.md) for advanced cluster configuration.
- Read [AGENTS.md](../AGENTS.md) for technical architecture details.
- Check [TESTING.md](../TESTING.md) for the full test suite.
