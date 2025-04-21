# Getting started with Pixi

Pixi makes cross-platform ROS 2 development easy. You can build and run both capture and playback nodes on **Linux and Windows**—with no system-wide ROS install.

---

## 📦 Install Pixi

**Linux**:

```bash
curl -fsSL https://pixi.sh/install.sh | bash
```

**Windows** (PowerShell):

```powershell
powershell -ExecutionPolicy ByPass -c "irm -useb https://pixi.sh/install.ps1 | iex"
```

---

## 🚀 Clone & Build Project

```bash
git clone https://github.com/CLFML/ROS2_Package_Template.git
cd ROS2_Package_Template
pixi install
pixi run build
```

Or launch VSCode with the environment:

```bash
pixi run vscode
```

> ✅ **Note (Windows):** Always build in **Release** or **RelWithDebInfo**, not Debug!  
> *(Ctrl+Shift+P → "CMake: Select Variant")*

---

## ⚡ Using as a Pixi Dependency

Want to use `custom_pkg` from another Pixi-based project?

### 1. Init a new project

```bash
mkdir my_project && cd my_project
pixi init
```

### 2. Edit `pixi.toml`

Add these:

```toml
[project]
channels = [
  "https://fast.prefix.dev/conda-forge",
  "https://prefix.dev/robostack-jazzy",
  "https://clfml.github.io/conda_ros2_jazzy_channel/"
]

[dependencies]
ros-jazzy-ros-base = "*"
ros-jazzy-custom_pkg = "*"
colcon-common-extensions = "*"
rosdep = "*"
```

### 🧠 Optional: VSCode Support

Add to your `pixi.toml`:

```toml
[target.linux-64.dependencies]
python-devtools = "*"
pybind11 = "*"
numpy = "*"

[target.win-64.dependencies]
python-devtools = "*"

[target.linux-64.tasks]
vscode = 'env -u LD_LIBRARY_PATH code .'

[target.win-64.tasks]
vscode = "code ."
```

---

### 3. Run the nodes

```bash
pixi install
pixi run ros2 run custom_pkg custom_node
```

---
