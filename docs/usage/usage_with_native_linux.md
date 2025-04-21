# 🐧 Getting Started with Native ROS

Prefer a traditional system-wide install? Use the prebuilt `.deb` package for **Ubuntu Noble / ROS 2 Jazzy**.

---

## 📦 Install package via `.deb`

Install the latest `.deb` package directly from [Releases](https://github.com/CLFML/ROS2_Audio_Tools/releases):

```bash
curl -s https://api.github.com/repos/CLFML/ROS2_Package_Template/releases/latest \
  | grep "browser_download_url.*deb" \
  | cut -d : -f 2,3 \
  | tr -d \" \
  | wget -qi -
sudo dpkg -i ./ros-jazzy-*.deb
```

---

## ✅ Run the Nodes

Make sure ROS is sourced:

```bash
source /opt/ros/jazzy/setup.sh
```

```bash
ros2 run custom_pkg custom_node
```
