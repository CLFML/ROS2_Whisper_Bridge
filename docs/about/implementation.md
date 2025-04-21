# 🌟 **Custom ROS 2 Node Example**

Publishes a simple “Hello World!” message every second using a custom message type. Ideal for getting started with ROS 2 publishers and timers.

---

### ✅ Topics

| Topic          | Type                          | Description                    |
|----------------|-------------------------------|--------------------------------|
| `/hello_world` | `custom_pkg/msg/Custom`       | Custom message with a string   |

---

### ⚙️ Parameters

_No parameters used in this node._

---

### 📦 Message Notes

- `Custom`:
  - `string custom_msg`

This node publishes the string `"Hello World!"` as the content of `custom_msg`.

---

### 🧩 Implementation Notes

- Built with **`rclcpp`**.
- Uses a **1-second timer** to trigger publishing.
- Simple custom message `custom_pkg::msg::Custom`.
- Logging is used to indicate node startup.

---

### 🏁 Launch Example

```bash
ros2 run custom_pkg custom_node
```

---

### 🧪 Dependencies

- `rclcpp`
- Custom message in `custom_pkg`:
  - `Custom.msg`
