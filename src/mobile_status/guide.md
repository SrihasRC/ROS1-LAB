# ✅ Custom ROS1 Message: Mobile Robot Status (Python + rospy)

---

## 🧭 Overview

We’ll create a ROS1 package that:

* Defines a **custom message**: `RobotStatus.msg`
* Publishes:

  * `float64 x`, `float64 y` (Location)
  * `string direction` (Direction of travel)
  * `time stamp` (Time of message)
* Includes **publisher** and **subscriber** nodes using `rospy`

---

## 🔧 Step-by-Step Instructions

---

### 🛠️ Step 1: Create the ROS Package

```bash
cd ~/srihasrc_ws/src
catkin_create_pkg mobile_status rospy std_msgs message_generation
```

Dependencies:

* `rospy`
* `std_msgs`
* `message_generation` (for custom messages)

---

### 📄 Step 2: Create the Custom Message

```bash
cd ~/srihasrc_ws/src/mobile_status
mkdir msg
gedit msg/RobotStatus.msg
```

Paste this inside `RobotStatus.msg`:

```
float64 x
float64 y
string direction
time stamp
```

---

### 🧪 Step 3: Modify `CMakeLists.txt`

Open `CMakeLists.txt` and **edit/uncomment/add** the following:

```cmake
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  RobotStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
```

---

### 📝 Step 4: Modify `package.xml`

Ensure the following tags are present:

```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>

<depend>rospy</depend>
<depend>std_msgs</depend>
```

---

### 🏗️ Step 5: Build the Package

```bash
cd ~/srihasrc_ws
catkin_make
source devel/setup.bash
```

✅ Now your custom message is ready to use.

---

### 🧾 Step 6: Create the Publisher Node

```bash
cd ~/srihasrc_ws/src/mobile_status
mkdir scripts
cd scripts
gedit publisher.py
```

Paste the following:

```python
#!/usr/bin/env python

import rospy
from mobile_status.msg import RobotStatus
import random

def publisher():
    pub = rospy.Publisher('robot_status', RobotStatus, queue_size=10)
    rospy.init_node('status_publisher', anonymous=True)
    rate = rospy.Rate(1)  # 1 Hz

    directions = ['North', 'East', 'South', 'West']

    while not rospy.is_shutdown():
        msg = RobotStatus()
        msg.x = round(random.uniform(0, 100), 2)
        msg.y = round(random.uniform(0, 100), 2)
        msg.direction = random.choice(directions)
        msg.stamp = rospy.Time.now()

        rospy.loginfo(f"Publishing: x={msg.x}, y={msg.y}, dir={msg.direction}, time={msg.stamp}")
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
```

Make it executable:

```bash
chmod +x publisher.py
```

---

### 📥 Step 7: Create the Subscriber Node

```bash
gedit subscriber.py
```

Paste the following:

```python
#!/usr/bin/env python

import rospy
from mobile_status.msg import RobotStatus

def callback(data):
    rospy.loginfo(f"Received: x={data.x}, y={data.y}, dir={data.direction}, time={data.stamp}")

def subscriber():
    rospy.init_node('status_subscriber', anonymous=True)
    rospy.Subscriber('robot_status', RobotStatus, callback)
    rospy.spin()

if __name__ == '__main__':
    subscriber()
```

Make it executable:

```bash
chmod +x subscriber.py
```

---

### 🚀 Step 8: Run Everything

**Terminal 1:**

```bash
roscore
```

**Terminal 2:**

```bash
source ~/srihasrc_ws/devel/setup.bash
rosrun mobile_status publisher.py
```

**Terminal 3:**

```bash
source ~/srihasrc_ws/devel/setup.bash
rosrun mobile_status subscriber.py
```

🎉 You should now see the custom `RobotStatus` messages being published and received!

---

## 📌 Final Notes

### ✅ Inspect the message:

```bash
rosmsg show mobile_status/RobotStatus
```

### ✅ View the topic:

```bash
rostopic echo /robot_status
```

---
