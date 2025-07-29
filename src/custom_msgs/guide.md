# **Lab Assignment: Creating Custom ROS Messages to Publish Name, Age, Height, and Weight**

## **Objective**

Create a ROS package that defines a custom message type to publish a person’s Name, Age, Height, and Weight, and implement both publisher and subscriber nodes using this message.

---

## **🛠 Step-by-Step Procedure**

### **1. Create a ROS Package**

```bash
cd ~/srihasrc_ws/src
catkin_create_pkg person_info std_msgs rospy
```

---

### **2. Create the Custom Message File**

```bash
cd ~/srihasrc_ws/src/person_info
mkdir msg
cd msg
touch Person.msg
```

---

### **3. Define Message Format in `Person.msg`**

```text
string name
int32 age
float32 height
float32 weight
```

---

### **4. Modify `CMakeLists.txt`**

Update your `person_info/CMakeLists.txt` as follows:

```cmake
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
  rospy
)

add_message_files(
  FILES
  Person.msg
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

### **5. Modify `package.xml`**

Add the following inside the `<package>` tags:

```xml
<build_depend>message_generation</build_depend>
<exec_depend>message_runtime</exec_depend>
```

---

### **6. Build the Package**

```bash
cd ~/srihasrc_ws
catkin_make
source devel/setup.bash
```

---

### **7. Create the Publisher Node**

Create the file `publisher.py` inside `~/srihasrc_ws/src/person_info/scripts/`:

```python
#!/usr/bin/env python

import rospy
from person_info.msg import Person

def publisher():
    pub = rospy.Publisher('person_topic', Person, queue_size=10)
    rospy.init_node('person_publisher', anonymous=True)
    rate = rospy.Rate(1)  # 1 Hz

    msg = Person()
    msg.name = "Alice"
    msg.age = 25
    msg.height = 165.5
    msg.weight = 58.0

    while not rospy.is_shutdown():
        rospy.loginfo(msg)
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

### **8. Create the Subscriber Node**

Create the file `subscriber.py` inside `~/srihasrc_ws/src/person_info/scripts/`:

```python
#!/usr/bin/env python

import rospy
from person_info.msg import Person

def callback(data):
    rospy.loginfo("Received Info:\nName: %s\nAge: %d\nHeight: %.2f\nWeight: %.2f",
                  data.name, data.age, data.height, data.weight)

def subscriber():
    rospy.init_node('person_subscriber', anonymous=True)
    rospy.Subscriber('person_topic', Person, callback)
    rospy.spin()

if __name__ == '__main__':
    subscriber()
```

Make it executable:

```bash
chmod +x subscriber.py
```

---

## **🚀 Run the Nodes**

Open **3 terminals** and run the following:

### **Terminal 1 (Start roscore):**

```bash
cd ~/srihasrc_ws
source devel/setup.bash
roscore
```

### **Terminal 2 (Run publisher):**

```bash
cd ~/srihasrc_ws
source devel/setup.bash
rosrun person_info publisher.py
```

### **Terminal 3 (Run subscriber):**

```bash
cd ~/srihasrc_ws
source devel/setup.bash
rosrun person_info subscriber.py
```

---

## **✅ Conclusion**

This lab demonstrates the complete flow of creating and using **custom ROS message types** using **ROS1** and **Python (rospy)**. You created a package, defined a message, configured build files, and implemented publisher and subscriber nodes to exchange structured data over ROS topics.

---

