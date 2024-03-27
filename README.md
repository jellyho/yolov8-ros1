# yolov8-ros1

Simple yolov8 ros1 wrapper.

Subscribes the image topic and inference by yolov8 to get bounding box and annotations.

You can use webcam as an image publisher.


### 1) Download this repository to workspace/src

```jsx
cd catckin_ws/src
git clone --recursive https://github.com/jellyho/yolov8-ros1.git
```

### 2) Install python dependencies of yolov5

```jsx
pip install ultralytics
```

### 3) Put pretrained weights

copy-paste the pretrained weights into catkin_ws/src/yolov8/src



### 4) Build the package and run!

```jsx
cd ~/catkin_ws
catkin build
source devel/setup.bash
```

There are three options when execute launch file.
- image(string) : Image topic name that you want to apply yolov5.
- verbose(bool) : Open popup window that shows the annotated results.
- publish(bool) : Publish the annotated image
- weights(stirng) : Pretrained weight name in src/ folder

Default setting is verbose:=false, publish:=true

If you don't put any option about weights, defuault yolov8s.pt will applied.

.

1. Subscribe existing image topic and yolo
```jsx
roslaunch yolov8 yolo.launch image:='/topic_name' verbose:=false publish:= true weights:=yolov8m.pt
```

2. Use webcam as an image publisher
```jsx
roslaunch yolov8 yolo_webcam.launch vebose:=false publish:=true
```

### 5) Topic lists

```jsx
/yolo_image - Image - Annotated inferenced image when publish:=true
/yolo_results - String - Bounding box information encoded into json format
```

> How to use /yolo_results ?

```python
# define String msg subscriber
rospy.Subscriber('/yolo_results', String, yolo_cb)
```

```python
# In callback, convert to json
import json

def yolo_cb(msg):
    result = json.loads(msg.data)
    print(result)
```

```bash
[{"name": "person", "class": 0, "confidence": 0.9249374270439148, "box": {"x1": 50.73822021484375, "y1": 1.416290283203125, "x2": 640.0, "y2": 478.8927917480469}},
 {"name": "person", "class": 0, "confidence": 0.7882765531539917, "box": {"x1": 0.37717437744140625,"y1": 164.11801147460938,"x2": 71.52581024169922, "y2": 478.4902648925781}}]
```
