# obj_detector
Object detection base yolo4-tiny for armadillo2. 
Face recognizer. 


## Dependencies
* ubuntu 16.04 LTS
* ROS kinect
* python 2.7.*
* [bguplp/gazebo_worlds package](https://github.com/bguplp/gazebo_worlds)

The following python2 packges are required:
* OpenCV
* scipy
* numpy
* cmake
* face_recognition

## installation

Assuming you already followed the instructions of [bguplp /robotican_demos_upgrade ](https://github.com/bguplp/robotican_demos_upgrade)

1. Install the face recognition:
```bash
pip install face_recognition
```
2. Install the forked darknet-ROS reposetory
```bash
cd obj_detector
git clone https://github.com/bguplp/darknet-ROS.git
```
3. Download the [weights](https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v3_optimal/yolov4.weights) to `/obj_detector/darknet-ROS/data` (Google-drive mirror [yolov4.weights](https://drive.google.com/open?id=1cewMfusmPjYWbrnuJRuKhPMwRe_b9PaT)).

4. Complie the darknet (cmake):
```bash
./build
```
Or with make:
```bash
make
```
## Runing
After launching armadillo (simulation/robot), Run Yolo-4:
```bash
rosrun obj_detector wrapper_yolo4.py
```


