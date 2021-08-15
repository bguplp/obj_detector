Open cv error:
	Issue:
	cv2.error: /io/opencv/modules/highgui/src/window.cpp:539: error: (-2) The function is not implemented. Rebuild the library with Windows, GTK+ 2.x or Carbon support. If you are on Ubuntu or Debian, install libgtk2.0-dev and pkg-config, then re-run cmake or configure script in function cvDestroyAllWindows
	Solution:
	pip install opencv-contrib-python

Python:
	Issue:
	*** Error in `python3': double free or corruption (!prev): 0x00000000419db1a0 ***
	Solution:
	sudo apt-get install libtcmalloc-minimal4
	export LD_PRELOAD="/usr/lib/libtcmalloc_minimal.so.4"

	Issue:
	ImportError: /opt/ros/kinetic/lib/python2.7/dist-packages/cv2.so: undefined symbol: PyCObject_Type
	Solution A(This won't allow you use cv with python2, i think):
	cd /opt/ros/kinetic/lib/python2.7/dist-packages/
	sudo mv cv2.so cv2_ros.so
	Solution B(In the script):
	-----------
	import sys
	sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
	import cv2
	sys.path.append('/opt/ros/kinetic/lib/python2.7/dist-packages') 
	-----------
