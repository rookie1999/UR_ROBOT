### 安装摄像头驱动
`sudo apt install ros-noetic-usb-cam`


### ROS图像接口
1. 安装SDK
    * 位置：`https://github.com/intel-ros/realsense/releases`
    ```
        mkdir build
        cd build 
        cmake ..
        make
        sudo make install
    ```
2. 安装ROS驱动
    * 位置：`https://github.com/IntelRealSense/librealsense/releases`
    ```
        catkin_make -DCATKIN_ENABLE_TESTING=FALSE -DCMAKE_BUILD_TYPE=Release
        catkin_make install
        echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    ```

### 相机内参标定流程
安装标定功能包  `sudo apt install ros-noetic-camera-calibration`
1. 打开摄像头（使用usbcam.launch）
2. 调用标定功能包 `rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.024 image:=/usb_cam/image_raw camera:=/usb_cam`
    * size : 标定棋盘格内部角点个数，这里使用的棋盘有六行，每行有八个内部角点
    * square : 对应每个棋盘格的边长 单位是米
    * image和camera : 设置摄像头发布的图像话题
3. 遇到的问题
    * opencv与cvbridge的版本不匹配，cvbridge对应的opencv版本是4.2，使用高版本会报错

### 安装opencv
```
sudo apt install ros-noetic-vision-opencv libopencv-dev 
sudo apt install python-opencv
```