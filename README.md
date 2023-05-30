# pie-ai-mapping

depends on project: https://github.com/turtlewizard73/pie-mapping-and-navigation.git

## Setup and install
1. **Clone repository and update submodules**
    ```
    mkdir -p workspace/src
    cd workspace/src
    git clone https://github.com/turtlewizard73/pie-mapping-and-navigation.git
    git clone https://github.com/turtlewizard73/pie-ai-mapping.git
    ```

2. **Checkout on development and update submodules**
    ```
    cd src/pie-mapping-and-navigation
    git fetch
    git checkout development
    git pull
    git submodule update --init --recursive

    cd src/pie-ai-mapping
    git fetch
    git checkout development
    git pull
    git submodule update --init --recursive
    ```

3. **Build gazebo actors plugin**
    ```
    cd src/pie-mapping-and-navigation/gazebo_actor_collisions_plugin
    mkdir build
    cd build
    cmake ..
    make
    export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$(pwd)
    ```

4. **Install dependencies and build the packages**
    ```
    cd workspace/src/pie-mapping-and-navigation
    sudo xargs -a apt-dependencies.txt apt install -y

    cd workspace/src/pie-ai-mapping
    sudo xargs -a apt-dependencies.txt apt install -y
    pip3 install -r pip-dependencies.txt

    cd workspace
    catkin_make
    source devel/setup.bash
    ```

5. **Download and load weights**
    ```
    cd workspace/src/pie-ai-mapping/pie_detection/weigths
    wget https://pjreddie.com/media/files/yolov3.weights

    python3 pie-ai-mapping/pie_detection/scripts/convert_weights.py
    ```

## How to use
1. **start the simulation** (gazebo & rviz)
    ```
    roslaunch pie_ai_bringup test.launch
    ```

2. **moving the robot**
    ```
    rosrun teleop_twist_keyboard teleop_twist_keyboard.py
    ```

3. **Start the object and people detection**
    ```
    rosrun pie_detection image_detector.py
    ```

4. **Start displaying markers**
    ```
    rosrun pie_detection pose_enplace.py
    ```

## stuff to sort out
cd yolo
pip install -r requirements.txt
catkin_make --make-args run_tests


cuda:
https://towardsdatascience.com/how-to-finally-install-tensorflow-gpu-on-wsl2-7be59e278f92
https://towardsdatascience.com/how-to-finally-install-tensorflow-gpu-on-windows-10-63527910f255
https://www.tensorflow.org/install/pip#windows-wsl2_1
https://docs.nvidia.com/cuda/wsl-user-guide/index.html

yolo weigths: https://pjreddie.com/darknet/yolo/

people detection:
    yolos scriptek:
https://machinelearningspace.com/yolov3-tensorflow-2-part-1/
https://machinelearningspace.com/yolov3-tensorflow-2-part-2/
https://machinelearningspace.com/yolov3-tensorflow-2-part-3/
https://machinelearningspace.com/yolov3-tensorflow-2-part-4/
https://github.com/RahmadSadli/Deep-Learning/tree/master/YOLOv3_TF2

    cuda telepítés:
        https://towardsdatascience.com/how-to-finally-install-tensorflow-gpu-on-windows-10-63527910f255
        https://towardsdatascience.com/how-to-finally-install-tensorflow-gpu-on-wsl2-7be59e278f92
        https://www.tensorflow.org/install/source#gpu

    image segmentation:
        https://medium.com/analytics-vidhya/image-segmentation-with-yolov3-and-grabcut-59a0abaafa3e
        https://github.com/Cuda-Chen/fish-yolo-grabcut/blob/master/utils/GrabCut.py
        https://pyimagesearch.com/2021/01/19/image-masking-with-opencv/

install: bottleneck