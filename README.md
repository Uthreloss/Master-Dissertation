# Orbbec camera installation, compilation and integration with ROS & Pepper installation and
Ubuntu version: 16.04 KINETIC

Naoqi drivers: Python 2.7 SDK 2.5.5 Linux 64

## 1.- Camera drivers installation (Ubuntu Kinetic)

First of all, make sure that the rules are set with the name "56-orbbec.rules" in /etc/udev/rules.d/ (556 is not correct as it does not consider the rules in the highest priority)

If "gedit" gives problems to edit the file, create the file outside of /etc/udev/rules.d, edit it and move back in BUT it should be enough following: http://wiki.ros.org/astra_camera

Now check that the camera is detected using lsusb -v (Look for the name of the camera; in this case ORBBEC) You may try lsusb but it may not display the camera's name.

Follow: https://github.com/orbbec/ros_astra_camera (You may need to install rgbd and freenect for kinetic)
```bash
sudo apt-get install ros-kinetic-rgbd-launch
sudo apt-get install ros-kinetic-freenect-launch
```
Now run the launch files using:
```bash
roslaunch astra_launch astra.launch
```
And use rqt to visualise it (Plugins --> Visualization --> Image) Try different topics to check that it is working:
```bash
rqt
```
## 2.- OPENI SDK installation

The next step is to install OPENI2SDK

Download it from the orbbec website, read the README files for pre-requisites and run the file "install.sh" (WATCH OUT for libudev. You may need another version, for instance, "libudev1")

Also CHECK that there is only one rules dile atthe "/etc/udev/rules.d" directory

Finally vewrify that it works by running one of the samples in SAMPLES or TOOLS folder inside the SDK (g.e. "./NiViewer")

## 3.- ORBBEC SDK installation

IF everything runs without errors, it is time to install the Astra Orbbec SDK.

NOTE that Software developers normally assume that you know how to compile the scripts given in the SDKs
In this case, you should first create a "build" folder inside the "Astra_SDK" directory and run the following commands so that the test files can be generated from the scripts provided by the SDK:

```bash
mkdir build && cd build
cmake ../samples/
make
```
 *(WARNING:)* if the compilation gives you any errors, check that "libsfml-dev" and "libcsfml-dev" are properly installed:

```bash
sudo apt-get install libsfml-dev
sudo apt-get install libcsfml-dev
```
*(If the are still libraries errors, install the remaining ones with "sudo apt-get MISSING_LIBRARY_NAME" TOO.)*

Then if you go to build/bin you will be able to find the test files and run them making use of ./<the file you want to run>
    The body tracking file is "./SimpleBodyViewer-SFML"

*(WARNING: Make sure that you terminate the example scripts using Ctrl+C. Otherwise, if you close the window, the script is not completely terminated and the camera cannot be used until those are ended or the computer is rebooted.)*

Use these two to terminate remaining camera commands:

Detect running commands:

```bash
ps -A
```
        
Stop them:

```bash
kill all <Name of the process>
```
        
## 4.- ROS Node integration

As the camera is properly working, the ros node can be safely installed and tested

ROS NODE:

Install the ROS shinsel node following the instruction on:

        https://github.com/shinselrobots


*(WARNING: The Astra_ROS_NODE needs ALL paths to be correct to know where to find the libraries and referencing of locations in the Astra SDK. If any errors check the paths in the CMakelists.txt and.xml files and change them to them ones in YOUR machine.)*

Search for something like:

        home/pepper/workspace/.........

Also be careful with the "intalls" folder. You may have to change the path to the correct directories ;)

Lastly, make sure the basrc file is sourced:

```bash
source ~/.bashrc
```

Export lines hsould ALWAYS be BEFORE the source lines. To gedit

```bash
gedit ~/.bashrc
```
## 5.- Pepper Pytho SDK installation

To have the Pepper robotic platform running you should carefully follow this instruction:


Do NOT follow the instructions in the ROS tutorials (http://wiki.ros.org/nao/Tutorials/Installation#NAOqi)

Follow: http://doc.aldebaran.com/2-4/dev/python/install_guide.html#python-install-guide
  BUT download from https://community.ald.softbankrobotics.com/en/resources/software/language/en-gb
  the SDK that is especifically for Pepper (Example:Pepper SDKs and documentation 2.5.5 View / Python 2.7 SDK 2.5.5 Linux 64 / )

THEN add the following line to your ~./bashrc
```bash
export PYTHONPATH=${PYTHONPATH}:/your_path_to/pynaoqi-python/lib/python2.7/site-packages
```
Where the path is to the SDK directory "site-packages" which contains the naoqi.py

Follow the instructions found in http://doc.aldebaran.com/2-5/dev/python/install_guide.html to check that it was properly installed

If everything is installed, referenced and running:

    * Check Pepper IP address --> For example: 192.168.x.xxx
    * Access Wi-Fi (I used the Assisted Living Studio Local network)
    * git clone git@git.brl.ac.uk:AssistedLivingGroup/brl_pepper.git

## 6.- Exercising + data collection module Installation

```bash
git clone https://github.com/Uthreloss/Master-Dissertation.git
git clone https://github.com/Uthreloss/user_srv.git
```

*(WARNING: Check that the paths for all scripts correspond to those on your machine)*

Launch files:

First configure and run:

```bash
python Folder_Creator.py (Set how many participants you want to collect data from and the sets to be be created)
roslaunch master_dissertation Record_Session.launch participant:=X set:=Y mode:=Z (Modes "e" = engaging // "ne" = not engaging)
roslaunch master_dissertation Replay.launch participant:=X set:=Y
```
After collecting data then run the rest of the files in the "data_management" folder in this order:
```bash
python ReadAndAverage.py
python Cluster_Creator.py
python Big_Cluster_Creator.py
python ReadAndAverageRest.py
python Big_Cluster_Creator_Rest.py
```
