# Quick start

## about the code

This script is used to detect turbines in images


>**Status:** <br>
><table>
><tr><td>Implemented features:</td><td>Getting images</td></tr>
><tr><td></td><td>Real-time slider for filters</td></tr>
><tr><td></td><td>Detect turbine based on lines (ie blades)</td></tr>
><tr><td></td><td>Find points on the turbine</td></tr>
><tr><td></td><td>PnP solver</td></tr>
><tr><td></td><td>Hard-coded camera parameters</td></tr>
><tr><td>Not jet implemented features:</td><td>Compiling with ROS</td></tr>
><tr><td></td><td>Reading the image over the ROS topic</td></tr>
><tr><td></td><td>Automatic tuning of the filters</td></tr>
><tr><td></td><td><br></td></tr>
><tr><td>Known bugs:</td><td>Segmentation fault when the PnPsolver ( detector::locate() ) is called with an incorrect calibration in the preceding steps </td></tr>
><tr><td></td><td>A-stable rotation matrix from the PnPSolver due to the geometric properties of the turbine</td></tr>
></table>


## installation

This is a step - by - step guide that enables you to build and run the turbine detection script

**prerequisite Ensure cmake installed**

1. Clone the github repository (html link)
2. Delete the /build directory
3. run the following to set-up the cmake project:

```bash
$ mkdir /build 
$ cd /build/
$ cmake ..  
$ make
```
4. Now a binary file named main is created, now lets make it executable

```bash 
$ sudo chmod +X main
```

5. running the binary. Syntax: main source(path to image / video) (video source is hardcoded in; default = /dev/video0)
```bash
$ ./main video 
or
$ ./main turbine.jpg
```
Use CTRL + P to bring up the 'display properties window' mark the PnP checkbox if all the result is similar / matching to the test image using the default settings

# Turbine detection




## Workflow

![Flowchart](https://raw.githubusercontent.com/Tihjs137/Turbine-detection/master/Resources/Turbine%20detection.jpg)
