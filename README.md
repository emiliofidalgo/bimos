# BIMOS
## Binary descriptor-based Image Mosaicing

BIMOS is an open source image mosaicing algorithm which can produce seamless mosaics on different scenarios and camera configurations in a reasonable amount of time. It implements a multi-threaded mosaicing architecture that permits to decouple the strategic steps involved in the process, speeding up the time required to estimate the final topology. In order to find the existent relations between the images, BIMOS employs [OBIndex] (http://github.com/emiliofidalgo/obindex), a binary visual dictionary that is built online, allowing the use of binary descriptors to accelerate the image description process.

The algorithm can generate mosaics from sequences where enough overlap exists between consecutive frames. BIMOS assumes that either the scene is planar or the distance from the camera to the scene is high enough so as to neglect the depth changes. It is also assumed that the camera is more or less perpendicular to the scene and at a more or less constant distance. Note that BIMOS is not a solution for generating rotational panoramas.

BIMOS is released as a ROS package, and relies on OpenCV, Ceres, OBIndex and Boost libraries. Note that BIMOS is research code. The authors are not responsible for any errors it may contain. Use it at your own risk!

## Related publications

BIMOS has been submitted to ICRA:

**Fast Image Mosaicing using Incremental Bags of Binary Words**     
Emilio Garcia-Fidalgo, Alberto Ortiz, Francisco Bonnin-Pascual and Joan P. Company     
IEEE International Conference on Robotics and Automation (ICRA)     
Stockholm (Sweden), 2016     

<!--The paper can be downloaded from [here] (http://emiliofidalgo.github.io/static/papers/conf_ETFA_Garcia2014.pdf). If you use this software in an academic work, please cite:

	@INPROCEEDINGS{GarciaFidalgoETFA14,
		author={Garcia-Fidalgo, Emilio and Ortiz, Alberto},
		booktitle={Emerging Technology and Factory Automation (ETFA), 2014 IEEE},
		title={On the use of binary feature descriptors for loop closure detection},
		year={2014},
		month={Sept},
		pages={1-8},
		doi={10.1109/ETFA.2014.7005121}
	}
	-->

## Conditions of use

BIMOS is distributed under the terms of the [GPL3 License] (http://github.com/emiliofidalgo/bimos/blob/master/LICENSE).

## Installation

1. First of all, you will need to have installed all library dependencies:

	`sudo apt-get install libflann-dev`   
	`sudo apt-get install libboost-system-dev`   
	`sudo apt-get install libboost-filesystem-dev`   

2. Clone the repository into your workspace:
	
	`cd ~/your_workspace/src`   
	`git clone https://github.com/emiliofidalgo/obindex.git`   

3. Compile the package using `catkin_make`:
	
	`cd ..`   
	`catkin_make -DCMAKE_BUILD_TYPE=Release`   

4. You can run an example:
	
	`rosrun obindex test_bindex /directory/of/images`   

## Usage

For an example of use, see the demo file (`src/demo/example.cpp`).

## Known limitations

- Despite BIMOS can deal with hundreds of images during the topology estimation, the blending step is an adaptation of the stitching OpenCV module. This module loads the images on the RAM memory. Due to this reason, if you plan to create a mosaic using a high number of images, you can run out of memory at this point.

- BIMOS cannot be used with unordered sequences. Images comprising the dataset should be enumerated consecutively when using the `batch` option.

<!-- - You should be sure that at least the first image can be considered as a KF -->

## Contact

If you have problems or questions using BIMOS, please contact the author (emilio.garcia@uib.es). [Feature requests](http://github.com/emiliofidalgo/bimos/issues) and [contributions](http://github.com/emiliofidalgo/bimos/pulls) are totally welcome.
