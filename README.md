# BIMOS

BIMOS (Binary descriptor-based Image Mosaicing) is an open source image mosaicing algorithm which can produce seamless mosaics on different scenarios and camera configurations in a reasonable amount of time. It is based on a multi-threaded mosaicing architecture that permits decoupling the strategic steps involved in the process, speeding up the time required to estimate the final topology. In order to find the existent relations between the images, BIMOS employs [OBIndex] (http://github.com/emiliofidalgo/obindex), a binary BoW scheme that is built online, allowing the use of binary descriptors to accelerate the image description process.

The algorithm can generate mosaics from sequences where consecutive frames present enough overlap between them. BIMOS assumes that either the scene is planar or the distance from the camera to the scene is high enough so as to neglect the depth changes. It is also assumed that the camera is more or less perpendicular to the scene and at a more or less constant distance. Note that BIMOS is not a solution for generating rotational panoramas.

BIMOS is released as a ROS package, and relies on [OpenCV](http://opencv.org), [Ceres Solver](http://ceres-solver.org), [OBIndex](http://github.com/emiliofidalgo/obindex) and [Boost](http://www.boost.org) libraries. Note that BIMOS is research code. The authors are not responsible for any errors it may contain. **Use it at your own risk!**

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

This installation process has been tested and verified using Ubuntu 14.04.1 LTS and ROS Indigo, but other configurations should also work.

1. First of all, follow the instructions to install [OBIndex] (http://github.com/emiliofidalgo/obindex) in your workspace.

2. Install BIMOS dependencies:

	```bash
	$ sudo apt-get install libceres-dev   
	$ sudo apt-get install libsuitesparse-dev   
	```

2. Clone the repository into your workspace:
	
	```bash
	$ cd ~/your_workspace/src   
	$ git clone https://github.com/emiliofidalgo/bimos.git`   
	```

3. Compile the package using `catkin_make`:
	
	```bash
	$ cd ..   
	$ catkin_make -DCMAKE_BUILD_TYPE=Release   
	```

4. Launch BIMOS:
	
	```bash
	$ roslaunch bimos bimos.launch   
	```

## Usage

### GUI

We composed a [rqt](http://wiki.ros.org/rqt)-based GUI to interact with BIMOS. To load the interface, open a new terminal and execute:

    $ rqt   

Next, go to *'Perspectives - Import...'* and open the file *resources/bimos.perspective* stored in the root package directory. You should see a window like this:   

![BIMOS Interface](https://github.com/emiliofidalgo/bimos/blob/develop/resources/rqt_num.png)   

The three different parts that the interface presents are:

1. The commands that can be send to interact with BIMOS.

2. A graph viewer to see the topology while it is estimated. This component is for debugging purposes and can introduce performance issues when building large mosaics. For more information, see the `pub_debug_info` option.

3. A `dynamic_reconfigure` component to set the different mosaicing options. The options are detailed in the following section.

### Options

Before generating a mosaic, some BIMOS options should be set according to the conditions of your dataset.

- `working_dir`: Directory where BIMOS operates and where the final results will be stored. **This directory should be writable!**.
- `img_descriptor`: Combination of detector/descriptor to be used. The available options are:
	- *ORB_ORB* (Default)
	- *ORB_BRIEF*
	- *ORB_LDB*
	- *FAST_BRIEF*
	- *FAST_LDB*
- `nkeypoints`: Number of features to find in each input image.
- `pub_debug_info`: This option controls if debug information should be published or not. Currently, only the graph image is published, which can be shown during the topology estimation selecting the corresponding topic in the GUI. For large mosaics, this options can severely affect to the performance of BIMOS and this option should be disabled.
- `lc_delay_kfs`: 
- `match_ratio`:
- `min_inliers`:
- `optim_every_kfs`:
- `blend_exp`:
- `blend_seams`:
- `kf_min_inliers`:
- `kf_overlap`:
- `batch`:
- `batch_images_dir`:
- `max_reproj_error`:

## Known limitations

- Despite BIMOS can deal with hundreds of images during the estimation of the topology, the blending step is an adaptation of the *stitching* OpenCV module. This module loads the images on the RAM memory. Due to this reason, if you plan to create a mosaic using a high number of images, you can run out of memory at this point.

- BIMOS cannot be used with unordered sequences. Images comprising the input dataset should be enumerated consecutively when using the `batch` option.

<!-- - You should be sure that at least the first image can be considered as a KF -->

## Contact

If you have problems or questions using BIMOS, please contact the author (emilio.garcia@uib.es). [Feature requests](http://github.com/emiliofidalgo/bimos/issues) and [contributions](http://github.com/emiliofidalgo/bimos/pulls) are totally welcome.
