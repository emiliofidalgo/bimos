# BIMOS

BIMOS (Binary descriptor-based Image Mosaicing) is an open source image mosaicing algorithm which can produce seamless mosaics on different scenarios and camera configurations in a reasonable amount of time. It is based on a multi-threaded mosaicing architecture that permits decoupling the strategic steps involved in the process, speeding up the time required to estimate the final topology. In order to find the existent relations between the images, BIMOS employs [OBIndex] (http://github.com/emiliofidalgo/obindex), a binary BoW scheme that is built online, allowing the use of binary descriptors to accelerate the image description process.

The algorithm can generate mosaics from sequences where consecutive frames present enough overlap between them. BIMOS assumes that either the scene is planar or the distance from the camera to the scene is high enough so as to neglect the depth changes. It is also assumed that the camera is more or less perpendicular to the scene and at a more or less constant distance. Note that BIMOS is not a solution for generating rotational panoramas.

BIMOS is released as a ROS package, and relies on [OpenCV](http://opencv.org), [Ceres Solver](http://ceres-solver.org), [OBIndex](http://github.com/emiliofidalgo/obindex) and [Boost](http://www.boost.org) libraries. Note that BIMOS is research code. The authors are not responsible for any errors it may contain. **Use it at your own risk!**

## Related publications

BIMOS was presented at ICRA conference. If you use BIMOS, we will be grateful if you cite:

**Fast Image Mosaicing using Incremental Bags of Binary Words**     
Emilio Garcia-Fidalgo, Alberto Ortiz, Francisco Bonnin-Pascual and Joan P. Company     
IEEE International Conference on Robotics and Automation (ICRA), pp. 1174-1180.         
Stockholm (Sweden), 2016      
Link to [IEEE Xplore] (http://ieeexplore.ieee.org/document/7487247/)        

## Conditions of use

BIMOS is distributed under the terms of the [GPL3 License] (http://github.com/emiliofidalgo/bimos/blob/master/LICENSE).

## Installation

This installation process has been tested and verified using Ubuntu 14.04.1 LTS and ROS Indigo, but other configurations should also work.

1. First of all, follow the instructions to install [OBIndex] (http://github.com/emiliofidalgo/obindex) in your workspace.

2. Install BIMOS dependencies:

Install ceres and its own dependencies as explained [here] (http://ceres-solver.org/building.html#linux). Note that installing ceres with the command

	```bash
	$ sudo apt-get install libceres-dev
	```
will cause compilation errors [see the bug report] (https://bugs.launchpad.net/ubuntu/+source/ceres-solver/+bug/1595692).

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

We composed a simple [rqt](http://wiki.ros.org/rqt)-based GUI to interact with BIMOS. To load the interface, open a new terminal and execute:

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

- `lc_delay_kfs`: Minimum inserted keyframes to consider a keyframe as a possible loop candidate. This is used to avoid closing loops with recently added keyframes.

- `match_ratio`: Nearest neighbour distance ratio to match descriptors. Usually, a value of 0.8 is a good option here.

- `min_inliers`: Minimum number of inliers to consider a loop closure detection. It should be configured depending on the movement of the camera.

- `optim_every_kfs`: Number of new inserted keyframes to launch a graph optimization. If large loops are not present in the trajectory, this value could be low.

- `blend_exp`: Exposure compensation during the blending step. This option is computationally demanding and can increase the time needed to create the final composite. It is recommended to uncheck it unless for a low number of images.

- `blend_seams`: Seam finding during the blending step. This option should be checked to obtain a seamless mosaic. However, it can be disabled to obtain a draft of the mosaic, since it can be also computationally demanding.

- `kf_min_inliers`: Minimum number of inliers between two consecutive images to consider an image as a new keyframe. Usually the higher the value, the larger the number of images used for the final mosaic.

- `kf_overlap`: Minimum overlap between two consecutive images to consider an image as a new keyframe. Usually the higher the value, the larger the number of images used for the final mosaic.

- `batch`: If checked, the input images are loaded directly from a local directory indicated in the `batch_images_dir` option. Otherwise, images are received through a ROS topic.

- `batch_images_dir`: When `batch` option is checked, this is the path to the directory where the input images are stored. The images should be named consecutively using the same number of digits. For instance: *image00001.jpg*, *image00002.jpg*, and so on.

- `max_reproj_error`: Maximum reprojection error when computing a homography between two images. The higher this value, the less number of inliers between the images. Depending on the scenario, this value could be set to a higher value to be less restrictive. The errors will be corrected by means of optimizations.

### Creating a mosaic

- After setting the corresponding options, we are ready to generate a mosaic. As examples, we provide two datasets. Download and unzip these files using the following links:

	- [ODEMAR](https://www.dropbox.com/s/dq3fi7h515auuu3/ODEMAR.tar.gz?dl=0), 64 images of size 480x270 pixels (~19MB), front-looking camera attached to an underwater vehicle (Thanks to Miquel Massot-Campos).

	- [Valldemossa](https://www.dropbox.com/s/9nqx2kgbbc70pd8/Valldemossa.tar.gz?dl=0), 201 images of size 320x180 pixels (~6MB), bottom-looking camera attached to an underwater vehicle (Thanks to Francisco Bonin-Font).

- Let's suppose that ODEMAR dataset has been unzipped in */home/user/ODEMAR/*. Then, set the `batch_images_dir` option to */home/user/ODEMAR/* (**Do not forget the '/' character!**).

- Check the `/bimos_node/init_mosaic` command to start the mosaicing process. Uncheck it as soon as log lines start to appear on the screen.

- If you want to see the topology, press *Refresh* in the graph viewer component and select the `/bimos_node/mosaic_graph` topic in the list.

- After all the images have been processed, check/uncheck the `/bimos_node/optim_and_blend` command to perform a final optimization and start the blending step.

- After a while, measures resulting from the mosaicing process, such as times and reprojection errors, will appear on the screen. The resulting mosaic is also saved as a JPG image in the working directory, along with other interesting files such as the optimized 2D poses. Note that two reduced versions of the mosaic have also been saved.

- Using the same image alignment, you can call the `/bimos_node/blend` command to create another version of the mosaic, modifying the blending options.

- If you want to repeat the image alignment or generate a mosaic using a different dataset, check/unckeck the `/bimos_node/stop_mosaic` to reinitialize BIMOS, modify the parameters as needed, and then check/uncheck the `/bimos_node/init_mosaic` again.

- As a result, you should obtain something like this (Valldemossa):

<!--![ODEMAR](https://github.com/emiliofidalgo/bimos/blob/develop/resources/mosaic_ODEMAR.jpg)   -->
![Valldemossa](https://github.com/emiliofidalgo/bimos/blob/develop/resources/mosaic_Valldemossa.jpg)

## Known limitations

- Despite BIMOS can deal with hundreds of images during the estimation of the topology, the blending step is an adaptation of the *stitching* OpenCV module. This module loads the images on the RAM memory. Due to this reason, if you plan to create a mosaic using a high number of images, you can run out of memory at this point.

- BIMOS cannot be used with unordered sequences. Images comprising the input dataset should be enumerated consecutively when using the `batch` option.

<!-- - You should be sure that at least the first image can be considered as a KF -->

## Contact

If you have problems or questions using BIMOS, please contact the author (emilio.garcia@uib.es). [Feature requests](http://github.com/emiliofidalgo/bimos/issues) and [contributions](http://github.com/emiliofidalgo/bimos/pulls) are totally welcome.
