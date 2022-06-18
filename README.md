# ofxLidarSlam

openFrameworks addon for performing SLAM using Lidar data.


Uses the [lidar-slam library]
(https://gitlab.kitware.com/keu-computervision/slam)

Currently only implemented for Ouster Lidars. 

Only tested on Macos BigSur on an Intel Mac.

## Requirements

* Use a nightly build of openFrameworks. It will not compile on the current release (0.11.2)
* [ofxOusterLidar](https://github.com/roymacdonald/ofxOusterLidar/tree/newLibs). It needs the newLibs branch rather than the master branch.
* [ofxDropdown](https://github.com/roymacdonald/ofxDropdown/)
* [ofxCeresLib](https://github.com/roymacdonald/ofxCeresLib)
* [ofxPCL](https://github.com/roymacdonald/ofxPCL)

Make sure you have g2o installed.

```
brew install g2o
```

## Important

### M1
On M1 macs, brew installs into a different directory, so you will need to modify the addon_config.mk file and put into it the correct file paths for the libraries


### Boost
Boost library is partially used within OF and it is included with it, but some other parts of it are needed so it is also installed using brew and wee need to link to those libraries.
You will need to remove from your project the paths to Boost included in OF.
On Xcode, go to the project settings and scroll all the way down and leave empty the following fields:

* LIB_BOOST_FS
* LIB_BOOST_SYSTEM
* HEADER_BOOST
