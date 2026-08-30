# Migrating source from V0.5.0 to V2.0.0

upgrading from Qt V6.10.2 to V6.11.2

These changes were made in 2 places:

- CmakeLists.txt

- Qt kit change

**The following config yaml file entries have been changed:**

- EnableCalOVR -> EnableCalibrationOVR

- calRangeBias -> RangeBias

- calRangeScale -> RangeScale
  
  

**The following config yaml file entries have been added:**

- ScanAngleWidth: 360.0

- StartScanAngle: 0.0

- FlattenScan: false

- RangeScale: 0.000978

- RangeBias: -525.0

- MinRange: 150.0

- MaxRange: 40000.0

- AlpaAngleStep: 0.602

- AlphaAngleBias: 1.15

- BetaAngle: 0.25

- XiAngle: 0.25

- ThetaAngleBias: 120.0

- MinRangeTrusted: 150.0
  
  

**Changes in l2lidar class for V2.0.0**

Depracated functions in the l2lidar class:

- **SetCalibrationOVR(calRangeScale_,calRangeBias_)**

- **GetCalibrationOVR(&calRangeScale_,&calRangeBias_)**

- **lidar_.ConvertL2data2pointcloud(frame, frame3d_, imu_adjust_, imuRollPitchOnly_, EnableCalRangeOVR_,calRangeScale_,calRangeBias_)**
  
  

**SetCalibrationOVR()** has been replaced by:

```
    lidar_.SetRangeBiasOVR(calRangeBias_);
    lidar_.SetRangeScaleOVR(calRangeScale_);
```



**GetCalibrationOVR(&calRangeScale_,&calRangeBias_)** has been replaced by:

```
calRangeBias_ =  lidar_.GetRangeBiasOVR();
calRangeScale_ = lidar_.SetRangeScaleOVR();
```

**lidar_.ConvertL2data2pointcloud(frame, frame3d_, imu_adjust_, imuRollPitchOnly_,
                                         EnableCalRangeOVR_,calRangeScale_,calRangeBias_)** has been replaced by:



```
lidar_.ConvertL2data2pointcloud(frame, frame3d_)
```

The parameters that were passed before are now class member variables set by class functions.

## For QtCreator 20.0.x, How to create a new Qt kit to use with l2lidar_node

This should be done for each platform type (GCC arm 64 bit, GCC amd64 ) you are working on.

start QtCreator

Do not open project

goto the menu Edit -> Preferences -> Kits

Clone the automatically managed Desktop Qt6.11.2 kit

rename it something like: ROS2-Jazzy-package %(Qt:Version)

configure to look like:

![63b9e0a9-792f-4081-a6ff-b4efcff82156](file:///C:/Users/photo/Pictures/Typedown/63b9e0a9-792f-4081-a6ff-b4efcff82156.png)

**Delete the build directory in your project folder for the l2lidar_node.**

**Delete the .QtCreator directory in your project for the l2lidar_node.**

When you open the l2lidar_node in QtCreator select this kit for configuration.
Also make sure to check both the Debug and Release to include in the configuration.

Select  'configure'

The next settings are to set the command line arguments.

This will be in the projects tab from the left pane.

Select the Run Settings.

![6bb76a0f-1c54-4c4f-9c71-ba14cb7bd602](file:///C:/Users/photo/Pictures/Typedown/6bb76a0f-1c54-4c4f-9c71-ba14cb7bd602.png)

Set the Command line arguments to:

```
--ros-args --params-file /home/robot/SoftwareDev/ros2_ws/src/l2lidar_node/config//l2lidar_node.yaml
```

You should adjust the filename path to match your project location.

You will need to do this for both the debug and release configurations.
