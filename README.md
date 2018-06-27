# object_builders_lib
　Object Orientation Corrected Bounding Box Fit for Point Cloud Clusters. 
<p align="center">
    <img src=".readme/MinMaxBox_vs_MinBox.png" width="688px" alt=""/>
</p>

## TODO list
- [x] Apollo's [Min-box Object Builder](https://github.com/ApolloAuto/apollo/blob/master/docs/specs/3d_obstacle_perception_cn.md#minbox-%E9%9A%9C%E7%A2%8D%E7%89%A9%E8%BE%B9%E6%A1%86%E6%9E%84%E5%BB%BA)
- [ ] TAS: [An Orientation Corrected Bounding Box Fit Based on the Convex Hull under Real Time Constraints](https://www.youtube.com/watch?v=ZGzwdzMygLI&t=0s&list=LLbz4mTfvxwOQQQwDGaXf19A&index=13), similar to Apollo, IV 2018.
- [ ] **M. Himmelsbach**'s Fit Oriented Box by a *RANSAC* algorithm to fit the _dominant line_ in the xy-projection of the segment’s points; the other axis is then taken to be perpendicular to the dominant line, IV 2012.
    ```bibtex
    @inproceedings{himmelsbach2012tracking,
      title={Tracking and classification of arbitrary objects with bottom-up/top-down detection},
      author={Himmelsbach, Michael and Wuensche, H-J},
      booktitle={Intelligent Vehicles Symposium (IV), 2012 IEEE},
      pages={577--582},
      year={2012},
      organization={IEEE}
    }
    ```
<p align="left">
    <img src="https://user-images.githubusercontent.com/6770853/36574754-27763efe-1882-11e8-8aa1-baf91eec25b1.png" width="588px" alt=""/>
</p>

## How to use
1. `git clone` as a ROS package, with [common_lib](https://github.com/LidarPerception/common_lib) as dependency.
2. `find_package` for this **object_builders_lib** local package.
    ```cmake
    find_package(catkin REQUIRED COMPONENTS
        # ROS core
        roscpp
        # ROS messages
        #std_msgs
        #sensor_msgs
        # ROS PCL
        #pcl_conversions
        #pcl_ros
        # 3rd modules for ROS, like Eigen
        cmake_modules
        # local packages
        common_lib
        object_builders_lib
    )
    ```
3. Use **Point Cloud Clusters' Object Builder** in your source code.
    ```cpp
    // 1.include files
    #include "object_builders/base_object_builder.hpp"
    #include "object_builders/object_builder_manager.hpp"
    
    // 2.define object builder
    boost::shared_ptr<object_builder::BaseObjectBuilder> object_builder_;
    
    // 3.create object builder by manager
    object_builder_ = object_builder::createObjectBuilder();
    
    // 4.build 3D orientation bounding box for clustering point cloud
    std::vector<PointICloudPtr> cloud_clusters;
    std::vector<ObjectPtr> objects;
    object_builder_->build(cloud_clusters, &objects);
    ```
