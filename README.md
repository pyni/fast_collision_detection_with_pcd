# fast_collision_detection_with_pcd
在抓取方面，通常点云碰撞检测需要看每个法线向上方向有没有物体，而极限搜索中pcl::transformPointCloud (*cloud_filtered, *transformed_cloud, transform_0);这个函数时间代价太大

这里呢，用了一种几何算法，取代pcl::transformPointCloud 

点到直线点到平面公式同时参考：
https://wenku.baidu.com/view/ed88f4bcfab069dc51220129.html
