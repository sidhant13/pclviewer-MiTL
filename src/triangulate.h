/*Project: PCL_Visualization
 *Description: Used to fetch the triangulated representation of Point Cloud
 *Methods:
 *  triangulate()  [Constructor]
 *                 The parameters have some default values for the calculation of trangle objects.
 *                 They are defined in the macro below. The new values can be passed as an argument.
 *                 The constructor object also takes point cloud ptr as an argument on which it makes triangles.
 *                 Constructor stores the triangle in pcl::PolygonMesh object. It uses Greedy Projection algorithm
 *                 to  triangulate documented at http://pointclouds.org/documentation/tutorials/greedy_projection.php
 *
 * pcl::PolygonMesh getPolygonMesh()  [method]
 *                  Returns the triangle object calculated by constructor
*/


#include <iostream>
#include <string>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>

//[Default Value] Maximum distance between connected points
#define SR 2.5
//[Default Value] Maximum acceptable distance for a point to be considered
#define MU 2.0
//[Default Value] How many neighbors are searched for
#define MN 30
//[Default Value] Maximum Surface Angle
#define MSA M_PI/4
//[Default Value] The minimum and maximum angles in each triangle
#define MIN_A M_PI/18
#define MAX_A 2*M_PI/3
#define NC false

#ifndef TRIANGULATE_H
#define TRIANGULATE_H

class triangulate{

private:
    pcl::PolygonMesh triangles;

public:
    triangulate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int scale_factor=1, double search_radius=SR, double mu=MU, int max_nbr=MN, double max_sfc_angle=MSA, double min_angle=MIN_A,
                double max_angle=MAX_A, bool normal_consistent= NC);
    pcl::PolygonMesh getPolygonMesh();
};

#endif // TRIANGULATE_H

