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

#include "triangulate.h"


triangulate::triangulate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, int scale_factor, double search_radius, double mu, int max_nbr, double max_sfc_angle, double min_angle,
                         double max_angle, bool normal_consistent)
{
    //std::cout << "Triangulating..." << std::endl;
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud);
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    n.setKSearch (20);
    n.compute (*normals);
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);


    // Create search tree*
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    tree2->setInputCloud (cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (search_radius*scale_factor);

    // Set typical values for the parameters
    gp3.setMu (mu*scale_factor);
    gp3.setMaximumNearestNeighbors (max_nbr);
    gp3.setMaximumSurfaceAngle(max_sfc_angle); // 45 degrees
    gp3.setMinimumAngle(min_angle); // 10 degrees
    gp3.setMaximumAngle(max_angle); // 120 degrees
    gp3.setNormalConsistency(normal_consistent);

    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);

}


pcl::PolygonMesh triangulate::getPolygonMesh(){
    return triangles;
}
