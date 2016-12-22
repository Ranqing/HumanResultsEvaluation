#include "eval_points.h"

Qing_Pointcloud_Evaluater::Qing_Pointcloud_Evaluater(const string &cloud_name):
    m_cloud_name(cloud_name), m_cloud(new QingPointcloudXYZ() ), m_cloud_filtered(new QingPointcloudXYZ() )
{
    qing_read_pointxyz_ply(cloud_name, m_cloud);
#if 1
    std::cout << "PointCloud Size: " << m_cloud->width * m_cloud->height
              << " data points (" << pcl::getFieldsList (*m_cloud) << ")." << std::endl;
#endif
}

Qing_Pointcloud_Evaluater::Qing_Pointcloud_Evaluater():
    m_cloud(new QingPointcloudXYZ()), m_cloud_filtered(new QingPointcloudXYZ()),
    m_normals(new pcl::PointCloud<pcl::Normal>()), m_cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>() )
{

}

Qing_Pointcloud_Evaluater::~Qing_Pointcloud_Evaluater()
{

}

void Qing_Pointcloud_Evaluater::uniform_resampling()
{

}

void Qing_Pointcloud_Evaluater::fast_triangulation()
{
    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (m_cloud);
    n.setInputCloud (m_cloud);
    n.setSearchMethod (tree);
    n.setKSearch (20);
    n.compute (*normals);
    cout << "normals done." << endl;
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*m_cloud, *normals, *cloud_with_normals);
    cout << "pointclouds with normals done." << endl;
    //* cloud_with_normals = cloud + normals

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (0.15);

    // Set typical values for the parameters
    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors (600);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (triangles);

    // Additional vertex information
    std::vector<int> parts = gp3.getPartIDs();
    std::vector<int> states = gp3.getPointStates();

    string savename = m_cloud_name.substr(0, m_cloud_name.rfind('.')) + "_mesh.vtk";
    pcl::io::saveVTKFile (savename, triangles);
    cout << "meshes done. saving " << savename << endl;
}
