#ifndef EVAL_POINTS_H
#define EVAL_POINTS_H

#include "../../Qing/qing_pointcloud.h"
#include "../../Qing/qing_ply.h"

class Qing_Pointcloud_Evaluater
{
public:
    Qing_Pointcloud_Evaluater(const string& cloud_name);
    Qing_Pointcloud_Evaluater();
    ~Qing_Pointcloud_Evaluater() ;

    string m_cloud_name ;                        //point_cloud_name
    QingPointcloudXYZPtr m_cloud;
    QingPointcloudXYZPtr m_cloud_filtered;

    pcl::PointCloud<pcl::Normal>::Ptr m_normals;
    pcl::PointCloud<pcl::PointNormal>::Ptr m_cloud_with_normals;


    void uniform_resampling();
    void fast_triangulation();                     //triangulation

};

#endif // EVAL_POINTS_H
