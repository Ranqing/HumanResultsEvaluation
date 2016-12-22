#include "eval_calib.h"
#include "eval_points.h"

int main()
{
//    string folder = "../../../../HumanDatas/20160711/Calib_Results";
//    Qing_Calibration_Evaluater calibEval(folder);
//    calibEval.eval_mono_calib();
//    calibEval.eval_bino_calib();


    string cloud_name = "../../HumanDatas/Pointclouds/pointcloud_Lu_nocolor.ply";
    Qing_Pointcloud_Evaluater pointEval(cloud_name);
    pointEval.fast_triangulation();

    return 1;
}
