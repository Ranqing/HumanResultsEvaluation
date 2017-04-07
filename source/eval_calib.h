#ifndef _EVAL_CALIB_H
#define _EVAL_CALIB_H

#include "../../../Qing/qing_common.h"
#include "../../../Qing/qing_string.h"
#include "../../../Qing/qing_dir.h"

#define CMOS_W 22.3
#define CMOS_H 14.9

class Qing_Calibration_Evaluater{
public:
    string m_folder;                                 //calibration results' folder
    string m_out_folder;                             //output folder
    vector<string> m_mono_files, m_bino_files;       //all calibration files

    Qing_Calibration_Evaluater(const string folder);
    ~Qing_Calibration_Evaluater() {}

    void get_files();                           //get files
    void eval_mono_calib();                     //evaluate mono-calibration result
    void eval_bino_calib();                     //evaluate bino-calibration result
};




#endif
