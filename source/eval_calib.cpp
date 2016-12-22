#include "eval_calib.h"

Qing_Calibration_Evaluater::Qing_Calibration_Evaluater(const string folder): m_folder(folder)
{
    m_out_folder = m_folder + "/../Evaluations";
    qing_create_dir(m_out_folder);

    get_files();
}

void Qing_Calibration_Evaluater::get_files()
{
    vector<string> allFiles;
    qing_get_all_files(m_folder, allFiles);
    sort(allFiles.begin(), allFiles.end());

    m_mono_files.resize(0);
    m_bino_files.resize(0);

    for(int i = 0; i < allFiles.size(); ++i)
    {
        string filename = allFiles[i];
        if(filename.find("calib") != -1)
        {
            m_mono_files.push_back(filename);
        }
        else if (filename.find("stereo") != -1)
        {
            m_bino_files.push_back(filename);
        }
    }
    cout << m_mono_files.size() << " mono files in " << m_folder << endl;
    cout << m_bino_files.size() << " bino files in " << m_folder << endl;
}

void Qing_Calibration_Evaluater::eval_mono_calib()
{
    string outfile = m_out_folder + "/eval_mono_calib.txt";
    fstream fout(outfile.c_str(), ios::out);
    if(fout.is_open() == false)
    {
        cerr << "failed to open " << outfile << endl;
        return ;
    }
    cout << outfile << endl;
    fout << "Camera\tFocal\tRms" << endl;

    int numOfFiles = m_mono_files.size();
    for(int i = 0; i < numOfFiles; ++i)
    {
        string filename = m_mono_files[i];
        string cameraname = filename.substr(filename.find('_')+1, 3);

        filename = m_folder + "/" + filename;
        FileStorage fs(filename, FileStorage::READ);
        if(fs.isOpened() == false)
        {
            cerr << "failed to open " << filename << endl;
            continue;
        }

        double err, focal;
        int imagew;
        Mat camera_matrix;

        if(cameraname[0] == 'A' || cameraname[0] == 'B' || cameraname[0] == 'C')
            fs["Image_Height"] >> imagew;
        else
            fs["Image_Width"] >> imagew;
        fs["Avg_Reprojection_Error"] >> err;
        fs["Camera_Matrix"] >> camera_matrix;

        focal =  (camera_matrix.at<double>(0,0) *  CMOS_W) / imagew;
        fout << cameraname  << ": ";
        fout << setw(7) << setfill(' ') << focal << '\t' << setw(7) << setfill(' ') <<  err  << endl;
    }
    fout.close();
    cout << "please check " << outfile << endl;
}

void Qing_Calibration_Evaluater::eval_bino_calib()
{
    string outfile = m_out_folder + "/eval_bino_calib.txt";  //up folder
    fstream fout(outfile.c_str(), ios::out);
    if(fout.is_open() == false)
    {
        cerr << "failed to open " << outfile << endl;
        return ;
    }
    cout << outfile << endl;
    fout << "Stereo\tRms\tAver_Recti_Err\tMax_Recti_Err\tAver_Recons_Err(mm)\tMax_Recons_Err(mm)" << endl;

    int numOfFiles = m_bino_files.size() ;
    for(int i = 0; i < numOfFiles; ++i)
    {
        string filename = m_folder + "/" + m_bino_files[i];

        FileStorage fs(filename, FileStorage::READ);
        if(fs.isOpened() == false)
        {
            cerr << "failed to open " << filename << endl;
            continue;
        }

        double rms, avg_rect_err, max_rect_err;
        double avg_recons_err, max_recons_err;
        string cam0, cam1;
        fs["Left_Camera_Name"]  >> cam0;
        fs["Right_Camera_Name"] >> cam1;
        fs["RMS"] >> rms;
        fs["Average_Rectified_Error"] >> avg_rect_err;
        fs["Max_Rectified_Error"] >> max_rect_err;
        fs["Average_Reconstruct_Error"] >> avg_recons_err;
        fs["Max_Reconstruct_Error"] >> max_recons_err;

        fout << cam0+cam1 << ": " ;
        fout << setw(10) << setfill(' ') << rms << '\t'
             << setw(10) << setfill(' ') << avg_rect_err << '\t'
             << setw(10) << setfill(' ') << max_rect_err << '\t'
             << setw(10) << setfill(' ') << avg_recons_err << '\t'
             << setw(10) << setfill(' ') << max_recons_err << '\t' << endl;
    }
    fout.close();
    cout << "please check " << outfile << endl;
}
