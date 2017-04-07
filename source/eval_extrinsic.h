#ifndef EVAL_EXTRINSIC_H
#define EVAL_EXTRINSIC_H

#include "../../../Qing/qing_common.h"

//to show cameras

class QingExtrinsicsEaluater
{
public:
    QingExtrinsicsEaluater();

    int m_num;                        //how many cameras
    Mat * m_extrinsics;               //extrinsics of all cameras


};

#endif // EVAL_EXTRINSIC_H
