/**
 * @file FastDynamicMesh.c
 * @author SpecialXuan (special.xuan@outlook.com)
 * @brief
 * @version 0.1
 * @date 2023-12-31
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "fdmUtils.h"
#include "udf.h"

/**
 * @brief calculate modal displacement
 *
 */
DEFINE_ON_DEMAND(ModeCalculation)
{
#if !RP_NODE                                 // run on host process
    system("ModeCalculation.bat");           // execute apdl batch
    if (read_row_column() == 0) // check if files generated
        Message("UDF[Host]: Files Generated\n");
#endif
}

/**
 * @brief preprocess for fluid solution
 *
 */
DEFINE_ON_DEMAND(Preprocess)
{
    int row = 0, column = 0, fileStatus = 0; // size of node coordinates and modal displacements, number of modes, status of input file(0 is noError, 1 is Missing files, 2 is Missing data
    double *nodeCoorDisp = NULL;                        // node coordinates and modal displacements

    iIter = 1; // initialise iteration index
    iTime = 0; // initialise time index

    // RP_Get_Real(char *s)-RP_Get_Real("flow-time")
    // RP_Set_Float(char *s, double v)-RP_Set_Float("flow-time", 0.2)
    RP_Set_Float("flow-time", 0); // set  flow time

#if !RP_NODE                                              // run in host process
    if (fileStatus = read_row_column() == 0) // input size of node coordinates and modal displacement
    {
        row = nNode, column = (nMode + 1) * 3;                                           // number of modes
        Message("UDF[Host]: r = %d, c = %d, m = %d\n", row, column, nMode); // print size of node coordinates and modal displacement and number of modes

        nodeCoorDisp = (double *)malloc(row * column * sizeof(double)); // allocate memory for nodeCoorDisp
        memset(nodeCoorDisp, 0, row * column * sizeof(double));         // initialize nodeCoorDisp
        modeFreq = (real *)malloc(nMode * sizeof(real));                // allocate memory for mode frequency
        memset(modeFreq, 0, nMode * sizeof(real));                      // initialize frequency

        for (int iMode = 0; iMode < nMode + 1; iMode++)                                  // input each modal displacement
            if (fileStatus = read_coor_mode(nodeCoorDisp, modeFreq, iMode)) // if file or data missing, print error
                Message("UDF[Host]: Error: %d in %d\n", fileStatus, iMode);

        Message("UDF[Host]: Mode frequency\n"); // print mode frequency
        for (int i = 0; i < nMode; i++)
            Message("           %f\n", modeFreq[i]);

        qsort(nodeCoorDisp, row, column * sizeof(double), cmp_node); // sort by node coordinate
    }
    else
        Message("UDF[Host]: Error: %d\n", fileStatus);
#endif

    host_to_node_int_1(fileStatus); // broadcast file status to all node process
    if (fileStatus == 0)            // if no error
    {
        host_to_node_int_4(row, column, nNode, nMode);                  // broadcast size to all node process
#if !RP_HOST                                                            // run in node process
        nodeCoorDisp = (double *)malloc(row * column * sizeof(double)); // allocate memory for nodeCoorDisp
        memset(nodeCoorDisp, 0, row * column * sizeof(double));         // initialize nodeCoorDisp
        modeFreq = (real *)malloc(nMode * sizeof(real));                // allocate memory for mode frequency
        memset(modeFreq, 0, nMode * sizeof(real));                      // initialize frequency
#endif

        host_to_node_double(nodeCoorDisp, row * column); // broadcast node coordinate and modal displacement to all node process
        host_to_node_double(modeFreq, nMode);            // broadcast mode frequency to node process

#if !RP_HOST // run in node process
        fill_modal_disp(nodeCoorDisp);
#endif
    }

    TimeSeq = (real *)malloc(100000 * sizeof(real));
    memset(TimeSeq, 0, 100000 * sizeof(real));
    modeForce = (real *)malloc(100000 * nMode * sizeof(real));
    memset(modeForce, 0, 100000 * nMode * sizeof(real));
    modeDisp = (real *)malloc(100000 * 3 * nMode * sizeof(real));
    memset(modeDisp, 0, 100000 * 3 * nMode * sizeof(real));
    initVelocity = (real *)malloc(4 * sizeof(real));
    memset(initVelocity, 0, 4 * sizeof(real));

    free(nodeCoorDisp);
}

/**
 * @brief construct a new grid motion method
 * 
 */
DEFINE_GRID_MOTION(FDM_method, pDomain, dt, time, dtime)
{
    real * const modeDispThisTime = modeDisp + iTime * nMode * 3, modeDispBuff[nMode * 3];
    real * const modeForceThisTime = modeForce + iTime * nMode, modeForceBuff[nMode];
    memset(modeDispThisTime, 0, nMode * 3 * sizeof(real));
    memset(modeDispBuff, 0, nMode * 3 * sizeof(real));
    memset(modeForceThisTime, 0, nMode * sizeof(real));
    memset(modeForceBuff, 0, nMode * sizeof(real));

#if !RP_NODE
    Message("\nUDF: ***      Begin: This is Host      ***\n");

    Message("UDF: ***      End:   This is Host      ***\n");
#endif
#if !RP_HOST
    Message("\nUDF: +++      Begin: This is Node      +++\n");

    get_mode_force(modeForceThisTime, pDomain);

    Message("UDF: +++      End:   This is Node      +++\n");
#endif

    PRF_GRSUM(modeForceThisTime, nMode, modeForceBuff);

#if !RP_NODE
    Message("\nUDF: ***      Begin: This is Host      ***\n");

    // if (iTime == 0)
    // {
    //     for (int i = 0; i < nMode; i++)
    //     {
    //         modeDisp[i * 3 + 0] = modeForceThisTime[i];
    //         modeDisp[i * 3 + 1] = initVelocity[i];
    //         modeDisp[i * 3 + 1] = 0;
    //     }
    // }
    // else
    // {
    //     const real *const modeForceLastTime = modeForce + (iTime - 1) * nMode;
    //     const real *const modeDispLastTime = modeDisp + (iTime - 1) * nMode * 3;
    //     for (int i = 0; i < nMode; i++)
    //     {
    //         // // the calculation of modal displacement,  acceleration, velocity using wilson-theta method
    //         // modeDispThisTime[3 * i] = modeForce[time_index - 1][i + 1] + theta * (modeForce[time_index][i + 1] - modeForce[time_index - 1][i + 1]);
    //         // modeDispThisTime[3 * i] += m * (6 / SQR(theta * dtime) * mode_disp[time_index - 1][3 * i] + 6 / (theta * dtime) * mode_disp[time_index - 1][3 * i + 1] + 2 * mode_disp[time_index - 1][3 * i + 2]);
    //         // modeDispThisTime[3 * i] += c * (3 / (theta * dtime) * mode_disp[time_index - 1][3 * i] + 2 * mode_disp[time_index - 1][3 * i + 1] + 0.5 * theta * dtime * mode_disp[time_index - 1][3 * i + 2]);
    //         // modeDispThisTime[3 * i] /= 6 * m / SQR(theta * dtime) + 3 * c / (theta * dtime) + SQR(2 * Pi * freq[i]);
    //         // // calculation of the acceleration
    //         // modeDispThisTime[3 * i + 2] = 6 / (SQR(theta * dtime) * theta) * (modeDispThisTime[3 * i] - mode_disp[time_index - 1][3 * i]) - 6 / (SQR(theta) * dtime) * mode_disp[time_index - 1][3 * i + 1] + (1 - 3 / theta) * mode_disp[time_index - 1][3 * i + 2];
    //         // // calculation of the velocity
    //         // modeDispThisTime[3 * i + 1] = mode_disp[time_index - 1][3 * i + 1] + 0.5 * dtime * (modeDispThisTime[3 * i + 2] + mode_disp[time_index - 1][3 * i + 2]);
    //         // // calculation of the displacement
    //         // modeDispThisTime[3 * i] = mode_disp[time_index - 1][3 * i] + dtime * mode_disp[time_index - 1][3 * i + 1] + SQR(dtime) / 6 * (mode_disp[time_index][3 * i + 2] + 2 * mode_disp[time_index - 1][3 * i + 2]);
    //     }
    // }

    Message("UDF: ***      End:   This is Host      ***\n");
#endif
}

DEFINE_EXECUTE_AT_END(setting_next_time_step)
{
#if !RP_NODE
    Message("\nUDF: ***      Begin: This is Host      ***\n");

    Message("UDF: ***      End:   This is Host      ***\n");
#endif
#if !RP_HOST
    Message("\nUDF: +++      Begin: This is Node      +++\n");

    Message("UDF: +++      End:   This is Node      +++\n");
#endif
}