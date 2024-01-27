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
DEFINE_ON_DEMAND(Mode_calculation)
{
#if !RP_NODE                       // run on host process
    system("ModeCalculation.bat"); // execute apdl batch
    if (read_nodes_modes() == 0)   // check if files generated
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
    double *nodeCoorDisp = NULL;             // node coordinates and modal displacements

    iIter = 1; // initialise iteration index
    iTime = 0; // initialise time index

    // RP_Get_Real(char *s)-RP_Get_Real("flow-time")
    // RP_Set_Float(char *s, double v)-RP_Set_Float("flow-time", 0.2)
    RP_Set_Float("flow-time", 0); // set  flow time

#if !RP_NODE                                  // run in host process
    if (fileStatus = read_nodes_modes() == 0) // input size of node coordinates and modal displacement
    {
        row = nNode, column = (nMode + 1) * N_DOF_PER_NODE;                 // number of modes
        Message("UDF[Host]: r = %d, c = %d, m = %d\n", row, column, nMode); // print size of node coordinates and modal displacement and number of modes

        nodeCoorDisp = (double *)malloc(row * column * sizeof(double)); // allocate memory for nodeCoorDisp
        memset(nodeCoorDisp, 0, row * column * sizeof(double));         // initialize nodeCoorDisp
        modeFreq = (real *)malloc(nMode * sizeof(real));                // allocate memory for mode frequency
        memset(modeFreq, 0, nMode * sizeof(real));                      // initialize frequency

        for (int iMode = 0; iMode < nMode + 1; iMode++)                     // input each modal displacement
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
    modeDisp = (real *)malloc(100000 * N_DOF_PER_NODE * nMode * sizeof(real));
    memset(modeDisp, 0, 100000 * N_DOF_PER_NODE * nMode * sizeof(real));
    initVelocity = (real *)malloc(4 * sizeof(real));
    memset(initVelocity, 0, 4 * sizeof(real));

    free(nodeCoorDisp);
}

/**
 * @brief construct a new grid motion method
 *
 */
DEFINE_GRID_MOTION(FDM_method, pDomain, dt, time, dTime)
{
    real *const modeDispThisTime = modeDisp + iTime * nMode * N_DOF_PER_NODE;        // modal displacement this time, buffer
    real *const modeForceThisTime = modeForce + iTime * nMode, modeForceBuff[nMode]; // modal force this time, buffer
    memset(modeDispThisTime, 0, nMode * N_DOF_PER_NODE * sizeof(real));              // allocate memory
    memset(modeForceThisTime, 0, nMode * sizeof(real));
    memset(modeForceBuff, 0, nMode * sizeof(real));

#if !RP_NODE
    // calculate modal force on structure
#endif
#if !RP_HOST
    get_mode_force(modeForceThisTime, pDomain); // get modal force
    // Message("UDF[Node][%d]: ", myid);
    // for (int i = 0; i < nMode; i++)
    //     Message("%5.1e, ", modeForceThisTime[i]);
    // Message("\n");
#endif

    PRF_GRSUM(modeForceThisTime, nMode, modeForceBuff); // global summation fo modal force this time
    node_to_host_real(modeForceThisTime, nMode);

#if !RP_NODE
    // Message("UDF[Host]: time step is %d, iteration is %d modal forces are ",iTime, iIter);
    // for (int i = 0; i < nMode; i++)
    //     Message("%5.1e ", modeForceThisTime[i]);
    // Message("\n");
    wilson_theta(modeForceThisTime, modeDispThisTime, dTime);
    // Message("UDF[Host]: time step is %d, iteration is %d modal displacement are ",iTime, iIter);
    // for (int i = 0; i < 9; i++)
    //     Message("%5.1e ", modeDispThisTime[i]);
    // Message("\n");
#endif

    host_to_node_real(modeDispThisTime, N_DOF_PER_NODE * nMode); // broadcast modal displacement to node process

#if !RP_HOST // run on node process
    move_grid(modeDispThisTime, pDomain);
#endif

    iIter++;
}

/**
 * @brief set next time step
 *
 */
DEFINE_EXECUTE_AT_END(Set_next_time_step)
{
#if !RP_NODE
    Message("UDF[Host]: time step is %d, time is %f, modal forces are ", iTime, TimeSeq[iTime]);
    for (int i = 0; i < nMode; i++)
        Message("%5.4e ", modeForce[iTime * nMode + i]);
    Message("\n");
#endif

    PRF_GSYNC();
    iIter = 1;
    iTime++;
    TimeSeq[iTime] = CURRENT_TIME;
}

/**
 * @brief clear memories at exit
 *
 */
DEFINE_EXECUTE_AT_EXIT(Finish_process)
{
    free(TimeSeq);
    free(modeDisp);
    free(modeFreq);
    free(modeForce);
    free(initVelocity);
    Message0("UDF[Host]: All memories freed!\n");
}

/**
 * @brief velocity inlet boundary condition
 * 
 */
DEFINE_PROFILE(Velocity_inlet, thread, variable_index)
{
    real U = 0.9;
    real x[3];
    real y;
    face_t f;
    begin_f_loop(f, thread)
    {
        F_CENTROID(x, f, thread);
        y = x[1];
        /* x[1] means the Y location, the X location was write as x[0] */
        F_PROFILE(f, thread, variable_index) = 1.5 * U * 4 / 0.1681 * y * (0.41 - y);
    }
    end_f_loop(f, thread)
}
