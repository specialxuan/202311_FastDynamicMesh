/**
 * @file FastDynamicMesh.c
 * @author SpecialXuan (special.xuan@outlook.com)
 * @brief
 * @version 1.1
 * @date 2023-12-31
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "fdmUtils.h"
#include <udf.h>

/**
 * @brief calculate modal displacement
 *
 */
DEFINE_ON_DEMAND(Mode_calculation)
{
#if !RP_NODE                           // run on host process
    system("ModeCalculation.bat");     // execute apdl batch
    if (read_fluid_nodes_modes() == 0) // check if files generated
        Message("UDF[Host]: Files Generated\n");
#endif
}

/**
 * @brief preprocess for fluid solution
 *
 */
DEFINE_ON_DEMAND(Preprocess)
{
    iIter = 1; // initialise iteration index
    iTime = 0; // initialise time index

    // RP_Get_Real(char *s)-RP_Get_Real("flow-time")
    // RP_Set_Float(char *s, double v)-RP_Set_Float("flow-time", 0.2)
    RP_Set_Float("flow-time", 0); // set  flow time

    int fileStatus = 0;                    // size of node coordinates and modal displacements, number of modes, status of input file(0 is noError, 1 is Missing files, 2 is Missing data
#if !RP_NODE                               // run in host process
    fileStatus = read_fluid_nodes_modes(); // input size of node coordinates and modal displacement
#endif

    host_to_node_int_1(fileStatus); // broadcast file status to all node process
    if (fileStatus == 0)            // if no error
    {
        host_to_node_int_2(nNodeFluid, nModeFluid);                       // broadcast size to all node process
        int row = nNodeFluid, column = (nModeFluid + 1) * N_DOF_PER_NODE; // number of modes

        double *nodeCoorDisp = NULL;                                    // node coordinates and modal displacements
        nodeCoorDisp = (double *)malloc(row * column * sizeof(double)); // allocate memory for nodeCoorDisp
        memset(nodeCoorDisp, 0, row * column * sizeof(double));         // initialize nodeCoorDisp
        modeFreq = (real *)malloc(nModeFluid * sizeof(real));           // allocate memory for mode frequency
        memset(modeFreq, 0, nModeFluid * sizeof(real));                 // initialize frequency

#if !RP_NODE                                                                     // run in host process
        Message("UDF[Host]: r = %d, c = %d, m = %d\n", row, column, nModeFluid); // print size of node coordinates and modal displacement and number of modes

        for (int iMode = 0; iMode < nModeFluid + 1; iMode++)                      // input each modal displacement
            if (fileStatus = read_fluid_coor_mode(nodeCoorDisp, modeFreq, iMode)) // if file or data missing, print error
                Message("UDF[Host]: Error: %d in %d\n", fileStatus, iMode);

        Message("UDF[Host]: Mode frequency\n"); // print mode frequency
        for (int i = 0; i < nModeFluid; i++)
            Message("           %f\n", modeFreq[i]);

        qsort(nodeCoorDisp, row, column * sizeof(double), cmp_node); // sort by node coordinate
#endif

        host_to_node_double(nodeCoorDisp, row * column); // broadcast node coordinate and modal displacement to all node process
        host_to_node_double(modeFreq, nModeFluid);       // broadcast mode frequency to node process

#if !RP_HOST // run in node process
        fill_modal_disp(nodeCoorDisp);
#endif

        TimeSeq = (real *)malloc(100000 * sizeof(real));
        memset(TimeSeq, 0, 100000 * sizeof(real));
        modeForce = (real *)malloc(100000 * nModeFluid * sizeof(real));
        memset(modeForce, 0, 100000 * nModeFluid * sizeof(real));
        modeDisp = (real *)malloc(100000 * N_DOF_PER_NODE * nModeFluid * sizeof(real));
        memset(modeDisp, 0, 100000 * N_DOF_PER_NODE * nModeFluid * sizeof(real));
        initVelocity = (real *)malloc(nModeFluid * sizeof(real));
        memset(initVelocity, 0, nModeFluid * sizeof(real));

        free(nodeCoorDisp);
    }
    else
        Message("UDF[Host]: Error: %d\n", fileStatus);
}

/**
 * @brief construct a new grid motion method
 *
 */
DEFINE_GRID_MOTION(FDM_method, pDomain, dt, time, dTime)
{
    real *const modeDispThisTime = modeDisp + iTime * nModeFluid * N_DOF_PER_NODE;             // modal displacement this time, buffer
    real *const modeForceThisTime = modeForce + iTime * nModeFluid, modeForceBuff[nModeFluid]; // modal force this time, buffer
    memset(modeDispThisTime, 0, nModeFluid * N_DOF_PER_NODE * sizeof(real));                   // allocate memory
    memset(modeForceThisTime, 0, nModeFluid * sizeof(real));
    memset(modeForceBuff, 0, nModeFluid * sizeof(real));

#if !RP_NODE
    // calculate modal force on structure
#endif
#if !RP_HOST
    get_fluid_mode_force(modeForceThisTime, pDomain); // get modal force
    // Message("UDF[Node][%d]: ", myid);
    // for (int i = 0; i < nModeFluid; i++)
    //     Message("%5.1e, ", modeForceThisTime[i]);
    // Message("\n");
#endif

    PRF_GRSUM(modeForceThisTime, nModeFluid, modeForceBuff); // global summation fo modal force this time
    node_to_host_real(modeForceThisTime, nModeFluid);

#if !RP_NODE
    // Message("UDF[Host]: time step is %d, iteration is %d modal forces are ",iTime, iIter);
    // for (int i = 0; i < nModeFluid; i++)
    //     Message("%5.1e ", modeForceThisTime[i]);
    // Message("\n");
    wilson_theta(modeForceThisTime, modeDispThisTime, dTime);
    // Message("UDF[Host]: time step is %d, iteration is %d modal displacement are ",iTime, iIter);
    // for (int i = 0; i < 9; i++)
    //     Message("%5.1e ", modeDispThisTime[i]);
    // Message("\n");
#endif

    host_to_node_real(modeDispThisTime, N_DOF_PER_NODE * nModeFluid); // broadcast modal displacement to node process

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
    for (int i = 0; i < nModeFluid; i++)
        Message("%5.4e ", modeForce[iTime * nModeFluid + i]);
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
    free(TimeSeq); // clear memories
    free(modeDisp);
    free(modeFreq);
    free(modeForce);
    free(initVelocity);
    Message0("UDF[Host]: All memories freed!\n");
}

/**
 * @brief clear memories at exit
 *
 */
DEFINE_ON_DEMAND(Clear_memories)
{
    free(TimeSeq); // clear memories
    free(modeDisp);
    free(modeFreq);
    free(modeForce);
    free(initVelocity);
    Message0("UDF[Host]: All memories cleared!\n");
}

/**
 * @brief velocity inlet boundary condition
 *
 */
DEFINE_PROFILE(Velocity_inlet, thread, iVar)
{
    real inVel = 0.9, x[3] = {0}, y = 0; // inlet velocity
    face_t pFace;                        // pointer of face
    begin_f_loop(pFace, thread)
    {
        F_CENTROID(x, pFace, thread); // coordinates of centroid of this face
        y = x[1];
        /* x[1] means the Y location, the X location was write as x[0] */
        F_PROFILE(pFace, thread, iVar) = 1.5 * inVel * 4 / 0.1681 * y * (0.41 - y);
    }
    end_f_loop(pFace, thread)
}
