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
#include <udf.h>

/**
 * @brief calculate modal displacement
 *
 */
DEFINE_ON_DEMAND(Mode_calculation)
{
#if !RP_NODE                            // run on host process
    system("ModeCalculation.bat");      // execute apdl batch command
    if (read_struct_nodes_modes() == 0) // check if structure files generated
        Message("UDF[Host]: Structure Files Generated\n");
    if (read_fluid_nodes_modes() == 0) // check if fluid files generated
        Message("UDF[Host]: Fluid Files Generated\n");
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
    RP_Set_Float("flow-time", 0); // set flow time

    int row = 0, column = 0, fileStatus = 0;           // size of matrix, status of input file(0 is noError, 1 is Missing files, 2 is Missing data
#if !RP_NODE                                           // run in host process
    if ((fileStatus = read_struct_nodes_modes()) == 0) // input number of nodes and modes in structure
    {
        row = nNodeStruct, column = nModeStruct * N_DOF_PER_NODE;                            // size of structure modal displacement matrix
        Message("UDF[Host]: Structure: r = %d, c = %d, m = %d\n", row, column, nModeStruct); // print size of structure modal displacement and number of modes

        structModeDisp = (double *)malloc(row * column * sizeof(double)); // allocate memories
        memset(structModeDisp, 0, row * column * sizeof(double));         // initialize
        structStiff = (double *)malloc(row * N_DOF_PER_NODE * sizeof(double));
        memset(structStiff, 0, row * N_DOF_PER_NODE * sizeof(double));
        structDamp = (double *)malloc(row * N_DOF_PER_NODE * sizeof(double));
        memset(structDamp, 0, row * N_DOF_PER_NODE * sizeof(double));

        for (int iMode = 0; iMode < nModeStruct; iMode++)                                   // input each modal displacement
            if (fileStatus = read_struct_coor_mode(structModeDisp, row, column, iMode + 1)) // if file or data missing, print error
                Message("UDF[Host]: Error: Structure: %d in %d\n", fileStatus, iMode);

        if (fileStatus = read_struct_stiff_damp(structStiff, structDamp, row, N_DOF_PER_NODE)) // input stiffness and damping vector
            Message("UDF[Host]: Error: Structure: %d\n", fileStatus);
    }
    else
        Message("UDF[Host]: Structure: Error: %d\n", fileStatus); // if no file print error
#endif

    double *nodeCoorDisp = NULL;                      // node coordinates and modal displacements
#if !RP_NODE                                          // run in host process
    if ((fileStatus = read_fluid_nodes_modes()) == 0) // input number of nodes and modes in fluid
    {
        row = nNodeFluid, column = (nModeFluid + 1) * N_DOF_PER_NODE;            // size of node coordinates and modal displacement matrix
        Message("UDF[Host]: r = %d, c = %d, m = %d\n", row, column, nModeFluid); // print size and number of modes

        nodeCoorDisp = (double *)malloc(row * column * sizeof(double)); // allocate memories
        memset(nodeCoorDisp, 0, row * column * sizeof(double));         // initialize
        modeFreq = (real *)malloc(nModeFluid * sizeof(real));
        memset(modeFreq, 0, nModeFluid * sizeof(real));

        for (int iMode = 0; iMode < nModeFluid + 1; iMode++)                                   // input each modal displacement
            if (fileStatus = read_fluid_coor_mode(nodeCoorDisp, modeFreq, row, column, iMode)) // if file or data missing, print error
                Message("UDF[Host]: Error: %d in %d\n", fileStatus, iMode);

        Message("UDF[Host]: Mode frequency\n"); // print mode frequency
        for (int i = 0; i < nModeFluid; i++)
            Message("           %f\n", modeFreq[i]);

        qsort(nodeCoorDisp, row, column * sizeof(double), cmp_node); // sort by node coordinate
    }
    else
        Message("UDF[Host]: Error: %d\n", fileStatus); // if no file print error
#endif
    host_to_node_int_1(fileStatus); // broadcast file status to all node process

    if (fileStatus == 0) // if no error
    {
        host_to_node_int_4(row, column, nNodeFluid, nModeFluid);        // broadcast size to all node process
#if !RP_HOST                                                            // run in node process
        nodeCoorDisp = (double *)malloc(row * column * sizeof(double)); // allocate memories
        memset(nodeCoorDisp, 0, row * column * sizeof(double));         // initialize
        modeFreq = (real *)malloc(nModeFluid * sizeof(real));
        memset(modeFreq, 0, nModeFluid * sizeof(real));
#endif
        host_to_node_double(nodeCoorDisp, row * column); // broadcast node coordinate and modal displacement to all node process
        host_to_node_double(modeFreq, nModeFluid);       // broadcast mode frequency to node process
#if !RP_HOST                                             // run in node process
        fill_modal_disp(nodeCoorDisp, row, column);                   // fill modal displacement to UDMI
#endif

        TimeSeq = (real *)malloc(100000 * sizeof(real)); // allocate memories
        memset(TimeSeq, 0, 100000 * sizeof(real));       // initialize
        modeForce = (real *)malloc(100000 * nModeFluid * sizeof(real));
        memset(modeForce, 0, 100000 * nModeFluid * sizeof(real));
        modeDisp = (real *)malloc(100000 * N_DOF_PER_NODE * nModeFluid * sizeof(real));
        memset(modeDisp, 0, 100000 * N_DOF_PER_NODE * nModeFluid * sizeof(real));
        initVelocity = (real *)malloc(4 * sizeof(real));
        memset(initVelocity, 0, 4 * sizeof(real));
    }
    free(nodeCoorDisp); // clear memory
}

/**
 * @brief construct a new grid motion method
 *
 */
DEFINE_GRID_MOTION(FDM_method, pDomain, dt, time, dTime)
{
    real *const modeDispThisTime = modeDisp + iTime * nModeFluid * N_DOF_PER_NODE;             // modal displacement this time
    real *const modeForceThisTime = modeForce + iTime * nModeFluid, modeForceBuff[nModeFluid]; // modal forces this time, buffer
    memset(modeDispThisTime, 0, nModeFluid * N_DOF_PER_NODE * sizeof(real));                   // clear memory
    memset(modeForceThisTime, 0, nModeFluid * sizeof(real));
    memset(modeForceBuff, 0, nModeFluid * sizeof(real));

#if !RP_NODE
    // calculate modal forces on structure
#endif
#if !RP_HOST
    get_mode_force(modeForceThisTime, pDomain); // get modal forces
    // Message("UDF[Node][%d]: ", myid);
    // for (int i = 0; i < nModeFluid; i++)
    //     Message("%5.1e, ", modeForceThisTime[i]);
    // Message("\n");
#endif

    PRF_GRSUM(modeForceThisTime, nModeFluid, modeForceBuff); // global summation fo modal forces this time
    node_to_host_real(modeForceThisTime, nModeFluid);        // send modal forces to host process

#if !RP_NODE
    // Message("UDF[Host]: time step is %d, iteration is %d modal forces are ",iTime, iIter);
    // for (int i = 0; i < nModeFluid; i++)
    //     Message("%5.1e ", modeForceThisTime[i]);
    // Message("\n");
    wilson_theta(modeForceThisTime, modeDispThisTime, dTime); // wilson theta method
    // Message("UDF[Host]: time step is %d, iteration is %d modal displacement are ",iTime, iIter);
    // for (int i = 0; i < 9; i++)
    //     Message("%5.1e ", modeDispThisTime[i]);
    // Message("\n");
#endif

    host_to_node_real(modeDispThisTime, N_DOF_PER_NODE * nModeFluid); // broadcast modal displacement to node process

#if !RP_HOST                              // run on node process
    move_mesh(modeDispThisTime, pDomain); // move mesh
#endif

    iIter++; // iteration index +1
}

/**
 * @brief set next time step
 *
 */
DEFINE_EXECUTE_AT_END(Set_next_time_step)
{
#if !RP_NODE                                                                                     // run in host
    Message("UDF[Host]: time step is %d, time is %f, modal forces are ", iTime, TimeSeq[iTime]); // print modal forces
    for (int i = 0; i < nModeFluid; i++)
        Message("%5.4e ", modeForce[iTime * nModeFluid + i]);
    Message("\n");
#endif

    PRF_GSYNC();                   // synchronize
    iIter = 1;                     // reset iteration index
    iTime++;                       // time index +1
    TimeSeq[iTime] = CURRENT_TIME; // record this time
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
    free(structModeDisp);
    free(structStiff);
    free(structDamp);
    Message0("UDF[Host]: All memories cleared!\n");
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
    free(structModeDisp);
    free(structStiff);
    free(structDamp);
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
