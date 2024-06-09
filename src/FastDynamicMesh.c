/**
 * @file FastDynamicMesh.c
 * @author SpecialXuan (special.xuan@outlook.com)
 * @brief
 * @version 1.2.2
 * @date 2024-04-15
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
    system("ModeCalculation.bat");      // execute apdl batch
    if (read_struct_nodes_modes() == 0) // check if files generated
        Message("UDF[Host]: Struct Files Generated\n");
    if (read_fluid_nodes_modes() == 0) // check if files generated
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
    RP_Set_Float("flow-time", 0); // set  flow time

    int fileStatus = 0;                                // size of node coordinates and modal displacements, number of modes, status of input file(0 is noError, 1 is Missing files, 2 is Missing data
#if !RP_NODE                                           // run in host process
    if ((fileStatus = read_struct_nodes_modes()) == 0) // input number of nodes and modes in structure
    {
        int row = nNodeStruct, column = nModeStruct * ND_ND;                        // size of structure modal displacement matrix
        Message("UDF[Host]: Structure: r = %d, c = %d, m = %d\n", row, column, nModeStruct); // print size of structure modal displacement and number of modes

        structModeDisp = (double *)malloc(row * column * sizeof(double)); // allocate memories
        memset(structModeDisp, 0, row * column * sizeof(double));         // initialize
        structStiff = (double *)malloc(row * ND_ND * sizeof(double));
        memset(structStiff, 0, row * ND_ND * sizeof(double));
        structDamp = (double *)malloc(row * ND_ND * sizeof(double));
        memset(structDamp, 0, row * ND_ND * sizeof(double));
        structLoad = (double *)malloc(row * ND_ND * sizeof(double));
        memset(structLoad, 0, row * ND_ND * sizeof(double));
        modeForceStruct = (real *)malloc(nModeStruct * sizeof(real));
        memset(modeForceStruct, 0, nModeStruct * sizeof(real));

        for (int iMode = 0; iMode < nModeStruct; iMode++)                                   // input each modal displacement
            if (fileStatus = read_struct_coor_mode(structModeDisp, row, column, iMode + 1)) // if file or data missing, print error
                Message("UDF[Host]: Error: Structure: %d in %d\n", fileStatus, iMode);

        if (fileStatus = read_struct_stiff_damp_load(structStiff, structDamp, structLoad, row, ND_ND)) // input stiffness and damping vector
            Message("UDF[Host]: Error: Structure: %d\n", fileStatus);
    }
    else
    {
        nModeStruct = nNodeStruct = 0;
        Message("UDF[Host]: Structure: Error: %d\n", fileStatus); // if no file print error
    }

    fileStatus = read_fluid_nodes_modes(); // input size of node coordinates and modal displacement
#endif

    host_to_node_int_1(fileStatus); // broadcast file status to all node process
    if (fileStatus == 0)            // if no error
    {
        host_to_node_int_2(nNodeFluid, nModeFluid);                       // broadcast size to all node process
        int row = nNodeFluid, column = (nModeFluid + 1) * ND_ND; // number of modes

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

        nModeFewer = nModeFluid < nModeStruct ? nModeFluid : nModeStruct; // get fewer mode

        TimeSeq = (real *)malloc(100000 * sizeof(real)); // allocate memories
        memset(TimeSeq, 0, 100000 * sizeof(real));
        modeForce = (real *)malloc(100000 * nModeFluid * sizeof(real));
        memset(modeForce, 0, 100000 * nModeFluid * sizeof(real));
        modeDisp = (real *)malloc(100000 * nModeFluid * 3 * sizeof(real));
        memset(modeDisp, 0, 100000 * nModeFluid * 3 * sizeof(real));
        initVelocity = (real *)malloc(nModeFluid * sizeof(real));
        memset(initVelocity, 0, nModeFluid * sizeof(real));

        initVelocity[0] = 0;
        initVelocity[1] = 0;
        initVelocity[2] = 0;
        initVelocity[3] = 0;
        initVelocity[4] = 0;

        free(nodeCoorDisp); // clear memory
    }
    else
        Message("UDF[Host]: Error: %d\n", fileStatus);
}

/**
 * @brief preprocess for fluid solution
 *
 */
DEFINE_EXECUTE_AFTER_DATA(AutoPreprocess, libudf)
{
    iIter = 1; // initialise iteration index
    iTime = 0; // initialise time index

    // RP_Get_Real(char *s)-RP_Get_Real("flow-time")
    // RP_Set_Float(char *s, double v)-RP_Set_Float("flow-time", 0.2)
    RP_Set_Float("flow-time", 0); // set  flow time

    int fileStatus = 0;                                // size of node coordinates and modal displacements, number of modes, status of input file(0 is noError, 1 is Missing files, 2 is Missing data
#if !RP_NODE                                           // run in host process
    if ((fileStatus = read_struct_nodes_modes()) == 0) // input number of nodes and modes in structure
    {
        int row = nNodeStruct, column = nModeStruct * ND_ND;                        // size of structure modal displacement matrix
        Message("UDF[Host]: Structure: r = %d, c = %d, m = %d\n", row, column, nModeStruct); // print size of structure modal displacement and number of modes

        structModeDisp = (double *)malloc(row * column * sizeof(double)); // allocate memories
        memset(structModeDisp, 0, row * column * sizeof(double));         // initialize
        structStiff = (double *)malloc(row * ND_ND * sizeof(double));
        memset(structStiff, 0, row * ND_ND * sizeof(double));
        structDamp = (double *)malloc(row * ND_ND * sizeof(double));
        memset(structDamp, 0, row * ND_ND * sizeof(double));
        structLoad = (double *)malloc(row * ND_ND * sizeof(double));
        memset(structLoad, 0, row * ND_ND * sizeof(double));
        modeForceStruct = (real *)malloc(nModeStruct * sizeof(real));
        memset(modeForceStruct, 0, nModeStruct * sizeof(real));

        for (int iMode = 0; iMode < nModeStruct; iMode++)                                   // input each modal displacement
            if (fileStatus = read_struct_coor_mode(structModeDisp, row, column, iMode + 1)) // if file or data missing, print error
                Message("UDF[Host]: Error: Structure: %d in %d\n", fileStatus, iMode);

        if (fileStatus = read_struct_stiff_damp_load(structStiff, structDamp, structLoad, row, ND_ND)) // input stiffness and damping vector
            Message("UDF[Host]: Error: Structure: %d\n", fileStatus);
    }
    else
    {
        nModeStruct = nNodeStruct = 0;
        Message("UDF[Host]: Structure: Error: %d\n", fileStatus); // if no file print error
    }

    fileStatus = read_fluid_nodes_modes(); // input size of node coordinates and modal displacement
#endif

    host_to_node_int_1(fileStatus); // broadcast file status to all node process
    if (fileStatus == 0)            // if no error
    {
        host_to_node_int_2(nNodeFluid, nModeFluid);                       // broadcast size to all node process
        int row = nNodeFluid, column = (nModeFluid + 1) * ND_ND; // number of modes

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

        nModeFewer = nModeFluid < nModeStruct ? nModeFluid : nModeStruct; // get fewer mode

        TimeSeq = (real *)malloc(100000 * sizeof(real)); // allocate memories
        memset(TimeSeq, 0, 100000 * sizeof(real));
        modeForce = (real *)malloc(100000 * nModeFluid * sizeof(real));
        memset(modeForce, 0, 100000 * nModeFluid * sizeof(real));
        modeDisp = (real *)malloc(100000 * nModeFluid * 3 * sizeof(real));
        memset(modeDisp, 0, 100000 * nModeFluid * 3 * sizeof(real));
        initVelocity = (real *)malloc(nModeFluid * sizeof(real));
        memset(initVelocity, 0, nModeFluid * sizeof(real));

        initVelocity[0] = 0;
        initVelocity[1] = 0;
        initVelocity[2] = 0;
        initVelocity[3] = 0;
        initVelocity[4] = 0;

        free(nodeCoorDisp); // clear memory
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
    real *const modeDispThisTime = modeDisp + iTime * nModeFluid * ND_ND;             // modal displacement this time, buffer
    real *const modeForceThisTime = modeForce + iTime * nModeFluid, modeForceBuff[nModeFluid]; // modal force this time, buffer
    memset(modeDispThisTime, 0, nModeFluid * ND_ND * sizeof(real));                   // allocate memory
    memset(modeForceThisTime, 0, nModeFluid * sizeof(real));
    memset(modeForceBuff, 0, nModeFluid * sizeof(real));

#if !RP_NODE
    // calculate modal force on structure
    if (nModeStruct > 0)
    {
        memset(modeForceStruct, 0, nModeStruct * sizeof(real)); // reset struct mode force
        get_struct_mode_force(modeForceStruct);                 // get struct mode force
    }
#endif

#if !RP_HOST
    get_fluid_mode_force(modeForceThisTime, pDomain); // get modal force
#endif

    PRF_GRSUM(modeForceThisTime, nModeFluid, modeForceBuff); // global summation fo modal force this time
    node_to_host_real(modeForceThisTime, nModeFluid);        // send mode force to host process

#if !RP_NODE
    for (int i = 0; i < nModeFewer; i++) // add struct force
        modeForceThisTime[i] += modeForceStruct[i];

    wilson_theta(modeForceThisTime, modeDispThisTime, dTime); // wilson theta method
#endif

    host_to_node_real(modeDispThisTime, ND_ND * nModeFluid); // broadcast modal displacement to node process

#if !RP_HOST                              // run on node process
    move_grid(modeDispThisTime, pDomain); // move grid
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
    Message("UDF[Host]: time step is %d, time is %f, modal forces are ", iTime, CURRENT_TIME);
    for (int i = 0; i < nModeFluid; i++)
        Message("%5.4e ", modeForce[iTime * nModeFluid + i]);
    Message("\n");

    FILE *fpOutput = fopen("Result.csv", "a");
    if (iTime == 0)
    {
        time_t timep;
        time(&timep);
        fprintf(fpOutput, "Calculation starts with %d processors, at %s", compute_node_count, asctime(gmtime(&timep)));
        fprintf(fpOutput, "                Time,");
        for (int i = 0; i < nModeFluid; i++)
        {
            fprintf(fpOutput, "             Force%2d,", i + 1);
        }
        for (int i = 0; i < nModeFluid; i++)
        {
            fprintf(fpOutput, "      Displacement%2d,", i * ND_ND + 0 + 1);
            fprintf(fpOutput, "      Displacement%2d,", i * ND_ND + 1 + 1);
            fprintf(fpOutput, "      Displacement%2d,", i * ND_ND + 2 + 1);
        }

        fprintf(fpOutput, "\n");
    }
    fprintf(fpOutput, "%20.10f,", CURRENT_TIME);
    for (int i = 0; i < nModeFluid; i++)
    {
        fprintf(fpOutput, "%20.10e,", modeForce[iTime * nModeFluid + i]);
    }
    for (int i = 0; i < nModeFluid; i++)
    {
        fprintf(fpOutput, "%20.10e,%20.10e,%20.10e,",
                modeDisp[iTime * nModeFluid * 3 + i * 3 + 0],
                modeDisp[iTime * nModeFluid * 3 + i * 3 + 1],
                modeDisp[iTime * nModeFluid * 3 + i * 3 + 2]);
    }
    fprintf(fpOutput, "\n");
    fclose(fpOutput);
#endif

    PRF_GSYNC();                   // synchronise
    iIter = 1;                     // reset index for iteration
    iTime++;                       // index for time +1
    TimeSeq[iTime] = CURRENT_TIME; // record time
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
