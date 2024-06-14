/**
 * @file FastDynamicMesh.c
 * @author SpecialXuan (special.xuan@outlook.com)
 * @brief
 * @version 1.3.0
 * @date 2024-06-14
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

    RP_Set_Float("flow-time", 0); // set  flow time
    free_memories();

    int fileStatus = 0;                                // size of node coordinates and modal displacements, number of modes, status of input file(0 is noError, 1 is Missing files, 2 is Missing data
#if !RP_NODE                                           // run in host process
    if ((fileStatus = read_struct_nodes_modes()) == 0) // input number of nodes and modes in structure
    {
        int row = nNodeStruct, column = nModeStruct * ND_ND;                                 // size of structure modal displacement matrix
        Message("UDF[Host]: Structure: r = %d, c = %d, m = %d\n", row, column, nModeStruct); // print size of structure modal displacement and number of modes

        structModeDisp = (double *)malloc(row * column * sizeof(double)); // allocate memories
        memset(structModeDisp, 0, row * column * sizeof(double));         // initialize
        structStiff = (double *)malloc(row * ND_ND * sizeof(double));
        memset(structStiff, 0, row * ND_ND * sizeof(double));
        structDamp = (double *)malloc(row * ND_ND * sizeof(double));
        memset(structDamp, 0, row * ND_ND * sizeof(double));
        structLoad = (double *)malloc(row * ND_ND * sizeof(double));
        memset(structLoad, 0, row * ND_ND * sizeof(double));
        structModeForce = (real *)malloc(nModeStruct * sizeof(real));
        memset(structModeForce, 0, nModeStruct * sizeof(real));

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

    fileStatus = read_fluid_nodes_modes();                            // input size of node coordinates and modal displacement
    nModeFewer = nModeFluid < nModeStruct ? nModeFluid : nModeStruct; // get fewer mode
#endif

    host_to_node_int_1(fileStatus); // broadcast file status to all node process
    if (fileStatus == 0)            // if no error
    {
        host_to_node_int_2(nNodeFluid, nModeFluid);              // broadcast size to all node process
        int row = nNodeFluid, column = (nModeFluid + 1) * ND_ND; // number of modes

        double *nodeCoorDisp = NULL;                                    // node coordinates and modal displacements
        nodeCoorDisp = (double *)malloc(row * column * sizeof(double)); // allocate memory for nodeCoorDisp
        memset(nodeCoorDisp, 0, row * column * sizeof(double));         // initialize nodeCoorDisp
        modeFreq = (real *)malloc(nModeFluid * sizeof(real));           // allocate memory for mode frequency
        memset(modeFreq, 0, nModeFluid * sizeof(real));                 // initialize frequency
        modeForce_ThisTime = (real *)malloc(nModeFluid * sizeof(real));
        memset(modeForce_ThisTime, 0, nModeFluid * sizeof(real));
        modeForce_LastTime = (real *)malloc(nModeFluid * sizeof(real));
        memset(modeForce_LastTime, 0, nModeFluid * sizeof(real));
        modeDisp_ThisTime = (real *)malloc(nModeFluid * 3 * sizeof(real));
        memset(modeDisp_ThisTime, 0, nModeFluid * 3 * sizeof(real));
        modeDisp_LastTime = (real *)malloc(nModeFluid * 3 * sizeof(real));
        memset(modeDisp_LastTime, 0, nModeFluid * 3 * sizeof(real));
        initVelocity = (real *)malloc(nModeFluid * sizeof(real));
        memset(initVelocity, 0, nModeFluid * sizeof(real));

#if !RP_NODE                                                                     // run in host process
        Message("UDF[Host]: r = %d, c = %d, m = %d\n", row, column, nModeFluid); // print size of node coordinates and modal displacement and number of modes

        for (int iMode = 0; iMode < nModeFluid + 1; iMode++)                      // input each modal displacement
            if (fileStatus = read_fluid_coor_mode(nodeCoorDisp, modeFreq, iMode)) // if file or data missing, print error
                Message("UDF[Host]: Error: %d in %d\n", fileStatus, iMode);

        Message("UDF[Host]: Mode frequency\n"); // print mode frequency
        for (int i = 0; i < nModeFluid; i++)
            Message("           %f\n", modeFreq[i]);

        qsort(nodeCoorDisp, row, column * sizeof(double), cmp_node); // sort by node coordinate
        read_paramater();
#endif

        host_to_node_double(nodeCoorDisp, row * column); // broadcast node coordinate and modal displacement to all node process
        host_to_node_double(modeFreq, nModeFluid);       // broadcast mode frequency to node process
        host_to_node_int_2(idFSI, idFluid);
        host_to_node_real(initVelocity, nModeFluid);

#if !RP_HOST // run in node process
        fill_modal_disp(nodeCoorDisp);
#endif
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
    Preprocess();
}

/**
 * @brief construct a new grid motion method
 *
 */
DEFINE_GRID_MOTION(FDM_method, pDomain, dt, time, dTime)
{
    real modeForceBuff[nModeFluid];                              // modal force this time, buffer
    memset(modeDisp_ThisTime, 0, nModeFluid * 3 * sizeof(real)); // allocate memory
    memset(modeForce_ThisTime, 0, nModeFluid * sizeof(real));
    memset(modeForceBuff, 0, nModeFluid * sizeof(real));

#if !RP_NODE
    // calculate modal force on structure
    if (nModeStruct > 0)
    {
        memset(structModeForce, 0, nModeStruct * sizeof(real)); // reset struct mode force
        get_struct_mode_force();                                // get struct mode force
    }
#endif

#if !RP_HOST
    get_fluid_mode_force(pDomain); // get modal force
#endif

    PRF_GRSUM(modeForce_ThisTime, nModeFluid, modeForceBuff); // global summation fo modal force this time
    node_to_host_real(modeForce_ThisTime, nModeFluid);        // send mode force to host process

#if !RP_NODE
    if (nModeStruct > 0)
        for (int i = 0; i < nModeFewer; i++) // add struct force
            modeForce_ThisTime[i] += structModeForce[i];

    wilson_theta(dTime); // wilson theta method
#endif

    host_to_node_real(modeDisp_ThisTime, nModeFluid * 3); // broadcast modal displacement to node process

#if !RP_HOST            // run on node process
    move_grid(pDomain); // move grid
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
        Message("%5.4e ", modeForce_ThisTime[i]);
    Message("\n");

    FILE *fpOutput = fopen("Result.csv", "a");
    if (iTime == 0)
    {
        time_t timep;
        time(&timep);
        fprintf(fpOutput, "Calculation starts with %d processors, at %s", compute_node_count, asctime(localtime(&timep)));
        fprintf(fpOutput, "                Time,");
        for (int i = 0; i < nModeFluid; i++)
        {
            fprintf(fpOutput, "             Force%2d,", i + 1);
        }
        for (int i = 0; i < nModeFluid; i++)
        {
            fprintf(fpOutput, "      Displacement%2d,", i + 1);
            fprintf(fpOutput, "          Velocity%2d,", i + 1);
            fprintf(fpOutput, "      Acceleration%2d,", i + 1);
        }

        fprintf(fpOutput, "\n");
    }
    fprintf(fpOutput, "%20.10f,", CURRENT_TIME);
    for (int i = 0; i < nModeFluid; i++)
    {
        fprintf(fpOutput, "%20.10e,", modeForce_ThisTime[i]);
    }
    for (int i = 0; i < nModeFluid; i++)
    {
        fprintf(fpOutput, "%20.10e,%20.10e,%20.10e,",
                modeDisp_ThisTime[i * 3 + 0],
                modeDisp_ThisTime[i * 3 + 1],
                modeDisp_ThisTime[i * 3 + 2]);
    }
    fprintf(fpOutput, "\n");
    fclose(fpOutput);
#endif

    memcpy(modeDisp_LastTime, modeDisp_ThisTime, nModeFluid * 3 * sizeof(real));
    memset(modeDisp_ThisTime, 0, nModeFluid * 3 * sizeof(real));
    memcpy(modeForce_LastTime, modeForce_ThisTime, nModeFluid * sizeof(real));
    memset(modeForce_ThisTime, 0, nModeFluid * sizeof(real));

    PRF_GSYNC(); // synchronise
    iIter = 1;   // reset index for iteration
    iTime++;     // index for time +1
}

/**
 * @brief clear memories at exit
 *
 */
DEFINE_ON_DEMAND(Clear_memories)
{
    free_memories();
}

/**
 * @brief clear memories at exit
 *
 */
DEFINE_EXECUTE_AT_EXIT(Finish_process)
{
    free_memories();
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
