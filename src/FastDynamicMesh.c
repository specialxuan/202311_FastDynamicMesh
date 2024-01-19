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

// static real ini_vel[4] = {0, 0, 0, 0};   // initial modal velocity
// static real mode_force[100000][5] = {0}; // modal force
// static real mode_disp[100000][12] = {0}; // modal-displacement,modal-velocity,acceleration repeat

DEFINE_GRID_MOTION(FDM_method, domain, dt, time, dtime)
{
#if !RP_NODE
    Message("\nUDF: ***      Begin: This is Host      ***\n");

    Message("UDF: ***      End:   This is Host      ***\n");
#endif
#if RP_HOST
    Message("\nUDF: +++      Begin: This is Node      +++\n");

    // Thread *tFSI = Lookup_Thread(domain, idFSI);
    // Thread *tFLD = Lookup_Thread(domain, idFluid);
    // cell_t cellFSI;
    // face_t faceFSI;
    // Node *nodeFSI;
    // real nNodes, *Press, Area[3] = {0};

    // memset(modeForce + iTime * 5, 0, 5 * sizeof(real));

    Message("UDF: +++      End:   This is Node      +++\n");
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