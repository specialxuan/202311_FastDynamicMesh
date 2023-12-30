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
    int row = 0, column = 0;                 // size of node coordinates and modal displacements, status of input file(0 is noError, 1 is Missing files, 2 is Missing data
    if (read_row_column(&row, &column) == 0) // check if files generated
        Message("UDF: Files Generated\n");
#endif
}

static int iter_index = 1; // record the number of the iteration steps
static int time_index = 0; // record the number of the time steps

/**
 * @brief preprocess for fluid solution
 * 
 */
DEFINE_ON_DEMAND(Preprocess)
{
    int row = 0, column = 0, fileStatus = 0; // size of node coordinates and modal displacements, status of input file(0 is noError, 1 is Missing files, 2 is Missing data
    double *nodeCoorDisp = NULL;             // node coordinates and modal displacements

    iter_index = 1; // initialise iteration index
    time_index = 0; // initialise time index

    // RP_Get_Real(char *s)-RP_Get_Real("flow-time")
    // RP_Set_Float(char *s, double v)-RP_Set_Float("flow-time", 0.2)
    RP_Set_Float("flow-time", 0); // set  flow time

#if !RP_NODE // run in host process
    if (fileStatus = read_row_column(&row, &column) == 0) // input size of node coordinates and modal displacement
    {
        Message("UDF: [Host] r = %d, c = %d\n", row, column);                      // print size of node coordinates and modal displacement
        nodeCoorDisp = (double *)malloc(row * column * sizeof(double)); // allocate memory for elas_mode
        memset(nodeCoorDisp, 0, row * column * sizeof(double));         // initialize elas_mode

        for (int iMode = 0; iMode < column / 3; iMode++)                       // input each modal displacement
            if (fileStatus = read_coor_mode(nodeCoorDisp, row, column, iMode)) // if file or data missing, print error
                Message("UDF: [Host] Error: %d in %d\n", fileStatus, iMode);

        qsort(nodeCoorDisp, row, column * sizeof(double), cmp_node); // sort by node coordinate
    }
    else
        Message("UDF: [Host] Error: %d\n", fileStatus);
#endif

    host_to_node_int_1(fileStatus); // broadcast file status to all node process
    if (fileStatus == 0)            // if no error
    {
        host_to_node_int_2(row, column); // broadcast size to all node process
#if !RP_HOST                             // run in node process
        nodeCoorDisp = (double *)malloc(row * column * sizeof(double)); // allocate memory for nodeCoorDisp
        memset(nodeCoorDisp, 0, row * column * sizeof(double));         // initialize nodeCoorDisp
#endif
        
        host_to_node_double(nodeCoorDisp, row * column); // broadcast node coordinate and modal displacement to all node process
    
#if !RP_HOST // run in node process
    fill_modal_disp(nodeCoorDisp, row, column);
#endif
    }

    free(nodeCoorDisp); 
}

// calculation of the modal aerodynamic force
// and modal displacement for fluid mesh
// Node motion for CFD mesh of the fluid zone
static real m = 1;                                      // modal mass = 1, which means normalization of principal mass
static real c = 0;                                      // modal damping
static real Pi = 3.141592654;                           // Pi
static real theta = 1.4;                                // wilson-theta method, the value of the theta
static int FSI_ID = 19;                                 // record the id of the fsi faces, shown in fluent       // TODO: read from file
static int Fluid_ID = 11;                               // record the id of the fluid cell zone, shown in fluent // TODO: read from file
static real freq[4] = {0.17839, 1.103, 3.0322, 5.8042}; // frequencies of the structure                          // TODO: read from file
static real ini_vel[4] = {0, 0, 0, 0};                  // initial modal velocity
static real mode_force[100000][5] = {0};                // modal force
static real mode_disp[100000][12] = {0};                // modal-displacement,modal-velocity,acceleration repeat

DEFINE_GRID_MOTION(FDM_method, domain, dt, time, dtime)
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