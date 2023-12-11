/**
 * @file test.c
 * @author Bronya19c
 * @brief
 * @version 0.1
 * @date 2023-01-13
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <math.h>
#include "udf.h" // note: move udf.h to last

static int iter_index = 1; // record the number of the iteration steps
static int time_index = 0; // record the number of the time steps

DEFINE_ON_DEMAND(Preprocess)
{
    FILE *fp = NULL;

    Domain *domain;
    cell_t c;
    Thread *t;
    Node *v;
    int row = 66654, column = 16;
    int n = 0, i = 0, count = 0; // record the number of total nodes
    double *elas_mode = NULL;    // elas_mode are the first 4 mode shapes array of the fluid mesh.
                              // node-No.,x-coor,y-coor,z-coor,shape-information
                              // 66654 row, 16column, Start at 0 when called

    iter_index = 1;
    time_index = 0;

    // RP_Get_Real(char *s)-RP_Get_Real("flow-time")
    // RP_Set_Float(char *s, double v)-RP_Set_Float("flow-time", 0.2)
    RP_Set_Float("flow-time", 0); // set  flow time

#if !RP_NODE
    elas_mode = (double *)malloc(row * column * sizeof(double)); // allocate memory for elas_mode
    memset(elas_mode, 0, row * column * sizeof(double));         // initialize elas_mode

    Message("iteration = %d, time = %d\n", iter_index, time_index);
    Message("hello world!\n");
#endif

#if !RP_HOST
    domain = Get_Domain(1); // for single-phase flows, domain_id is 1 and Get_Domain(1) returns the fluid domain pointer
    Message("\n ---         This is Node          ---\n");
#endif
}