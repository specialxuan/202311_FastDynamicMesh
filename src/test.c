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
#include "udf.h" // note: move udf.h to the last

// define the read_mode()  to read input the array from the .txt file
int read_my_mode(double *elas_mode, const int row, const int column)
{
#if !RP_NODE
    FILE *fp = NULL;  // file pointer
    int i = 0, j = 0; // indexes

    fp = fopen("elas_mode.txt", "r"); // open the file in the read-only mode
    if (fp == NULL)                   // file not exists -> print error
    {
        Message(" ---        Error: No file.        ---\n");
        return 1;
    }

    for (i = 0; i < row; i++) // fill the array
        for (j = 0; j < column; j++)
            if (fscanf(fp, "%lf", elas_mode + i * column + j) <= 0)
            {
                Message(" ---   Error: Lack of variables.   ---\n From (%d,%d)", i, j);
                fclose(fp);
                return 1;
            }

    fclose(fp); // close file

    Message(" ---    The elas_mode array is:    ---\n"); // validation for the initialization of the elas_mode
    for (j = 0; j < column; j++)                         // print variables
        if (j == 0)
            Message("%f \t", elas_mode[(row - 1) * column + j]);
        else
            Message("%f \t", elas_mode[(row - 1) * column + j]);
    Message(" ---      Validation is done!      ---\n");
#endif

    return 0;
}

static int iter_index = 1; // record the number of the iteration steps
static int time_index = 0; // record the number of the time steps

DEFINE_ON_DEMAND(Preprocess)
{
    FILE *fp = NULL;

    int row = 66654, column = 16;
    int n = 0, i = 0, nCount = 0, fileStatus = 0; // record the number of total nodes
    double *elas_mode = NULL;                    // elas_mode are the first 4 mode shapes array of the fluid mesh.
                                                 // node-No.,x-coor,y-coor,z-coor,shape-information
                                                 // 66654 row, 16column, Start at 0 when called

    iter_index = 1;
    time_index = 0;

    // RP_Get_Real(char *s)-RP_Get_Real("flow-time")
    // RP_Set_Float(char *s, double v)-RP_Set_Float("flow-time", 0.2)
    RP_Set_Float("flow-time", 0); // set  flow time

    elas_mode = (double *)malloc(row * column * sizeof(double)); // allocate memory for elas_mode
    memset(elas_mode, 0, row * column * sizeof(double));         // initialize elas_mode

#if !RP_NODE
    Message("\n ***      Begin: This is Host      ***\n");

    if (fileStatus = read_my_mode(elas_mode, row, column))
        Message(" ---      Error: read_my_mode      ---\n");

    Message(" ***      End:   This is Host      ***\n");
#endif

    host_to_node_int_1(fileStatus);
    host_to_node_double(elas_mode, row * column);

#if !RP_HOST // TODO: add ! when compile
    Message("\n +++      Begin: This is Node      +++\n");

    Domain *domain;
    cell_t cell;
    Thread *thread;
    Node *node;
    double *nCoor = (double *)malloc(row * 4 * sizeof(double));
    memset(nCoor, 0, row * 4 * sizeof(double));

    domain = Get_Domain(1); // for single-phase flows, domain_id is 1 and Get_Domain(1) returns the fluid domain pointer
    if (fileStatus == 0)
    {
        Message(" ---    The elas_mode array is:    ---\n"); // validation for the initialization of the elas_mode
        for (i = 0; i < column; i++)                         // print variables
            if (i == 0)
                Message("%f \t", elas_mode[(row - 1) * column + i]);
            else
                Message("%f \t", elas_mode[(row - 1) * column + i]);
        Message(" ---      Validation is done!      ---\n");
    }
    // NODE_X(node);

    thread_loop_c(thread, domain)
    {
        begin_c_loop(cell, thread)
        {
            c_node_loop(cell, thread, n)
            {
                node = C_NODE(cell, thread, n);
                // set the value of the first user define node memory as 0
                // used to identify if the node has stored the shape information or not
                N_UDMI(node, 12) = 0;
            }
        }
        end_c_loop(cell, thread)
    }
    thread_loop_c(thread, domain)
    {
        begin_c_loop(cell, thread)
        {
            c_node_loop(cell, thread, n)
            {
                node = C_NODE(cell, thread, n);
                if (N_UDMI(node, 12) == 0)
                {
                    N_UDMI(node, 12) = 1;
                    nCount++;
                }
            }
        }
        end_c_loop(cell, thread)
    }
    Message("Total Number of nodes in NODE %d is %d\n", myid, nCount);

    Message(" +++      End:   This is Node      +++\n");
#endif
}