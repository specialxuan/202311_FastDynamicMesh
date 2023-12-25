/**
 * @file test.c
 * @author SpecialXuan
 * @brief
 * @version 0.1
 * @date 2023-01-13
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <math.h>
#include "udf.h" // note: move udf.h to the last
#define NODE_MATCH_TOLERANCE 1e-6

// define the read_mode()  to read input the array from the .txt file
int read_coor_mode(double *elas_mode, const int row, const int column, const int i_Mode)
{
#if !RP_NODE
    FILE *fpInput = NULL;                           // file pointer
    char inFileName[20] = {0}, line_buf[256] = {0}; // input file name
    if (i_Mode == 0)
        sprintf(inFileName, "mode/NodeCoor.csv");
    else
        sprintf(inFileName, "mode/NodeDisp%d.csv", i_Mode);

    fpInput = fopen(inFileName, "r"); // open the file in the read-only mode
    if (fpInput == NULL)              // file not exists -> print error
    {
        Message(" ---        Error: No file.        ---\n");
        return 1;
    }

    fgets(line_buf, 256, fpInput);
    for (int i = 0; i < row; i++) // fill the array
        for (int j = 0; j < 3; j++)
            if (fscanf(fpInput, "%lf,", elas_mode + i * column + i_Mode * 3 + j) <= 0)
            {
                Message(" ---   Error: Lack of variables.   ---\n From (%d,%d)", i, j);
                fclose(fpInput);
                return -2;
            }

    fclose(fpInput); // close file
#endif

    return 0;
}

int read_row_column(int *row, int *column)
{
#if !RP_NODE
    FILE *fpInput = NULL; // file pointer
    double n_node = 0, n_mode = 0;
    fpInput = fopen("mode/NodeCoor.csv", "r"); // open the file in the read-only mode
    if (fpInput == NULL)                       // file not exists -> print error
    {
        Message(" ---        Error: No file.        ---\n");
        return -1;
    }
    if (fscanf(fpInput, "%lf,", &n_node) <= 0 ||
        fscanf(fpInput, "%lf,", &n_node) <= 0 || // input number of nodes
        fscanf(fpInput, "%lf,", &n_mode) <= 0)   // input number of modes
    {
        Message(" ---   Error: Lack of variables.   ---\n");
        fclose(fpInput);
        return -2;
    }
    *row = (int)n_node;
    *column = ((int)n_mode + 1) * 3;
    fclose(fpInput);
#endif
    return 0;
}

int cmp_node(const void *a, const void *b)
{
    const double *da = (double *)a, *db = (double *)b;
    int xCmp = *(da + 0) - *(db + 0) < -NODE_MATCH_TOLERANCE ? -1 : *(da + 0) - *(db + 0) > NODE_MATCH_TOLERANCE ? 1 : 0;
    int yCmp = *(da + 1) - *(db + 1) < -NODE_MATCH_TOLERANCE ? -1 : *(da + 1) - *(db + 1) > NODE_MATCH_TOLERANCE ? 1 : 0;
    int zCmp = *(da + 2) - *(db + 2) < -NODE_MATCH_TOLERANCE ? -1 : *(da + 2) - *(db + 2) > NODE_MATCH_TOLERANCE ? 1 : 0;
    return xCmp ? xCmp : yCmp ? yCmp : zCmp ? zCmp : 0;
}

static int iter_index = 1; // record the number of the iteration steps
static int time_index = 0; // record the number of the time steps

DEFINE_ON_DEMAND(Preprocess)
{
    int row = 0, column = 0;
    int fileStatus = 0;       // status of input file, 0 is noError, 1 is Error
    double *elas_mode = NULL; // elas_mode are the first 4 mode shapes array of the fluid mesh.
                              // node-No.,x-coor,y-coor,z-coor,shape-information
                              // 66654 row, 16column, Start at 0 when called

    iter_index = 1;
    time_index = 0;

    // RP_Get_Real(char *s)-RP_Get_Real("flow-time")
    // RP_Set_Float(char *s, double v)-RP_Set_Float("flow-time", 0.2)
    RP_Set_Float("flow-time", 0); // set  flow time

#if !RP_NODE
    Message("\n ***      Begin: This is Host      ***\n");

    if (fileStatus = read_row_column(&row, &column))
        Message(" Error: %d\n", fileStatus);
    else
    {
        Message(" r = %d, c = %d\n", row, column);
        elas_mode = (double *)malloc(row * column * sizeof(double)); // allocate memory for elas_mode
        memset(elas_mode, 0, row * column * sizeof(double));         // initialize elas_mode

        for (int i_Mode = 0; i_Mode < column / 3; i_Mode++)
        {
            if (fileStatus = read_coor_mode(elas_mode, row, column, i_Mode))
            {
                Message(" Error: %d in %d\n", fileStatus, i_Mode);
            }
        }

        qsort(elas_mode, row, column * sizeof(double), cmp_node);

        if (fileStatus == 0)
        {
            Message(" ---    The elas_mode array is:    ---\n"); // validation for the initialization of the elas_mode
            for (int i = 0; i < column; i++)                     // print variables
                if (i == 0)
                    Message("%e \t", elas_mode[3 * column + i]);
                else
                    Message("%e \t", elas_mode[3 * column + i]);
            Message(" ---      Validation is done!      ---\n");
        }
    }

    Message(" ***      End:   This is Host      ***\n");
#endif

    host_to_node_int_1(fileStatus);
    if (fileStatus == 0)
    {
        host_to_node_int_2(row, column);
#if !RP_HOST
        elas_mode = (double *)malloc(row * column * sizeof(double)); // allocate memory for elas_mode
        memset(elas_mode, 0, row * column * sizeof(double));         // initialize elas_mode
#endif
        host_to_node_double(elas_mode, row * column);
    }

    #if !RP_HOST // TODO: add ! when compile
        Message("\n +++      Begin: This is Node      +++\n");

        Domain *domain;
        cell_t cell;
        Thread *thread;
        Node *pNode;

        int n = 0, node_count = 0; // total number of node
        double node_coor[3] = {0}, *this_node = NULL;

        char outFileName[20] = {0}; // output file name
        sprintf(outFileName, "outputNode%d.csv", myid);
        FILE *fpOutput = fopen(outFileName, "w+");

        domain = Get_Domain(1); // for single-phase flows, domain_id is 1 and Get_Domain(1) returns the fluid domain pointer
        if (fileStatus == 0)
        {
            Message(" ---    The elas_mode array is:    ---\n"); // validation for the initialization of the elas_mode
            for (int i = 0; i < column; i++)                     // print variables
                if (i == 0)
                    Message("%f \t", elas_mode[3 * column + i]);
                else
                    Message("%f \t", elas_mode[3 * column + i]);
            Message(" ---      Validation is done!      ---\n");
        }

        thread_loop_c(thread, domain)
        {
            begin_c_loop(cell, thread)
            {
                c_node_loop(cell, thread, n)
                {
                    pNode = C_NODE(cell, thread, n);
                    // set the value of the first user define node memory as 0
                    // used to identify if the node has stored the shape information or not
                    N_UDMI(pNode, column) = 0;
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
                    pNode = C_NODE(cell, thread, n);
                    if (N_UDMI(pNode, column) == 0)
                    {
                        node_coor[0] = NODE_X(pNode);
                        node_coor[1] = NODE_Y(pNode);
                        node_coor[2] = NODE_Z(pNode);

                        this_node = (double *)bsearch(node_coor, elas_mode, row, column * sizeof(double), cmp_node);

                        // fprintf(fpOutput, "%d,", nCount + 1);
                        // fprintf(fpOutput, "%f, %f, %f, %f, %f, %f,\n", this_node[0], this_node[1], this_node[2], this_node[3], this_node[4], this_node[5]);

                        if (this_node != NULL)
                        {
                            for (int i = 0; i < column; i++)
                            {
                                N_UDMI(pNode, i) = this_node[i];
                                fprintf(fpOutput, " %f,", N_UDMI(pNode, i));
                            }
                        }
                        else
                        {
                            fprintf(fpOutput, " %f, %f, %f", node_coor[0], node_coor[1], node_coor[2]);
                        }
                        

                        fprintf(fpOutput, "\n");
                        N_UDMI(pNode, column) = 1;
                        node_count++;
                        // if (myid == 0)
                        // {
                        //     Message("%d\n", nCount);
                        // }
                    }

                }
            }
            end_c_loop(cell, thread)
        }
        fclose(fpOutput);
        Message("Total Number of nodes in NODE %d is %d\n", myid, node_count);

        Message(" +++      End:   This is Node      +++\n");
    #endif
}