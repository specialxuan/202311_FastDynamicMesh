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

// calculate modal displacement
DEFINE_ON_DEMAND(ModeCalculation)
{
#if !RP_NODE                                        // run on host process
    system("ModeCalculation.bat");                  // execute apdl batch
    FILE *fpTest = fopen("mode/NodeCoor.csv", "r"); // check if modal files are generated
    if (fpTest)
        Message("Mode files generated\n");
    else
        Message("Error: No mode files generated\n");
    fclose(fpTest);
#endif
}

// input node coordinates and modal displacement
int read_coor_mode(double *nodeCoorDisp, const int row, const int column, const int iMode)
{
#if !RP_NODE                                        // run in host process
    char inFileName[20] = {0}, line_buf[256] = {0}; // input file name, ignore first line
    if (iMode == 0)
        sprintf(inFileName, "mode/NodeCoor.csv"); // read coordinate at first time
    else
        sprintf(inFileName, "mode/NodeDisp%d.csv", iMode); // read modal displacement

    FILE *fpInput = fopen(inFileName, "r"); // open the file in the read-only mode
    if (fpInput == NULL)                    // file not exists, print error
    {
        Message(" Error: No file.\n");
        return 1;
    }

    fgets(line_buf, 256, fpInput); // ignore first line
    for (int i = 0; i < row; i++)  // fill the array of node coordinates and modal displacements
        for (int j = 0; j < 3; j++)
            if (fscanf(fpInput, "%lf,", nodeCoorDisp + i * column + iMode * 3 + j) <= 0) // data missing, print error
            {
                Message(" Error: Lack of variables in Mode %d, Node %d, Displacement%d.\n", iMode, i, j);
                fclose(fpInput);
                return 2;
            }

    fclose(fpInput);
#endif

    return 0;
}

// input size of node coordinates and modal displacement
int read_row_column(int *row, int *column)
{
#if !RP_NODE                                         // run in host process
    double nNode = 0, nMode = 0;                     // number of nodes, number of modes
    FILE *fpInput = fopen("mode/NodeCoor.csv", "r"); // open the file in the read-only mode
    if (fpInput == NULL)                             // file not exists, print error
    {
        Message(" ---        Error: No file.        ---\n");
        return 1;
    }
    if (fscanf(fpInput, "%lf,", &nNode) <= 0 || // ignore first number
        fscanf(fpInput, "%lf,", &nNode) <= 0 || // input number of nodes
        fscanf(fpInput, "%lf,", &nMode) <= 0)   // input number of modes
    {
        Message(" ---   Error: Lack of variables.   ---\n");
        fclose(fpInput);
        return 2;
    }
    *row = (int)nNode;              // rows of node coordinates and modal displacement
    *column = ((int)nMode + 1) * 3; // columns of node coordinates and modal displacement
    fclose(fpInput);
#endif

    return 0;
}

// compare nodes in order of x, y, z
int cmp_node(const void *a, const void *b)
{
    const double *da = (double *)a, *db = (double *)b;
    int xCmp = *(da + 0) - *(db + 0) < -NODE_MATCH_TOLERANCE ? -1 : *(da + 0) - *(db + 0) > NODE_MATCH_TOLERANCE ? 1 : 0; // compare x
    int yCmp = *(da + 1) - *(db + 1) < -NODE_MATCH_TOLERANCE ? -1 : *(da + 1) - *(db + 1) > NODE_MATCH_TOLERANCE ? 1 : 0; // compare y
    int zCmp = *(da + 2) - *(db + 2) < -NODE_MATCH_TOLERANCE ? -1 : *(da + 2) - *(db + 2) > NODE_MATCH_TOLERANCE ? 1 : 0; // compare z
    return xCmp ? xCmp : yCmp ? yCmp : zCmp ? zCmp : 0; // if x, then y, then z, then equal
}

static int iter_index = 1; // record the number of the iteration steps
static int time_index = 0; // record the number of the time steps

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
    Message("\n ***      Begin: This is Host      ***\n");

    if (fileStatus = read_row_column(&row, &column)) // input size of node coordinates and modal displacement
        Message(" Error: %d\n", fileStatus);
    else
    {
        Message(" r = %d, c = %d\n", row, column);                      // print size of node coordinates and modal displacement
        nodeCoorDisp = (double *)malloc(row * column * sizeof(double)); // allocate memory for elas_mode
        memset(nodeCoorDisp, 0, row * column * sizeof(double));         // initialize elas_mode

        for (int i_Mode = 0; i_Mode < column / 3; i_Mode++)                     // input each modal displacement
            if (fileStatus = read_coor_mode(nodeCoorDisp, row, column, i_Mode)) // if file or data missing, print error
                Message(" Error: %d in %d\n", fileStatus, i_Mode);

        qsort(nodeCoorDisp, row, column * sizeof(double), cmp_node); // sort by node coordinate

        // if (fileStatus == 0)
        // {
        //     Message(" ---    The elas_mode array is:    ---\n"); // validation for the initialization of the elas_mode
        //     for (int i = 0; i < column; i++)                     // print variables
        //         if (i == 0)
        //             Message("%e \t", elas_mode[3 * column + i]);
        //         else
        //             Message("%e \t", elas_mode[3 * column + i]);
        //     Message(" ---      Validation is done!      ---\n");
        // }
    }

    Message(" ***      End:   This is Host      ***\n");
#endif

    host_to_node_int_1(fileStatus); // broadcast file status to all node process
    if (fileStatus == 0)            // if no error
    {
        host_to_node_int_2(row, column); // broadcast size to all node process
#if !RP_HOST
        nodeCoorDisp = (double *)malloc(row * column * sizeof(double)); // allocate memory for nodeCoorDisp
        memset(nodeCoorDisp, 0, row * column * sizeof(double));         // initialize nodeCoorDisp
#endif
        host_to_node_double(nodeCoorDisp, row * column); // broadcast node coordinate and modal displacement to all node process
    }

#if !RP_HOST // note: add ! when compile, run in node process
    Message("\n +++      Begin: This is Node      +++\n");

    Domain *pDomain; // pointer of domain
    cell_t pCell;    // pointer of cell
    Thread *pThread; // pointer of thread
    Node *pNode;     // pointer of node

    int iNode = 0, node_count = 0, UDMI_column = column - 3; // node index, total number of node, columns of UDMI
    double node_coor[3] = {0}, *this_node = NULL;            // buffer of node coordinate, pointer of this node

    char outFileName[20] = {0};                     // output file name
    sprintf(outFileName, "outputNode%d.csv", myid); // output file name of each node process
    FILE *fpOutput = fopen(outFileName, "w+");      // open output file in write mode

    // if (fileStatus == 0)
    // {
    //     Message(" ---    The elas_mode array is:    ---\n"); // validation for the initialization of the elas_mode
    //     for (int i = 0; i < column; i++)                     // print variables
    //         if (i == 0)
    //             Message("%f \t", elas_mode[3 * column + i]);
    //         else
    //             Message("%f \t", elas_mode[3 * column + i]);
    //     Message(" ---      Validation is done!      ---\n");
    // }

    if (fileStatus == 0) // if no error 
    {    
        pDomain = Get_Domain(1);        // for single-phase flows, domain_id is 1 and Get_Domain(1) returns the fluid domain pointer
        thread_loop_c(pThread, pDomain) // loop thread in domain
        {
            begin_c_loop(pCell, pThread) // loop cell in thread
            {
                c_node_loop(pCell, pThread, iNode) // loop node in cell
                {
                    pNode = C_NODE(pCell, pThread, iNode); // get node pointer
                    N_UDMI(pNode, UDMI_column) = 0;        // last column of UDMI is 0 indicates not set, 1 indicates set
                }
            }
            end_c_loop(pCell, pThread) // finish cell loop
        }
        thread_loop_c(pThread, pDomain) // loop thread in domain
        {
            begin_c_loop(pCell, pThread) // loop cell in thread
            {
                c_node_loop(pCell, pThread, iNode) // loop node in cell
                {
                    pNode = C_NODE(pCell, pThread, iNode); // get node pointer
                    if (N_UDMI(pNode, UDMI_column) == 0)   // if node not set
                    {
                        node_coor[0] = NODE_X(pNode); // get coordinate of this node
                        node_coor[1] = NODE_Y(pNode);
                        node_coor[2] = NODE_Z(pNode);

                        this_node = (double *)bsearch(node_coor, nodeCoorDisp, row, column * sizeof(double), cmp_node); // search for this node

                        if (this_node != NULL)                    // if this node found
                            for (int i = 0; i < UDMI_column; i++) // fill UDMI with modal displacement
                            {
                                N_UDMI(pNode, i) = this_node[i + 3];
                                fprintf(fpOutput, " %10.5e,", N_UDMI(pNode, i)); // output UDMI to file
                            }
                        else
                            fprintf(fpOutput, "Error: no node at (%f, %f, %f)", node_coor[0], node_coor[1], node_coor[2]); // if this node not found, output error in file

                        fprintf(fpOutput, "\n");
                        N_UDMI(pNode, UDMI_column) = 1; // this node set
                        node_count++;                   // count how many nodes in this process
                    }
                }
            }
            end_c_loop(pCell, pThread) // finish cell loop
        }
    }
    fclose(fpOutput);                                                      // close file
    Message("Total Number of nodes in NODE %d is %d\n", myid, node_count); // print how many nodes in this process

    Message(" +++      End:   This is Node      +++\n");
#endif
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
    Message("\n ***      Begin: This is Host      ***\n");

    Message(" ***      End:   This is Host      ***\n");
#endif
#if !RP_HOST
    Message("\n +++      Begin: This is Node      +++\n");

    Message(" +++      End:   This is Node      +++\n");
#endif
}