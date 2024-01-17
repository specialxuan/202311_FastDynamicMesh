/**
 * @file fdmUtils.h
 * @author Special (special.xuan@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2023-12-30
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include <math.h>
#include "udf.h" // note: move udf.h to the last
#define NODE_MATCH_TOLERANCE 1e-6

/**
 * @brief HOST_ONLY input node coordinates and modal displacement
 *
 * @param nodeCoorDisp node coordinates and modal displacements
 * @param modeFreq mode frequency
 * @param row row of nodeCoorDisp
 * @param column column of nodeCoorDisp
 * @param iMode which mode to input
 * @return int
 */
int read_coor_mode(double *nodeCoorDisp, real *modeFreq, const int row, const int column, const int iMode)
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
        Message("UDF[Host]: Error: No file.\n");
        return 1;
    }

    if (iMode > 0)
        fscanf(fpInput, "%lf,", modeFreq + iMode - 1); // read mode frequency
    
    fgets(line_buf, 256, fpInput); // ignore first line
    for (int i = 0; i < row; i++)  // fill the array of node coordinates and modal displacements
        if (fscanf(fpInput, "%lf,", nodeCoorDisp + i * column + iMode * 3 + 0) <= 0 ||
            fscanf(fpInput, "%lf,", nodeCoorDisp + i * column + iMode * 3 + 1) <= 0 ||
            fscanf(fpInput, "%lf,", nodeCoorDisp + i * column + iMode * 3 + 2) <= 0) // data missing, print error
        {
            Message("UDF[Host]: Error: Lack of variables in Mode %d, Node %d.\n", iMode, i);
            fclose(fpInput);
            return 2;
        }

    fclose(fpInput);
#endif

    return 0;
}

/**
 * @brief HOST_ONLY input size of node coordinates and modal displacement
 *
 * @param row row of nodeCoorDisp
 * @param column column of nodeCoorDisp
 * @return int
 */
int read_row_column(int *row, int *column)
{
#if !RP_NODE                                         // run in host process
    double nNode = 0, nMode = 0;                     // number of nodes, number of modes
    FILE *fpInput = fopen("mode/NodeCoor.csv", "r"); // open the file in the read-only mode
    if (fpInput == NULL)                             // file not exists, print error
    {
        Message("UDF[Host]: Error: No file.\n");
        return 1;
    }
    if (fscanf(fpInput, "%lf,", &nNode) <= 0 || // ignore first number
        fscanf(fpInput, "%lf,", &nNode) <= 0 || // input number of nodes
        fscanf(fpInput, "%lf,", &nMode) <= 0)   // input number of modes
    {
        Message("UDF[Host]: Error: Lack of variables.\n");
        fclose(fpInput);
        return 2;
    }
    *row = (int)nNode;              // rows of node coordinates and modal displacement
    *column = ((int)nMode + 1) * 3; // columns of node coordinates and modal displacement
    fclose(fpInput);
#endif

    return 0;
}

/**
 * @brief compare nodes in order of x, y, z
 *
 * @param a pointer of first node
 * @param b pointer of second node
 * @return int
 */
int cmp_node(const void *a, const void *b)
{
    const double *da = (double *)a, *db = (double *)b;
    int xCmp = *(da + 0) - *(db + 0) < -NODE_MATCH_TOLERANCE ? -1 : *(da + 0) - *(db + 0) > NODE_MATCH_TOLERANCE ? 1 : 0; // compare x
    int yCmp = *(da + 1) - *(db + 1) < -NODE_MATCH_TOLERANCE ? -1 : *(da + 1) - *(db + 1) > NODE_MATCH_TOLERANCE ? 1 : 0; // compare y
    int zCmp = *(da + 2) - *(db + 2) < -NODE_MATCH_TOLERANCE ? -1 : *(da + 2) - *(db + 2) > NODE_MATCH_TOLERANCE ? 1 : 0; // compare z
    return xCmp ? xCmp : yCmp ? yCmp : zCmp ? zCmp : 0; // if x, then y, then z, then equal
}

/**
 * @brief NODE_ONLY fill node coordinate and modal displacement into UDMI
 * 
 * @param nodeCoorDisp node coordinate and modal displacement
 * @param row row of nodeCoorDisp
 * @param column column of nodeCoorDisp
 * @return int 
 */
int fill_modal_disp(const double *nodeCoorDisp, const int row, const int column)
{
    Domain *pDomain; // pointer of domain
    cell_t pCell;    // pointer of cell
    Thread *pThread; // pointer of thread
    Node *pNode;     // pointer of node

    int iNode = 0, nodeCount = 0, UDMIColumn = column - 3; // node index, total number of node, columns of UDMI
    double node_coor[3] = {0}, *thisNode = NULL;           // buffer of node coordinate, pointer of this node

    char outFileName[20] = {0};                     // output file name
    sprintf(outFileName, "outputNode%d.csv", myid); // output file name of each node process
    FILE *fpOutput = fopen(outFileName, "w+");      // open output file in write mode

    pDomain = Get_Domain(1);        // for single-phase flows, domain_id is 1 and Get_Domain(1) returns the fluid domain pointer
    thread_loop_c(pThread, pDomain) // loop thread in domain
    {
        begin_c_loop_int_ext(pCell, pThread) // loop cell in thread
        {
            c_node_loop(pCell, pThread, iNode) // loop node in cell
            {
                pNode = C_NODE(pCell, pThread, iNode); // get node pointer
                N_UDMI(pNode, UDMIColumn) = 0;         // last column of UDMI is 0 indicates not set, 1 indicates set
            }
        }
        end_c_loop_int_ext(pCell, pThread) // finish cell loop
    }
    thread_loop_c(pThread, pDomain) // loop thread in domain
    {
        begin_c_loop_int_ext(pCell, pThread) // loop cell in thread
        {
            c_node_loop(pCell, pThread, iNode) // loop node in cell
            {
                pNode = C_NODE(pCell, pThread, iNode); // get node pointer
                if (N_UDMI(pNode, UDMIColumn) == 0)    // if node not set
                {
                    node_coor[0] = NODE_X(pNode); // get coordinate of this node
                    node_coor[1] = NODE_Y(pNode);
                    node_coor[2] = NODE_Z(pNode);

                    thisNode = (double *)bsearch(node_coor, nodeCoorDisp, row, column * sizeof(double), cmp_node); // search for this node

                    if (thisNode != NULL)                    // if this node found
                        for (int i = 0; i < UDMIColumn; i++) // fill UDMI with modal displacement
                        {
                            N_UDMI(pNode, i) = thisNode[i + 3];
                            // fprintf(fpOutput, " %10.5e,", N_UDMI(pNode, i)); // output UDMI to file
                        }
                    else
                        fprintf(fpOutput, "Error: no node at (%f, %f, %f)", node_coor[0], node_coor[1], node_coor[2]); // if this node not found, output error in file

                    fprintf(fpOutput, "\n");
                    N_UDMI(pNode, UDMIColumn) = 1; // this node set
                    nodeCount++;                   // count how many nodes in this process
                }
            }
        }
        end_c_loop_int_ext(pCell, pThread) // finish cell loop
    }
    fclose(fpOutput);                                                                // close file
    Message("UDF[Node]: Total Number of nodes in NODE %d is %d\n", myid, nodeCount); // print how many nodes in this process

    return 0;
}