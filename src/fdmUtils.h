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

#if RP_3D
#define N_DOF_PER_NODE 3
#endif
#if RP_2D
#define N_DOF_PER_NODE 2
#endif

static int iIter = 1; // record the number of the iteration steps
static int iTime = 0; // record the number of the time steps

static int nNode = 0;
static int nMode = 0;
static real *TimeSeq = NULL;      // time sequence
static real *modeFreq = NULL;     // frequencies of the structure // TODO: read from file
static real *modeForce = NULL;    // modal force
static real *modeDisp = NULL;     // modal-displacement,modal-velocity,acceleration repeat
static real *initVelocity = NULL; // initial modal velocity

static int idFSI = 19;   // record the id of the fsi faces, shown in fluent       // TODO: read from file
static int idFluid = 11; // record the id of the fluid cell zone, shown in fluent // TODO: read from file

/**
 * @brief compare nodes in order of x, y, z
 *
 * @param a pointer of first node
 * @param b pointer of second node
 * @return int
 */
int cmp_node(const void *const a, const void *const b)
{
    const double *da = (double *)a, *db = (double *)b;
    int xCmp = *(da + 0) - *(db + 0) < -NODE_MATCH_TOLERANCE ? -1 : *(da + 0) - *(db + 0) > NODE_MATCH_TOLERANCE ? 1 : 0; // compare x
    int yCmp = *(da + 1) - *(db + 1) < -NODE_MATCH_TOLERANCE ? -1 : *(da + 1) - *(db + 1) > NODE_MATCH_TOLERANCE ? 1 : 0; // compare y
    int zCmp = *(da + 2) - *(db + 2) < -NODE_MATCH_TOLERANCE ? -1 : *(da + 2) - *(db + 2) > NODE_MATCH_TOLERANCE ? 1 : 0; // compare z
    return xCmp ? xCmp : yCmp ? yCmp : zCmp ? zCmp : 0; // if x, then y, then z, then equal
}

#if !RP_NODE
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
int read_coor_mode(double *const nodeCoorDisp, real *const modeFreq, const int iMode)
{
    const int row = nNode, column = (nMode + 1) * N_DOF_PER_NODE;
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
        if (fscanf(fpInput, "%lf,", nodeCoorDisp + i * column + iMode * N_DOF_PER_NODE + 0) <= 0 ||
            fscanf(fpInput, "%lf,", nodeCoorDisp + i * column + iMode * N_DOF_PER_NODE + 1) <= 0 ||
            fscanf(fpInput, "%lf,", nodeCoorDisp + i * column + iMode * N_DOF_PER_NODE + 2) <= 0) // data missing, print error
        {
            Message("UDF[Host]: Error: Lack of variables in Mode %d, Node %d.\n", iMode, i);
            fclose(fpInput);
            return 2;
        }

    fclose(fpInput);

    return 0;
}

/**
 * @brief HOST_ONLY input size of node coordinates and modal displacement
 *
 * @return int
 */
int read_nodes_modes()
{
    double nNodeFromFile = 0, nModeFromFile = 0;     // number of nodes, number of modes
    FILE *fpInput = fopen("mode/NodeCoor.csv", "r"); // open the file in the read-only mode
    if (fpInput == NULL)                             // file not exists, print error
    {
        Message("UDF[Host]: Error: No file.\n");
        return 1;
    }
    if (fscanf(fpInput, "%lf,", &nNodeFromFile) <= 0 || // ignore first number
        fscanf(fpInput, "%lf,", &nNodeFromFile) <= 0 || // input number of nodes
        fscanf(fpInput, "%lf,", &nModeFromFile) <= 0)   // input number of modes
    {
        Message("UDF[Host]: Error: Lack of variables.\n");
        fclose(fpInput);
        return 2;
    }
    nNode = (int)nNodeFromFile; // rows of node coordinates and modal displacement
    nMode = (int)nModeFromFile; // columns of node coordinates and modal displacement
    fclose(fpInput);

    return 0;
}

/**
 * @brief wilson theta method
 *
 * @param modeForceThisTime modal force at this time
 * @param modeForceLastTime modal force at last time
 * @param modeDispThisTime modal displacement at this time
 * @param modeDispLastTime modal displacement at last time
 * @param dTime time gap
 * @return int
 */
int wilson_theta(const real *const modeForceThisTime,
                 const real *const modeForceLastTime,
                 real *const modeDispThisTime,
                 const real *const modeDispLastTime,
                 const real dTime)
{
    static const real Mass = 1;    // modal mass = 1, which means normalization of principal mass
    static const real Damp = 0;    // modal damping
    static const real Theta = 1.4; // wilson-theta method, the value of the theta

    if (iTime == 0) // initiate wilson-theta method
    {
        for (int i = 0; i < nMode; i++)
        {
            modeDisp[i * N_DOF_PER_NODE + 0] = 0;
            modeDisp[i * N_DOF_PER_NODE + 1] = initVelocity[i];
            modeDisp[i * N_DOF_PER_NODE + 2] = modeForceThisTime[i];
        }
    }
    else
    {
        for (int i = 0; i < nMode; i++)
        {
            real dis = 0, vel = 0, acc = 0;
            const real disLast = modeDispLastTime[i * N_DOF_PER_NODE + 0],
                       velLast = modeDispLastTime[i * N_DOF_PER_NODE + 1],
                       accLast = modeDispLastTime[i * N_DOF_PER_NODE + 2];

            // calculate modal displacement, acceleration, velocity using wilson-theta method
            dis = modeForceLastTime[i] + Theta * (modeForceThisTime[i] - modeForceLastTime[i]);
            dis += Mass * (6 / SQR(Theta * dTime) * disLast + 6 / (Theta * dTime) * velLast + 2 * accLast);
            dis += Damp * (3 / (Theta * dTime) * disLast + 2 * velLast + 0.5 * Theta * dTime * accLast);
            dis /= 6 * Mass / SQR(Theta * dTime) + 3 * Damp / (Theta * dTime) + SQR(2 * M_PI * modeFreq[i]);
            // calculate acceleration
            acc = 6 / (SQR(Theta * dTime) * Theta) * (dis - disLast) - 6 / (SQR(Theta) * dTime) * velLast + (1 - 3 / Theta) * accLast;
            // calculate velocity
            vel = velLast + 0.5 * dTime * (acc + accLast);
            // calculate displacement
            dis = disLast + dTime * velLast + SQR(dTime) / 6 * (acc + 2 * accLast);

            modeDispThisTime[i * N_DOF_PER_NODE + 0] = dis;
            modeDispThisTime[i * N_DOF_PER_NODE + 1] = vel;
            modeDispThisTime[i * N_DOF_PER_NODE + 2] = acc;
        }
    }

    return 0;
}

#endif

#if !RP_HOST
/**
 * @brief NODE_ONLY fill node coordinate and modal displacement into UDMI
 *
 * @param nodeCoorDisp node coordinate and modal displacement
 * @param row row of nodeCoorDisp
 * @param column column of nodeCoorDisp
 * @return int
 */
int fill_modal_disp(const double *const nodeCoorDisp)
{
    Domain *pDomain; // pointer of domain
    cell_t pCell;    // pointer of cell
    Thread *pThread; // pointer of thread
    Node *pNode;     // pointer of node

    const int row = nNode, column = (nMode + 1) * N_DOF_PER_NODE;
    int iNode = 0, nodeCount = 0, UDMIColumn = column - N_DOF_PER_NODE; // node index, total number of node, columns of UDMI
    double node_coor[N_DOF_PER_NODE] = {0}, *thisNode = NULL;           // buffer of node coordinate, pointer of this node

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
                            N_UDMI(pNode, i) = thisNode[i + N_DOF_PER_NODE];
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

/**
 * @brief get mode force object
 *
 * @param modeForce modal force at this time
 * @param pDomain pointer of domain
 * @return int
 */
int get_mode_force(real *const modeForce, Domain *const pDomain)
{
    Thread *pThread = Lookup_Thread(pDomain, idFSI);
    face_t pFace;
    Node *pNode;
    real AreaVector[N_DOF_PER_NODE] = {0}, Press = 0;
    int nNodePerFace = 0, iNode = 0;

    begin_f_loop(pFace, pThread)
    {
        F_AREA(AreaVector, pFace, pThread);
        Press = F_P(pFace, pThread);
        nNodePerFace = F_NNODES(pFace, pThread);
        f_node_loop(pFace, pThread, iNode)
        {
            pNode = F_NODE(pFace, pThread, iNode);
            for (int i = 0; i < nMode; i++)
                modeForce[i] += 1 / nNodePerFace * Press * (
                AreaVector[0] * N_UDMI(pNode, i * N_DOF_PER_NODE + 0) + 
                AreaVector[1] * N_UDMI(pNode, i * N_DOF_PER_NODE + 1) + 
                AreaVector[2] * N_UDMI(pNode, i * N_DOF_PER_NODE + 2));
        }
    }
    end_f_loop(pFace, pThread);

    return 0;
}

/**
 * @brief move grid
 *
 * @param modeDispThisTime modal displacement at this time
 * @param modeDispLastTime modal displacement at last time
 * @param pDomain
 * @return int
 */
int move_grid(const real *const modeDispThisTime,
              const real *const modeDispLastTime,
              Domain *const pDomain)
{
    Thread *pThread = Lookup_Thread(pDomain, idFluid);
    cell_t pCell;
    Node *pNode;
    real dispUpdate[N_DOF_PER_NODE] = {0}, deltaDisp = 0;
    int iNode = 0;

    begin_c_loop_int_ext(pCell, pThread)
    {
        c_node_loop(pCell, pThread, iNode)
        {
            pNode = C_NODE(pCell, pThread, iNode);
            if (NODE_POS_NEED_UPDATE(pNode))
            {
                memset(dispUpdate, 0, N_DOF_PER_NODE * sizeof(real));
                for (int i = 0; i < nMode; i++)
                {
                    deltaDisp = modeDispThisTime[N_DOF_PER_NODE * i] - modeDispLastTime[N_DOF_PER_NODE * i];
                    dispUpdate[0] = deltaDisp * N_UDMI(pNode, i * N_DOF_PER_NODE + 0);
                    dispUpdate[1] = deltaDisp * N_UDMI(pNode, i * N_DOF_PER_NODE + 1);
                    dispUpdate[2] = deltaDisp * N_UDMI(pNode, i * N_DOF_PER_NODE + 2);
                }
                NV_V(NODE_COORD(pNode), +=, dispUpdate);
                NODE_POS_UPDATED(pNode);
            }
        }
    }
    end_c_loop_int_ext(pCell, pThread)

    return 0;
}

#endif
