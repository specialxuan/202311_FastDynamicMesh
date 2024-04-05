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
#include <udf.h> // note: move udf.h to the last

// node match tolerance
#define NODE_MATCH_TOLERANCE 4e-7

// number of dof per node
#if RP_3D
#define N_DOF_PER_NODE 3
#endif
#if RP_2D
#define N_DOF_PER_NODE 2
#endif

static int iIter = 1; // record the number of the iteration steps
static int iTime = 0; // record the number of the time steps

static int nNodeFluid = 0;            // number of nodes in fluid
static int nModeFluid = 0;            // number of modes in fluid
static int nModeFewer = 0;            // number of modes in fluid
static int nNodeStruct = 0;           // number of nodes in fluid
static int nModeStruct = 0;           // number of modes in fluid
static double *structModeDisp = NULL; // structure modal displacement
static double *structStiff = NULL;    // structure stiffness
static double *structDamp = NULL;     // structure damping
static double *structLoad = NULL;    // structure forces
static real *TimeSeq = NULL;          // time sequence
static real *modeFreq = NULL;         // frequencies of the structure
static real *modeForce = NULL;        // modal force
static real *modeForceStruct = NULL;        // modal force
static real *modeDisp = NULL;         // modal-displacement,modal-velocity,acceleration repeat
static real *initVelocity = NULL;     // initial modal velocity

static int idFSI = 23;   // record the id of the fsi faces, shown in fluent
static int idFluid = 14; // record the id of the fluid cell zone, shown in fluent

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
 * @brief input number of nodes and modes of structure
 *
 * @return int
 */
int read_struct_nodes_modes()
{
    double nNodeFromFile = 0, nModeFromFile = 0;     // number of nodes, number of modes
    char inFileName[30] = "mode/StructNodeCoor.csv"; // input file name
    FILE *fpInput = fopen(inFileName, "r");          // open file read only
    if (fpInput == NULL)                             // file not exists print error
    {
        Message("UDF[Host]: Error: No Struct files.\n");
        return 1;
    }
    if (fscanf(fpInput, "%lf,", &nNodeFromFile) <= 0 || // ignore first number
        fscanf(fpInput, "%lf,", &nNodeFromFile) <= 0 || // input number of nodes
        fscanf(fpInput, "%lf,", &nModeFromFile) <= 0)   // input number of modes
    {
        Message("UDF[Host]: Error: Lack of Struct variables.\n");
        fclose(fpInput);
        return 2;
    }
    nNodeStruct = (int)nNodeFromFile; // number of nodes of structure
    nModeStruct = (int)nModeFromFile; // number of modes of structure
    fclose(fpInput);

    return 0;
}

/**
 * @brief input node coordinates and modal displacement
 *
 * @param structModeDisp node coordinates and modal displacements
 * @param row row of matrix
 * @param column column of matrix
 * @param iMode which mode to input
 * @return int
 */
int read_struct_coor_mode(double *const structModeDisp,
                          const int row,
                          const int column,
                          const int iMode)
{
    char inFileName[30] = {0}, line_buf[256] = {0}; // input file name, ignore first line
    if (iMode == 0)
        sprintf(inFileName, "mode/StructNodeCoor.csv"); // read coordinate at iMode == 0
    else
        sprintf(inFileName, "mode/StructNodeDisp%d.csv", iMode); // read modal displacement

    FILE *fpInput = fopen(inFileName, "r"); // open file read only
    if (fpInput == NULL)                    // file not exists print error
    {
        Message("UDF[Host]: Error: No Struct files.\n");
        return 1;
    }

    fgets(line_buf, 256, fpInput); // ignore first line
    for (int i = 0; i < row; i++)  // fill the array of node coordinates and modal displacements
        if (fscanf(fpInput, "%lf,", structModeDisp + i * column + (iMode - 1) * N_DOF_PER_NODE + 0) <= 0 ||
            fscanf(fpInput, "%lf,", structModeDisp + i * column + (iMode - 1) * N_DOF_PER_NODE + 1) <= 0 ||
            fscanf(fpInput, "%lf,", structModeDisp + i * column + (iMode - 1) * N_DOF_PER_NODE + 2) <= 0) // data missing, print error
        {
            Message("UDF[Host]: Error: Lack of variables in Mode %d, Node %d.\n", iMode, i);
            fclose(fpInput);
            return 2;
        }

    fclose(fpInput);

    return 0;
}

/**
 * @brief input structure stiffness and damping matrices
 *
 * @param structStiff structure stiffness
 * @param structDamp structure damping
 * @param row row of matrices
 * @param column column of matrices
 * @return int
 */
int read_struct_stiff_damp_load(double *const structStiff,
                                double *const structDamp,
                                double *const structLoad,
                                const int row,
                                const int column)
{
    char line_buf[256] = {0};                           // ignore first line
    FILE *fpInput = fopen("mode/StructStiff.csv", "r"); // open file read only
    if (fpInput == NULL)                                // file not exists print error
    {
        Message("UDF[Host]: Error: No StructStiff file.\n");
        return 1;
    }
    fgets(line_buf, 256, fpInput); // ignore first line
    for (int i = 0; i < row; i++)  // fill the array of node coordinates and modal displacements
        if (fscanf(fpInput, "%lf,", structStiff + i * column + 0) <= 0 ||
            fscanf(fpInput, "%lf,", structStiff + i * column + 1) <= 0 ||
            fscanf(fpInput, "%lf,", structStiff + i * column + 2) <= 0) // data missing, print error
        {
            Message("UDF[Host]: Error: Lack of stiffness in Node %d.\n", i);
            fclose(fpInput);
            return 2;
        }
    fclose(fpInput);

    fpInput = fopen("mode/StructDamp.csv", "r"); // open file read only
    if (fpInput == NULL)                         // file not exists print error
    {
        Message("UDF[Host]: Error: No StructDamp file.\n");
        return 1;
    }
    fgets(line_buf, 256, fpInput); // ignore first line
    for (int i = 0; i < row; i++)  // fill the array of node coordinates and modal displacements
        if (fscanf(fpInput, "%lf,", structDamp + i * column + 0) <= 0 ||
            fscanf(fpInput, "%lf,", structDamp + i * column + 1) <= 0 ||
            fscanf(fpInput, "%lf,", structDamp + i * column + 2) <= 0) // data missing, print error
        {
            Message("UDF[Host]: Error: Lack of damp in Node %d.\n", i);
            fclose(fpInput);
            return 2;
        }
    fclose(fpInput);

    fpInput = fopen("mode/StructLoad.csv", "r"); // open file read only
    if (fpInput == NULL)                         // file not exists print error
    {
        Message("UDF[Host]: Error: No StructLoad file.\n");
        return 1;
    }
    fgets(line_buf, 256, fpInput); // ignore first line
    for (int i = 0; i < row; i++)  // fill the array of node coordinates and modal displacements
        if (fscanf(fpInput, "%lf,", structLoad + i * column + 0) <= 0 ||
            fscanf(fpInput, "%lf,", structLoad + i * column + 1) <= 0 ||
            fscanf(fpInput, "%lf,", structLoad + i * column + 2) <= 0) // data missing, print error
        {
            Message("UDF[Host]: Error: Lack of damp in Node %d.\n", i);
            fclose(fpInput);
            return 2;
        }
    fclose(fpInput);

    return 0;
}

/**
 * @brief input size of node coordinates and modal displacement
 *
 * @return int
 */
int read_fluid_nodes_modes()
{
    double nNodeFromFile = 0, nModeFromFile = 0;     // number of nodes, number of modes
    FILE *fpInput = fopen("mode/FluidNodeCoor.csv", "r"); // open the file in the read-only mode
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
    nNodeFluid = (int)nNodeFromFile; // rows of node coordinates and modal displacement
    nModeFluid = (int)nModeFromFile; // columns of node coordinates and modal displacement
    fclose(fpInput);
   
    return 0;
}

/**
 * @brief input node coordinates and modal displacement
 *
 * @param nodeCoorDisp node coordinates and modal displacements
 * @param modeFreq mode frequency
 * @param row row of nodeCoorDisp
 * @param column column of nodeCoorDisp
 * @param iMode which mode to input
 * @return int
 */
int read_fluid_coor_mode(double *const nodeCoorDisp, real *const modeFreq, const int iMode)
{
    const int row = nNodeFluid, column = (nModeFluid + 1) * N_DOF_PER_NODE;
    char inFileName[20] = {0}, line_buf[256] = {0}; // input file name, ignore first line
    if (iMode == 0)
        sprintf(inFileName, "mode/FluidNodeCoor.csv"); // read coordinate at first time
    else
        sprintf(inFileName, "mode/FluidNodeDisp%d.csv", iMode); // read modal displacement

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
 * @brief Get structural modal force
 *
 * @param modeForceStruct structural modal force
 * @return int
 */
int get_struct_mode_force(real *const modeForceStruct)
{
    if (iTime <= 1) // skip first time step
        return 0;

    const real Dis = modeDisp[(iTime - 1) * nModeFluid * 3 + 0]; // modal displacement last time
    const real Vel = modeDisp[(iTime - 1) * nModeFluid * 3 + 1]; // modal displacement last time
    double *Stiff = (double *)malloc(nModeStruct * sizeof(double));
    memset(Stiff, 0, nModeStruct * sizeof(double));
    double *Damp = (double *)malloc(nModeStruct * sizeof(double));
    memset(Damp, 0, nModeStruct * sizeof(double));
    double *Load = (double *)malloc(nModeStruct * sizeof(double));
    memset(Load, 0, nModeStruct * sizeof(double));

    for (int iMode = 0; iMode < nModeStruct; iMode++)
    {
        for (int iNode = 0; iNode < nNodeStruct; iNode++)
        {
            int M = iNode * nModeStruct * N_DOF_PER_NODE + iMode * N_DOF_PER_NODE, N = iNode * N_DOF_PER_NODE;
            // if (iMode == nModeStruct -1 && iNode == nNodeStruct - 1)
            // {
            //     Message("M = %d, N = %d\n", M, N);
            // }
            
            Stiff[iMode] += structStiff[N + 0] * structModeDisp[N + 0] * structModeDisp[M + 0] +
                            structStiff[N + 1] * structModeDisp[N + 1] * structModeDisp[M + 1] +
                            structStiff[N + 2] * structModeDisp[N + 2] * structModeDisp[M + 2];
            Damp[iMode] += structDamp[N + 0] * structModeDisp[N + 0] * structModeDisp[M + 0] +
                           structDamp[N + 1] * structModeDisp[N + 1] * structModeDisp[M + 1] +
                           structDamp[N + 2] * structModeDisp[N + 2] * structModeDisp[M + 2];
            Load[iMode] += structLoad[N + 0] * structModeDisp[M + 0] +
                           structLoad[N + 1] * structModeDisp[M + 1] +
                           structLoad[N + 2] * structModeDisp[M + 2];
        }
        modeForceStruct[iMode] = (real)(Dis * Stiff[iMode] + Vel * Damp[iMode] + Load[iMode]);
    }
\
    free(Stiff);
    free(Damp);
    free(Load);
    return 0;
}

/**
 * @brief wilson theta method
 *
 * @param modeForceThisTime modal force at this time
 * @param modeDispThisTime modal displacement at this time
 * @param dTime time gap
 * @return int
 */
int wilson_theta(const real *const modeForceThisTime,
                 real *const modeDispThisTime,
                 const real dTime)
{
    static const real Mass = 1;    // modal mass = 1, which means normalization of principal mass
    static const real Damp = 0;    // modal damping
    static const real Theta = 1.4; // wilson-theta method, the value of the theta

    if (iTime <= 1) // initiate wilson-theta method
    {
        for (int i = 0; i < nModeFluid; i++)
        {
            modeDispThisTime[i * 3 + 0] = 0;
            modeDispThisTime[i * 3 + 1] = initVelocity[i];
            modeDispThisTime[i * 3 + 2] = modeForceThisTime[i];
        }
    }
    else
    {
        const real *const modeDispLastTime = modeDisp + (iTime - 1) * nModeFluid * 3;
        const real *const modeForceLastTime = modeForce + (iTime - 1) * nModeFluid;
        for (int i = 0; i < nModeFluid; i++)
        {
            real dis = 0, vel = 0, acc = 0;
            const real disLast = modeDispLastTime[i * 3 + 0],
                       velLast = modeDispLastTime[i * 3 + 1],
                       accLast = modeDispLastTime[i * 3 + 2];

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

            modeDispThisTime[i * 3 + 0] = dis;
            modeDispThisTime[i * 3 + 1] = vel;
            modeDispThisTime[i * 3 + 2] = acc;
        }
    }

    return 0;
}

#endif

#if !RP_HOST
/**
 * @brief fill node coordinate and modal displacement into UDMI
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

    const int row = nNodeFluid, column = (nModeFluid + 1) * N_DOF_PER_NODE;
    int iNode = 0, nodeCount = 0, UDMIColumn = column - N_DOF_PER_NODE; // node index, total number of node, columns of UDMI
    double node_coor[N_DOF_PER_NODE] = {0}, *thisNode = NULL;           // buffer of node coordinate, pointer of this node

    char outFileName[30] = {0};                     // output file name
    sprintf(outFileName, "FluidOutputNode%d.csv", myid); // output file name of each node process
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
                            fprintf(fpOutput, "%f,", N_UDMI(pNode, i)); // if this node not found, output error in file
                        }
                    else
                    {
                        fprintf(fpOutput, "Error: no node at (%f, %f, %f)", node_coor[0], node_coor[1], node_coor[2]); // if this node not found, output error in file
                        Message("UDF[Node]: Error: no node at (%f, %f, %f)\n", node_coor[0], node_coor[1], node_coor[2]);
                    }

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
 * @param modeForceThisTime modal force at this time
 * @param pDomain pointer of domain
 * @return int
 */
int get_fluid_mode_force(real *const modeForceThisTime, Domain *const pDomain)
{
    Thread *pThread = Lookup_Thread(pDomain, idFSI);  // pointer of thread
    face_t pFace;                                     // pointer of face
    Node *pNode;                                      // pointer of node
    real AreaVector[N_DOF_PER_NODE] = {0}, Press = 0; // normal vector of area, pressure
    int nNodePerFace = 0, iNode = 0;                  // number of node per face, node index
    static int modeForceCount = 0;                    // modal force counter

    begin_f_loop(pFace, pThread) // begin face loop
    {
        F_AREA(AreaVector, pFace, pThread);      // get normal vector
        Press = F_P(pFace, pThread);             // get pressure
        nNodePerFace = F_NNODES(pFace, pThread); // get number of nodes per face
        f_node_loop(pFace, pThread, iNode)       // begin node loop
        {
            if (PRINCIPAL_FACE_P(pFace, pThread)) // if face belong to this process
            {
                pNode = F_NODE(pFace, pThread, iNode); // get this node
                // if(myid == 0 && modeForceCount > 49 && modeForceCount % 50 == 0)
                //     Message("UDF[Node]: Modal Force: ");
                for (int i = 0; i < nModeFluid; i++) // for each mode
                {
                    modeForceThisTime[i] += 1 / (double)nNodePerFace * Press * (// modal force is summation of pressure times normal vector times modal displacement
                                            AreaVector[0] * N_UDMI(pNode, i * N_DOF_PER_NODE + 0) + 
                                            AreaVector[1] * N_UDMI(pNode, i * N_DOF_PER_NODE + 1) + 
                                            AreaVector[2] * N_UDMI(pNode, i * N_DOF_PER_NODE + 2));
                    // if (myid == 0 && modeForceCount > 49 && modeForceCount % 50 == 0)
                    //     Message("%5.1e, ", modeForceThisTime[i]);
                }
                // if(myid == 0 && modeForceCount > 49 && modeForceCount % 50 == 0)
                //     Message("%5.1e, %5.1e, %5d",AreaVector[0], Press, nNodePerFace);
                // if (myid == 0 && modeForceCount > 49 && modeForceCount % 50 == 0)
                //     Message("\n");
                modeForceCount++; // modal force counter +1
            }
        }
    }
    end_f_loop(pFace, pThread); // end face loop

    // Message("UDF[Node][%d]: ", myid);
    // for (int i = 0; i < nModeFluid; i++)
    //     Message("%5.1e, ", modeForceThisTime[i]);
    // Message("\n");

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
              Domain *const pDomain)
{
    if (iTime <= 1) // skip first time step
        return 0;

    Thread *pThread = Lookup_Thread(pDomain, idFluid);                                    // pointer of thread
    cell_t pCell;                                                                         // pointer of cell
    Node *pNode;                                                                          // pointer of node
    real dispUpdate[N_DOF_PER_NODE] = {0}, deltaDisp = 0;                                 // update displacement, displacement gap
    int iNode = 0;                                                                        // node index
    const real *const modeDispLastTime = modeDisp + (iTime - 1) * nModeFluid * N_DOF_PER_NODE; // modal displacement last time
    // Message("\nUDF[Node][%d]: time step is %d, iteration is %d modal displacement are ", myid, iTime, iIter);
    // for (int i = 0; i < 9; i++)
    //     Message("%5.1e ", modeDispThisTime[i]);
    // Message("\n");

    begin_c_loop_int_ext(pCell, pThread) // begin all cell loop
    {
        c_node_loop(pCell, pThread, iNode) // begin node loop
        {
            pNode = C_NODE(pCell, pThread, iNode); // get node pointer
            if (NODE_POS_NEED_UPDATE(pNode))       // if node not yet updated
            {
                memset(dispUpdate, 0, N_DOF_PER_NODE * sizeof(real)); // clear update displacement
                for (int i = 0; i < nModeFluid; i++)
                {
                    deltaDisp = modeDispThisTime[N_DOF_PER_NODE * i] - modeDispLastTime[N_DOF_PER_NODE * i]; // displacement gap
                    dispUpdate[0] += deltaDisp * N_UDMI(pNode, i * N_DOF_PER_NODE + 0);                      // update displacement is summation of displacement gap times each modal displacement
                    dispUpdate[1] += deltaDisp * N_UDMI(pNode, i * N_DOF_PER_NODE + 1);
                    dispUpdate[2] += deltaDisp * N_UDMI(pNode, i * N_DOF_PER_NODE + 2);
                }
                NV_V(NODE_COORD(pNode), +=, dispUpdate); // update node
                NODE_POS_UPDATED(pNode);                 // indicate node updated
                // real x = NODE_X(pNode), y = NODE_Y(pNode), z = NODE_Z(pNode);
                // if (x >= 0.49 && x <= 0.51 && y >= 0.18 && y <=0.22 && z == 0)
                //     Message("UDF[Node][%d]: %5.1e + %5.1e, %5.1e + %5.1e, %5.1e + %5.1e\n", myid, x, dispUpdate[0], y, dispUpdate[1], z, dispUpdate[2]);
            }
        }
    }
    end_c_loop_int_ext(pCell, pThread) // end cell loop

    return 0;
}

#endif
