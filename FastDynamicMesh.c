// 			 Dynamic mesh UDF edited by LKD-2023
// 	   fluid structure algorithm proposed By LKD & WZR
//          Refactored by SpecialXuan at 2023.11

#include "udf.h"

// define the read_mode()  to read input the array from the .txt file
int read_my_mode(double *elas_mode, const int row, const int column)
{
	FILE *fp = NULL; // file pointer
	int i = 0, j = 0; // indexes

	fp = fopen("elas_mode.txt", "r"); // open the file in the read-only mode
	if (fp == NULL)					  // file not exists -> print error
	{
		Message("\n ---        Error: No file.        ---\n");
		return 1;
	}

	for (i = 0; i < row; i++) // fill the array
		for (j = 0; j < column; j++)
			if (fscanf(fp, "%lf", elas_mode + i * column + j) <= 0)
			{
				Message("\n ---   Error: Lack of variables.   ---\n From (%d,%d)", i, j);
				fclose(fp);
				return 1;
			}

	fclose(fp); // close file

	Message("\n ---    The elas_mode array is:    ---\n"); // validation for the initialization of the elas_mode
	for (j = 0; j < column; j++)						 // print variables
		if (j == 0)
			Message("%f \t", elas_mode[row * column + j]);
		else
			Message("%6.5e \t", elas_mode[row * column + j]);
	Message("\n ---      Validation is done!      ---\n");

	return 0;
}

static int iter_index = 1; // record the number of the iteration steps
static int time_index = 0; // record the number of the time steps

DEFINE_ON_DEMAND(Preparation)
{
	FILE *fp = NULL;

	Domain *domain;
	cell_t c;
	Thread *t;
	Node *v;
	int row = 66654, column = 16;
	int n = 0, i = 0, count = 0;   // record the number of total nodes
	double *elas_mode = NULL; // elas_mode are the first 4 mode shapes array of the fluid mesh.
					   // node-No.,x-coor,y-coor,z-coor,shape-information
					   // 66654 row, 16column, Start at 0 when called

	iter_index = 1;
	time_index = 0;

	// RP_Get_Real(char *s)-RP_Get_Real("flow-time")
	// RP_Set_Float(char *s, double v)-RP_Set_Float("flow-time", 0.2)
	RP_Set_Float("flow-time", 0); // set  flow time

	domain = Get_Domain(1);									   // for single-phase flows, domain_id is 1 and Get_Domain(1) returns the fluid domain pointer
	elas_mode = (double *)malloc(row * column * sizeof(double)); // allocate memory for elas_mode
	memset(elas_mode, 0, row * column * sizeof(double));		   // initialize elas_mode

	if (read_my_mode(elas_mode, row, column))
		Message("\n ---      Error: read_my_mode      ---\n");
	else
	{
		fp = fopen("numberMatch.csv", "w");

		// Store the modal shape information in user-defined node memory
		thread_loop_c(t, domain){
			begin_c_loop(c, t){
				c_node_loop(c, t, n){
					v = C_NODE(c, t, n);
					
					// set the value of the first user define node memory as 0
					// used to identify if the node has stored the shape information or not
					N_UDMI(v, 12) = 0;
				}
			}
			end_c_loop(c, t)
		}

		thread_loop_c(t, domain){
			begin_c_loop(c, t){
				c_node_loop(c, t, n){
					v = C_NODE(c, t, n);
					
					fprintf(fp, "%5d, %5d,", v, count);

					if (N_UDMI(v, 12) == 0) // hold the modal shape information using the UDMI
					{
						for (i = 0; i < 12; i++)
							N_UDMI(v, i) = elas_mode[count * column + i + 1];
					
						fprintf(fp, "%5d, %5d,", v, count);
						count++;
						N_UDMI(v, 12) = 1;
					}

					fprintf(fp, "\n");
				}
			}
			end_c_loop(c, t)
		}

		fclose(fp);

		Message("\n --- The number of total nodes is: ---\n");
		Message("%d \n", count);
	}
	free(elas_mode);
}

// calculation of the modal aerodynamic force
// and modal displacement for fluid mesh
// Node motion for CFD mesh of the fluid zone
static real m = 1;										// modal mass = 1, which means normalization of principal mass
static real c = 0;										// modal damping
static real Pi = 3.141592654;							// Pi
static real theta = 1.4;								// wilson-theta method, the value of the theta
static int FSI_ID = 19;									// record the id of the fsi faces, shown in fluent       // TODO: read from file
static int Fluid_ID = 11;								// record the id of the fluid cell zone, shown in fluent // TODO: read from file
static real freq[4] = {0.17839, 1.103, 3.0322, 5.8042}; // frequencies of the structure                          // TODO: read from file
static real ini_vel[4] = {0, 0, 0, 0};					// initial modal velocity
static real mode_force[100000][5] = {0};				// modal force
static real mode_disp[100000][12] = {0};				// modal-displacement,modal-velocity,acceleration repeat
DEFINE_GRID_MOTION(FDM_method, domain, dt, time, dtime)
{
	Thread *t_fsi = Lookup_Thread(domain, FSI_ID); // pointer to the thread of FSI faces
	Thread *t = Lookup_Thread(domain, Fluid_ID);   // pointer to a thread
	real N_nodes, Pres, Area[3];				   // number of nodes in a face, pressure, normal vector of a face
	real disp[3] = {0};							   // record the displacement of the cfd mesh node
	cell_t c;									   // An integer data type that identifies a particular cell within a cell thread.
	face_t f;									   // An integer data type that identifies a particular face within a face thread.
	Node *node;									   // A structure data type that stores data associated with a mesh point
	int node_index, i;
	for (i = 1; i < 5; i++) //TODO: use memset
	{
		mode_force[time_index][i] = 0;
	}
	begin_f_loop(f, t_fsi) // loop over all faces in a given face thread.
	{
		F_AREA(Area, f, t_fsi); //(A,f,t):A[ND_ND], face_t f, Thread *t, A (area vector)

		Pres = F_P(f, t_fsi);			  //(f,t): face_t f, Thread *t, pressure
		N_nodes = F_NNODES(f, t_fsi);	  //(f,t), face_t f, Thread *t, number of nodes in a face
		f_node_loop(f, t_fsi, node_index) // loops over all nodes of a given face
		{
			node = F_NODE(f, t_fsi, node_index); // the F_NODE macro to obtain the global face node number
			for (i = 1; i < 5; i++)				 // calculation of the modal aerodynamic force
			{
				mode_force[time_index][i] += 1 / N_nodes * Pres * (Area[0] * N_UDMI(node, 3 * i - 3) + Area[1] * N_UDMI(node, 3 * i - 2) + Area[2] * N_UDMI(node, 3 * i - 1));
			}
		}
	}
	end_f_loop(f, t_fsi)
		Message("(");
	Message("%d", iter_index);
	Message(")");
	//    calculation of modal displacement and modal velocity
	if (time_index == 0)
	{
		for (i = 1; i < 5; i++)
		{
			mode_disp[time_index][3 * i - 1] = mode_force[time_index][i];
			mode_disp[time_index][3 * i - 2] = ini_vel[i - 1];
			mode_disp[time_index][3 * i - 3] = 0;
		}
		Message("\nThe vibration is initialized.\n");
	}
	else
	{
		for (i = 0; i < 4; i++)
		{
			// the calculation of modal displacement,  acceleration, velocity using wilson-theta method
			mode_disp[time_index][3 * i] = mode_force[time_index - 1][i + 1] + theta * (mode_force[time_index][i + 1] - mode_force[time_index - 1][i + 1]);
			mode_disp[time_index][3 * i] += m * (6 / SQR(theta * dtime) * mode_disp[time_index - 1][3 * i] + 6 / (theta * dtime) * mode_disp[time_index - 1][3 * i + 1] + 2 * mode_disp[time_index - 1][3 * i + 2]);
			mode_disp[time_index][3 * i] += c * (3 / (theta * dtime) * mode_disp[time_index - 1][3 * i] + 2 * mode_disp[time_index - 1][3 * i + 1] + 0.5 * theta * dtime * mode_disp[time_index - 1][3 * i + 2]);
			mode_disp[time_index][3 * i] /= 6 * m / SQR(theta * dtime) + 3 * c / (theta * dtime) + SQR(2 * Pi * freq[i]);
			// calculation of the acceleration
			mode_disp[time_index][3 * i + 2] = 6 / (SQR(theta * dtime) * theta) * (mode_disp[time_index][3 * i] - mode_disp[time_index - 1][3 * i]) - 6 / (SQR(theta) * dtime) * mode_disp[time_index - 1][3 * i + 1] + (1 - 3 / theta) * mode_disp[time_index - 1][3 * i + 2];
			// calculation of the velocity
			mode_disp[time_index][3 * i + 1] = mode_disp[time_index - 1][3 * i + 1] + 0.5 * dtime * (mode_disp[time_index][3 * i + 2] + mode_disp[time_index - 1][3 * i + 2]);
			// calculation of the displacement
			mode_disp[time_index][3 * i] = mode_disp[time_index - 1][3 * i] + dtime * mode_disp[time_index - 1][3 * i + 1] + SQR(dtime) / 6 * (mode_disp[time_index][3 * i + 2] + 2 * mode_disp[time_index - 1][3 * i + 2]);
		}
	}
	if (time_index > 0)
	{
		begin_c_loop(c, t)
		{
			c_node_loop(c, t, node_index)
			{
				node = C_NODE(c, t, node_index);
				if (NODE_POS_NEED_UPDATE(node))
				{
					// set the value of disp[3] with zeros to calculate the displacement for the current time step
					for (i = 0; i < 3; i++)
					{
						disp[i] = 0;
					}
					// indicate that the position of the node has been updated to avoid updating more than once
					NODE_POS_UPDATED(node);
					// calculation of nodal displacement for the current node
					for (i = 0; i < 4; i++)
					{
						disp[0] += (mode_disp[time_index][3 * i] - mode_disp[time_index - 1][3 * i]) * N_UDMI(node, 3 * i);
						disp[1] += (mode_disp[time_index][3 * i] - mode_disp[time_index - 1][3 * i]) * N_UDMI(node, 3 * i + 1);
						disp[2] += (mode_disp[time_index][3 * i] - mode_disp[time_index - 1][3 * i]) * N_UDMI(node, 3 * i + 2);
					}
					NV_V(NODE_COORD(node), +=, disp);
				}
			}
		}
		end_c_loop(c, t)
	}
	iter_index += 1; /**increase the total number of iteration steps**/
}

// keep the flow_time information and increase the number of time steps
DEFINE_EXECUTE_AT_END(setting_next_time_step)
{
	int i;
	Message("\n**************************************\n");
	Message("%d", time_index);
	Message("\n The current modal_aerodynamic_force are: \n");
	for (i = 0; i < 5; i++)
	{
		Message("%12.10e\t", mode_force[time_index][i]);
	}
	Message("\nThe current modal_displacement_velocity_acceleration are:\n");
	for (i = 0; i < 12; i++)
	{
		Message("%12.10e\t", mode_disp[time_index][i]);
	}
	Message("\n**************************************\n");
	iter_index = 1;							  // set the value of the iterations number to be 1
	time_index += 1;						  // increase the total number of time steps, used as the index for mode_force and mode_disp
	mode_force[time_index][0] = CURRENT_TIME; // record the flow time at the current time step
}

// define_on_demand
// to output the calculated modal force, displacement and velocity to a txt file
DEFINE_ON_DEMAND(Output_disp)
{
	FILE *fp;
	int i, j, k;
	/**new the file in the write mode**/
	fp = fopen("Modal_Displacement.txt", "w+");
	if (fp == NULL)
		Message(" Can't open the file. Sorry.\n");
	else
	{
		for (i = 0; i <= time_index; i++)
		{
			for (j = 0; j < 5; j++)
			{
				if (j == 0)
					fprintf(fp, "%12.10e", mode_force[i][j]);
				else
					fprintf(fp, "\t %12.10e", mode_force[i][j]);
			}
			for (k = 0; k < 12; k++)
			{
				if (k != 11)
					fprintf(fp, "\t%12.10e", mode_disp[i][k]);
				else
					fprintf(fp, "\t%12.10e\n ", mode_disp[i][k]);
			}
		}
	}
	fclose(fp);
	Message("\n The calculated modal_force_displacement are output to Modal_Displacement.txt \n");
}