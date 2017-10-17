/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
#include <math.h>
#include "mex.h"
#include <stdio.h>
#include <stdlib.h>
#include "plannerheader.h"

/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ROBOT_IN	prhs[1]
#define	GOAL_IN     prhs[2]

/* Output Arguments */
#define	ACTION_OUT	plhs[0]

/*access to the map is shifted to account for 0-based indexing in the map, whereas
 * 1-based indexing in matlab (so, robotpose and goalpose are 1-indexed)*/
#define GETMAPINDEX(X, Y, XSIZE, YSIZE) ((Y-1)*XSIZE + (X-1))

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

/* Primitives Information */
#define NUMOFDIRS 8
#define NUMOFPRIMS 5
#define NUMOFINTERSTATES 10
#define NUMOFDIM 3

#define RES 0.1

#define EPS 1 // Weight for weighted A-Star

typedef float PrimArray[NUMOFDIRS][NUMOFPRIMS][NUMOFINTERSTATES][NUMOFDIM];

int temp = 0;

bool applyaction(double *map, int x_size, int y_size, float robotposeX, float robotposeY, float robotposeTheta,
    float *newx, float *newy, float *newtheta, PrimArray mprim, int dir, int prim)
{
    int i;
    for (i = 0; i < NUMOFINTERSTATES; i++) {
        *newx = robotposeX + mprim[dir][prim][i][0];
        *newy = robotposeY + mprim[dir][prim][i][1];
        *newtheta = mprim[dir][prim][i][2];
        
         //int gridposx = (int)(*newx / RES);
         //int gridposy = (int)(*newy / RES);
        
        int gridposx = (int)(*newx / RES + 0.5);
        int gridposy = (int)(*newy / RES + 0.5);
        
        /* check validity */
        if (gridposx < 1 || gridposx > x_size || gridposy < 1 || gridposy > y_size){
            return false;
        }
        if ((int)map[GETMAPINDEX(gridposx, gridposy, x_size, y_size)] != 0){
            return false;
        }
    }
    return true;
}

int getPrimitiveDirectionforRobotPose(float angle)
{
    /* returns the direction index with respect to the PrimArray */
    /* normalize bw 0 to 2pi */
    if (angle < 0.0) {
        angle += 2 * M_PI;
    }
    //int dir = (int)(angle / (2 * M_PI / NUMOFDIRS));
    int dir = (int)(angle / (2 * M_PI / NUMOFDIRS) + 0.5);
    if (dir == 8) {
        dir = 0;
    }
    return dir;
}

static void planner(
    double*	map,
    int x_size,
    int y_size,
    float robotposeX,
    float robotposeY,
    float robotposeTheta,
    float goalposeX,
    float goalposeY,
    PrimArray mprim,
    int *prim_id)
{

    // Initialization of Variables
    temp = temp+1;
    printf("step number : %d\n", temp);
    bool dijkstra = true;
    //short int map_g_val[x_size][y_size];
    short int i,j,k;
    int dir;
    int prim;
    *prim_id = 0; /* arbitrary action */

    short int **map_g_val = (short int **)malloc(x_size * sizeof(short int *));
    for (i=0; i<x_size; i++)
     map_g_val[i] = (short int *)malloc(y_size * sizeof(short int));

 if (dijkstra){

   //bool closed_set_dijkstra[x_size][y_size];
    bool **closed_set_dijkstra = (bool **)malloc(x_size * sizeof(bool*));
    for (i=0; i<x_size; i++)
       closed_set_dijkstra[i] = (bool *)malloc(y_size * sizeof(bool));


   Node* Dijkstra_Nodes[x_size][y_size];
   for (i=0;i<x_size;i++){
    for (j=0;j<y_size;j++){
        map_g_val[i][j] = 10000;
        closed_set_dijkstra[i][j] = false;
        Dijkstra_Nodes[i][j] = NULL;
    }
}

 // Initializing Backward Dijkstra by setting g value of goal to zero and adding the goal state to the sorted open set
Open_Set_Sorted_Linked_List* open_set;
open_set = malloc(sizeof(Open_Set_Sorted_Linked_List));
open_set->head = NULL;

Node* GoalNode;
GoalNode = malloc(sizeof(Node));
GoalNode->x_index = (int)(goalposeX/RES + 0.5) - 1;
GoalNode->y_index = (int)(goalposeY/RES + 0.5) - 1;
map_g_val[GoalNode->x_index][GoalNode->y_index] = 0;
GoalNode->g = 0;
GoalNode->next = NULL;
GoalNode->prev = NULL;
open_set = add_node(open_set, GoalNode);

    /*printf("robot: %d %d; ", robotposeX, robotposeY);*/
    /*printf("goal: %d %d;", goalposeX, goalposeY);*/
dir = getPrimitiveDirectionforRobotPose(robotposeTheta);

    // DIJKSTRA STARTS
   // Assuming 4-connected grid for Dijkstra
short int x_change[] ={1,-1,0,0};
short int y_change[] ={0,0,1,-1};
short int x_curr_index, y_curr_index;
short int iter;
Node* CurrentNode;  
    //printf("Dijkstra Loop Starts: \n");
while(open_set->head!=NULL){
        CurrentNode = open_set->head; // Choosing the head as the Node with min g value as the linked list is sorted.
        open_set = remove_first_node(open_set);
        closed_set_dijkstra[CurrentNode->x_index][CurrentNode->y_index] = true;

        // Getting Succesors and updating the open list
        for(iter = 0; iter<4;iter++){
            x_curr_index = CurrentNode->x_index + x_change[iter];
            y_curr_index = CurrentNode->y_index + y_change[iter];
            // Out of bounds check
            if ((x_curr_index < 0) || (x_curr_index >= x_size) || (y_curr_index < 0)  || (y_curr_index >= y_size)){
                continue;
            }
            // Obstacle check
            if ((int)map[GETMAPINDEX(x_curr_index + 1, y_curr_index + 1, x_size, y_size)] != 0){
                continue;
            }
            // Check if the node is already closed;
            if(closed_set_dijkstra[x_curr_index][y_curr_index] == 1){
                continue;
            }

            short int updated_g = map_g_val[CurrentNode->x_index][CurrentNode->y_index] + 1 ;
            if (map_g_val[x_curr_index][y_curr_index] > updated_g){
                Node* PredecessorNode;
                PredecessorNode = malloc(sizeof(Node));
                map_g_val[x_curr_index][y_curr_index] = updated_g;
                PredecessorNode->x_index = x_curr_index;
                PredecessorNode->y_index = y_curr_index;
                PredecessorNode->g = updated_g;
                PredecessorNode->next = NULL;
                PredecessorNode->prev = NULL;

                if (Dijkstra_Nodes[PredecessorNode->x_index][PredecessorNode->y_index]== NULL){
                    Dijkstra_Nodes[PredecessorNode->x_index][PredecessorNode->y_index] = PredecessorNode;
                    open_set = add_node(open_set, PredecessorNode);
                } else{
                    //delete and add the node again
                    delete_node(open_set,Dijkstra_Nodes[PredecessorNode->x_index][PredecessorNode->y_index]);
                    Dijkstra_Nodes[PredecessorNode->x_index][PredecessorNode->y_index] = PredecessorNode;
                    open_set = add_node(open_set, PredecessorNode);
                }
            }
        }
        free(CurrentNode); 
    }

    free(open_set);
    Node* TempNode;
    TempNode = open_set->head;
    while(TempNode!=NULL){
        open_set->head = open_set->head->next;
        free(TempNode);
        TempNode = open_set->head ;
    }
    
}

// FILE *fp;
//     fp = fopen("Output.txt", "w");
//     for (i=0;i<x_size;i++){
//         for (j=0;j<y_size;j++){
//             fprintf(fp,"%d,",map_g_val[i][j]);
//         }
//         fprintf(fp,"\n");
//     }
    // DIJKSTRA ENDS

    // A-STAR STARTS
    Open_Set_Sorted_Linked_List3d* open_set_astar;
    open_set_astar = malloc(sizeof(Open_Set_Sorted_Linked_List3d));
    open_set_astar->head = NULL;

    Open_Set_Sorted_Linked_List3d* closed_set_astar;
    closed_set_astar = malloc(sizeof(Open_Set_Sorted_Linked_List3d));
    closed_set_astar->head = NULL;

    Node3d* StartNodeAstar;
    StartNodeAstar = malloc(sizeof(Node3d));
    StartNodeAstar->x_index = (int)(robotposeX/RES + 0.5) - 1;
    StartNodeAstar->y_index = (int)(robotposeY/RES + 0.5) - 1;
    StartNodeAstar->theta_index = dir;
    StartNodeAstar->g = 0;
    StartNodeAstar->h = map_g_val[StartNodeAstar->x_index][StartNodeAstar->y_index]; // Using Backward Dijkstra's g values as the heuristic value
    StartNodeAstar->next = NULL;
    StartNodeAstar->prev = NULL;
    StartNodeAstar->primitive_num = 0;
    open_set_astar = add_node3d(open_set_astar, StartNodeAstar);

    // //check if the neighbourhood of the goal is closed. If yes, return. Else, repeat A-Star
    bool ret;
    short int cur_x_index, cur_y_index, cur_theta_index;
    Node3d* CurrentNode_astar;
    float newx, newy, newtheta;
    short int thresh = 4; // 4 cells
    bool first_iter = true;
    short int current_val;
    bool revisited = false;
    //print_linked_list_val3d(open_set_astar);
    //printf("A Star Starts: \n");
    while (open_set_astar->head!=NULL){
        CurrentNode_astar = open_set_astar->head; // Choosing the head as the Node with min g + eps*h value as the linked list is sorted.
        open_set_astar = remove_first_node3d(open_set_astar);
        closed_set_astar = add_node_head3d(closed_set_astar, CurrentNode_astar);
        if (map_g_val[CurrentNode_astar->x_index][CurrentNode_astar->y_index] <= thresh){
            *prim_id = CurrentNode_astar->primitive_num;
            break;
        }
        robotposeX = CurrentNode_astar->x_index*RES + 0.1;
        robotposeY = CurrentNode_astar->y_index*RES + 0.1;
        robotposeTheta = CurrentNode_astar->theta_index*2*M_PI/NUMOFDIRS;
        for (prim = 0; prim < NUMOFPRIMS; prim++) {
            ret = applyaction(map, x_size, y_size, robotposeX, robotposeY, robotposeTheta, &newx, &newy, &newtheta, mprim, CurrentNode_astar->theta_index, prim);        
    //     /* skip action that leads to collision */
            if (ret) {
                cur_x_index = (int)(newx/RES + 0.5) - 1;
                cur_y_index = (int)(newy/RES + 0.5) - 1;
                cur_theta_index = getPrimitiveDirectionforRobotPose(newtheta);

                 // Check if the node is already closed;
                if(is_in_closed3d(closed_set_astar,  cur_x_index, cur_y_index, cur_theta_index)){
                 continue;
             }

             current_val = CurrentNode_astar->g + 1;    
             revisited = is_in_open3d(open_set_astar, StartNodeAstar, cur_x_index, cur_y_index, cur_theta_index);
             if (!revisited){
                StartNodeAstar = malloc(sizeof(Node3d));
                StartNodeAstar->x_index = cur_x_index;
                StartNodeAstar->y_index = cur_y_index;
                StartNodeAstar->theta_index = cur_theta_index;
                StartNodeAstar->g = CurrentNode_astar->g + 1;
                StartNodeAstar->h = map_g_val[cur_x_index][cur_y_index];
                StartNodeAstar->next = NULL;
                StartNodeAstar->prev = NULL;
                if (first_iter){
                    StartNodeAstar->primitive_num = prim;
                }else{
                    StartNodeAstar->primitive_num = CurrentNode_astar->primitive_num;
                }
                open_set_astar = add_node3d(open_set_astar, StartNodeAstar);}

                else{
                    if ((StartNodeAstar->g) > (current_val)){
                        delete_node3d(open_set_astar,StartNodeAstar);
                        StartNodeAstar = malloc(sizeof(Node3d));
                        StartNodeAstar->x_index = cur_x_index;
                        StartNodeAstar->y_index = cur_y_index;
                        StartNodeAstar->theta_index = cur_theta_index;
                        StartNodeAstar->g = CurrentNode_astar->g + 1;
                        StartNodeAstar->h = map_g_val[cur_x_index][cur_y_index];
                        StartNodeAstar->next = NULL;
                        StartNodeAstar->prev = NULL;
                        open_set_astar = add_node3d(open_set_astar, StartNodeAstar);
                    }
                }
            }
        }
        first_iter = false; 
    }
    // // A-STAR ENDS

    printf("action %d\n", *prim_id);

        // Node3d* TempNode3d;
        // TempNode3d = open_set_astar->head;
        // while(TempNode3d!=NULL){
        //     open_set_astar->head = open_set_astar->head->next;
        //     free(TempNode3d);
        //     TempNode3d = open_set_astar->head ;
        // }

    return;
}

/*prhs contains input parameters (3):
 * 1st is matrix with all the obstacles
 * 2nd is a row vector <x,y> for the robot pose
 * 3rd is a row vector <x,y> for the target pose
 * plhs should contain output parameters (1):
 * 1st is a row vector <dx,dy> which corresponds to the action that the robot should make*/

void parseMotionPrimitives(PrimArray mprim)
{
    FILE * fp;
    fp = fopen ("unicycle_8angles.mprim", "r+");
    char skip_c[100];
    int skip_f;
    float resolution;
    int num_angles;
    int num_mprims;
    fscanf(fp, "%s %f", skip_c, &resolution);
    fscanf(fp, "%s %d", skip_c, &num_angles);
    fscanf(fp, "%s %d", skip_c, &num_mprims);

    int i, j, k;
    for (i = 0; i < NUMOFDIRS; ++i) {
        for (j = 0; j < NUMOFPRIMS; ++j) {
            fscanf(fp, "%s %d", skip_c, &skip_f);
            for (k = 0; k < NUMOFINTERSTATES; ++k) {
                fscanf(fp, "%f %f %f", &mprim[i][j][k][0], &mprim[i][j][k][1], &mprim[i][j][k][2]);
            }

        }
    }
}

void mexFunction( int nlhs, mxArray *plhs[],
    int nrhs, const mxArray*prhs[] )
{
    /* Read motion primtives */
    PrimArray motion_primitives;
    parseMotionPrimitives(motion_primitives);

    /* Check for proper number of arguments */
    if (nrhs != 3) {
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidNumInputs",
            "Three input arguments required.");
    } else if (nlhs != 1) {
        mexErrMsgIdAndTxt( "MATLAB:planner:maxlhs",
            "One output argument required.");
    }

    /* get the dimensions of the map and the map matrix itself*/
    int x_size = mxGetM(MAP_IN);
    int y_size = mxGetN(MAP_IN);
    double* map = mxGetPr(MAP_IN);

    /* get the dimensions of the robotpose and the robotpose itself*/
    int robotpose_M = mxGetM(ROBOT_IN);
    int robotpose_N = mxGetN(ROBOT_IN);
    if(robotpose_M != 1 || robotpose_N != 3){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidrobotpose",
            "robotpose vector should be 1 by 3.");
    }
    double* robotposeV = mxGetPr(ROBOT_IN);
    float robotposeX = (float)robotposeV[0];
    float robotposeY = (float)robotposeV[1];
    float robotposeTheta = (float)robotposeV[2];

    /* get the dimensions of the goalpose and the goalpose itself*/
    int goalpose_M = mxGetM(GOAL_IN);
    int goalpose_N = mxGetN(GOAL_IN);
    if(goalpose_M != 1 || goalpose_N != 3){
        mexErrMsgIdAndTxt( "MATLAB:planner:invalidgoalpose",
            "goalpose vector should be 1 by 3.");
    }
    double* goalposeV = mxGetPr(GOAL_IN);
    float  goalposeX = (float)goalposeV[0];
    float  goalposeY = (float)goalposeV[1];

    /* Create a matrix for the return action */
    ACTION_OUT = mxCreateNumericMatrix( 1, 1, mxINT8_CLASS, mxREAL);
    int* action_ptr = (int*) mxGetData(ACTION_OUT);

    /* Do the actual planning in a subroutine */
    planner(map, x_size, y_size, robotposeX, robotposeY, robotposeTheta, goalposeX, goalposeY, motion_primitives, &action_ptr[0]);
    return;
}