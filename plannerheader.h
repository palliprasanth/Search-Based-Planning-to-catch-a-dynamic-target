#ifndef PLANNER_HEADER_H   
#define PLANNER_HEADER_H

typedef struct Node Node;
struct Node{
    short int x_index;
    short int y_index;
    short int g;
    Node* next; // Initialize to null whenever created;
    Node* prev; // Initialize to null whenever created;
};

typedef struct Open_Set_Sorted_Linked_List{
    Node* head;  // Initialize to null whenever created;
}Open_Set_Sorted_Linked_List;


// Use 3d Nodes for Forward A-Star
typedef struct Node3d Node3d;
struct Node3d{
    short int x_index;
    short int y_index;
    short int theta_index;
    short int g;
    short int h; // Use backward Dijkstra as heuristic values
    Node3d* next; // Initialize to null whenever created;
    Node3d* prev; // Initialize to null whenever created;
    short int primitive_num;
};

typedef struct Open_Set_Sorted_Linked_List3d{
    Node3d* head;  // Initialize to null whenever created;
}Open_Set_Sorted_Linked_List3d;

//Function declrations go here
// Function declarations for 2d Dijkstra
Open_Set_Sorted_Linked_List* remove_first_node(Open_Set_Sorted_Linked_List* open_list);
Open_Set_Sorted_Linked_List* add_node(Open_Set_Sorted_Linked_List* open_list, Node* node_a);
void print_linked_list_val(Open_Set_Sorted_Linked_List* open_list);
void delete_node(Open_Set_Sorted_Linked_List* open_list, Node* node_del);

// Function declarations for 3d Astar
Open_Set_Sorted_Linked_List3d* remove_first_node3d(Open_Set_Sorted_Linked_List3d* open_list);
Open_Set_Sorted_Linked_List3d* add_node3d(Open_Set_Sorted_Linked_List3d* open_list, Node3d* node_a);
Open_Set_Sorted_Linked_List3d* add_node_head3d(Open_Set_Sorted_Linked_List3d* open_list, Node3d* node_a);
void print_node_val3d(Node3d* CurrentNode);
void print_linked_list_val3d(Open_Set_Sorted_Linked_List3d* open_list);
void delete_node3d(Open_Set_Sorted_Linked_List3d* open_list, Node3d* node_del);
bool is_in_open3d(Open_Set_Sorted_Linked_List3d* open_list, Node3d* node_a, short int x, short int y, short int theta);
bool is_in_closed3d(Open_Set_Sorted_Linked_List3d* open_list, short int x, short int y, short int theta);
#endif 