#include <math.h>
#include "mex.h"
#include <stdio.h>
#include <stdlib.h>
#include "plannerheader.h"

#define EPS 1 // Weight for weighted A-Star

Open_Set_Sorted_Linked_List* remove_first_node(Open_Set_Sorted_Linked_List* open_list)
{
    if(open_list->head == NULL)
        return open_list;
    open_list->head = open_list->head->next; 
    if (open_list->head != NULL){ 
        //free(open_list->head->prev);
        open_list->head->prev = NULL;
    }
    return open_list;
}

Open_Set_Sorted_Linked_List* add_node(Open_Set_Sorted_Linked_List* open_list, Node* node_a)
{
    Node* current;

    if(open_list->head == NULL){
        open_list->head = node_a;
        return open_list;
    }
    else if(open_list->head->g >= node_a->g){
        node_a->next = open_list->head;
        node_a->next->prev = node_a;
        open_list->head = node_a;
    }

    else{
        current = open_list->head;
        while (current->next != NULL &&  current->next->g < node_a->g)
            current = current->next;
        node_a->next = current->next;
        if (current->next != NULL)
            node_a->next->prev = node_a;

        current->next = node_a;
        node_a->prev = current;
    }
    return open_list;
}

void delete_node(Open_Set_Sorted_Linked_List* open_list, Node* node_del){
    if (node_del->next == NULL){
        if (node_del->prev != NULL){
            node_del->prev->next = NULL;
            free(node_del);
            return;
        }
    }
    if (node_del->prev == NULL){ 
        if ( node_del->next != NULL){
         node_del->next->prev = NULL;
         open_list->head =  node_del->next;
         free(node_del);
         return;
     }
 }

 node_del->prev->next = node_del->next;
 node_del->next->prev = node_del->prev;
 free(node_del);
 return;
}

void print_linked_list_val(Open_Set_Sorted_Linked_List* open_list){
    Node* CurrentNode;
    CurrentNode = open_list->head;
    while(CurrentNode != NULL){
        printf("g value : %d\n", CurrentNode->g);
        printf("Current Node Address : %p\n", CurrentNode);
        printf("Next Node Address : %p\n", CurrentNode->next);
        printf("Prev Node Address : %p\n", CurrentNode->prev);
        CurrentNode = CurrentNode->next;
    } 
}

// A-Star function declarations
Open_Set_Sorted_Linked_List3d* remove_first_node3d(Open_Set_Sorted_Linked_List3d* open_list)
{
    if(open_list->head == NULL)
        return open_list;
    open_list->head = open_list->head->next; 
    if (open_list->head != NULL){ 
        //free(open_list->head->prev);
        open_list->head->prev = NULL;
    }
    return open_list;
}

Open_Set_Sorted_Linked_List3d* add_node3d(Open_Set_Sorted_Linked_List3d* open_list, Node3d* node_a)
{
    Node3d* current;
    if(open_list->head == NULL){
        open_list->head = node_a;
        return open_list;
    }

    else if((open_list->head->g + EPS*open_list->head->h) >= (node_a->g + EPS*node_a->h)){
        node_a->next = open_list->head;
        node_a->next->prev = node_a;
        open_list->head = node_a;
    }

    else{
        current = open_list->head;
        while (current->next != NULL &&  ((current->next->g + EPS*open_list->head->h) <(node_a->g + EPS*node_a->h)))
            current = current->next;

        node_a->next = current->next;
        if (current->next != NULL)
            node_a->next->prev = node_a;

        current->next = node_a;
        node_a->prev = current;
    }
    return open_list;
}

void delete_node3d(Open_Set_Sorted_Linked_List3d* open_list, Node3d* node_del){
    if (node_del->next == NULL){
        if (node_del->prev != NULL){
            node_del->prev->next = NULL;
            free(node_del);
            return;
        }
    }
    if (node_del->prev == NULL){ 
        if ( node_del->next != NULL){
         node_del->next->prev = NULL;
         open_list->head =  node_del->next;
         free(node_del);
         return;
     }
 }
 node_del->prev->next = node_del->next;
 node_del->next->prev = node_del->prev;
 free(node_del);
 return;
}

void print_linked_list_val3d(Open_Set_Sorted_Linked_List3d* open_list){
    Node3d* CurrentNode;
    CurrentNode = open_list->head;
    while(CurrentNode != NULL){
        // printf("g value: %d\n", CurrentNode->g);
        // printf("h value : %d\n", CurrentNode->h);
        // printf("g + eps(h) value : %d\n", CurrentNode->g + EPS*CurrentNode->h);
        // printf("Prim Id : %d\n",CurrentNode->primitive_num);
        printf("X index : %d\n", CurrentNode->x_index);
        printf("Y index : %d\n", CurrentNode->y_index);
        printf("Theta index : %d\n", CurrentNode->theta_index);
        // printf("Current Node Address : %p\n", CurrentNode);
        // printf("Next Node Address : %p\n", CurrentNode->next);
        // printf("Prev Node Address : %p\n\n", CurrentNode->prev);
        CurrentNode = CurrentNode->next;
    } 
}

void print_node_val3d(Node3d* CurrentNode){
    if(CurrentNode != NULL){
        printf("g value: %d\n", CurrentNode->g);
        printf("h value : %d\n", CurrentNode->h);
        printf("g + eps(h) value : %d\n", CurrentNode->g + EPS*CurrentNode->h);
        printf("Prim Id : %d\n",CurrentNode->primitive_num);
        printf("X index : %d\n", CurrentNode->x_index);
        printf("Y index : %d\n", CurrentNode->y_index);
        printf("Theta index : %d\n", CurrentNode->theta_index);
        printf("Current Node Address : %p\n", CurrentNode);
        printf("Next Node Address : %p\n", CurrentNode->next);
        printf("Prev Node Address : %p\n\n", CurrentNode->prev);
    } 
}

bool is_in_open3d(Open_Set_Sorted_Linked_List3d* open_list, Node3d* node_a, short int x, short int y, short int theta){
   Node3d* CurrentNode;
   CurrentNode = open_list->head;
   while(CurrentNode != NULL){
    if ((CurrentNode->x_index == x) && (CurrentNode->y_index == y) && (CurrentNode->theta_index == theta)){
        node_a = CurrentNode;
        return true;
    }
    else{
        CurrentNode = CurrentNode->next;
    }
}
return false; 
}


Open_Set_Sorted_Linked_List3d* add_node_head3d(Open_Set_Sorted_Linked_List3d* open_list, Node3d* node_a)
{
    if(open_list->head == NULL){
        open_list->head = node_a;
        return open_list;
    }
    else{
        node_a->next = open_list->head;
        node_a->next->prev = node_a;
        open_list->head = node_a;
    }
    return open_list;
}

bool is_in_closed3d(Open_Set_Sorted_Linked_List3d* open_list, short int x, short int y, short int theta){
   Node3d* CurrentNode;
   CurrentNode = open_list->head;
   while(CurrentNode != NULL){
    if ((CurrentNode->x_index == x) && (CurrentNode->y_index == y) && (CurrentNode->theta_index == theta)){
        return true;
    }
    else{
        CurrentNode = CurrentNode->next;
    }
} 
return false;
}
