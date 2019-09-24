/*

  node_list.h
  
  A list of fsm nodes ordered by fan-in, used when coding the fsm with the "FanIn" method
  
*/

#ifndef _node_list_H
#define _node_list_H

#include <stdio.h>
#include "fsm.h"


typedef struct _node_list *NODE_LIST;

struct _node_list
{
  int node_id;  /* the ID of the node in the fsm */
  int fan_in;   /* the fan in of the corresponding node */
  int cntDC;    /* the total number of DCs in the condition of the input edges */
  NODE_LIST next;   
};



/* NODE_LIST */

/* INTERFACE functions */

NODE_LIST   node_OpenNodeList       ();
NODE_LIST   node_DeleteNodeFromList (NODE_LIST nodes);
void        node_CloseNodeList      (NODE_LIST nodes);
NODE_LIST   node_AddNodeByFanIn     (NODE_LIST nodes, int node_id, int fan_in, int cntDC);

/* Function for WRITING a node list */

void        node_WriteList          (NODE_LIST nodes, fsm_type fsm);


#endif
