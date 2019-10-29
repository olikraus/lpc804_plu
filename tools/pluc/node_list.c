/*

  node_list.c

   A list of fsm nodes ordered by fan-in

  Copyright (C) 2019 Oliver Kraus (olikraus@gmail.com)

  This file is part of "pluc".

  "pluc" is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  "pluc" is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with DGC; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA  


*/

#include <stdlib.h>
#include <stdio.h>
#include <strings.h>
#include "node_list.h"
#include "fsm.h"

/* NODE_LIST */

/*-- node_OpenNodeList -------------------------------------------------------------*/

NODE_LIST node_OpenNodeList()
{
  return (NODE_LIST) NULL;
}

/*-- node_DeleteNodeFromList --------------------------------------------------------*/

NODE_LIST node_DeleteNodeFromList(NODE_LIST nodes)
{
  /* deletes the first element from the list anode returns the resulting list 
   */
   
  NODE_LIST tmp;
  if(nodes == NULL) return NULL;
  
  tmp = nodes->next;
  
  free(nodes);
  
  return tmp;

}

/*-- node_CloseNodeList -----------------------------------------------------------*/

void node_CloseNodeList(NODE_LIST nodes)
{
  while(nodes != NULL)
    nodes = node_DeleteNodeFromList(nodes);

}

/*-- node_AddNodeByFanIn -----------------------------------------------------------*/

NODE_LIST node_AddNodeByFanIn(NODE_LIST nodes, int node_id, int fan_in, int cntDC)
{
  /* adds a node in the list such that the list is ordered by the node fan in 
   */
  NODE_LIST tmp, prev, aux;

  if((aux = (NODE_LIST) malloc(sizeof(struct _node_list))) == NULL)
    return NULL;
  
  aux->node_id = node_id;
  aux->fan_in = fan_in;
  aux->cntDC = cntDC;
  
  if( ( nodes==NULL ) || fan_in > nodes->fan_in)
  {
    /* insert it at the beginning of the list */
    
    aux->next = nodes;
    return aux;
  }
  
  /* finde its place and then insert it */
  
  prev = nodes;
  tmp = prev->next;
  
  while(tmp && fan_in < tmp->fan_in )
  {
    prev = tmp;
    tmp = tmp->next;
  }
  
  while(tmp && (fan_in < tmp->fan_in) && ( cntDC > tmp->cntDC ))
  {
    prev = tmp;
    tmp = tmp->next;
  }
  
  /* insert it after prev */
  aux->next = tmp;
  prev->next = aux;
  
  return nodes;
}

/*-- node_WriteList -----------------------------------------------------------*/

void node_WriteList(NODE_LIST nodes, fsm_type fsm)
{
  NODE_LIST tmp;
  if(nodes == NULL) return;
  
  printf("\n -----BEGIN LIST------\n");
  for (tmp = nodes; tmp != NULL; tmp = tmp->next)
  {
    printf("%s %d\n", fsm_GetNodeName(fsm, tmp->node_id), tmp->fan_in);
  }
  printf("\n -----END LIST------\n");
}
