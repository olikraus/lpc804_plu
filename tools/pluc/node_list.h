/*

  node_list.h
  
  A list of fsm nodes ordered by fan-in, used when coding the fsm with the "FanIn" method

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
