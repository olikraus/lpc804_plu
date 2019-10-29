/*

  index_table.h
  
  Description of the index table for fsm nodes;
  Creates and ordering of the fsm nodes continuous from 0 to maxNodes - 1

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

#ifndef _INDEX_TABLE_H
#define _INDEX_TABLE_H

#include "fsm.h"


typedef struct _index_table *INDEX_TABLE;

struct _index_table
{
  int nr_nodes;
  int *table;
};

INDEX_TABLE index_Open(int nr);
void index_Close(INDEX_TABLE tab);
INDEX_TABLE index_CreateTable( fsm_type fsm);

int index_GetNodeId(INDEX_TABLE tab, int pos);
int index_SetNodeId(INDEX_TABLE tab, int pos, int node_id);
int index_FindNodeIndex(INDEX_TABLE tab, int node_id);

void index_WriteTable(INDEX_TABLE tab, fsm_type fsm);


#endif
