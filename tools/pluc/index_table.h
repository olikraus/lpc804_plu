/*

  index_table.h
  
  Description of the index table for fsm nodes;
  Creates and ordering of the fsm nodes continuous from 0 to maxNodes - 1
  
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
