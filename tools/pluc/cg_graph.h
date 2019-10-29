/*

  cg_graph.h
  
  Description of the input constraints as a graph, created from an input constraint list

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

#ifndef _CG_GRAPH_H
#define _CG_GRAPH_H

#include <stdio.h>

#include "dcube.h"
#include "fsm.h"
#include "ic_list.h"


typedef struct _kid_list *KID_LIST;

struct _kid_list
{
  int id;
  KID_LIST next;   
};


/* the CONSTR_NODE : one node in the graph */

typedef struct _constr_node *CONSTR_NODE;

struct _constr_node
{
  dcube constr;   /* the constraint as a cube with no input, just output */
  int nr_kids;    /* how many children the node has */
  KID_LIST kids;    /* the list of kids in the reverse order of cardinality, ie first are
                   * coming the kids with a higher cardinality
                   */
                   
  int is_valid;
};


/* the CONSTR_GRAPH : the whole graph */


typedef struct _constr_graph *CONSTR_GRAPH;

struct _constr_graph
{
  int nrNodes;          /* the number of nodes in the graph */
  CONSTR_NODE* nodes;   /* the vector with the nodes structures */
};

/* INTERFACE functions */

CONSTR_GRAPH cg_OpenGraph( int nrNodes, pinfo *pConstr);
void cg_CloseGraph(CONSTR_GRAPH cGraph);
CONSTR_GRAPH cg_CreateGraph(IC_LIST icList, pinfo *pConstr);
int cg_GetNrPrimConstr(CONSTR_GRAPH cGraph);

/* AUXILIARY functions */

/* KID_LIST */

KID_LIST cg_OpenKidList();
KID_LIST cg_DeleteTopKid(KID_LIST kidList);
KID_LIST cg_DeleteKid(KID_LIST kidList, KID_LIST kid);
void cg_CloseKidList(KID_LIST kids);
KID_LIST cg_AddKidAsTopInList(KID_LIST kids, int id);
KID_LIST cg_AddKidOrderedDescInList(KID_LIST kids, int id, CONSTR_GRAPH cGraph, pinfo *pConstr);




/* CONSTR_NODE */

CONSTR_NODE cg_OpenConstrNode();
void cg_CloseConstrNode(CONSTR_NODE cNode);
void cg_CopyConstr(CONSTR_GRAPH cGraph, int cId, dcube *constr, pinfo *pConstr);

/* CONSTR_GRAPH */

CONSTR_GRAPH cg_ReadaptGraph(CONSTR_GRAPH cGraph, KID_LIST bad, pinfo *pConstr);
CONSTR_GRAPH cg_ChangeOrderOfPrimConstr(CONSTR_GRAPH cGraph, pinfo *pConstr);
int cg_GetConstrId(CONSTR_GRAPH cGraph, dcube *constr, pinfo *pConstr);
int cg_IsAlreadyIncluded (CONSTR_GRAPH cGraph, int node_id, dcube *constr, pinfo *pConstr);
CONSTR_GRAPH cg_AddNewPrimConstrs(CONSTR_GRAPH cGraph, KID_LIST next_constr, pinfo *pConstr);
CONSTR_GRAPH cg_DeletePrimConstrFromGraph(CONSTR_GRAPH cGraph, KID_LIST kids);
int cg_GetMinNrBits1(CONSTR_GRAPH cGraph, pinfo *pConstr);
int cg_GetMinNrBits2(CONSTR_GRAPH cGraph, pinfo *pConstr);

/* function for WRITING the constraint graph */

void cg_WriteGraph(CONSTR_GRAPH cGraph, fsm_type fsm, INDEX_TABLE tab,  pinfo *pConstr);


#endif
