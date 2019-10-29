/*

  min_symb_list.h

  the minimized symbolic implicants list of the fsm 

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

#ifndef _MIN_SYMB_LIST_H
#define _MIN_SYMB_LIST_H

#include <stdio.h>

#include "dcube.h"
#include "fsm.h"
#include "index_table.h"

/* one implicant */

typedef struct _min_symb_list *MIN_SYMB_LIST;

struct _min_symb_list
{
  dcube* condition;   /* condition for the transition; has as pinfo the ConditionPNFO from
                       * the fsm
                       */
  dcube *group;       /* the group that makes the transition */
  dcube *dest;        /* the destination for the transition; group and dest MUST have
                       * another pinfo which MUST be created somewhere; in = 0;
                       * out = nr of states
                       */
  int is_valid;       /* used for creating the minimized symbolic list 
                       */
                       
  MIN_SYMB_LIST next;
};

/* The functions */

/* INTERFACE FUNCTIONS */

MIN_SYMB_LIST min_OpenList();
void min_CloseList(MIN_SYMB_LIST minList);
int min_GetCnt(MIN_SYMB_LIST minList);

/* the actual functions for building the minimum symbolic list = the groups */

MIN_SYMB_LIST min_BuildGroupsInclude(fsm_type fsm,  INDEX_TABLE tab, pinfo *pConstr);
MIN_SYMB_LIST min_BuildGroupsComplex(fsm_type fsm,  INDEX_TABLE tab, pinfo *pConstr);
MIN_SYMB_LIST min_BuildGroupsEasy(fsm_type fsm,  INDEX_TABLE tab, pinfo *pConstr);
MIN_SYMB_LIST min_BuildGroups(fsm_type fsm, INDEX_TABLE tab, pinfo *pConstr, char *method);

/* AUXILIARY functions not to be called externly */

MIN_SYMB_LIST min_AllocTerm();
MIN_SYMB_LIST min_CopyTerm(MIN_SYMB_LIST term, dcube *condition, dcube *group, dcube * dest, 
  pinfo *pCond, pinfo *pGroup);
MIN_SYMB_LIST min_AddTerm(MIN_SYMB_LIST minList, dcube *condition, dcube *group, dcube * dest, 
  pinfo *pCond, pinfo *pGroup);
void min_CloseTerm(MIN_SYMB_LIST term);
MIN_SYMB_LIST min_DeleteInvalideTerms(MIN_SYMB_LIST minList);

void min_InvalidateAll(MIN_SYMB_LIST minList);
void min_SetValid(MIN_SYMB_LIST minList, dcube *group, pinfo *pConstr);

/* functions for WRITING the list */

void min_WriteList(MIN_SYMB_LIST minList, pinfo *pCond, pinfo *pGroup );
void min_WriteList1(MIN_SYMB_LIST minList, fsm_type fsm, INDEX_TABLE tab, pinfo *pGroup );
int min_ExistTerm(MIN_SYMB_LIST start, MIN_SYMB_LIST end, dcube * cond, dcube *group, 
  pinfo *pCond, pinfo * pConstr);


#endif
