/*

  code_list.h
  
  Description of the list of constraints and assigned codes; uses the graph of
  constraints described in "cg_graph.h"

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

#ifndef _CODE_LIST_H
#define _CODE_LIST_H

#include <stdio.h>

#include "dcube.h"
#include "cg_graph.h"

/* just one list */

typedef struct _code_list *CODE_LIST;

struct _code_list
{
  dcube *constr; /* the constraint is given as a dcube with no input part, just output */ 
  dcube *code;   /* the code is given as a dcube with no output, just input part */
  CODE_LIST next;
};

/* an array of lists, organized by constraint level in the constraint graph */

typedef struct _code_list_vector *CODE_LIST_VECTOR;

struct _code_list_vector
{
  int nr_levels;      /* how big the vector is */
  CODE_LIST* lists;  /* the array of lists, organized by levels */
};


/* INTERFACE functions */

CODE_LIST_VECTOR code_OpenCodeListVector(int n);
void code_CloseCodeListVector(CODE_LIST_VECTOR clv);
CODE_LIST_VECTOR code_AddPairToListVector(CODE_LIST_VECTOR clv, int level, dcube *constr, 
  dcube *code, pinfo *pConstr, pinfo *pCode);
CODE_LIST_VECTOR code_ReallocCodeListVector(CODE_LIST_VECTOR clv, int new_n);

/* assign codes to the graph */
int code_AssignCodes(CONSTR_GRAPH cGraph, CODE_LIST_VECTOR codeList, pinfo *pConstr, pinfo *pCode, 
  KID_LIST *bad);
int code_AssignCodeToPrimaryConstr(CONSTR_GRAPH cGraph, int primNode, int listLevel, 
  CODE_LIST_VECTOR codeList, pinfo *pConstr, pinfo *pCode);
int code_AssignCodeToKidConstr(CONSTR_GRAPH cGraph, int kidNode, int fatherNode, dcube fatherCode, 
  int listLevel, CODE_LIST_VECTOR codeList, pinfo *pConstr, pinfo *pCode, int* user_data);

int code_CopyCodes(CODE_LIST_VECTOR codeList, fsm_type fsm, 
  INDEX_TABLE tab, pinfo *pConstr, pinfo *pCode);

/* AUXILIARY functions */

/* CODE_LIST */

CODE_LIST code_OpenCodeList();
CODE_LIST code_AddPairToList(CODE_LIST codeList, dcube *constr, dcube *code, pinfo *pConstr, pinfo *pCode);
CODE_LIST code_DeleteTopPairFromList(CODE_LIST codeList);
CODE_LIST code_DeleteConstrFromList (CODE_LIST codeList, dcube *constr, pinfo *pConstr);
void code_CloseCodeList(CODE_LIST codeList);
void code_WriteCodeList(CODE_LIST codeList, pinfo *pConstr, pinfo *pCode);
int code_FindConstrInList(CODE_LIST codeList, dcube *constr, dcube *code, pinfo *pConstr, pinfo *pCode);
int code_FindCodeInList(CODE_LIST codeList, dcube *code, pinfo *pCode);


/* CODE_LIST_VECTOR */

int code_FindConstrOnLevelInListVector(CODE_LIST_VECTOR clv, int level, dcube *constr, 
  pinfo *pConstr, dcube *code, pinfo *pCode);
int code_FindConstrInListVector(CODE_LIST_VECTOR clv,  dcube *constr, 
  pinfo *pConstr, dcube *code, pinfo *pCode);
int code_FindCodeInListVector(CODE_LIST_VECTOR clv,  dcube *code, pinfo *pCode);
CODE_LIST_VECTOR code_DeleteNodeFromListVector(CODE_LIST_VECTOR clv, int level, 
  CONSTR_GRAPH cGraph, int cNode, pinfo *pConstr);
int code_IsCompatible(CODE_LIST_VECTOR clv,  dcube *constr1, dcube* constr2, dcube *code1, 
  dcube * code2, pinfo * pConstr, pinfo *pCode);
int code_SetSatisfiedConstr(MIN_SYMB_LIST minList, CODE_LIST_VECTOR codeList, pinfo *pConstr);
CODE_LIST_VECTOR code_CopyCodeListVector(CODE_LIST_VECTOR clv, pinfo *pConstr, pinfo *pCode);

/* function for WRITING the code list vector */

void code_WriteCodeListVector(CODE_LIST_VECTOR clv, pinfo *pConstr, pinfo *pCode);

#endif
