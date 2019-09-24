/*

  mis.h

  minimization of finite state machines

  Copyright (C) 2001 Oliver Kraus (olikraus@yahoo.com)

  This file is part of DGC.

  DGC is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  DGC is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with DGC; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA  

  Reference: Chris J. Myers "Asynchronous Circuit Design", Wiley 2001
  pages 140 - 154    

*/

#ifndef _MIS_H
#define _MIS_H

#include "xbm.h"
#include "fsm.h"
#include "b_bcp.h"

struct _pr_struct
{
  int s1;
  int s2;
};
typedef struct _pr_struct pr_struct;

#define PAIR_T_UNKNOWN 0
#define PAIR_T_ALWAYS_COMPATIBLE 1
#define PAIR_T_CONDITIONAL_COMPATIBLE 2
#define PAIR_T_NOT_COMPATIBLE 3

#define PAIR_T_NOT_DONE 4
#define PAIR_T_DONE 5


/*  
  pair_type has different meaning in each context...
  as part of the half-matrix:
    t, pr_ptr and pr_cnt are used
  as part of the c_list:
    compatibles, pr_ptr and pr_cnt are used
*/

struct _pair_struct
{
  int t;
  dcube compatible;         /* pi_cl */
  pr_struct *pr_ptr;
  size_t pr_cnt;
};
typedef struct _pair_struct *pair_type;

struct _mis_struct
{
  fsm_type fsm;
  xbm_type xbm;
  
  int state_cnt;
  int *state_to_node;
  
  int node_cnt;
  int *node_to_state;
  
  
  pair_type *pair_list_ptr;   /* a half matrix */
  size_t pair_list_size;      /* (state_cnt-1)*state_cnt/2; */
  
  
  b_pl_type c_list;           /* a list of pair_type objects */
  
  
  pinfo *pi_imply;
  pinfo *pi_cl;               /* compatible list */
  
  pinfo *pi_mcl;              /* maximum compatible list (problem info) */
  dclist cl_mcl;              /* maximum compatible list */
  
  b_bcp_type bcp;             /* binate cover problem */
  
};
typedef struct _mis_struct *mis_type;

#define mis_GetCListElement(m,p) ((pair_type)b_pl_GetVal(((m)->c_list),p))

mis_type mis_Open(fsm_type fsm, xbm_type xbm);
void mis_Close(mis_type m);


void mis_MarkAlwaysCompatible(mis_type m);
void mis_MarkNotCompatible(mis_type m);
int mis_MarkConditionalCompatible(mis_type m);
void mis_UnmarkCondictionalCompatible(mis_type m);
int mis_CalculateMCL(mis_type m);
int mis_CopyMCLtoCList(mis_type m);
int mis_CalculateClassSets(mis_type m);
int mis_ShowClassSets(mis_type m);
int mis_BuildPrimeCompatibleList(mis_type m);
int mis_CalculateBCP(mis_type m);
int mis_CalculateGroups(mis_type m);
void mis_ShowStates(mis_type m);

#endif
