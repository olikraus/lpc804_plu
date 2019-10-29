/*

  ic_list.h

  the input constraints list, build from the minimum symbolic list of the fsm

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

#ifndef _IC_LIST_H
#define _IC_LIST_H

#include <stdio.h>

#include "min_symb_list.h"
#include "dcube.h"

typedef struct _ic_list *IC_LIST;

struct _ic_list
{
  dcube *constraint;        /* the constraint */
  int weight;               /* means how many times the constraint appears in the symbolic list of the fsm */
                       
  IC_LIST next;
};

/* INTERFACE functions */

IC_LIST ic_OpenList();
void ic_CloseList(IC_LIST icList);
int ic_Cnt(IC_LIST icList);
IC_LIST ic_CreateICList(MIN_SYMB_LIST constrList, pinfo *pConstr);
IC_LIST ic_CreateICClosure(IC_LIST icList, pinfo *pConstr);

/* AUXILIARY functions, NOT to be used externly */

IC_LIST ic_AddConstr(IC_LIST icList,  dcube *constraint, int weight, pinfo *pConstr );
IC_LIST ic_AddUniversum(IC_LIST icList, pinfo *pConstr);
IC_LIST ic_AddStates(IC_LIST icList, pinfo *pConstr);

/* function for WRITING the ic list */

void ic_WriteList(IC_LIST icList, pinfo *pConstr);

/* functions for converting to and from a dclist */

int ic_ICTodclist(IC_LIST icList, dclist *dcl, pinfo *pConstr);
IC_LIST ic_DCToiclist(dclist dcl, pinfo *pConstr);


#endif
