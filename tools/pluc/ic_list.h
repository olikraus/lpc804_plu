/*

  ic_list.h

  the input constraints list, build from the minimum symbolic list of the fsm
  
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
