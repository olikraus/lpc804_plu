/*

  ic_list.c

  the input constraints list

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
#include "ic_list.h"
#include "min_symb_list.h"
#include "dcube.h"
#include "extra.h"

/* IC_LIST */

/* --------- ic_OpenList --------------------------------------*/

IC_LIST ic_OpenList()
{
  return NULL;
}


/* --------- ic_AddConstr --------------------------------------*/

IC_LIST ic_AddConstr(IC_LIST icList,  dcube *constraint, int weight, pinfo *pConstr )
{
  /* adds a constraint to the list such that the list is sorted (desc)
   * a new constraint has the weight = 1; 
   * if constraint exists in the lsit then the corresponding weight is incremented
   */
  
  IC_LIST tmp, prev, aux;
  
  
  if( ( icList==NULL ) || (extra_dcOutOneCnt(constraint, pConstr) > extra_dcOutOneCnt(icList->constraint, pConstr)))
  {
    /* insert it at the beginning of the list */
    if((aux = (IC_LIST) malloc(sizeof(struct _ic_list))) == NULL)
      return NULL;
    
    if((aux->constraint = (dcube*) malloc(sizeof(dcube))) == NULL)
      return NULL;
    
    dcInit(pConstr, aux->constraint);
    dcCopy(pConstr, aux->constraint, constraint);
    aux ->weight = weight;
    aux->next = icList;
    return aux;
  }
  
  /* test if it is not equal with the first one */
  if(dcIsEqual(pConstr, constraint, icList->constraint))
  {
    icList->weight++;
    return icList;
  }
  
  /* find its place and then insert it */
  
  prev = icList;
  tmp = prev->next;
  
  while(tmp && (extra_dcOutOneCnt(constraint, pConstr) < extra_dcOutOneCnt(tmp->constraint, pConstr)) )
  {
    prev = tmp;
    tmp = tmp->next;
  }
  
  while(tmp && (extra_dcOutOneCnt(constraint, pConstr) == extra_dcOutOneCnt(tmp->constraint, pConstr)))  
  {
    if(dcIsEqual(pConstr, constraint, tmp->constraint))
    {
      tmp->weight ++;
      return icList;
    }
    else
    {
      prev = tmp;
      tmp = tmp->next;
    }
  }
  
  /* insert it after prev */
  
  if((aux = (IC_LIST) malloc(sizeof(struct _ic_list))) == NULL)
    return NULL;
 
  if((aux->constraint = (dcube*) malloc(sizeof(dcube))) == NULL)
    return NULL;
  
  dcInit(pConstr, aux->constraint);

  dcCopy(pConstr, aux->constraint, constraint);
  aux->weight = weight;
  aux->next = tmp;
  prev->next = aux;
  return icList;
}

/* --------- ic_CloseList --------------------------------------*/

void ic_CloseList(IC_LIST icList)
{
  IC_LIST tmp, aux;

  for(tmp = icList; tmp != NULL;)
  {
    aux = tmp;
    tmp = tmp->next;
    
    dcDestroy(aux->constraint);
    
    free(aux->constraint);
    
    free(aux);
  }
}

/* --------- ic_WriteList --------------------------------------*/

void ic_WriteList(IC_LIST icList, pinfo *pConstr)
{
  IC_LIST tmp;
  int cnt;
  
  cnt = 0;

  printf("----BEGIN INPUT CONTRAINTS LIST----\n");

  for(tmp = icList; tmp != NULL; tmp = tmp->next)
  {
    printf("%s; weight = %d\n", dcToStr(pConstr, tmp->constraint, "", ""), tmp->weight);
    cnt++;
  }
  printf("There were %d\n", cnt);
  printf("----END INPUT CONSTRAINTS LIST----\n");

}


/* --------- ic_Cnt --------------------------------------*/

int ic_Cnt(IC_LIST icList)
{
  IC_LIST tmp;
  int cnt = 0;
  
  for(tmp = icList; tmp; tmp=tmp->next)
    cnt++;
  
  return cnt;
}

/* --------- ic_CreateICList --------------------------------------*/

IC_LIST ic_CreateICList(MIN_SYMB_LIST constrList, pinfo *pConstr)
{
  MIN_SYMB_LIST tmp;
  IC_LIST icList;
  
  icList = ic_OpenList();
  
  for(tmp = constrList; tmp != NULL; tmp = tmp->next)
  {
    if((icList = ic_AddConstr(icList,  tmp->group, 1, pConstr )) == NULL)
      return NULL;
  }
  
  return icList;
  
}


/* --------- ic_AddUniversum --------------------------------------*/

IC_LIST ic_AddUniversum(IC_LIST icList, pinfo *pConstr)
{
  dcube univ;
  int k, i;
  
  dcInit(pConstr, &univ);
  
  k = pinfoGetOutCnt(pConstr);
  
  for(i=0; i< k; i++) 
    dcSetOut( &univ, i, 1);
  
  icList = ic_AddConstr(icList, &univ,1,  pConstr);
  
  dcDestroy(&univ);
 
  return icList; 
}

/* --------- ic_AddStates --------------------------------------*/

IC_LIST ic_AddStates(IC_LIST icList, pinfo *pConstr)
{
  dcube state;
  int nrStates = pinfoGetOutCnt(pConstr);
  int i;
  
  dcInit(pConstr, &state);
  
  for(i=0; i< nrStates; i++)
  {
    dcOutSetAll(pConstr, &state, 0);
    dcSetOut(&state, i, 1);
    icList = ic_AddConstr(icList, &state,1,  pConstr);
  }
  
  dcDestroy(&state);
  return icList;
}

/* --------- ic_CreateICClosure --------------------------------------*/

IC_LIST ic_CreateICClosure(IC_LIST icList, pinfo *pConstr)
{
  dcube inters;
  IC_LIST tmp, prev;
  IC_LIST icList1;
  
  icList1 = ic_OpenList();
  
  dcInit(pConstr, &inters);
  
  for(prev = icList; prev != NULL; prev = prev->next)
    for(tmp = prev->next; tmp != NULL; tmp = tmp->next)
      if ( dcIntersection(pConstr, &inters, tmp->constraint, prev->constraint) != 0)
        icList1 = ic_AddConstr(icList1, &inters,1,  pConstr);
  
  
  for(tmp = icList; tmp != NULL; tmp = tmp->next)
    icList1 = ic_AddConstr(icList1, tmp->constraint, tmp->weight, pConstr);
  
  icList1 = ic_AddUniversum(icList1, pConstr);
  icList1 = ic_AddStates(icList1, pConstr);        
  
  dcDestroy(&inters);
 
  ic_CloseList(icList);
  
  return icList1;
}


/* --------- ic_ICTodclist --------------------------------------*/

int ic_ICTodclist(IC_LIST icList, dclist *dcl, pinfo *pConstr)
{
  IC_LIST tmp;
  
  for(tmp = icList; tmp != NULL; tmp = tmp->next)
    if( (dclAdd(pConstr, *dcl, tmp->constraint)) < 0)
      return 0;
  
  return 1;
}

/* --------- ic_DCToiclist --------------------------------------*/

IC_LIST ic_DCToiclist(dclist dcl, pinfo *pConstr)
{
  IC_LIST icList;
  int cnt, i;
  
  cnt = dclCnt(dcl);
  
  icList = ic_OpenList();
  
  for(i = 0; i < cnt; i++)
    icList = ic_AddConstr(icList, dclGet(dcl, i),1, pConstr);
  
  return icList;
  
}
