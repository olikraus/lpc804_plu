/*

  min_symb_list.c

  the minimized symbolic implicants list  

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
#include <string.h>

#include "fsm.h"
#include "index_table.h"
#include "min_symb_list.h"
#include "extra.h"

/* MIN_SYMB_LIST */

/* --------- min_OpenList --------------------------------------*/

MIN_SYMB_LIST min_OpenList()
{
  return NULL;
}

MIN_SYMB_LIST min_AllocTerm()
{
  MIN_SYMB_LIST term;
    
  if( (term = (MIN_SYMB_LIST) malloc(sizeof(struct _min_symb_list))) == NULL)
    return NULL;
  
  if ( (term->group = (dcube*) malloc(sizeof(dcube))) == NULL)
    return NULL;

  if ( (term->dest = (dcube*) malloc(sizeof(dcube))) == NULL)
    return NULL;

  if ( (term->condition = (dcube*) malloc(sizeof(dcube))) == NULL)
    return NULL;

    
  return term;

}

MIN_SYMB_LIST min_CopyTerm(MIN_SYMB_LIST term, dcube *condition, dcube *group, dcube * dest, pinfo *pCond, pinfo *pGroup)
{
  if(!term)
    return NULL;
    
  dcInit(pGroup, term->group);
  dcInit(pGroup, term->dest);
  dcInit(pCond, term->condition);

    
  dcCopy(pGroup, term->group, group);
  dcCopy(pGroup, term->dest, dest);
  dcCopy(pCond, term->condition, condition);

  term->is_valid = 1;

  return term;
}


MIN_SYMB_LIST min_AddTerm(MIN_SYMB_LIST minList, dcube *condition, dcube *group, dcube * dest, pinfo *pCond, pinfo *pGroup)
{
  /* adds a term at the end og the corresponding group */
  
  MIN_SYMB_LIST tmp, aux, prev;
  
  if( (aux = min_AllocTerm()) == NULL)
    return NULL;
    
  aux = min_CopyTerm(aux,condition,  group, dest,  pCond, pGroup);
    
  if(minList == NULL)
  { 
    aux->next = NULL;
    return aux;
  }
  
  for(tmp = minList; tmp && !dcIsEqual(pGroup, tmp->dest, dest);  tmp = tmp->next);
  
  if(!tmp)
  {
    /* insert at the real end */
    for(prev = minList; prev->next != tmp; prev = prev->next);
    aux->next = NULL;
    prev->next = aux;
    return minList;
  }
  
  for( ; tmp && dcIsEqual(pGroup, tmp->dest, dest) &&  (extra_dcOutOneCnt(group, pGroup) < extra_dcOutOneCnt(tmp->group, pGroup))
   ;  tmp = tmp->next);
  
  /* insert after prev */
  if(tmp == minList)
  {
    aux->next = minList;
    minList = aux;
  }
  else
  {
    for(prev = minList; prev->next != tmp; prev = prev->next);

    aux->next = tmp;
    prev->next = aux;
  }
 
  return minList;
 
}

/* --------- min_CloseTerm --------------------------------------*/

void min_CloseTerm(MIN_SYMB_LIST term)
{
  if(term)
  {
    dcDestroy(term->group);
    dcDestroy(term->dest);
    dcDestroy(term->condition);
    
    free(term->group);
    free(term->dest);
    free(term->condition);
    
    free(term);
  }
}

/* --------- min_CloseList --------------------------------------*/

void min_CloseList(MIN_SYMB_LIST minList)
{
  MIN_SYMB_LIST tmp, aux;

  for(tmp = minList; tmp != NULL;)
  {
    aux = tmp;
    tmp = tmp->next;
    
    min_CloseTerm(aux);
  }
}

/* --------- min_DeleteInvalideTerms --------------------------------------*/

MIN_SYMB_LIST min_DeleteInvalideTerms(MIN_SYMB_LIST minList)
{
  MIN_SYMB_LIST tmp, aux, prev;
  
  if(!minList)
    return NULL;
    
  while(!minList->is_valid)
  {
    aux = minList;
    minList = minList->next;
    min_CloseTerm(aux);
  }
  
  /* now minList points to something valid */
  for(prev = minList, tmp = minList->next; tmp;)
  {
    if(tmp->is_valid)
    {
      prev = tmp;
      tmp = tmp->next;
    }
    else
    {
      aux = tmp;
      tmp = tmp->next;
      prev->next = tmp;
      min_CloseTerm(aux);
    }
  }
  
  return minList;
}

/* --------- min_WriteList --------------------------------------*/

void min_WriteList(MIN_SYMB_LIST minList, pinfo *pCond, pinfo *pGroup )
{
  MIN_SYMB_LIST tmp;
  int cnt;
    
  cnt = 0;

  printf("----BEGIN MINIMIZED SYMBOLIC LIST----\n");

  for(tmp = minList; tmp != NULL; tmp = tmp->next)
  {
      printf("%s ", dcToStr(pCond, tmp->condition, "",""));
      printf("%s ", dcToStr(pGroup, tmp->group, "",""));
      printf("%s ", dcToStr(pGroup, tmp->dest, "",""));
      if(tmp->is_valid)
        printf(" is valid\n");
      else
        printf(" is NOT valid\n");
      cnt++;
   }
  printf("There were %d\n", cnt);
  printf("----END MINIMIZED SYMBOLIC LIST----\n");

}

/* --------- min_WriteList --------------------------------------*/

void min_WriteList1(MIN_SYMB_LIST minList, fsm_type fsm, INDEX_TABLE tab, pinfo *pGroup )
{
  MIN_SYMB_LIST tmp;
  int k, i;
  int cnt, node_id;
  pinfo *pCond;
  
  pCond = fsm_GetConditionPINFO(fsm);
  
  k = pinfoGetOutCnt(pGroup);
  
  cnt = 0;

  printf("----BEGIN MINIMIZED SYMBOLIC LIST----\n");

  for(tmp = minList; tmp != NULL; tmp = tmp->next)
  {
    
      printf("%s ", dcToStr(pCond, tmp->condition, "",""));
      
      for(i = 0; i < k; i++)
      {
        if(dcGetOut(tmp->group, i) == 1)
        {
          node_id = index_GetNodeId(tab, i);
          printf("%s ", fsm_GetNodeName(fsm, node_id));
        }
      }

      printf("-->");
      
      for(i = 0; i < k; i++)
      {
        if(dcGetOut(tmp->dest, i) == 1)
        {
          node_id = index_GetNodeId(tab, i);
          printf("%s ", fsm_GetNodeName(fsm, node_id));
        }
      }
            
      cnt++;
   }
  printf("There were %d\n", cnt);
  printf("----END MINIMIZED SYMBOLIC LIST----\n");

}


/* --------- min_ExistTerm --------------------------------------*/

int min_ExistTerm(MIN_SYMB_LIST start, MIN_SYMB_LIST end, dcube * cond, dcube *group, pinfo *pCond, pinfo * pConstr)
{
  MIN_SYMB_LIST tmp;
  
  for(tmp = start; tmp != end; tmp = tmp->next)
    if( dcIsEqual(pCond, cond, tmp->condition) && dcIsEqual(pConstr, group, tmp->group) )
      return 1;
      
  return 0;
}


/* --------- min_BuildGroupsInclude --------------------------------------*/

MIN_SYMB_LIST min_BuildGroupsInclude(fsm_type fsm,  INDEX_TABLE tab, pinfo *pConstr)
{
  MIN_SYMB_LIST constrList;
  unsigned int cnt, cntCond, i, j;
  int dest_node_id, ind_dest, src_node_id, ind_src, edge_id, loop;
  dclist inters;   /* holds the list with all intersections of conditions for a given destination*/
  dcube group;    /* holds the source group */
  dcube dest;
  dcube *c;
  pinfo *pCond = fsm_GetConditionPINFO(fsm);
  int state_cnt = b_set_Cnt(fsm->nodes);
  int flag;
  
  pinfoSetInCnt(pConstr, 0);
  pinfoSetOutCnt(pConstr, state_cnt);

  if(!dclInit( &inters))
    return NULL;
    
  if(!dcInit(pConstr, &group))
    return NULL;
    
  if(!dcInit(pConstr, &dest))
    return NULL;
  
  constrList = min_OpenList();
  
  dest_node_id = -1;
  while( fsm_LoopNodes(fsm, &dest_node_id) != 0 )
  {
    ind_dest = index_FindNodeIndex(tab, dest_node_id);
    dcOutSetAll(pConstr, &dest, 0);
    dcSetOut(&dest, ind_dest, 1);
   
    dclClear(inters);
    dcOutSetAll(pConstr, &group, 0);
    
    loop = -1;
    while( fsm_LoopNodeInEdges(fsm, dest_node_id, &loop, &edge_id) != 0 )
    {
      if( dclSCCUnion(pCond, inters, fsm_GetEdgeCondition(fsm, edge_id))==0)
        return NULL;
    }
    
    /*printf("The original list:\n");
    dclShow(pCond, inters);*/
    
    dclPrimesInv(pCond, inters);
    
    /*printf("The list with the intersesctions:\n");
    dclShow(pCond, inters);*/
   
    cnt = dclCnt(inters);
    for(i = 0; i< cnt; i++)
    {
      dcOutSetAll(pConstr, &group, 0);
      
      c = dclGet(inters, i);

      flag = 0;

      loop = -1;
      while( fsm_LoopNodeInEdges(fsm, dest_node_id, &loop, &edge_id) != 0 )
      {
        cntCond = dclCnt(fsm_GetEdgeCondition(fsm, edge_id));
        for(j = 0; j < cntCond; j++)
        {
          if(dcIsEqual(pCond, dclGet(fsm_GetEdgeCondition(fsm, edge_id), j), c))
          {
            flag = 1;
            src_node_id = fsm_GetEdgeSrcNode(fsm, edge_id);
            ind_src = index_FindNodeIndex(tab, src_node_id);
            dcSetOut(&group, ind_src, 1);
          }
        }
      }
      
      if(flag)
      {
        loop = -1;
        while( fsm_LoopNodeInEdges(fsm, dest_node_id, &loop, &edge_id) != 0 )
        {
          cntCond = dclCnt(fsm_GetEdgeCondition(fsm, edge_id));
          for(j = 0; j < cntCond; j++)
          {
            if(dcIsSubSet(pCond, dclGet(fsm_GetEdgeCondition(fsm, edge_id), j), c))
            {
              src_node_id = fsm_GetEdgeSrcNode(fsm, edge_id);
              ind_src = index_FindNodeIndex(tab, src_node_id);
              dcSetOut(&group, ind_src, 1);
            }
          }
        }
      }

      if(extra_dcOutOneCnt(&group, pConstr) > 1)
        constrList = min_AddTerm(constrList, c, &group, &dest,  pCond, pConstr);
    }
  }

  dclDestroy(inters);
  dcDestroy(&group);  
  dcDestroy(&dest);  
   
  return constrList;
}


/* --------- min_BuildGroupsComplex --------------------------------------*/

MIN_SYMB_LIST min_BuildGroupsComplex(fsm_type fsm,  INDEX_TABLE tab, pinfo *pConstr)
{
  MIN_SYMB_LIST constrList;
  unsigned int cnt, cntCond, i, j;
  int dest_node_id, ind_dest, src_node_id, ind_src, edge_id, loop;
  dclist inters;   /* holds the list with all intersections of conditions for a given destination*/
  dcube group;    /* holds the source group */
  dcube dest;
  dcube *c;
  pinfo *pCond = fsm_GetConditionPINFO(fsm);
  int state_cnt = b_set_Cnt(fsm->nodes);
  
  pinfoSetInCnt(pConstr, 0);
  pinfoSetOutCnt(pConstr, state_cnt);

  if(!dclInit( &inters))
    return NULL;
    
  if(!dcInit(pConstr, &group))
    return NULL;
    
  if(!dcInit(pConstr, &dest))
    return NULL;
  
  constrList = min_OpenList();
  
  dest_node_id = -1;
  while( fsm_LoopNodes(fsm, &dest_node_id) != 0 )
  {
    ind_dest = index_FindNodeIndex(tab, dest_node_id);
    dcOutSetAll(pConstr, &dest, 0);
    dcSetOut(&dest, ind_dest, 1);
   
    dclClear(inters);
    dcOutSetAll(pConstr, &group, 0);
    
    loop = -1;
    while( fsm_LoopNodeInEdges(fsm, dest_node_id, &loop, &edge_id) != 0 )
    {
      if( dclSCCUnion(pCond, inters, fsm_GetEdgeCondition(fsm, edge_id))==0)
        return NULL;
    }
    
    /*printf("The original list:\n");
    dclShow(pCond, inters);*/
    
    dclPrimesInv(pCond, inters);
    
    /*printf("The list with the intersesctions:\n");
    dclShow(pCond, inters);*/
   
    cnt = dclCnt(inters);
    for(i = 0; i< cnt; i++)
    {
      dcOutSetAll(pConstr, &group, 0);
      
      c = dclGet(inters, i);
     
      loop = -1;
      while( fsm_LoopNodeInEdges(fsm, dest_node_id, &loop, &edge_id) != 0 )
      {
        cntCond = dclCnt(fsm_GetEdgeCondition(fsm, edge_id));
        for(j = 0; j < cntCond; j++)
        {
          if(dcIsSubSet(pCond, dclGet(fsm_GetEdgeCondition(fsm, edge_id), j), c))
          {
            src_node_id = fsm_GetEdgeSrcNode(fsm, edge_id);
            ind_src = index_FindNodeIndex(tab, src_node_id);
            dcSetOut(&group, ind_src, 1);
          }
        }
      }
      
      if(extra_dcOutOneCnt(&group, pConstr) > 1)
        constrList = min_AddTerm(constrList, c, &group, &dest,  pCond, pConstr);
    }
  }

  dclDestroy(inters);
  dcDestroy(&group);  
  dcDestroy(&dest);  
   
  return constrList;

}


/* --------- min_BuildGroupsEasy --------------------------------------*/

MIN_SYMB_LIST min_BuildGroupsEasy(fsm_type fsm,  INDEX_TABLE tab, pinfo *pConstr)
{
  MIN_SYMB_LIST constrList;
  unsigned int cnt, cntCond, i, j;
  int dest_node_id, ind_dest, src_node_id, ind_src, edge_id, loop;
  dclist inters;   /* holds the list with all intersections of conditions for a given destination*/
  dcube group;    /* holds the source group */
  dcube dest;
  dcube *c;
  pinfo *pCond = fsm_GetConditionPINFO(fsm);
  int state_cnt = b_set_Cnt(fsm->nodes);
  
  pinfoSetInCnt(pConstr, 0);
  pinfoSetOutCnt(pConstr, state_cnt);

  if(!dclInit( &inters))
    return NULL;
    
  if(!dcInit(pConstr, &group))
    return NULL;
    
  if(!dcInit(pConstr, &dest))
    return NULL;
  
  constrList = min_OpenList();
  
  dest_node_id = -1;
  while( fsm_LoopNodes(fsm, &dest_node_id) != 0 )
  {
    ind_dest = index_FindNodeIndex(tab, dest_node_id);
    dcOutSetAll(pConstr, &dest, 0);
    dcSetOut(&dest, ind_dest, 1);
   
    dclClear(inters);
    dcOutSetAll(pConstr, &group, 0);
    
    loop = -1;
    while( fsm_LoopNodeInEdges(fsm, dest_node_id, &loop, &edge_id) != 0 )
    {
      if( dclSCCUnion(pCond, inters, fsm_GetEdgeCondition(fsm, edge_id))==0)
        return NULL;
    }
    
    /*printf("The original list:\n");
    dclShow(pCond, inters);*/
    
    dclPrimesInv(pCond, inters);
    
    /*printf("The list with the intersesctions:\n");
    dclShow(pCond, inters);*/
   
    cnt = dclCnt(inters);
    for(i = 0; i< cnt; i++)
    {
      dcOutSetAll(pConstr, &group, 0);
      
      c = dclGet(inters, i);
     
      loop = -1;
      while( fsm_LoopNodeInEdges(fsm, dest_node_id, &loop, &edge_id) != 0 )
      {
        cntCond = dclCnt(fsm_GetEdgeCondition(fsm, edge_id));
        for(j = 0; j < cntCond; j++)
        {
          if(dcIsEqual(pCond, dclGet(fsm_GetEdgeCondition(fsm, edge_id), j), c))
          {
            src_node_id = fsm_GetEdgeSrcNode(fsm, edge_id);
            ind_src = index_FindNodeIndex(tab, src_node_id);
            dcSetOut(&group, ind_src, 1);
          }
        }
      }
      
      if(extra_dcOutOneCnt(&group, pConstr) > 1)
        constrList = min_AddTerm(constrList, c, &group, &dest,  pCond, pConstr);
    }
  }

  dclDestroy(inters);
  dcDestroy(&group);  
  dcDestroy(&dest);  
   
  return constrList;
    
}

/* --------- min_BuildGroups --------------------------------------*/

MIN_SYMB_LIST min_BuildGroups(fsm_type fsm, INDEX_TABLE tab, pinfo *pConstr, char *method)
{
  MIN_SYMB_LIST constrList;

  if(strcmp(method, "EASY")==0)
    constrList = (MIN_SYMB_LIST) min_BuildGroupsEasy(fsm, tab, pConstr);
  else
    if(strcmp(method, "COMPLEX")==0)
      constrList = (MIN_SYMB_LIST) min_BuildGroupsComplex(fsm, tab, pConstr);
    else
      if(strcmp(method, "INCLUDE")==0)
        constrList = (MIN_SYMB_LIST) min_BuildGroupsInclude(fsm, tab, pConstr);
      else
        constrList = (MIN_SYMB_LIST) min_BuildGroupsEasy(fsm, tab, pConstr);

  return constrList;
}



/* --------- min_InvalidateAll --------------------------------------*/

void min_InvalidateAll(MIN_SYMB_LIST minList)
{
  MIN_SYMB_LIST tmp;
  
  for(tmp = minList; tmp; tmp = tmp->next)
    tmp->is_valid = 0;
}

/* --------- min_SetValid --------------------------------------*/

void min_SetValid(MIN_SYMB_LIST minList, dcube *group, pinfo *pConstr)
{
  MIN_SYMB_LIST tmp;
  
  for(tmp = minList; tmp; tmp = tmp->next)
  {
    if(dcIsEqual(pConstr, group, tmp->group))
      tmp->is_valid = 1;
  }
}


/* --------- min_GetCnt --------------------------------------*/

int min_GetCnt(MIN_SYMB_LIST minList)
{
  MIN_SYMB_LIST tmp;
  int cnt = 0;
  
  for(tmp = minList; tmp; tmp = tmp->next)
    cnt++;
    
  return cnt;
}
