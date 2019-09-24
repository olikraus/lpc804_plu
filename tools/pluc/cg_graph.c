/*

  cg_graph.c

 
  
*/

#include <stdlib.h>
#include <stdio.h>
#include <strings.h>
#include <math.h>

#include "cg_graph.h"
#include "extra.h"

/* KID_LIST */

/*-- cg_OpenKidList -------------------------------------------------------------*/

KID_LIST cg_OpenKidList()
{
  return (KID_LIST) NULL;
}

/*-- cg_DeleteTopKid --------------------------------------------------------*/

KID_LIST cg_DeleteTopKid(KID_LIST kidList)
{
  /* deletes the first element from the list and returns the resulting list 
   */
   
  KID_LIST tmp;
  if(kidList == NULL) return NULL;
  
  tmp = kidList->next;
  
  free(kidList);
  
  return tmp;

}

/*-- cg_DeleteKid --------------------------------------------------------*/

KID_LIST cg_DeleteKid(KID_LIST kidList, KID_LIST kid)
{
  KID_LIST tmp;
  
  if(kid == kidList)
    return cg_DeleteTopKid(kidList);
    
  for(tmp = kidList; tmp && tmp->next != kid; tmp = tmp->next);
  
  if(!tmp)
    return kidList;
    
  tmp->next = kid->next;
  
  free(kid);
  return kidList;
    
}

/*-- cg_CloseKidList -----------------------------------------------------------*/

void cg_CloseKidList(KID_LIST kids)
{
  while(kids != NULL)
    kids = cg_DeleteTopKid(kids);

}

/*-- cg_AddKidAsTopInList -----------------------------------------------------------*/

KID_LIST cg_AddKidAsTopInList(KID_LIST kids, int id)
{
  /* adds a kid id at the beginning of the list and returns the new list
   */
  KID_LIST tmp;

  if((tmp = (KID_LIST) malloc(sizeof(struct _kid_list))) == NULL)
    return NULL;

  tmp->id = id;
  tmp->next = kids;

  return tmp;
}

/*-- cg_AddKidOrderedDescInList -----------------------------------------------------------*/

KID_LIST cg_AddKidOrderedDescInList(KID_LIST kids, int id, CONSTR_GRAPH cGraph, pinfo *pConstr)
{
  /* adds a kid id in the list, such that the kid list is in descending order concerning cardinality
   */
  KID_LIST tmp, prev, aux;
  
  
  if( ( kids==NULL ) || (extra_dcOutOneCnt(&(cGraph->nodes[id]->constr), pConstr) >= extra_dcOutOneCnt(&(cGraph->nodes[kids->id]->constr), pConstr)))
  {
    /* insert it at the beginning of the list */
    if((aux = (KID_LIST) malloc(sizeof(struct _kid_list))) == NULL)
      return NULL;
    
    aux->id = id;
    aux->next = kids;
    return aux;
  }
  
  /* find its place and then insert it */
  
  prev = kids;
  tmp = prev->next;
  
  while(tmp && (extra_dcOutOneCnt(&(cGraph->nodes[id]->constr), pConstr) < extra_dcOutOneCnt(&(cGraph->nodes[kids->id]->constr), pConstr)) )
  {
    prev = tmp;
    tmp = tmp->next;
  }
  
  /* insert it after prev */
  if((aux = (KID_LIST) malloc(sizeof(struct _kid_list))) == NULL)
    return NULL;
 
  aux->id = id;
  aux->next = tmp;
  prev->next = aux;
  
  return kids;
}



/* CONSTR_NODE */

/*-- cg_OpenConstrNode -------------------------------------------------------------*/

CONSTR_NODE cg_OpenConstrNode(pinfo *pConstr)
{
  CONSTR_NODE cNode;
  
  if((cNode = (CONSTR_NODE) malloc(sizeof(struct _constr_node))) == NULL)
    return NULL;
    
  if(dcInit(pConstr, &(cNode->constr) ) == 0)
    return NULL;
    
  cNode->nr_kids = 0;
  cNode->is_valid = 1;

  cNode->kids = cg_OpenKidList();
  return cNode;
}

/*-- cg_CloseConstrNode -------------------------------------------------------------*/

void cg_CloseConstrNode(CONSTR_NODE cNode)
{
  if(cNode ==NULL)
    return;
  
  dcDestroy(&(cNode->constr));
  
  cg_CloseKidList(cNode->kids);  
  
  free(cNode);
}

/*-- cg_CopyConstr ------------------------------------------------------------*/

void cg_CopyConstr(CONSTR_GRAPH cGraph, int cId, dcube *constr, pinfo *pConstr)
{
  if(cId >=0 && cId < cGraph->nrNodes)
    dcCopy(pConstr, &(cGraph->nodes[cId]->constr), constr);
}

/* CONSTR_GRAPH */

/*-- cg_OpenGraph -------------------------------------------------------------*/

CONSTR_GRAPH cg_OpenGraph( int nrNodes, pinfo *pConstr)
{
  CONSTR_GRAPH cGraph;
  int i;
  
  if((cGraph = (CONSTR_GRAPH) malloc(sizeof(struct _constr_graph))) == NULL)
    return NULL;

  cGraph->nrNodes = nrNodes;
  
  if((cGraph->nodes = (CONSTR_NODE*) malloc(nrNodes * sizeof(struct _constr_node)) )==NULL)
    return NULL;
  
  for(i=0; i< nrNodes; i++)
    if((cGraph->nodes[i] = cg_OpenConstrNode(pConstr)) == NULL)
      return NULL;
    
  return cGraph; 
}

/*-- cg_CloseGraph -------------------------------------------------------------*/

void cg_CloseGraph(CONSTR_GRAPH cGraph)
{
  int i;

  if(cGraph == NULL)
     return;
  
  for(i=0; i< cGraph->nrNodes; i++)
    cg_CloseConstrNode(cGraph->nodes[i]);
    
  free(cGraph->nodes);
  
  free(cGraph);
  
  return;
}

/*-- cg_GetConstrId -------------------------------------------------------------*/

int cg_GetConstrId(CONSTR_GRAPH cGraph, dcube *constr, pinfo *pConstr)
{
  /* search constr in the graph and returns the id (position) or -1 if it is not there
   */
   
  int i;
   
  if(cGraph == NULL)
     return -1;

  for(i=0; i< cGraph->nrNodes; i++)
    if(cGraph->nodes[i]->is_valid && dcIsEqual(pConstr, constr, &(cGraph->nodes[i]->constr)))
      return i;
      
  return -1;
}

/*-- cg_IsAlreadyIncluded -----------------------------------------------------------*/

int cg_IsAlreadyIncluded (CONSTR_GRAPH cGraph, int node_id, dcube *constr, pinfo *pConstr)
{
  /* searches if the constraint constr is already included in one of the
   * constraints of the list cGraph->nodes[node_id]->kids
   */
   
  KID_LIST tmp;
  
   if(cGraph == NULL)
     return 0;
  
  for(tmp = cGraph->nodes[node_id]->kids; tmp!=NULL; tmp = tmp->next)
  {
    if(dcIsSubSet(pConstr, &(cGraph->nodes[tmp->id]->constr), constr))
      return 1;
  }
  return 0;
}

/*-- cg_CreateConstrGraph -----------------------------------------------------------*/

CONSTR_GRAPH cg_CreateGraph(IC_LIST icList, pinfo *pConstr)
{


  CONSTR_GRAPH cGraph;
  int id, i;
  int nrNodes;
  IC_LIST tmp, prev;

  dcube *possibleFather, *possibleKid;

  nrNodes = ic_Cnt(icList);
  if((cGraph = cg_OpenGraph(nrNodes, pConstr)) == NULL)
    return NULL;

  for(tmp = icList, i=0; tmp != NULL; tmp = tmp->next, i++)
  {
    cg_CopyConstr(cGraph,i, tmp->constraint, pConstr);
  }

  for(prev = icList, i=0; prev != NULL; prev = prev->next, i++)
  {
    for(tmp = prev->next; tmp != NULL; tmp = tmp->next)
    {
      possibleFather = prev->constraint;
      possibleKid = tmp->constraint;

      if(dcIsSubSet(pConstr, possibleFather, possibleKid) != 0)
        if(! cg_IsAlreadyIncluded(cGraph, i, possibleKid, pConstr))
        {
          id = cg_GetConstrId(cGraph, possibleKid, pConstr);
          cGraph->nodes[i]->nr_kids ++;
          cGraph->nodes[i]->kids = cg_AddKidOrderedDescInList(cGraph->nodes[i]->kids, id, cGraph, pConstr);

        }
    }
  }

  return cGraph;
}

/*-- cg_WriteGraph -----------------------------------------------------------*/

void cg_WriteGraph(CONSTR_GRAPH cGraph, fsm_type fsm, INDEX_TABLE tab,  pinfo *pConstr)
{
  KID_LIST tmp;
  
  int i, j;
  int k,node_id;
  
  if(cGraph == NULL)
     return;
     
   k = pinfoGetOutCnt(pConstr);
     
  printf("\n----BEGIN GRAPH----\n");
  printf("%d nodes\n", cGraph->nrNodes);
  
  for(i=0; i < cGraph->nrNodes; i++)
  {
    if(cGraph->nodes[i]->is_valid)
    {
      printf("\nConstraint: ");
      for(j=0; j< k; j++)
      {
        if(dcGetOut(&(cGraph->nodes[i]->constr), j) == 1)
        {
          node_id = index_GetNodeId(tab, j);
          printf("%s+", fsm_GetNodeName(fsm, node_id));
        }
      }
      printf("\nNr kids: %d\n", cGraph->nodes[i]->nr_kids);
      printf("Kids list:\n");
    
      for(tmp = cGraph->nodes[i]->kids; tmp!= NULL; tmp = tmp->next)
      {
        printf("\n New kid\n");
        for(j=0; j< k; j++)
        {
          if(dcGetOut(&(cGraph->nodes[tmp->id]->constr), j) == 1)
          {
            node_id = index_GetNodeId(tab, j);
            printf("%s; ", fsm_GetNodeName(fsm, node_id));
          }
        }
      }
    }
  }
  
  printf("\n----END GRAPH----\n");

}



/*-- cg_AddNewPrimConstrs -----------------------------------------------------------*/

CONSTR_GRAPH cg_AddNewPrimConstrs(CONSTR_GRAPH cGraph, KID_LIST next_constr, pinfo *pConstr)
{
  /* adds the kids of the cGragh->nodes[primConstr->id] as children of the universal constraint 
   * (graph root)
   */
   
  /*KID_LIST next_constr = cGraph->nodes[primConstr->id]->kids;*/
  KID_LIST tmp;
  
  for(tmp = next_constr; tmp != NULL; tmp = tmp->next)
  {
    if(! cg_IsAlreadyIncluded (cGraph, 0, &(cGraph->nodes[tmp->id]->constr), pConstr))
    {
      cGraph->nodes[0]->kids = cg_AddKidOrderedDescInList(cGraph->nodes[0]->kids, tmp->id, cGraph, pConstr );
      cGraph->nodes[0]->nr_kids++;
    }
  }
  
  return cGraph;
}

CONSTR_GRAPH cg_DeletePrimConstrFromGraph(CONSTR_GRAPH cGraph, KID_LIST kids)
{
  cGraph->nodes[kids->id]->is_valid = 0;
  
  cGraph->nodes[0]->kids = cg_DeleteKid(cGraph->nodes[0]->kids, kids);
  cGraph->nodes[0]->nr_kids--;
  
  return cGraph;
}

CONSTR_GRAPH cg_ReadaptGraph(CONSTR_GRAPH cGraph, KID_LIST bad, pinfo *pConstr)
{
  KID_LIST aux;
  
  aux = cGraph->nodes[bad->id]->kids;
  cGraph = (CONSTR_GRAPH) cg_DeletePrimConstrFromGraph(cGraph, bad);
  cGraph = (CONSTR_GRAPH) cg_AddNewPrimConstrs(cGraph, aux, pConstr);
  
  return cGraph;

}


/*-- cg_ChangeOrderOfPrimConstr -----------------------------------------------------------*/

CONSTR_GRAPH cg_ChangeOrderOfPrimConstr(CONSTR_GRAPH cGraph, pinfo *pConstr)
{
  KID_LIST tmp, aux;
  int c1;
  
  c1 = cGraph->nodes[0]->kids->id;
  
  for(tmp = cGraph->nodes[0]->kids; tmp->next != NULL ; tmp = tmp->next);
  
  aux = cGraph->nodes[0]->kids->next;
  cGraph->nodes[0]->kids->next = tmp->next;
  tmp->next = cGraph->nodes[0]->kids;
  cGraph->nodes[0]->kids = aux;
  
  return cGraph;
}


/*-- cg_GetNrPrimConstr -----------------------------------------------------------*/

int cg_GetNrPrimConstr(CONSTR_GRAPH cGraph)
{
  if (!cGraph)
    return 0;
    
  return cGraph->nodes[0]->nr_kids;
}

/*-- cg_GetMinNrBits1 -----------------------------------------------------------*/

int cg_GetMinNrBits1(CONSTR_GRAPH cGraph, pinfo *pConstr)
{
  int maxPrimConstr, maxConstrCard, maxConstrLevel;
  
  maxPrimConstr = cGraph->nodes[0]->kids->id;
  maxConstrCard = extra_dcOutOneCnt( &(cGraph->nodes[maxPrimConstr]->constr), pConstr);
  maxConstrLevel = (int) ceil (log(maxConstrCard)/log(2));
  return maxConstrLevel + 1;
}

/*-- cg_GetMinNrBits2 -----------------------------------------------------------*/

int cg_GetMinNrBits2(CONSTR_GRAPH cGraph, pinfo *pConstr)
{
  int i=0, cnt=0;
  
  while(cGraph->nodes[i]->kids != NULL)
  {
    i = cGraph->nodes[i]->kids->id;
    cnt++;
  }
  return cnt+1;
  
}
