/*

  mis.c

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

#include <stdlib.h>
#include <assert.h>
#include "mis.h"

/*-- half matrix access -----------------------------------------------------*/

static void hm_normal(int *x, int *y)
{
  if ( *x > *y )
  {
    int tt;
    tt = *x;
    *x = *y;
    *y = tt;
  }
}

static int hm_xy_to_pos(int x, int y)
{
  if ( x == y ) 
    return -1;
  if ( x > y )
    return (x-1)*x/2 + y;
  return (y-1)*y/2 + x;
}


/*-- fsm helper function ----------------------------------------------------*/

static int dcl_is_mealy_output_compatible(pinfo *pi, dclist c1, dclist c2)
{
  dclist a, b, c;
  int i, cnt;
  if ( dclInitVA(3, &a, &b, &c) == 0 )
    return 0;
  /* make a copy of the lists */
  if ( dclCopy(pi, a, c1) == 0 )
    return dclDestroyVA(3, a, b, c), 0;
  if ( dclCopy(pi, b, c2) == 0 )
    return dclDestroyVA(3, a, b, c), 0;
    
  /* remove the output parts (only consider inputs) */
  cnt = dclCnt(a);
  for( i = 0; i < cnt; i++ )
    dcSetOutTautology(pi, dclGet(a, i));
  cnt = dclCnt(b);
  for( i = 0; i < cnt; i++ )
    dcSetOutTautology(pi, dclGet(b, i));
    
  /* calculate the intersection: c = intersection(a,b) */
  if ( dclIntersectionList(pi, c, a, b) == 0 )
    return dclDestroyVA(3, a, b, c), 0;
    
  /* output is compatible, if the intersection is empty */
  if ( dclCnt(c) == 0 )
    return dclDestroyVA(3, a, b, c), 1;

  /* output is compatible if a and b are equal under the same input */
    
  /* restrict c1 into a and c2 into b */
  if ( dclIntersectionList(pi, a, c1, c) == 0 )
    return dclDestroyVA(3, a, b, c), 0;
  if ( dclIntersectionList(pi, b, c2, c) == 0 )
    return dclDestroyVA(3, a, b, c), 0;
  
  /* now check if a and b are equal */
  if ( dclIsEquivalent(pi, a, b) != 0 )
    return dclDestroyVA(3, a, b, c), 1;

  return dclDestroyVA(3, a, b, c), 0;
}

static int fsm_IsOutputCompatible(fsm_type fsm, int n1, int n2)
{
  dclist c1, c2;
  pinfo *pi;
  int l1, l2;
  int e1, e2;
  int dn1, dn2;
  int g1, g2;

  assert(fsm != NULL);
  
  if ( n1 == n2 )
    return 1;
    
  pi = fsm_GetOutputPINFO(fsm);
  
  if ( dclInitVA(2, &c1, &c2) == 0 )
    return 0;
  if ( fsm_GetNodeInOCover(fsm, n1, c1) == 0 )
    return dclDestroyVA(2, c1, c2), 0;
  if ( fsm_GetNodeInOCover(fsm, n2, c2) == 0 )
    return dclDestroyVA(2, c1, c2), 0;
  if ( dcl_is_mealy_output_compatible(pi, c1, c2) == 0 )
  {
    return dclDestroyVA(2, c1, c2), 0;
  }
  dclDestroyVA(2, c1, c2);
  return 1;
}

static int xbm_IsOutputCompatible(xbm_type x, int n1, int n2)
{

  if ( dclIsIntersection(xbm_GetPiIn(x), 
          xbm_GetSt(x, n1)->in_self_cl, xbm_GetSt(x, n2)->in_self_cl) != 0 )
  {
    /*
    xbm_Log(x, 1, "XBM mis: States '%s' and '%s' have input intersection.",
        xbm_GetStNameStr(x, n1), xbm_GetStNameStr(x, n2));
    */
    if ( dcIsEqualOut(xbm_GetPiOut(x), 
            &(xbm_GetSt(x, n1)->out), &(xbm_GetSt(x, n2)->out)) == 0 )
    {
      /* not compatible */
      return 0;
    }
  }
  return 1;
}

static int xbm_IsNotCompatible(xbm_type x, int n1, int n2)
{
  dcube *c1;
  dcube *c2;
  int l1, l2, t1, t2;
  
  if ( xbm_IsOutputCompatible(x, n1, n2) == 0 )
    return 1;
    
  if ( x->is_sync != 0 )
    return 0;   /* compatible! */
    
  /* async state machines require additional checking */

  if ( x->is_safe_opt != 0 )
  {

    /* 1. is there a conflict between in and out transitions? */

    if ( xbm_IsHFInOutTransitionPossible(x, n1, n2) == 0 )
      return 1;

    if ( xbm_IsHFInOutTransitionPossible(x, n2, n1) == 0 )
      return 1;

    /* 2. do the outgoing transition cubes intersect? */
    /* FIXME: no check required???? ... seems to be   */

    /* done by conditional checks...
    if ( xbm_IsHFOutOutTransitionPossible(x, n1, n2) == 0 )
      return 1;
    */
  }
  
  return 0;
}


static int fsm_IsUncoditionalCompatible(fsm_type fsm, int n1, int n2)
{
  dclist c1, c2;
  pinfo *pi;
  int l1, l2;
  int e1, e2;
  int dn1, dn2;
  int g1, g2;
  
  assert(fsm != NULL);
  
  if ( n1 == n2 )
    return 0;

    
  /* two states are not compatible, if the output is not compatible */
  
  pi = fsm_GetOutputPINFO(fsm);
  
  if ( dclInitVA(2, &c1, &c2) == 0 )
    return 0;
  if ( fsm_GetNodeInOCover(fsm, n1, c1) == 0 )
    return dclDestroyVA(2, c1, c2), 0;
  if ( fsm_GetNodeInOCover(fsm, n2, c2) == 0 )
    return dclDestroyVA(2, c1, c2), 0;
  if ( dcl_is_mealy_output_compatible(pi, c1, c2) == 0 )
  {
    return dclDestroyVA(2, c1, c2), 0;
  }
  dclDestroyVA(2, c1, c2);
    
  /* two states are not compatible, if there exists one condition for two */
  /* different transitions */

  pi = fsm_GetConditionPINFO(fsm);
  l1 = -1;
  e1 = -1;
  while ( fsm_LoopNodeOutEdges(fsm, n1, &l1, &e1) != 0 )
  {
    l2 = -1;
    e2 = -1;
    while ( fsm_LoopNodeOutEdges(fsm, n2, &l2, &e2) != 0 )
    {
      c1 = fsm_GetEdgeCondition(fsm,e1);
      c2 = fsm_GetEdgeCondition(fsm,e2);
      if ( dclIsIntersection(pi, c1, c2) != 0 )
      {
        dn1 = fsm_GetEdgeDestNode(fsm, e1);
        dn2 = fsm_GetEdgeDestNode(fsm, e2);
        g1 = fsm_GetNodeGroupIndex(fsm, dn1);
        g2 = fsm_GetNodeGroupIndex(fsm, dn2);
        /* if ( g1 < 0 || g2 < 0 || g1 != g2 ) */
          if ( dn1 != dn2 )
          {
            fsm_Log(fsm, "FSM: States '%s' and '%s' have identical conditions for different target states.",
              fsm_GetNodeNameStr(fsm, n1), fsm_GetNodeNameStr(fsm, n2));
            return 0;
          }
      }
    }
  }
  
  fsm_Log(fsm, "FSM: States '%s' and '%s' are compatible.",
    fsm_GetNodeNameStr(fsm, n1), fsm_GetNodeNameStr(fsm, n2));
  
  return 1;
}

static int xbm_IsUncoditionalCompatible(xbm_type x, int n1, int n2)
{
  dcube *c1;
  dcube *c2;
  int l1, l2, t1, t2;
  int i, cnt;

  /* not compatible, if there are some conflicts ... */
  if ( x->is_sync == 0 )
  {
    if ( x->is_safe_opt != 0 )
    {
      if ( xbm_IsHFInOutTransitionPossible(x, n1, n2) == 0 )
        return 0;
      if ( xbm_IsHFInOutTransitionPossible(x, n2, n1) == 0 )
        return 0;
      if ( xbm_IsHFOutOutTransitionPossible(x, n1, n2) == 0 )
        return 0;
    }
  }

  
  /* two states are not compatible, if the output is not compatible */

  if ( dclIsIntersection(xbm_GetPiIn(x), 
          xbm_GetSt(x, n1)->in_self_cl, xbm_GetSt(x, n2)->in_self_cl) != 0 )
  {
    if ( dcIsEqualOut(xbm_GetPiOut(x), 
            &(xbm_GetSt(x, n1)->out), &(xbm_GetSt(x, n2)->out)) == 0 )
    {
      /* not compatible */
      xbm_Log(x, 1, "XBM mis: States '%s' and '%s' have different output values.",
        xbm_GetStNameStr(x, n1), xbm_GetStNameStr(x, n2));
      return 0;
    }
  }

  /* two states are not compatible, if there exists one condition for two */
  /* different transitions */

  l1 = -1;
  t1 = -1;
  while( xbm_LoopStOutTr(x, n1, &l1, &t1) != 0 )
  {
    c1 = &(xbm_GetTr(x, t1)->in_end_cond);
    l2 = -1;
    t2 = -1;
    while( xbm_LoopStOutTr(x, n2, &l2, &t2) != 0 )
    {
      c2 = &(xbm_GetTr(x, t2)->in_end_cond);
      if ( xbm_GetTrDestStPos(x, t1) != xbm_GetTrDestStPos(x, t2) )
      {
        if ( dcIsDeltaInNoneZero(xbm_GetPiIn(x), c1, c2) == 0 )
        {
          /* not compatible */
          xbm_Log(x, 1, "XBM mis: States '%s' and '%s' have identical conditions for different target states.",
            xbm_GetStNameStr(x, n1), xbm_GetStNameStr(x, n2));
          return 0;
        }
      }
    }
  }
  
  l1 = -1;
  t1 = -1;
  while( xbm_LoopStOutTr(x, n1, &l1, &t1) != 0 )
  {
    c1 = &(xbm_GetTr(x, t1)->in_end_cond);
    if ( xbm_GetTrDestStPos(x, t1) != n2 )
    {
      if ( dclIsIntersectionCube(xbm_GetPiIn(x), xbm_GetSt(x, n2)->in_self_cl, c1) != 0 )
      {
        xbm_Log(x, 1, "XBM mis: Self transition for state '%s' conflicts with target state of '%s'.",
          xbm_GetStNameStr(x, n2), xbm_GetStNameStr(x, n1));
        return 0;
      }
    }
  }

  l2 = -1;
  t2 = -1;
  while( xbm_LoopStOutTr(x, n2, &l2, &t2) != 0 )
  {
    c2 = &(xbm_GetTr(x, t2)->in_end_cond);
    if ( xbm_GetTrDestStPos(x, t2) != n1 )
    {
      if ( dclIsIntersectionCube(xbm_GetPiIn(x), xbm_GetSt(x, n1)->in_self_cl, c2) != 0 )
      {
        xbm_Log(x, 1, "XBM mis: Self transition for state '%s' conflicts with target state of '%s'.",
          xbm_GetStNameStr(x, n1), xbm_GetStNameStr(x, n2));
        return 0;
      }
    }
  }
  
  /* compatible */

  xbm_Log(x, 1, "XBM mis: States '%s' and '%s' are compatible.",
    xbm_GetStNameStr(x, n1), xbm_GetStNameStr(x, n2));
  
  return 1;
}


/*-- pair functions ---------------------------------------------------------*/


static pair_type pair_Open(mis_type m)
{
  pair_type p;
  p = (pair_type)malloc(sizeof(struct _pair_struct));
  if ( p != NULL )
  {
    p->t = PAIR_T_UNKNOWN;
    if ( dcInit(m->pi_cl, &(p->compatible)) != 0 )
    {
      p->pr_ptr = NULL;
      p->pr_cnt = 0;
      return p;
    }
    free(p);
  }
  return NULL;
}

static void pair_ClearRefs(pair_type p)
{
  p->pr_cnt = 0;
  if ( p->pr_ptr != NULL )
    free(p->pr_ptr);
  p->pr_ptr = NULL;
}

static void pair_Close(pair_type p)
{
  pair_ClearRefs(p);
  dcDestroy(&(p->compatible));
  free(p);
}

static int pair_FindRef(pair_type p, int s1, int s2)
{
  int i;
  hm_normal(&s1, &s2);
  for( i = 0; i < p->pr_cnt; i++ )
    if ( p->pr_ptr[i].s1 == s1 && p->pr_ptr[i].s2 == s2 )
      return i;
  return -1;
}

static int pair_AddRef(pair_type p, int s1, int s2)
{
  pr_struct *ptr;
  if ( p->pr_ptr == NULL )
    ptr = (pr_struct *)malloc(sizeof(pr_struct));
  else
    ptr = (pr_struct *)realloc(p->pr_ptr, (p->pr_cnt+1)*sizeof(pr_struct));
  if ( ptr == NULL )
    return 0;
  p->pr_ptr = ptr;
  hm_normal(&s1, &s2);
  p->pr_ptr[p->pr_cnt].s1 = s1;
  p->pr_ptr[p->pr_cnt].s2 = s2;
  p->pr_cnt++;
  return 1;
}

static int pair_AddUniqueRef(pair_type p, int s1, int s2)
{
  if ( pair_FindRef(p, s1, s2) >= 0 )
    return 1;
  if ( pair_AddRef(p, s1, s2) != 0 )
    return 1;
  return 0;
}

static int pair_AddUniqueCRef(pair_type p, int s1, int s2)
{
  if ( dcGetOut(&(p->compatible), s1) != 0 && dcGetOut(&(p->compatible), s2) != 0 )
    return 1;
  if ( pair_FindRef(p, s1, s2) >= 0 )
    return 1;
  if ( pair_AddRef(p, s1, s2) != 0 )
    return 1;
  return 0;
}


static int pair_AppendUniqueCRefs(pair_type dest, pair_type src)
{
  int i;
  for( i = 0; i < src->pr_cnt; i++ )
    if ( pair_AddUniqueCRef(dest, src->pr_ptr[i].s1, src->pr_ptr[i].s2) == 0 )
      return 0;
  return 1;
}

/* Is b subset of a? */
static int pair_IsSubsetRefs(pair_type a, pair_type b)
{
  int i;
  for( i = 0; i < b->pr_cnt; i++ )
    if ( pair_FindRef(a, b->pr_ptr[i].s1, b->pr_ptr[i].s2) < 0 )
      return 0;
  return 1;
}

/*-- minimize states (init/destroy) -----------------------------------------*/

static int mis_InitRefs(mis_type m)
{
  int n = -1;
  int state = 0;
  int i;

  for( i = 0; i < m->state_cnt; i++ )
    m->state_to_node[i] = -1;
  
  for( i = 0; i < m->node_cnt; i++ )
    m->node_to_state[i] = -1;
  
  if ( m->fsm != NULL )
  {

    n = -1;
    state = 0;
    while( fsm_LoopNodes(m->fsm, &n) != 0 )
    {
      m->state_to_node[state] = n;
      m->node_to_state[n] = state;
      state++;
    }
  }
  else if ( m->xbm != NULL )
  {
    n = -1;
    state = 0;
    while( xbm_LoopSt(m->xbm, &n) != 0 )
    {
      m->state_to_node[state] = n;
      m->node_to_state[n] = state;
      state++;
    }
  }
  else
    return 0;
    
  return 1;
}

mis_type mis_Open(fsm_type fsm, xbm_type xbm)
{
  int i;
  int inputs;
  mis_type m;
  m = (mis_type)malloc(sizeof(struct _mis_struct));
  if ( m != NULL )
  {
    m->fsm = fsm;
    m->xbm = xbm;
    if ( fsm != NULL )
    {
      m->state_cnt = b_set_Cnt(fsm->nodes);
      m->node_cnt = b_set_Max(fsm->nodes);
      inputs = fsm_GetInCnt(fsm);
    }
    else if ( xbm != NULL )
    {
      m->state_cnt = b_set_Cnt(xbm->states);
      m->node_cnt = b_set_Max(xbm->states);
      inputs = xbm->inputs;
    }
    else
    {
      free(m);
      return NULL;
    }
    m->pair_list_size = (m->state_cnt-1)*m->state_cnt/2;
    m->state_to_node = (int *)malloc(sizeof(int)*m->state_cnt);
    if ( m->state_to_node != NULL )
    {
      m->node_to_state = (int *)malloc(sizeof(int)*m->node_cnt);
      if ( m->node_to_state != NULL )
      {
        m->pi_cl = pinfoOpenInOut(0, m->state_cnt);
        if ( m->pi_cl != NULL )
        {
          m->pair_list_ptr = (pair_type *)malloc(sizeof(pair_type)*m->pair_list_size);
          if ( m->pair_list_ptr != NULL )
          {
            for( i = 0; i < m->pair_list_size; i++ )
              m->pair_list_ptr[i] = NULL;
            for( i = 0; i < m->pair_list_size; i++ )
            {
              m->pair_list_ptr[i] = pair_Open(m);
              if ( m->pair_list_ptr[i] == NULL )
                break;
            }
            if ( i == m->pair_list_size )
            {
              mis_InitRefs(m);

              m->pi_imply = pinfoOpenInOut(inputs, m->state_cnt);
              if ( m->pi_imply != NULL )
              {
                m->pi_mcl = pinfoOpenInOut(m->state_cnt, 0);
                if ( m->pi_imply != NULL )
                {
                  if ( dclInit(&(m->cl_mcl)) != 0 )
                  {
                    m->bcp = b_bcp_Open();
                    if ( m->bcp != NULL )
                    {
                      m->c_list = b_pl_Open();
                      if ( m->c_list != NULL )
                      {
                        return m;
                      }
                      b_bcp_Close(m->bcp);
                    }
                    dclDestroy(m->cl_mcl);
                  }
                  pinfoClose(m->pi_mcl);
                }
                pinfoClose(m->pi_imply);
              }
            }
            for( i = 0; i < m->pair_list_size; i++ )
              if ( m->pair_list_ptr[i] != NULL )
                pair_Close(m->pair_list_ptr[i]);
          }
          pinfoClose(m->pi_cl);
        }
        free(m->node_to_state);
      }
      free(m->state_to_node);
    }
    free(m);
  }
  return NULL;
}

void mis_Close(mis_type m)
{
  int i;

  b_bcp_Close(m->bcp);

  for( i = 0; i < b_pl_GetCnt(m->c_list); i++ )
    pair_Close(mis_GetCListElement(m, i));
  b_pl_Close(m->c_list);

  dclDestroy(m->cl_mcl);
  pinfoClose(m->pi_mcl);

  pinfoClose(m->pi_cl);
  pinfoClose(m->pi_imply);
  for( i = 0; i < m->pair_list_size; i++ )
    if ( m->pair_list_ptr[i] != NULL )
      pair_Close(m->pair_list_ptr[i]);
  free(m->pair_list_ptr);
  free(m->node_to_state);
  free(m->state_to_node);
  free(m);
}

/*-- minimize states (access) -----------------------------------------------*/

static pair_type mis_GetPair(mis_type m, int s1, int s2)
{
  return m->pair_list_ptr[hm_xy_to_pos(s1, s2)];
}

static void mis_SetPairT(mis_type m, int s1, int s2, int t)
{
  mis_GetPair(m, s1, s2)->t = t;
}

static int mis_GetPairT(mis_type m, int s1, int s2)
{
  return mis_GetPair(m, s1, s2)->t;
}

static int mis_ToN(mis_type m, int s)
{
  return m->state_to_node[s];
}

static int mis_ToS(mis_type m, int n)
{
  return m->node_to_state[n];
}

static int mis_AddUniqueReference(mis_type m, int s1, int s2, int t1, int t2)
{
  return pair_AddUniqueRef(mis_GetPair(m, s1, s2), t1, t2);
}

static const char *mis_GetStateStr(mis_type m, int s)
{
  if ( m->fsm != NULL )
    return fsm_GetNodeNameStr(m->fsm, mis_ToN(m, s));
  else if ( m->xbm != NULL )
    return xbm_GetStNameStr(m->xbm, mis_ToN(m, s));
  return "";
}

static int mis_GetPairCnt(mis_type m, int s1, int s2)
{
  return mis_GetPair(m, s1, s2)->pr_cnt;
}

static pair_type mis_AddNewCListElement(mis_type m)
{
  pair_type p;
  p = pair_Open(m);
  if ( b_pl_Add(m->c_list, p) >= 0 )
    return p;
  pair_Close(p);
  return NULL;
}

/*-- minimize states (algorithm) --------------------------------------------*/

/*
  first step of the algorithm, 
  mark those state that are unconditionaly compatibel 
*/

void mis_MarkAlwaysCompatible(mis_type m)
{
  int s1, s2;
  int n1, n2;
  for( s1 = 0; s1 < m->state_cnt; s1++ )
  {
    for( s2 = 0; s2 < s1; s2++ )
    {
      if ( mis_GetPairT(m, s1, s2) == PAIR_T_UNKNOWN )
      {
        n1 = mis_ToN(m, s1);
        n2 = mis_ToN(m, s2);
        if ( m->fsm != NULL )
        {
          if ( fsm_IsUncoditionalCompatible(m->fsm, n1, n2) != 0 )
            mis_SetPairT(m, s1, s2, PAIR_T_ALWAYS_COMPATIBLE);
        }
        else if ( m->xbm != NULL )
        {
          if ( xbm_IsUncoditionalCompatible(m->xbm, n1, n2) != 0 )
            mis_SetPairT(m, s1, s2, PAIR_T_ALWAYS_COMPATIBLE);
        }
      }
    }
  }
}

/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/

/*
  second step of the algorithm, 
  mark those state that will never be compatible
*/

void mis_MarkNotCompatible(mis_type m)
{
  int s1, s2;
  int n1, n2;
  for( s1 = 0; s1 < m->state_cnt; s1++ )
  {
    for( s2 = 0; s2 < s1; s2++ )
    {
      if ( mis_GetPairT(m, s1, s2) == PAIR_T_UNKNOWN )
      {
        n1 = mis_ToN(m, s1);
        n2 = mis_ToN(m, s2);
        if ( m->fsm != NULL )
        {
          if ( fsm_IsOutputCompatible(m->fsm, n1, n2) == 0 )
            mis_SetPairT(m, s1, s2, PAIR_T_NOT_COMPATIBLE);
        }
        else if ( m->xbm != NULL )
        {
          if ( xbm_IsNotCompatible(m->xbm, n1, n2) != 0 )
            mis_SetPairT(m, s1, s2, PAIR_T_NOT_COMPATIBLE);
        }
        else
          mis_SetPairT(m, s1, s2, PAIR_T_NOT_COMPATIBLE);
      }
    }
  }
}

/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/

/*
  third step of the algorithm
  calculate the list of other states that must be compatible in
  order to make a pair compatible
*/


static int mis_GetStateImplyCL(mis_type m, int src_state, dclist cl)
{
  int src_node;
  int dest_node;
  int dest_state;
  int loop, edge;
  int i;
  dclist ecl;
  dcube *c = &(m->pi_imply->tmp[6]);
  dcube *e;
  
  dclClear(cl);

  src_node = mis_ToN(m, src_state);
  
  if ( m->fsm != NULL )
  {
    loop = -1;
    edge = -1;
    while( fsm_LoopNodeOutEdges(m->fsm, src_node, &loop, &edge) != 0 )
    {
      dest_node = fsm_GetEdgeDestNode(m->fsm, edge);
      dest_state = mis_ToS(m, dest_node);
      ecl = fsm_GetEdgeOutput(m->fsm, edge);

      for( i = 0; i < dclCnt(ecl); i++ )
      {
        dcCopyInToIn(m->pi_imply, c, 0, fsm_GetOutputPINFO(m->fsm), dclGet(ecl, i));
        dcOutSetAll(m->pi_imply, c, CUBE_OUT_MASK);
        dcSetOut(c, dest_state, 0);
        if ( dclAdd(m->pi_imply, cl, c) < 0 )
          return 0;
      }
    }
  }
  else if ( m->xbm != NULL )
  {      
    loop = -1;
    edge = -1;
    while( xbm_LoopStOutTr(m->xbm, src_node, &loop, &edge) != 0 )
    {
      dest_node = xbm_GetTrDestStPos(m->xbm, edge);
      dest_state = mis_ToS(m, dest_node);
      e = &(xbm_GetTr(m->xbm, edge)->in_end_cond);
      /* e = xbm_GetTrSuper(m->xbm, edge); */

      dcCopyInToIn(m->pi_imply, c, 0, xbm_GetPiIn(m->xbm), e);
      dcOutSetAll(m->pi_imply, c, CUBE_OUT_MASK);
      dcSetOut(c, dest_state, 0);
      if ( dclAdd(m->pi_imply, cl, c) < 0 )
        return 0;
    }
    
    for( i = 0; i < dclCnt(xbm_GetSt(m->xbm, src_node)->in_self_cl); i++ )
    {
      dcCopyInToIn(m->pi_imply, c, 0, xbm_GetPiIn(m->xbm), 
        dclGet(xbm_GetSt(m->xbm, src_node)->in_self_cl, i));
      dcOutSetAll(m->pi_imply, c, CUBE_OUT_MASK);
      dcSetOut(c, src_state, 0);
      if ( dclAdd(m->pi_imply, cl, c) < 0 )
        return 0;
    }
  }
  else
  {
    return 0;
  }
  
  return 1;  
}

static int mis_AddCondition(mis_type m, int s1, int s2, int t1, int t2)
{
  if ( s1 == t1 && s2 == t2 )
    return 1;
  if ( s1 == t2 && s2 == t1 )
    return 1;
  if ( t1 == t2 )
    return 1;
  if ( s1 == s2 )
    return 1;
  return mis_AddUniqueReference(m, s1, s2, t1, t2);
}

static int mis_ApplyConditions(mis_type m, int s1, int s2, dclist cl)
{
  int i, cnt = dclCnt(cl);
  int t1, t2;
  dcube *c;
  for( i = 0; i < cnt; i++ )
  {
    c = dclGet(cl, i);
    for( t1 = 0; t1 < m->state_cnt; t1++ )
      if ( dcGetOut(c, t1) == 0 )
        for( t2 = t1+1; t2 < m->state_cnt; t2++ )
          if ( dcGetOut(c, t2) == 0 )
            if ( mis_AddCondition(m, s1, s2, t1, t2) == 0 )
              return 0;
  }
  return 1;
}



/* check out-out transitions */
/* see also xbm_IsHFOutOutTransitionPossible() */
static int mis_ApplyXBMOutOutConditions(mis_type m, int state1, int state2)
{
  xbm_type x = m->xbm;
  dcube *s1;
  dcube *e1;
  dcube *s2;
  dcube *e2;
  dcube *trc1 = &(xbm_GetPiIn(x)->tmp[8]);
  dcube *trc2 = &(xbm_GetPiIn(x)->tmp[9]);
  int l1, tr1, l2, tr2;
  int n1 = mis_ToN(m, state1);
  int n2 = mis_ToN(m, state2);

  l1 = -1;
  tr1 = -1;
  while( xbm_LoopStOutTr(x, n1, &l1, &tr1 ) != 0 )
  {
    s1 = &(xbm_GetTr(x, tr1)->in_start_cond);
    e1 = &(xbm_GetTr(x, tr1)->in_end_cond);
    
    dcCopy(xbm_GetPiIn(x), trc1, xbm_GetTrSuper(x, tr1));
    
    l2 = -1;
    tr2 = -1;
    while( xbm_LoopStOutTr(x, n2, &l2, &tr2 ) != 0 )
    {
      s2 = &(xbm_GetTr(x, tr2)->in_start_cond);
      e2 = &(xbm_GetTr(x, tr2)->in_end_cond);

      dcCopy(xbm_GetPiIn(x), trc2, xbm_GetTrSuper(x, tr2));
      
      if ( tr1 != tr2 )
      {
        xbm_Log(x, 0, "XBM mis: %s/%s t/t  %s -> %s: %s    %s -> %s: %s.",
              xbm_GetStNameStr(x, n1),
              xbm_GetStNameStr(x, n2),
              xbm_GetStNameStr(x, xbm_GetTrSrcStPos(x, tr1)),
              xbm_GetStNameStr(x, xbm_GetTrDestStPos(x, tr1)),
              dcToStr(xbm_GetPiIn(x), trc1, "", ""),
              xbm_GetStNameStr(x, xbm_GetTrSrcStPos(x, tr2)),
              xbm_GetStNameStr(x, xbm_GetTrDestStPos(x, tr2)),
              dcToStr2(xbm_GetPiIn(x), trc2, "", "")
            );

        if ( dcIsDeltaInNoneZero(xbm_GetPiIn(x), trc1, trc2) == 0 )
        {
          if ( dcIsEqual(xbm_GetPiIn(x), s1, s2) == 0 || dcIsEqual(xbm_GetPiIn(x), e1, e2) == 0 )
          {
            xbm_Log(x, 1, "XBM mis: Conditional compatible states '%s' and '%s': Conflict between %s -> %s and %s -> %s.",
              xbm_GetStNameStr(x, n1),
              xbm_GetStNameStr(x, n2),
              xbm_GetStNameStr(x, xbm_GetTrSrcStPos(x, tr1)),
              xbm_GetStNameStr(x, xbm_GetTrDestStPos(x, tr1)),
              xbm_GetStNameStr(x, xbm_GetTrSrcStPos(x, tr2)),
              xbm_GetStNameStr(x, xbm_GetTrDestStPos(x, tr2))
              );
            if ( mis_AddCondition(m, 
                mis_ToS(m, xbm_GetTrSrcStPos(x, tr1)), 
                mis_ToS(m, xbm_GetTrSrcStPos(x, tr2)), 
                mis_ToS(m, xbm_GetTrDestStPos(x, tr1)), 
                mis_ToS(m, xbm_GetTrDestStPos(x, tr2))) == 0 )
            {
              return 0;
            }
            if ( mis_AddCondition(m, 
                mis_ToS(m, xbm_GetTrSrcStPos(x, tr1)), 
                mis_ToS(m, xbm_GetTrSrcStPos(x, tr2)), 
                mis_ToS(m, xbm_GetTrSrcStPos(x, tr1)), 
                mis_ToS(m, xbm_GetTrDestStPos(x, tr2))) == 0 )
            {
              return 0;
            }
            if ( mis_AddCondition(m, 
                mis_ToS(m, xbm_GetTrSrcStPos(x, tr1)), 
                mis_ToS(m, xbm_GetTrSrcStPos(x, tr2)), 
                mis_ToS(m, xbm_GetTrDestStPos(x, tr1)), 
                mis_ToS(m, xbm_GetTrSrcStPos(x, tr2))) == 0 )
            {
              return 0;
            }
          }
        }
      }
    }
  }

  return 1;
  
}

/* check out-out transitions */
/* see also xbm_IsHFInOutTransitionPossible() */
/* NOT USED AT THE MOMENT */
static int mis_ApplyXBMInOutConditions(mis_type m, int state1, int state2)
{
  xbm_type x = m->xbm;
  int n1 = mis_ToN(m, state1);
  int n2 = mis_ToN(m, state2);
  
  dclist cl;
  dcube *s1;
  dcube *e1;
  dcube *s2;
  dcube *e2;
  int l1, tr1, l2, tr2;

  if ( dclInit(&cl) == 0 )
    return 0;
    
  l1 = -1;
  tr1 = -1;
  while( xbm_LoopStOutTr(x, n1, &l1, &tr1 ) != 0 )
  {
    s1 = &(xbm_GetTr(x, tr1)->in_ddc_start_cond);
    e1 = &(xbm_GetTr(x, tr1)->in_end_cond);
    dclClear(cl);
    if ( dclAdd(xbm_GetPiIn(x), cl, xbm_GetTrSuper(x, tr1)) < 0 )
      return dclDestroy(cl), 0;
      
    if ( dclSubtractCube(xbm_GetPiIn(x), cl, s1) == 0 )
      return dclDestroy(cl), 0;
    /* FIXME
    if ( dclSubtractCube(xbm_GetPiIn(x), cl, e1) == 0 )
      return dclDestroy(cl), 0;
    */

    l2 = -1;
    tr2 = -1;
    while( xbm_LoopStInTr(x, n2, &l2, &tr2 ) != 0 )
    {
      e2 = &(xbm_GetTr(x, tr2)->in_end_cond);
      xbm_Log(x, 0, "XBM mis: %s/%s t/e  %s -> %s: %s    %s -> %s: %s.",
            xbm_GetStNameStr(x, n1),
            xbm_GetStNameStr(x, n2),
            xbm_GetStNameStr(x, xbm_GetTrSrcStPos(x, tr1)),
            xbm_GetStNameStr(x, xbm_GetTrDestStPos(x, tr1)),
            dcToStr(xbm_GetPiIn(x), xbm_GetTrSuper(x, tr1), "", ""),
            xbm_GetStNameStr(x, xbm_GetTrSrcStPos(x, tr2)),
            xbm_GetStNameStr(x, xbm_GetTrDestStPos(x, tr2)),
            dcToStr2(xbm_GetPiIn(x), e2, "", "")
          );
      /* dclShow(xbm_GetPiIn(x), cl); */
      if ( dclIsIntersectionCube(xbm_GetPiIn(x), cl, e2) != 0 )
      {
        /* called from mis.c */
        xbm_Log(x, 1, "XBM mis: States '%s' and '%s' are not compatible: Conflict between %s -> %s and %s -> %s.",
          xbm_GetStNameStr(x, n1),
          xbm_GetStNameStr(x, n2),
          xbm_GetStNameStr(x, xbm_GetTrSrcStPos(x, tr1)),
          xbm_GetStNameStr(x, xbm_GetTrDestStPos(x, tr1)),
          xbm_GetStNameStr(x, xbm_GetTrSrcStPos(x, tr2)),
          xbm_GetStNameStr(x, xbm_GetTrDestStPos(x, tr2))
          );
        if ( mis_AddCondition(m, 
            mis_ToS(m, xbm_GetTrDestStPos(x, tr1)), 
            mis_ToS(m, xbm_GetTrSrcStPos(x, tr2)), 
            mis_ToS(m, xbm_GetTrSrcStPos(x, tr1)), 
            mis_ToS(m, xbm_GetTrDestStPos(x, tr2))) == 0 )
          return dclDestroy(cl), 0;
        if ( mis_AddCondition(m, 
            mis_ToS(m, xbm_GetTrDestStPos(x, tr1)), 
            mis_ToS(m, xbm_GetTrSrcStPos(x, tr2)), 
            mis_ToS(m, xbm_GetTrDestStPos(x, tr1)), 
            mis_ToS(m, xbm_GetTrDestStPos(x, tr2))) == 0 )
          return dclDestroy(cl), 0;
        if ( mis_AddCondition(m, 
            mis_ToS(m, xbm_GetTrDestStPos(x, tr1)), 
            mis_ToS(m, xbm_GetTrSrcStPos(x, tr2)), 
            mis_ToS(m, xbm_GetTrSrcStPos(x, tr1)), 
            mis_ToS(m, xbm_GetTrSrcStPos(x, tr2))) == 0 )
          return dclDestroy(cl), 0;
      }
    }
  }

  return dclDestroy(cl), 1;
}

int mis_MarkConditionalCompatible(mis_type m)
{
  dclist cl1, cl2, cli;
  int s1, s2;
  
  if ( dclInitVA(3, &cl1, &cl2, &cli) == 0 )
    return 0;
  
  for( s1 = 0; s1 < m->state_cnt; s1++ )
  {
    if ( mis_GetStateImplyCL(m, s1, cl1) == 0 )
      return dclDestroyVA(3, cl1, cl2, cli), 0;
    for( s2 = 0; s2 < s1; s2++ )
    {
      if ( mis_GetPairT(m, s1, s2) == PAIR_T_UNKNOWN )
      {
        if ( mis_GetStateImplyCL(m, s2, cl2) == 0 )
          return dclDestroyVA(3, cl1, cl2, cli), 0;

        dclClear(cli);
        if ( dclIntersectionList(m->pi_imply, cli, cl1, cl2) == 0 )
          return dclDestroyVA(3, cl1, cl2, cli), 0;
        if ( mis_ApplyConditions(m, s1, s2, cli) == 0 )
          return dclDestroyVA(3, cl1, cl2, cli), 0;

        if ( m->xbm != NULL )
        {
          if ( m->xbm->is_safe_opt != 0 ) 
            if ( mis_ApplyXBMOutOutConditions(m, s1, s2) == 0 )
              return 0;
        }

        if ( mis_GetPairCnt(m, s1, s2) == 0 )
          mis_SetPairT(m, s1, s2, PAIR_T_ALWAYS_COMPATIBLE);
        else
          mis_SetPairT(m, s1, s2, PAIR_T_CONDITIONAL_COMPATIBLE);
          
        
      }
    }
  }
  
  dclDestroyVA(3, cl1, cl2, cli);
  return 1;
}

/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/

void mis_UnmarkCondictionalCompatible(mis_type m)
{
  int s1, s2;
  int t1, t2;
  int i, cnt;
  int is_changed = 1;
  
  while( is_changed != 0 )
  {
    is_changed = 0;
    for( s1 = 0; s1 < m->state_cnt; s1++ )
    {
      for( s2 = 0; s2 < s1; s2++ )
      {
        if ( mis_GetPairT(m, s1, s2) == PAIR_T_CONDITIONAL_COMPATIBLE )
        {
          cnt = mis_GetPairCnt(m, s1, s2);
          for( i = 0; i < cnt; i++ )
          {
            t1 = mis_GetPair(m, s1, s2)->pr_ptr[i].s1;
            t2 = mis_GetPair(m, s1, s2)->pr_ptr[i].s2;
            if ( mis_GetPairT(m, t1, t2) == PAIR_T_NOT_COMPATIBLE )
            {
              mis_SetPairT(m, s1, s2, PAIR_T_NOT_COMPATIBLE);
              /*
              printf("{%s,%s}: reference to incompatible pair {%s,%s}.\n",
                mis_GetStateStr(m, s1), mis_GetStateStr(m, s2),
                mis_GetStateStr(m, t1), mis_GetStateStr(m, t2)
              );
              */
              is_changed = 1;
            }
          }
        }
      }
    }
  }
}

/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/

/* calculate maximum compatibles */
/* second approach of myers' book (p144) */
/* assigns the maximum compatibles to m->cl_mcl */

int mis_CalculateMCL(mis_type m)
{
  int s1, s2;
  dcube *c;

  dclClear(m->cl_mcl);

  for( s1 = 0; s1 < m->state_cnt; s1++ )
  {
    for( s2 = 0; s2 < s1; s2++ )
    {
      if ( mis_GetPairT(m, s1, s2) == PAIR_T_NOT_COMPATIBLE )
      {
        c = dclAddEmptyCube(m->pi_mcl, m->cl_mcl);
        if ( c == NULL )
          return 0;
        dcInSetAll(m->pi_mcl, c, CUBE_IN_MASK_DC);
        dcSetIn(c, s1, 1);
        dcSetIn(c, s2, 1);
      }
    }
  }
  if ( dclConvertFromPOS(m->pi_mcl, m->cl_mcl) == 0 )
    return 0;
  if ( dclPrimes(m->pi_mcl, m->cl_mcl) == 0 )
    return 0;
  /* dclShow(m->pi_mcl, m->cl_mcl); */
  return 1;
}

/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/

int mis_CopyMCLtoCList(mis_type m)
{
  int i, cnt = dclCnt(m->cl_mcl);
  int j;
  pair_type p;
  for( i = 0; i < cnt; i++ )
  {
    p = mis_AddNewCListElement(m);
    if ( p == NULL )
      return 0;
    p->t = PAIR_T_NOT_DONE;
    dcOutSetAll(m->pi_cl, &(p->compatible), 0);
    for( j = 0; j < m->pi_mcl->in_cnt; j++)
      if ( dcGetIn(dclGet(m->cl_mcl, i), j) == 3 )
        dcSetOut(&(p->compatible), j, 1);
  }
  return 1;
}

/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/

int mis_CalculateClassSet(mis_type m, pair_type p)
{
  int s1, s2;
  pair_ClearRefs(p);  
  for( s1 = 0; s1 < m->state_cnt; s1++ )
    if ( dcGetOut(&(p->compatible), s1) != 0 )
      for( s2 = s1+1; s2 < m->state_cnt; s2++ )
        if ( dcGetOut(&(p->compatible), s2) != 0 )
          if ( pair_AppendUniqueCRefs(p, mis_GetPair(m, s1, s2)) == 0 )
            return 0;
  return 1;
}

int mis_CalculateClassSets(mis_type m)
{
  int i, cnt = b_pl_GetCnt(m->c_list);
  pair_type p;
  for( i = 0; i < cnt; i++ )
  {
    p = mis_GetCListElement(m, i);
    if ( mis_CalculateClassSet(m, p) == 0 )
      return 0;
  }
  return 1;
}

/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/

void mis_ShowClassSet(mis_type m, pair_type p)
{
  int s;
  int i;
  int is_first = 1;
  printf("{");
  for( s = 0; s < m->state_cnt; s++ )
    if ( dcGetOut(&(p->compatible), s) != 0 )
    { 
      if ( is_first == 0 )
        printf(",");
      is_first = 0;
      printf("%s", mis_GetStateStr(m, s));
    }
  printf("} ");

  for( i = 0; i < p->pr_cnt; i++ )
    printf("(%s,%s) ",
      mis_GetStateStr(m, p->pr_ptr[i].s1), mis_GetStateStr(m, p->pr_ptr[i].s2));
  
  printf("\n");
}

int mis_ShowClassSets(mis_type m)
{
  int i, cnt = b_pl_GetCnt(m->c_list);
  pair_type p;
  for( i = 0; i < cnt; i++ )
  {
    p = mis_GetCListElement(m, i);
    mis_ShowClassSet(m, p);
  }
  return 1;
}


/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/

/* procedure from myers's book, page 146 */
/* 'done' set is implemented as a flag */
/* q, p, s and k have the same meaning */
int mis_BuildPrimeCompatibleList(mis_type m)
{
  int i, j, k, cnt = b_pl_GetCnt(m->c_list);
  int state;
  pair_type p;
  pair_type s;
  pair_type q;
  int is_prime;
  
  s = pair_Open(m);
  if ( s == NULL )
    return 0;
  for( k = m->state_cnt-1; k > 0; k-- )
  {
    cnt = b_pl_GetCnt(m->c_list);
    for( i = 0; i < cnt; i++ )
    {
      p = mis_GetCListElement(m, i);
      if ( dcOutCnt(m->pi_cl, &(p->compatible)) == k )
      {
        /* printf("processing: "); mis_ShowClassSet(m, p); */
        dcCopyOut(m->pi_cl, &(s->compatible), &(p->compatible));
        for( state = 0; state < m->state_cnt; state++ )
        {
          if ( dcGetOut(&(s->compatible), state) != 0 )
          {
            dcSetOut(&(s->compatible), state, 0);
            
            /* step 1: check if this compatible already exists */
            for( j = 0; j < cnt; j++ )
            {
              q = mis_GetCListElement(m, j);
              if ( q->t == PAIR_T_DONE )
                if ( dcIsEqual(m->pi_cl, &(s->compatible), &(q->compatible)) != 0 )
                  break;
            }
            
            /* step 2: only if the compatible has not been checked */  
            if ( j >= cnt )
            {
              mis_CalculateClassSet(m, s);
              is_prime = 1;
              /* we can reuse j here... */
              for( j = 0; j < cnt; j++ )
              {
                q = mis_GetCListElement(m, j);
                if ( dcOutCnt(m->pi_cl, &(q->compatible)) >= k )
                {
                  /* Is q subset of s? yes --> s can not be a prime */
                  if ( pair_IsSubsetRefs(s, q) != 0 )
                  {
                    is_prime = 0;
                    break;
                  }
                }
              }
              if ( is_prime != 0 )
              { 
                /* s has been checked again larger primes */
                s->t = PAIR_T_DONE;
                /* store s int the array... it will be free'd later */
                if ( b_pl_Add(m->c_list, s) < 0 )
                  return pair_Close(s), 0;
                /* allocate a new 's' */
                /* printf("new prime: "); mis_ShowClassSet(m, s); */
                s = pair_Open(m);
                if ( s == NULL )
                  return 0;
                dcCopyOut(m->pi_cl, &(s->compatible), &(p->compatible));
              }
              else
              {
                /* printf("no new prime: "); mis_ShowClassSet(m, s); */
              }
            }
            dcSetOut(&(s->compatible), state, 1);
          }
        } /* if... with all subsets */
      } /* for */
    }
  }
  pair_Close(s);
  return 1;
}


/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/

int mis_CalculateBCP(mis_type m)
{
  int state;
  int i, j, k;
  pair_type p;
  pair_type q;
  int y = 0;
  int cnt;
  
  cnt = b_pl_GetCnt(m->c_list);
  for( state = 0; state < m->state_cnt; state++ )
  {
    for( i = 0; i < cnt; i++ )
    {
      p = mis_GetCListElement(m, i);
      if ( dcGetOut(&(p->compatible), state) != 0 )
        if ( b_bcp_Set(m->bcp, i, y, BCN_VAL_ONE) == 0 )
          return 0;
    }
    y++;
  }
  
  for( i = 0; i < cnt; i++ )
  {
    p = mis_GetCListElement(m, i);
    for( j = 0; j < p->pr_cnt; j++ )
    {
      if ( b_bcp_Set(m->bcp, i, y, BCN_VAL_ZERO) == 0 )
        return 0;
      for( k = 0; k < cnt; k++ )
      {
        q = mis_GetCListElement(m, k);
        if ( dcGetOut(&(q->compatible), p->pr_ptr[j].s1) != 0 && 
             dcGetOut(&(q->compatible), p->pr_ptr[j].s2) != 0 )
        {
          if ( b_bcp_Set(m->bcp, k, y, BCN_VAL_ONE) == 0 )
            return 0;
        }
      }
      y++;
    }
  }

  /* b_bcp_Show(m->bcp); */
  b_bcp_Do(m->bcp);
  
  /*
  for( i = 0; i < cnt; i++ )
  {
    if ( b_bcp_Get(m->bcp, i) == BCN_VAL_ONE )
      printf("x");
    else
      printf("-");
  }
  printf("\n");
  */
  
  
  return 1;
  
}

/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/

int mis_CalculateGroups(mis_type m)
{
  int i, cnt = b_pl_GetCnt(m->c_list);
  int j, k;
  pair_type p;
  pair_type q;
  int n1, n2;
  int g, g1, g2;
  int state, node;

  for( i = 0; i < cnt; i++ )
    if ( b_bcp_Get(m->bcp, i) == BCN_VAL_ONE )
    {
      p = mis_GetCListElement(m, i);
      for( j = 0; j < p->pr_cnt; j++ )
      {
        n1 = mis_ToN(m, p->pr_ptr[j].s1);
        n2 = mis_ToN(m, p->pr_ptr[j].s2);
        if ( m->fsm != NULL )
        {
          g1 = fsm_GetNodeGroupIndex(m->fsm, n1);
          g2 = fsm_GetNodeGroupIndex(m->fsm, n2);
        }
        else if ( m->xbm != NULL )
        {
          g1 = xbm_GetStGrPos(m->xbm, n1);
          g2 = xbm_GetStGrPos(m->xbm, n2);
        }
        else
        {
          g1 = -1;
          g2 = -1;
        }
      
        if ( g1 < 0 && g2 < 0 )
        {
          for( k = 0; k < cnt; k++ )
          {
            if ( b_bcp_Get(m->bcp, k) == BCN_VAL_ONE )
            {
              q = mis_GetCListElement(m, k);
              if ( dcGetOut(&(q->compatible), p->pr_ptr[j].s1) != 0 && 
                   dcGetOut(&(q->compatible), p->pr_ptr[j].s2) != 0 )
              {
                break;
              }
            }
          }
          assert( k < cnt);
          
          if ( m->fsm != NULL )
          {
            if ( fsm_SetNodeGroup(m->fsm, n1, k) == 0 )
            {
              fsm_Log(m->fsm, "FSM mis: Can not assign node group.");
              return 0;
            }
            if ( fsm_SetNodeGroup(m->fsm, n2, k) == 0 )
            {
              fsm_Log(m->fsm, "FSM mis: Can not assign node group.");
              return 0;
            }
          }
          else if ( m->xbm != NULL )
          {
            if ( xbm_SetStGr(m->xbm, n1, k) == 0 )
            {
              xbm_Error(m->xbm, "XBM mis: Can not assign node group.");
              return 0;
            }
            if ( xbm_SetStGr(m->xbm, n2, k) == 0 )
            {
              xbm_Error(m->xbm, "XBM mis: Can not assign node group.");
              return 0;
            }
          }
          else
            return 0;
        }
        else if ( g1 >= 0 && g2 >= 0 )
        {
          if ( g1 != g2 )
          {
            /* the two states should be merged, but they aren't ... */
            return 0;
          }
        }
        else
        {
          if ( g1 >= 0 )
          {
            q = mis_GetCListElement(m, g1);
            if ( dcGetOut(&(q->compatible), p->pr_ptr[j].s2) != 0 )
            {
              if ( m->fsm != NULL )
              {
                if ( fsm_SetNodeGroup(m->fsm, n2, g1) == 0 )
                {
                  fsm_Log(m->fsm, "FSM mis: Can not assign node group.");
                  return 0;
                }
              }
              else if ( m->xbm != NULL )
              {
                if ( xbm_SetStGr(m->xbm, n2, g1) == 0 )
                {
                  xbm_Error(m->xbm, "XBM mis: Can not assign node group.");
                  return 0;
                }
              }
              else
                return 0;
            }
            else
            { 
              /* not possible to merge the states */
              if ( m->fsm != NULL )
              {
                fsm_Log(m->fsm, "FSM mis: Incompatible states ('%s' and '%s').",
                  fsm_GetNodeNameStr(m->fsm, n1), fsm_GetNodeNameStr(m->fsm, n2));
              }
              else if ( m->xbm != NULL )
              {
                xbm_Error(m->xbm, "XBM mis: Incompatible states ('%s' and '%s').",
                  xbm_GetStNameStr(m->xbm, n1), xbm_GetStNameStr(m->xbm, n2));
              }
              
              
              return 0;
            }
          }
          else if ( g2 >= 0 )
          {
            q = mis_GetCListElement(m, g2);
            if ( dcGetOut(&(q->compatible), p->pr_ptr[j].s1) != 0 )
            {
              if ( m->fsm != NULL )
              {
                if ( fsm_SetNodeGroup(m->fsm, n1, g2) == 0 )
                {
                  fsm_Log(m->fsm, "FSM mis: Can not assign node group.");
                  return 0;
                }
              }
              else if ( m->xbm != NULL )
              {
                if ( xbm_SetStGr(m->xbm, n1, g2) == 0 )
                {
                  xbm_Error(m->xbm, "XBM mis: Can not assign node group.");
                  return 0;
                }
              }
              else
                return 0;
              
            }
            else
            { 
              /* not possible to merge the states */
              if ( m->fsm != NULL )
              {
                fsm_Log(m->fsm, "FSM mis: Incompatible states ('%s' and '%s').",
                  fsm_GetNodeNameStr(m->fsm, n1), fsm_GetNodeNameStr(m->fsm, n2));
              }
              else if ( m->xbm != NULL )
              {
                xbm_Error(m->xbm, "XBM mis: Incompatible states ('%s' and '%s').",
                  xbm_GetStNameStr(m->xbm, n1), xbm_GetStNameStr(m->xbm, n2));
              }
              return 0;
            }
          }
          else
          {
            /* internal error */
            if ( m->fsm != NULL )
              fsm_Log(m->fsm, "FSM mis: Internal error.");
            else if ( m->xbm != NULL )
              xbm_Error(m->xbm, "XBM mis: Internal error.");
            return 0;
          }
        }
      } /* for */
    }

  for( state = 0; state < m->state_cnt; state++ )
  {
    node = mis_ToN(m, state);
    if ( m->fsm != NULL )
      g = fsm_GetNodeGroupIndex(m->fsm, node);
    else if ( m->xbm != NULL )
      g = xbm_GetStGrPos(m->xbm, node);
    else
      return 0;
    
    if ( g < 0 )
    {
      for( i = 0; i < cnt; i++ )
      {
        if ( b_bcp_Get(m->bcp, i) == BCN_VAL_ONE )
        {
          p = mis_GetCListElement(m, i);
          if ( dcGetOut(&(p->compatible), state) != 0 )
          {
            if ( m->fsm != NULL )
            {
              if ( fsm_SetNodeGroup(m->fsm, node, i) == 0 )
              {
                fsm_Log(m->fsm, "FSM mis: Can not assign node group.");
                return 0;
              }
            }
            else if ( m->xbm != NULL )
            {
              if ( xbm_SetStGr(m->xbm, node, i) == 0 )
              {
                xbm_Error(m->xbm, "XBM mis: Can not assign node group.");
                return 0;
              }
            }
            break;
          }
        }
      }
      if ( i >= cnt )
      {
        if ( m->fsm != NULL )
          fsm_Log(m->fsm, "FSM mis: No cover found.");
        else if ( m->xbm != NULL )
          xbm_Error(m->xbm, "XBM mis: No cover found.");
        return 0;
      }
    }
  }


  return 1;  
}

/*- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -*/


void mis_ShowStates(mis_type m)
{
  int s1, s2;
  int i;
  for( s1 = 0; s1 < m->state_cnt; s1++ )
  {
    for( s2 = 0; s2 < s1; s2++ )
    {
      printf("{%s,%s}: ", mis_GetStateStr(m, s1), mis_GetStateStr(m, s2));
      switch(mis_GetPairT(m, s1, s2))
      {
        case PAIR_T_UNKNOWN:
          printf("STATUS UNKNOWN\n");
          break;
        case PAIR_T_ALWAYS_COMPATIBLE:
          printf("always compatible\n");
          break;
        case PAIR_T_CONDITIONAL_COMPATIBLE:
          printf("--> %ldx ", mis_GetPair(m, s1, s2)->pr_cnt);
          for( i = 0; i < mis_GetPair(m, s1, s2)->pr_cnt; i++ )
            printf("{%s,%s} ",
              mis_GetStateStr(m, mis_GetPair(m, s1, s2)->pr_ptr[i].s1),
              mis_GetStateStr(m, mis_GetPair(m, s1, s2)->pr_ptr[i].s2));
          printf("\n");
          break;
        case PAIR_T_NOT_COMPATIBLE:
          printf("not compatible\n");
          break;
      }
    }
  }
}
