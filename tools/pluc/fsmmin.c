/*

  fsmmin.c

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



  The current algorithm could be improved very very much!!!
  Probably we had to solve a binate cover problem to get an exact 
  result.

*/

#include "b_il.h"
#include "fsm.h"

/*
static int dcl_is_fbo_compatible(pinfo *pi, dclist c1, dclist c2)
{
  int i, cnt;
  
  if ( dclPrimes(pi, c1) == 0 )
    return 0;
  if ( dclPrimes(pi, c2) == 0 )
    return 0;
    
  cnt = dclCnt(c1);
  if ( cnt > 1 )
    for( i = 1; i < cnt; i++ )
      if ( dcIsEqualOut(pi, dclGet(c1, 0), dclGet(c1, i)) != 0 )
        return 0;
    
  cnt = dclCnt(c2);
  if ( cnt > 1 )
    for( i = 1; i < cnt; i++ )
      if ( dcIsEqualOut(pi, dclGet(c2, 0), dclGet(c2, i)) != 0 )
        return 0;
    
  for( i = 0; i < pi->out_cnt; i++ )
    if ( dcGetOut(dclGet(c1, 0), i) != dcGetOut(dclGet(c2, 0), i) )
      return 1;
  
  return 0;
}
*/

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

int dcl_is_output_compatible(pinfo *pi, dclist c1, dclist c2)
{
  dcube *r1 = &(pi->tmp[13]);
  dcube *r2 = &(pi->tmp[14]);
  dclOrElements(pi, r1, c1);
  dclOrElements(pi, r2, c2);

  return dcIsEqualOut(pi, r1, r2);
}


static int hm_xy_to_pos(int x, int y)
{
  if ( x == y ) 
    return -1;
  if ( x > y )
    return (x-1)*x/2 + y;
  return (y-1)*y/2 + x;
}



/*---------------------------------------------------------------------------*/

struct _hm_struct
{
  fsm_type fsm;
  int n;
  int *m;
  int m_size;
  int *flag_list;
  b_pl_type pl;   /* list of compatible sets */
};
typedef struct _hm_struct *hm_type;


#include <assert.h>
#include <stdlib.h>
#include <string.h>


hm_type hm_Open(fsm_type fsm, int n)
{
  hm_type hm;
  hm = (hm_type)malloc(sizeof(struct _hm_struct));
  if ( hm != NULL )
  {
    hm->fsm = fsm;
    hm->n = n;
    hm->m_size = (n-1)*n/2;
    hm->m = (int *)malloc(sizeof(int)*hm->m_size);
    if ( hm->m != NULL )
    {
      memset(hm->m, 0, sizeof(int)*hm->m_size);
      assert(hm_xy_to_pos(0, 0) == -1 );
      assert(hm_xy_to_pos(0, 1) == 0 );
      assert(hm_xy_to_pos(1, 0) == 0 );
      assert(hm_xy_to_pos(0, 2) == 1 );
      assert(hm_xy_to_pos(1, 2) == 2 );
      assert(hm_xy_to_pos(2, 0) == 1 );
      assert(hm_xy_to_pos(2, 1) == 2 );
      assert(hm_xy_to_pos(0, 3) == 3 );
      assert(hm_xy_to_pos(1, 3) == 4 );
      assert(hm_xy_to_pos(2, 3) == 5 );
      assert(hm_xy_to_pos(3, 0) == 3 );
      assert(hm_xy_to_pos(3, 1) == 4 );
      assert(hm_xy_to_pos(3, 2) == 5 );
      hm->pl = b_pl_Open();
      if ( hm->pl != NULL )
      {
        hm->flag_list = (int *)malloc(sizeof(int)*hm->n);
        if ( hm->flag_list != NULL )
        {
          memset(hm->flag_list, 0, sizeof(int)*hm->n);
          return hm;
          /*
          hm->pi_imply = pinfoOpenInOut(b_set_Cnt(fsm->nodes), b_set_Cnt(fsm->nodes))
          if ( hm->pi_imply != NULL )
          {
            if ( dclInit(&(hm->cl_imply)) != 0 )
            {
            }
          }
          pinfoClose(hm->pi_imply);
          */
        }
        b_pl_Close(hm->pl);
      }
      free(hm->m);
    }
    free(hm);
  }
  return NULL;
}

b_il_type hm_GetSet(hm_type hm, int pos)
{
  return (b_il_type)b_pl_GetVal(hm->pl, pos);
}


void hm_Close(hm_type hm)
{
  int i;
  free(hm->flag_list);
  for( i = 0; i < b_pl_GetCnt(hm->pl); i++ )
    b_il_Close(hm_GetSet(hm, i));
  b_pl_Close(hm->pl);
  free(hm->m);
  free(hm);
}

void hm_Set(hm_type hm, int x, int y, int v)
{
  int pos = hm_xy_to_pos(x, y);
  assert(pos >= 0 && pos < hm->m_size);
  hm->m[pos] = v;
}

int hm_Get(hm_type hm, int x, int y)
{
  int pos = hm_xy_to_pos(x, y);
  assert(pos >= 0 && pos < hm->m_size);
  return hm->m[pos];
}

int hm_IsSetCompatible(hm_type hm, b_il_type il, int s)
{
  int i;
  for( i = 0; i < b_il_GetCnt(il); i++ )
    if ( hm_Get(hm, b_il_GetVal(il, i), s) == 0 )
      return 0;
  return 1;
}

/* returns position */
int hm_AddEmptySet(hm_type hm)
{
  int pos;
  b_il_type il;
  il = b_il_Open();
  if ( il != NULL )
  {
    pos = b_pl_Add(hm->pl, il);
    if ( pos >= 0 )
      return pos;
    b_il_Close(il);
  }
  return -1;
}

int hm_BuildCompatibleSets(hm_type hm)
{
  int n1, n2;
  int pos;
  memset(hm->flag_list, 0, sizeof(int)*hm->n);
  n1 = -1;
  while( fsm_LoopNodes(hm->fsm, &n1) != 0 )
  {
    if ( hm->flag_list[n1] == 0 )
    {
      pos = hm_AddEmptySet(hm);
      if ( pos < 0 )
        return 0;
      if ( b_il_Add(hm_GetSet(hm, pos), n1) < 0 )
        return 0;
      hm->flag_list[n1] = 1;
      if ( fsm_SetNodeGroup(hm->fsm, n1, pos) == 0 )
        return 0;
      n2 = -1;
      while( fsm_LoopNodes(hm->fsm, &n2) != 0 )
      {
        if ( hm->flag_list[n2] == 0 )
        {
          if ( hm_IsSetCompatible(hm, hm_GetSet(hm, pos), n2) != 0 )
          {
            hm->flag_list[n2] = 1;
            if ( fsm_SetNodeGroup(hm->fsm, n2, pos) == 0 )
              return 0;
            if ( b_il_Add(hm_GetSet(hm, pos), n2) < 0 )
              return 0;
          }
        }
      }
    }
  }
  return 1;
}

#define BLEN 100
void hm_ShowCompatibleSets(hm_type hm)
{
  int i, j;
  b_il_type il;
  char s[BLEN];
  for( i = 0; i < b_pl_GetCnt(hm->pl); i++ )
  {
    sprintf(s, "FSM: Compatible group %d { ", i);
    il = hm_GetSet(hm, i);
    for( j = 0; j < b_il_GetCnt(il); j++ )
    {
      if( strlen(s) < BLEN-10-26-10 )
      {
        sprintf(s+strlen(s),"%s ", fsm_GetNodeNameStr(hm->fsm, b_il_GetVal(il, j)));
      }
      else
      {
        sprintf(s+strlen(s),"...");
        break;
      }
    }
    sprintf(s+strlen(s),"}");
    fsm_Log(hm->fsm, s);
  }
}

/*
static int fsm_GetAllInEdgeOutput(fsm_type fsm, int n, dclist cl)
{
  int loop, in_n;
  dclist cl_tmp;
  pinfo *pi = fsm_GetOutputPINFO(fsm);

  cl_tmp = fsm_GetNodeSelfOutput(fsm, n);
  if ( dclPrimes(pi, cl_tmp) == 0 )
    return 0;
  if ( dclCopy(pi, cl, fsm_GetNodeSelfOutput(fsm, n) ) == 0 )
    return 0;
  
  loop = -1;
  while( fsm_LoopNodeInNodes(fsm, n, &loop, &in_n) != 0 )
  {
    cl_tmp = fsm_GetNodeSelfOutput(fsm, in_n);
    if ( dclPrimes(pi, cl_tmp) == 0 )
      return 0;
    if ( dclSCCUnion( pi, cl, cl_tmp ) == 0 )
      return 0;
  }

  return 1;
}
*/

/*
static int fsm_IsFBOCompatible(fsm_type fsm, int n1, int n2)
{
  dclist c1, c2;
  pinfo *pi = fsm_GetOutputPINFO(fsm);
  dcube *or = &(pi->tmp[13]);
  dcube *and = &(pi->tmp[14]);
  int i;
  
  
  if ( dclInitVA(2, &c1, &c2) == 0 )
    return 0;

  if ( fsm_GetAllInEdgeOutput(fsm, n1, c1) == 0 )
    return dclDestroyVA(2, c1, c2), 0;
  if ( fsm_GetAllInEdgeOutput(fsm, n2, c2) == 0 )
    return dclDestroyVA(2, c1, c2), 0;

  fsm_LogDCL(fsm, "FSM: ", 2, "c1", pi, c1, "c2", pi, c2);
    
  dclAndElements(pi, and, c1);
  dclOrElements(pi, or, c2);
  for( i = 0; i < pi->out_cnt; i++ )
    if ( dcGetOut(and, i) != 0 && dcGetOut(or, i) == 0 )
      return dclDestroyVA(2, c1, c2), 1;
  
  dclAndElements(pi, and, c2);
  dclOrElements(pi, or, c1);
  for( i = 0; i < pi->out_cnt; i++ )
    if ( dcGetOut(and, i) != 0 && dcGetOut(or, i) == 0 )
      return dclDestroyVA(2, c1, c2), 1;

  return dclDestroyVA(2, c1, c2), 0;
}
*/

static int fsm_IsCompatible(fsm_type fsm, int n1, int n2, int is_fbo)
{
  dclist c1, c2;
  pinfo *pi;
  int l1, l2;
  int e1, e2;
  int dn1, dn2;
  //int g1, g2;
  
  if ( n1 == n2 )
    return 0;
    
  pi = fsm_GetOutputPINFO(fsm);
  
  /*
  if ( is_fbo != 0 )
  {
    if ( fsm_IsFBOCompatible(fsm, n1, n2) != 0 )
    {
      fsm_Log(fsm, "FSM: States '%s' and '%s' are fbo compatible.",
        fsm_GetNodeNameStr(fsm, n1), fsm_GetNodeNameStr(fsm, n2));
      return 1;
    }
  }
  */
  
  if ( dclInitVA(2, &c1, &c2) == 0 )
    return 0;

  
  /*
  fsm_GetNodePreOCover(fsm, n1, c1);
  fsm_GetNodePreOCover(fsm, n2, c2);
  */

  if ( fsm_GetNodeInOCover(fsm, n1, c1) == 0 )
    return dclDestroyVA(2, c1, c2), 0;
  if ( fsm_GetNodeInOCover(fsm, n2, c2) == 0 )
    return dclDestroyVA(2, c1, c2), 0;

  /* two states are not compatible, if the output is not compatible */

  if ( dcl_is_mealy_output_compatible(pi, c1, c2) == 0 )
  {
    fsm_Log(fsm, "FSM: States '%s' and '%s' have different outputs (not compatible).",
      fsm_GetNodeNameStr(fsm, n1), fsm_GetNodeNameStr(fsm, n2));
    /*
    fsm_LogDCL(fsm, "FSM: ", 2, fsm_GetNodeNameStr(fsm, n1), pi, c1, 
      fsm_GetNodeNameStr(fsm, n2), pi, c2);
    */
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
        //g1 = fsm_GetNodeGroupIndex(fsm, dn1);
        //g2 = fsm_GetNodeGroupIndex(fsm, dn2);
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

void hm_BuildStateMatrix(hm_type hm, int is_fbo)
{
  int n1, n2;
  n1 = -1;
  while( fsm_LoopNodes(hm->fsm, &n1) != 0 )
  {
    n2 = -1;
    while( fsm_LoopNodes(hm->fsm, &n2) != 0 )
      if ( n1 < n2 )
        hm_Set(hm, n1, n2, fsm_IsCompatible(hm->fsm, n1, n2, is_fbo));
  }
}

void hm_Show(hm_type hm)
{
  int n1, n2;
  for( n1 = 0; n1 < hm->n; n1++ )
  {
    for( n2 = 0; n2 < n1; n2++ )
    {
      if ( hm_Get(hm, n1, n2) == 0 )
        printf(".");
      else
        printf("*");
    }
    printf("\n");
  }
}

/* well, this assigns the group-index */
int fsm_minimize_states(fsm_type fsm, int is_fbo)
{
  hm_type hm;
  hm = hm_Open(fsm, b_set_Max(fsm->nodes));
  if ( hm == NULL )
    return 0;
  hm_BuildStateMatrix(hm, is_fbo);
  hm_BuildCompatibleSets(hm);
  hm_ShowCompatibleSets(hm);

  hm_Close(hm);
  return 1;
}

int fsm_OldMinimizeStates(fsm_type fsm, int is_fbo)
{
  return fsm_minimize_states(fsm, is_fbo);
}

