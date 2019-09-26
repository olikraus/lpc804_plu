/*

  xbmustt.c

  finite state machine for eXtended burst mode machines

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

  This modul does a 
    Unicode Single Transition Time (USTT)
  state encoding of the FSM.
  This encoding is required for asynchronous (race free)
  state machines.
  
  Reference:
    James H. Tracey
    "Internal State Assignment for Asynchronous Sequential Machines"
    IEEE Transactions on electronic computers, 1966
    
    Luciano Lavagno, Alberto Sangiovanni-Vincentelli
    Algorithms for synthesis and testing of asynchronous curcuits, 2nd printing
    Kluwer Academic Publishers, 1996
    page 56

*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "xbm.h"
#include "b_sp.h"

void xbm_DestroyAsync(xbm_type x)
{
  if ( x->pi_async != NULL )
    pinfoClose(x->pi_async);
  x->pi_async = NULL;
  if ( x->cl_async != NULL )
    dclDestroy(x->cl_async);
  x->cl_async = NULL;
}

int xbm_InitAsync(xbm_type x)
{
  xbm_DestroyAsync(x);
  x->pi_async = pinfoOpen();
  if ( x->pi_async == NULL )
    return 0;
  /* pinfoInitProgress(x->pi_async); */
  if ( pinfoSetInCnt(x->pi_async, b_set_Cnt(x->groups)) == 0 )
    return 0;
  if ( dclInit(&(x->cl_async)) == 0 )
    return 0;
  return 1;
}

/*---------------------------------------------------------------------------*/
static int xbm_separate_groups(xbm_type x, int g11, int g12, int g21, int g22)
{
  dcube *c = &(x->pi_async->tmp[11]);
  int i11;
  int i12;
  int i21;
  int i22;
  
  assert(g11 >= 0);
  assert(g12 >= 0);
  assert(g21 >= 0);
  assert(g22 >= 0);

  i11 = xbm_GetGr(x, g11)->user_val;
  i12 = xbm_GetGr(x, g12)->user_val;
  i21 = xbm_GetGr(x, g21)->user_val;
  i22 = xbm_GetGr(x, g22)->user_val;

  if ( g11 == g21 || g11 == g22 )
    return 1;
  if ( g12 == g21 || g12 == g22 )
    return 1;

  dcInSetAll(x->pi_async, c, CUBE_IN_MASK_DC);

  dcSetIn(c, i11, 1);
  dcSetIn(c, i12, 1);
  dcSetIn(c, i21, 2);
  dcSetIn(c, i22, 2);

  if ( dcInOneCnt(x->pi_async, c) != 0 && dcInZeroCnt(x->pi_async, c) != 0 )
  {
    xbm_Log(x, 0, "XBM: Adding partition %d: %s.", 
      dclCnt(x->cl_async), dcToStr(x->pi_async, c, "", ""));
    if ( dclAdd(x->pi_async, x->cl_async, c) < 0 )
    {
      xbm_Error(x, "xbm_separate_groups: Out of Memory (dclAdd).");
      return 0;
    }
  }
  return 1;
}


/*
  perhaps fbo should avoid the construction of separations at all.
  if the output values separate the two state pairs, adding the
  separation could be avoided?
*/

static int xbm_separate_states(xbm_type x, int s11, int s12, int s21, int s22)
{
  int g11 = xbm_GetSt(x, s11)->gr_pos;
  int g12 = xbm_GetSt(x, s12)->gr_pos;
  int g21 = xbm_GetSt(x, s21)->gr_pos;
  int g22 = xbm_GetSt(x, s22)->gr_pos;
  int i;
  int o1, o2;
  
  if ( x->is_fbo != 0 )
  {
    /* try to find a output variable that separates the two pairs */

    for( i = 0; i < xbm_GetPiOut(x)->out_cnt; i++ )
    {
      o1 = (dcGetOut(&(xbm_GetSt(x, s11)->out), i)+1)|
           (dcGetOut(&(xbm_GetSt(x, s12)->out), i)+1);
      o2 = (dcGetOut(&(xbm_GetSt(x, s21)->out), i)+1)|
           (dcGetOut(&(xbm_GetSt(x, s22)->out), i)+1);

      if ( o1 == 1 && o2 == 2 )
        return 1;
      if ( o1 == 2 && o2 == 1 )
        return 1;
    }
  }
  
  return xbm_separate_groups(x, g11, g12, g21, g22);
}

/*---------------------------------------------------------------------------*/

static int xbm_SeparateTrTr(xbm_type x)
{
  int t1, t2;
  pinfo *pi = xbm_GetPiIn(x);
  dcube *a;
  dcube *b;
  t1 = -1;
  while( xbm_LoopTr(x, &t1) != 0 )
  {
    a = &(xbm_GetTr(x, t1)->in_end_cond);
    t2 = t1;
    while( xbm_LoopTr(x, &t2) != 0 )
    {
      b = &(xbm_GetTr(x, t2)->in_end_cond);
      if ( dcIsDeltaInNoneZero(pi, a, b) == 0 )
      {
        if ( xbm_separate_states(x, 
          xbm_GetTrSrcStPos(x, t1), xbm_GetTrDestStPos(x, t1), 
          xbm_GetTrSrcStPos(x, t2), xbm_GetTrDestStPos(x, t2) ) == 0 )
            return 0;
      }
    }
  }
  return 1;
}

/*---------------------------------------------------------------------------*/

static int xbm_SeparateTrSt(xbm_type x)
{
  int t, s;
  pinfo *pi = xbm_GetPiIn(x);
  dclist cl;
  dcube *c;
  t = -1;
  while( xbm_LoopTr(x, &t) != 0 )
  {
    s = -1;
    c = &(xbm_GetTr(x, t)->in_end_cond);
    while( xbm_LoopSt(x, &s) != 0 )
    {
      cl = xbm_GetSt(x, s)->in_self_cl;
      if ( dclIsIntersectionCube(pi, cl, c) != 0 )
      {
        if ( xbm_separate_states(x, 
          xbm_GetTrSrcStPos(x, t), xbm_GetTrDestStPos(x, t), 
          s, s ) == 0 )
            return 0;
      }
    }
  }
  return 1;
}

/*---------------------------------------------------------------------------*/

static int xbm_SeparateGrGr(xbm_type x)
{
  int g1;
  int g2;

  int st1, st2;
  int separation_required;
  
  g1 = -1;
  while( xbm_LoopGr(x, &g1) != 0 )
  {
    g2 = g1;
    while( xbm_LoopGr(x, &g2) != 0 )
    {
      if ( x->is_fbo != 0 )
      {
        separation_required = 0;
        st1 = -1;
        while( xbm_LoopSt(x, &st1) != 0 && separation_required == 0)
        {
          if ( xbm_GetSt(x, st1)->gr_pos == g1 )
          {
            st2 = -1;
            while( xbm_LoopSt(x, &st2) != 0 && separation_required == 0 )
            {
              if ( xbm_GetSt(x, st2)->gr_pos == g2 )
              {
                if ( dcIsEqual(xbm_GetPiOut(x), 
                        &(xbm_GetSt(x, st1)->out), &(xbm_GetSt(x, st2)->out)) != 0 )
                {
                  separation_required = 1;
                  xbm_Log(x, 2, "XBM: Group separation %d/%d  required (state '%s'/'%s', out %s/%s).",
                    g1, g2, 
                    xbm_GetStNameStr(x, st1), xbm_GetStNameStr(x, st2),
                    dcToStr(xbm_GetPiOut(x), &(xbm_GetSt(x, st1)->out), "", ""),
                    dcToStr2(xbm_GetPiOut(x), &(xbm_GetSt(x, st2)->out), "", ""));
                }
              }
            }
          }
        }
      }
      else
        separation_required = 1;
      
      if ( separation_required != 0 )
      {
        if ( xbm_separate_groups(x, g1, g1, g2, g2) == 0 )
          return 0;
      }
      else
      {
        xbm_Log(x, 2, "XBM: Group separation %d/%d not required.", g1, g2);
      }
      
    }
  }
  return 1;
}

/*---------------------------------------------------------------------------*/

int xbm_BuildPartitionTable(xbm_type x)
{
  int g1;
  int g2;
  
  g1 = -1;
  g2 = 0;
  while( xbm_LoopGr(x, &g1) != 0 )
  {
    xbm_GetGr(x, g1)->user_val = g2;
    g2++;
  }

  if ( x->is_sync == 0 )
  {
    if ( xbm_SeparateTrTr(x) == 0 )
      return 0;
    if ( xbm_SeparateTrSt(x) == 0 )
      return 0;
  }
  if ( xbm_SeparateGrGr(x) == 0 )
    return 0;
  /*
  if ( x->is_fbo != 0 )
    if ( xbm_ApplyFeedbackOptimization(x) == 0 )
      return 0;
  */

  if ( x->pla_file_xbm_partitions != NULL )
  {
    xbm_Log(x, 4, "XBM: Writing XBM partitions to PLA file '%s'.", 
      x->pla_file_xbm_partitions);
    dclWritePLA(x->pi_async, x->cl_async, x->pla_file_xbm_partitions);
  }
  
  xbm_Log(x, 2, "XBM: Partitions: %d.", dclCnt(x->cl_async));

  dclSCCInv(x->pi_async, x->cl_async);

  xbm_Log(x, 2, "XBM: Partitions after SCCInv operation: %d.", dclCnt(x->cl_async));

  return 1;
}

/*===========================================================================*/

void xbm_MaxZeroPartitionTable(xbm_type x)
{
  dcube *c;
  int i, cnt = dclCnt(x->cl_async);
  int one_w, zero_w;
  int j;
  int st_pos;
  for( i = 0; i < cnt; i++ )
  {
    c = dclGet(x->cl_async, i);
    one_w = 0;
    zero_w = 0;
    st_pos = -1;
    while( xbm_LoopSt(x, &st_pos) != 0 )
    {
      for( j = 0; j < x->pi_async->in_cnt; j++ )
      {
        if ( xbm_GetSt(x, st_pos)->gr_pos == j )
        {
          if ( dcGetIn(c, j) == 1 )
            zero_w += 1;
          else if ( dcGetIn(c, j) == 2 )
            one_w += 1;
        }
      }
    }
    
    /*
    if ( dcInZeroCnt(x->pi_async, c) < x->pi_async->in_cnt/2 )
      dcInvIn(x->pi_async, c);
    */
    if ( zero_w < one_w )
      dcInvIn(x->pi_async, c);
  }
}

/*
void xbm_ResetZeroPartitionTable(xbm_type x)
{
  dcube *c;
  int i, cnt = dclCnt(x->cl_async);
  if ( x->reset_node_id < 0 )
  {
    fsm_MaxZeroPartitionTable(fsm);
    return;
  }
  for( i = 0; i < cnt; i++ )
  {
    c = dclGet(x->cl_async, i);
    if ( dcGetIn(c, x->reset_node_id) == 2 )
      dcInvIn(x->pi_async, c);
  }
}
*/

/*===========================================================================*/

static void xbm_do_msg(void *data, char *fmt, va_list va)
{
  xbm_type x = (xbm_type)data;
  xbm_LogVA(x, 2, fmt, va);
}

int xbm_MinimizePartitionTable(xbm_type x)
{
  if ( dclMinimizeUSTT(x->pi_async, x->cl_async, xbm_do_msg, x, "XBM: ",
    x->pla_file_prime_partitions) == 0 )
    return 0;

  if ( x->pla_file_min_partitions != NULL )
  {
    xbm_Log(x, 4, "XBM: Writing minimized partitions to PLA file '%s'.", 
      x->pla_file_min_partitions);
    dclWritePLA(x->pi_async, x->cl_async, x->pla_file_min_partitions);
  }

  xbm_Log(x, 2, "XBM: Partitions after minimization: %d.", dclCnt(x->cl_async));

  return 1;
}

/*===========================================================================*/

static void xbm_assign_state_code(xbm_type x)
{
  int st_pos;
  int column;
  int i, cnt = dclCnt(x->cl_async);
  dcube *c;
  int v;

  st_pos = -1;
  while( xbm_LoopSt(x, &st_pos) != 0 )
  {
    assert(xbm_GetSt(x, st_pos)->gr_pos >= 0);
    column = xbm_GetGr(x, xbm_GetSt(x, st_pos)->gr_pos)->user_val;
    c = &(xbm_GetSt(x, st_pos)->code);
    for( i = 0; i < cnt; i++ )
    {
      v = dcGetIn(dclGet(x->cl_async, i), column);
      if ( v == 3 )
        v = 1;
      dcSetOut(c, i, v-1);
    }
    xbm_Log(x, 2, "XBM: Code '%s' --> state '%s'/group %d.",
      dcToStr(xbm_GetPiCode(x), c, "", ""), 
      xbm_GetStNameStr(x, st_pos), 
      xbm_GetSt(x, st_pos)->gr_pos);
  }
}

static int xbm_EncodeWithPartitionTable(xbm_type x)
{
  int code_with = dclCnt(x->cl_async);
  
  if ( xbm_SetCodeWidth(x, code_with) == 0 )
    return 0;
    
  xbm_assign_state_code(x);
  
  return 1;
}

/*===========================================================================*/

int xbm_encode_line(xbm_type x, char *s)
{
  char *state_name;
  int st_pos;
  
  state_name = b_getid(&s);
  if ( state_name == NULL )
    return 0;
  st_pos = xbm_FindStByName(x, state_name);
  if ( st_pos < 0 )
  {
    xbm_Error(x, "XBM: Can not find state '%s'.", state_name);
    return 0;
  }
  b_skipspace(&s);
  
  if ( dcSetOutByStr(xbm_GetPiCode(x), &(xbm_GetSt(x, st_pos)->code), s) == 0 )
    return 0;
  
  xbm_Log(x, 2, "XBM: Code '%s' --> state '%s'.", 
    dcToStr(xbm_GetPiCode(x), &(xbm_GetSt(x, st_pos)->code), "", ""), 
    state_name);
  
  return 1;
}

int xbm_encode_with_fp(xbm_type x, FILE *fp)
{
  int is_init = 0;
  char *s;
  for(;;)
  {
    if ( fgets(x->labelbuf, 1024, fp) == NULL )
      break;
    s = x->labelbuf;
    b_skipspace(&s);

    if ( s[0] == ';' || s[0] == '#' || s[0] == '\0' )
    {
      /* nothing */
    }
    else if ( s[0] == '.' && is_init == 0 )
    {
      if ( strncmp(s, ".s ", 3) == 0 )
      {
        if ( xbm_SetCodeWidth(x, atoi(s+3)) == 0 )
          return 0;
      }
      else
      {
        xbm_Error(x, "XBM: Illegal command (.s <state-bits> expected).");
        return 0;
      }
    }
    else if ( s[0] == '.' )
    {
      xbm_Error(x, "XBM: Dot commands are not allowed any more.");
      return 0;
    }
    else
    {
      is_init = 1;
      if ( xbm_encode_line(x, s) == 0 )
        return 0;
    }
  }
  return 1;
}

int xbm_EncodeWithFile(xbm_type x, const char *name)
{
  FILE *fp;
  fp = fopen(name, "r");
  if ( fp == NULL )
  {
    xbm_Error(x, "XBM: File '%s' not found.", name);
    return 0;
  }
  if ( xbm_encode_with_fp(x, fp) == 0 )
  {
    fclose(fp);
    return 0;
  }
  fclose(fp);
  xbm_Log(x, 4, "XBM: Import of state encoding '%s' done.", name);
  return 1;
}

/*===========================================================================*/

static int xbm_encode_synchronous_state_machine(xbm_type x)
{
  int cnt = xbm_GetGrCnt(x);
  int log_2_cnt;
  int gr, st;
  dcube *c = &(xbm_GetPiCode(x)->tmp[9]);
  
  if ( cnt == 0 )
    return 1;
  
  cnt--;
  log_2_cnt = 0;
  while( cnt != 0 )
  {
    cnt /= 2;
    log_2_cnt++;
  }

  if ( xbm_SetCodeWidth(x, log_2_cnt) == 0 )
    return 0;
    
  xbm_Log(x, 0, "XBM: Group count %d, state count %d.",
    xbm_GetGrCnt(x), xbm_GetStCnt(x));

  dcOutSetAll(xbm_GetPiCode(x), c, 0);

  gr = -1;  
  while( xbm_LoopGr(x, &gr) != 0 )
  {
    st = -1;
    while( xbm_LoopSt(x, &st) != 0 )
    {
      if ( xbm_GetSt(x, st)->gr_pos == gr )
      {
        dcCopyOut(xbm_GetPiCode(x), &(xbm_GetSt(x, st)->code), c);
        xbm_Log(x, 2, "XBM: State '%s' code '%s'.", 
          xbm_GetStNameStr(x, st), 
          dcToStr(xbm_GetPiCode(x), c, "", ""));
      }
    }
    dcIncOut(xbm_GetPiCode(x), c);
  }

  return 1;  
}

/*===========================================================================*/

int xbm_EncodeStates(xbm_type x)
{
  /* file import */

  if ( x->import_file_state_codes != NULL )
    return xbm_EncodeWithFile(x, x->import_file_state_codes);


  /* calculate state encoding */

  if ( x->is_sync != 0 )
    return xbm_encode_synchronous_state_machine(x);
    
  /* note: The following code are also able to generate a synchronous */
  /*       state encoding. See xbm_BuildPartitionTable() above. */

  if ( xbm_InitAsync(x) == 0 )
    return 0;
  xbm_Log(x, 3, "XBM: Build partition table for state encoding.");
  if ( xbm_BuildPartitionTable(x) == 0 )
    return xbm_DestroyAsync(x), 0;

  xbm_Log(x, 3, "XBM: Minimize partition table for state encoding.");
  if ( xbm_MinimizePartitionTable(x) == 0 )
    return xbm_DestroyAsync(x), 0;

  xbm_Log(x, 3, "XBM: Optimize encoding.");
  xbm_MaxZeroPartitionTable(x);
  
  xbm_Log(x, 3, "XBM: Encode states.");
  if ( xbm_EncodeWithPartitionTable(x) == 0 )
    return xbm_DestroyAsync(x), 0;

  return xbm_DestroyAsync(x), 1;
}


/*===========================================================================*/

int xbm_WriteEncodingFP(xbm_type x, FILE *fp)
{
  int st_pos;
  fprintf(fp, ".s %d\n", xbm_GetPiCode(x)->out_cnt);
  
  st_pos = -1;
  while( xbm_LoopSt(x, &st_pos) != 0 )
  {
    fprintf(fp, "%s", xbm_GetStNameStr(x, st_pos));
    fprintf(fp, " ");
    fprintf(fp, "%s", 
      dcToStr(xbm_GetPiCode(x), &(xbm_GetSt(x, st_pos)->code), "", ""));
    fprintf(fp, "\n");
  }
  return 1;
}

int xbm_WriteEncoding(xbm_type x, const char *name)
{
  FILE *fp;
  fp = fopen(name, "w");
  if ( fp == NULL )
    return 0;
  if ( xbm_WriteEncodingFP(x, fp) == 0 )
  {
    fclose(fp);
    return 0;
  }
  fclose(fp);
  return 1;
}

