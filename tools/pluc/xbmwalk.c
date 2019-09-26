/*

  xbmwalk.c

  finite state machine for eXtended burst mode machines
  
  walk through the graph

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

*/

#include "xbm.h"
#include <assert.h>

/*---------------------------------------------------------------------------*/

static void xbm_relax(xbm_type x, int st_src_pos, int st_dest_pos)
{
  if ( xbm_GetSt(x, st_dest_pos)->d > xbm_GetSt(x, st_src_pos)->d + 1)
  {
    xbm_GetSt(x, st_dest_pos)->d = xbm_GetSt(x, st_src_pos)->d + 1;
    xbm_GetSt(x, st_dest_pos)->p = st_src_pos;
  }
}

static int xbm_check(xbm_type x, int st_src_pos, int st_dest_pos)
{
  if ( xbm_GetSt(x, st_dest_pos)->d > xbm_GetSt(x, st_src_pos)->d + 1)
    return 0;
  return 1;
}

static void xbm_init_bellman_ford(xbm_type x, int st_start_pos)
{
  int st_pos;
  st_pos = -1;
  while( xbm_LoopSt(x, &st_pos) != 0 )
  {
    xbm_GetSt(x, st_pos)->d = (1<<((8*sizeof(int))-2));
    xbm_GetSt(x, st_pos)->p = -1;
  }
  xbm_GetSt(x, st_start_pos)->d = 0;
}

/*
  The weight is '1' so I could also use Dijkstra's algorithm, but 
  the Bellman-Ford algorithm seems to be a little bit easier to implement.
  It doesn't require a priority queue...
  well, yes, Bellman-Ford is slower..
  
  taken from CLR, Introduction to algorithms, p 520ff
*/

static int xbm_do_bellman_ford(xbm_type x)
{
  int st_pos, tr_pos;
  
  st_pos = -1;
  while( xbm_LoopSt(x, &st_pos) != 0 )
  {
    tr_pos = -1;
    while( xbm_LoopTr(x, &tr_pos) != 0 )
      xbm_relax(x, xbm_GetTrSrcStPos(x, tr_pos), xbm_GetTrDestStPos(x, tr_pos));
  }
  
  tr_pos = -1;
  while( xbm_LoopTr(x, &tr_pos) != 0 )
    if ( xbm_check(x, xbm_GetTrSrcStPos(x, tr_pos), xbm_GetTrDestStPos(x, tr_pos)) == 0 )
      return 0;
    
  return 1;
}

int xbm_bellman_ford(xbm_type x, int st_start_pos)
{
  xbm_init_bellman_ford(x, st_start_pos);
  return xbm_do_bellman_ford(x);
}

/*---------------------------------------------------------------------------*/

static int xbm_GetTrVisitMax(xbm_type x, int tr_pos)
{
  int i;
  int visit_max = 1;
  dcube *s = &(xbm_GetTr(x, tr_pos)->in_start_cond);
  dcube *e = &(xbm_GetTr(x, tr_pos)->in_end_cond);

  if ( x->is_all_dc_change != 0 )
    return visit_max;

  for ( i = 0; i < x->inputs; i++ )
  {
    if ( dcGetIn(s, i) == 3 )
    {
      /* a directed don't care might have two different values */
      visit_max *= 2;
    }
  }
  return visit_max;
}

int xbm_GetNthTrVec(xbm_type x, int tr_pos, dcube *cs, dcube *ct, int n)
{
  int i;
  int pos = 0;
  dcube *s = &(xbm_GetTr(x, tr_pos)->in_start_cond);
  dcube *e = &(xbm_GetTr(x, tr_pos)->in_end_cond);
  dcube *dc = &(xbm_GetTr(x, tr_pos)->in_ddc_start_cond);
  int in_cnt = x->inputs;
  int out_cnt = x->outputs;
  int code_cnt = xbm_GetPiCode(x)->out_cnt;
  int st_src_pos = xbm_GetTrSrcStPos(x, tr_pos);
  int st_dest_pos = xbm_GetTrDestStPos(x, tr_pos);
  
  for ( i = 0; i < in_cnt; i++ )
  {
    if ( dcGetIn(s, i) == 3 )
    {
      if ( x->is_all_dc_change != 0 )
      {
        dcSetIn(cs, i, dcGetIn(dc, i));
        dcSetIn(ct, i, 3-dcGetIn(dc, i));
      }
      else
      {
        dcSetIn(cs, i, (n>>pos)&1);
        dcSetIn(ct, i, (n>>pos)&1);
        pos++;
      }
    }
    else
    {
      dcSetIn(cs, i, dcGetIn(s, i));
      dcSetIn(ct, i, dcGetIn(e, i));
    }
  }
  
  dcCopyOutToIn( xbm_GetPiMachine(x), cs, in_cnt, 
    xbm_GetPiCode(x), &(xbm_GetSt(x, st_src_pos)->code));
  if ( x->is_fbo != 0 )
     dcCopyOutToIn( xbm_GetPiMachine(x), cs, in_cnt+code_cnt, 
      xbm_GetPiOut(x), &(xbm_GetSt(x, st_src_pos)->out));
  dcCopyOutToOut( xbm_GetPiMachine(x), cs, 0, 
    xbm_GetPiCode(x), &(xbm_GetSt(x, st_src_pos)->code));
  dcCopyOutToOut( xbm_GetPiMachine(x), cs, code_cnt, 
    xbm_GetPiOut(x), &(xbm_GetSt(x, st_src_pos)->out));
  
  dcCopyOutToIn( xbm_GetPiMachine(x), ct, in_cnt, 
    xbm_GetPiCode(x), &(xbm_GetSt(x, st_src_pos)->code));
  if ( x->is_fbo != 0 )
    dcCopyOutToIn( xbm_GetPiMachine(x), ct, in_cnt+code_cnt, 
      xbm_GetPiOut(x), &(xbm_GetSt(x, st_src_pos)->out));
  dcCopyOutToOut( xbm_GetPiMachine(x), ct, 0, 
    xbm_GetPiCode(x), &(xbm_GetSt(x, st_dest_pos)->code));
  dcCopyOutToOut( xbm_GetPiMachine(x), ct, code_cnt, 
    xbm_GetPiOut(x), &(xbm_GetSt(x, st_dest_pos)->out));

  return 1;
}

static void xbm_init_walk(xbm_type x)
{
  int tr_pos = -1;
  while( xbm_LoopTr(x, &tr_pos) != 0 )
  {
    xbm_GetTr(x, tr_pos)->visit_cnt = 0;    /* all is white */
    xbm_GetTr(x, tr_pos)->visit_max = xbm_GetTrVisitMax(x, tr_pos);
  }
}

static int xbm_exec_st_st(xbm_type x, int st1_pos, int st2_pos, int n)
{
  int tr_pos;
  int loop;
  int is_reset = 0;
  dcube *cs;
  dcube *ct;
  
  cs = &(x->pi_machine->tmp[3]);
  ct = &(x->pi_machine->tmp[4]);
  
  if ( xbm_GetSt(x, st1_pos)->p < 0 )
  {
    assert(st1_pos == x->reset_st_pos);
    is_reset = 1;
  }
  tr_pos = -1;
  loop = -1;
  while ( xbm_LoopStOutTr(x, st1_pos, &loop, &tr_pos) != 0 )
    if ( xbm_GetTrDestStPos(x, tr_pos) == st2_pos )
    {
      xbm_GetNthTrVec(x, tr_pos, cs, ct, n);
      if ( x->walk_cb(x, x->walk_data, tr_pos, cs, ct, is_reset, 0) == 0 )
        return 0;
      break;
    }
  
  return 1;
}

static int xbm_do_quick_walk(xbm_type x, int st_pos)
{
  if ( xbm_GetSt(x, st_pos)->p >= 0 )
  {
    xbm_do_quick_walk(x, xbm_GetSt(x, st_pos)->p);
    if ( xbm_exec_st_st(x, xbm_GetSt(x, st_pos)->p, st_pos, 0) == 0 )
      return 0;
  }
  return 1;
}

static int xbm_exec_tr(xbm_type x, int tr_pos, int n, int is_reset)
{
  dcube *cs;
  dcube *ct;
  
  cs = &(x->pi_machine->tmp[3]);
  ct = &(x->pi_machine->tmp[4]);

  xbm_GetNthTrVec(x, tr_pos, cs, ct, n);

  return x->walk_cb(x, x->walk_data, tr_pos, cs, ct, is_reset, 1) ;
}

static int xbm_do_depth_walk(xbm_type x, int st_pos)
{
  int tr_pos, loop;
  int st_curr_pos;
  int st_next_pos;
  int is_reset = 0;
  if ( st_pos == x->reset_st_pos )
    is_reset = 1;
  
  st_curr_pos = st_pos;
  do
  {
    tr_pos = -1;
    loop = -1;
    st_next_pos = -1;
    while ( xbm_LoopStOutTr(x, st_curr_pos, &loop, &tr_pos) != 0 )
    {
      if ( xbm_GetTr(x, tr_pos)->visit_cnt < xbm_GetTr(x, tr_pos)->visit_max )
      {
        if ( xbm_exec_tr( x, tr_pos, xbm_GetTr(x, tr_pos)->visit_cnt, is_reset) == 0 )
          return 0;
        is_reset = 0;
        xbm_GetTr(x, tr_pos)->visit_cnt++;
        st_next_pos = xbm_GetTrDestStPos(x, tr_pos);
        break;
      }
    }
    st_curr_pos = st_next_pos;
  } while( st_next_pos >= 0 );
  
  return 1;
}

/* check if there are still transition that are not visited */
static int xbm_is_white(xbm_type x, int st_pos)
{
  int tr_pos, loop;
  tr_pos = -1;
  loop = -1;
  while ( xbm_LoopStOutTr(x, st_pos, &loop, &tr_pos) != 0 )
    if ( xbm_GetTr(x, tr_pos)->visit_cnt < xbm_GetTr(x, tr_pos)->visit_max )
      return 1;
  return 0;
}

/*
  idea: go to a state, try to go as far as possible
        loop as long as there are non-visited transitions
*/

static int xbm_do_walk_loop(xbm_type x, int st_pos)
{
  while( xbm_is_white(x, st_pos) != 0 )
  {
    if ( xbm_do_quick_walk(x, st_pos) == 0 )
      return 0;
    if ( xbm_do_depth_walk(x, st_pos) == 0 )
      return 0;
  }
  return 1;
}

static int xbm_do_walk(xbm_type x)
{
  int st_pos;

  /* a reset state is really required */
  if ( x->reset_st_pos < 0 )
    return 0;

  /* our quick walk follows the shortest path, so calculate it first */
  if ( xbm_bellman_ford(x, x->reset_st_pos) == 0 )
    return 0;
    
  /* reset all transitions */
  xbm_init_walk(x);

  /* setup has finished, walk starts here */    

  /* start walking from the reset state first */
  if ( xbm_do_walk_loop(x, x->reset_st_pos) == 0 )
    return 0;

  /* start walking from the remaining states */  
  st_pos = -1;
  while( xbm_LoopSt(x, &st_pos) != 0 )
    if ( xbm_do_walk_loop(x, st_pos) == 0 )
      return 0;
  
  return 1;
}

/* cs, ct: x->pi_machine */
int xbm_DoWalk(xbm_type x, int (*walk_cb)(xbm_type x, void *data, int tr_pos, dcube *cs, dcube *ct, int is_reset, int is_unique), void *data)
{
  if ( x->pi_machine == NULL || x->cl_machine == NULL )
    return 0;
  x->walk_cb = walk_cb;
  x->walk_data = data;
  return xbm_do_walk(x);
}

/*---------------------------------------------------------------------------*/

static int xbm_stdout_walk_cb(xbm_type x, void *data, int tr_pos, dcube *cs, dcube *ct, int is_reset, int is_unique)
{
  if ( is_reset != 0 )
    printf("\n%s", xbm_GetStNameStr(x, xbm_GetTrSrcStPos(x, tr_pos)));
  if ( is_unique != 0 )
    printf(" --> %s", xbm_GetStNameStr(x, xbm_GetTrDestStPos(x, tr_pos)));
  else
    printf(" ==> %s", xbm_GetStNameStr(x, xbm_GetTrDestStPos(x, tr_pos)));
  return 1;
}

int xbm_ShowWalk(xbm_type x)
{
  xbm_DoWalk(x, xbm_stdout_walk_cb, NULL);
  puts("");
  return 1;
}
