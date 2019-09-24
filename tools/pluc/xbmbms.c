/*

  xbmbms.c

  finite state machine for eXtended burst mode machines
  parser for BMS file format

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

  symbol  alternative  description

  A+                   rising edge
  A-                   falling edge
  A*                   directed don't care
  A#      [A+]         high level
  A~      [A-]         low level
  

*/

#include "xbm.h"
#include "b_ff.h"
#include <stdio.h>
#include <assert.h>
#include <ctype.h>
#include <string.h>

/*
  start   end    description          start  end   ddc_start ddc_end
    0      1     rising edge            0     1        0        1
    1      0     falling edge           1     0        1        0
    1      -     directed dc down       -     -        1        1
    0      -     directed dc up         -     -        0        0
*/

#define XBM_CHR_RISE_EDGE '+'
#define XBM_CHR_FALL_EDGE '-'
#define XBM_CHR_DONT_CARE '*'
#define XBM_CHR_HIGH_LEVEL '#'
#define XBM_CHR_LOW_LEVEL '~'

/*---------------------------------------------------------------------------*/
/* check incoming and outgoing transition of a state of an xbm circuit */

/* tc: transition cube */
/* s, e: start and end cube */
/* if tc(s,e) # s # e of an outgoing arc has a none-empty    */
/* intersection with e of an incoming arc --> not compatible */

int xbm_IsHFInOutTransitionPossible(xbm_type x, int n1, int n2)
{
  dclist cl;
  dcube *s1;
  //dcube *e1;
  //dcube *s2;
  dcube *e2;
  int l1, tr1, l2, tr2;

  if ( dclInit(&cl) == 0 )
    return 0;
    
  l1 = -1;
  tr1 = -1;
  while( xbm_LoopStOutTr(x, n1, &l1, &tr1 ) != 0 )
  {
    s1 = &(xbm_GetTr(x, tr1)->in_ddc_start_cond);
    //e1 = &(xbm_GetTr(x, tr1)->in_end_cond);
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
      xbm_Log(x, 0, "XBM: %s/%s t/e  %s -> %s: %s    %s -> %s: %s.",
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
        if ( n1 == n2 )
        {
          /* called from xbmbms. */
          xbm_Error(x, "XBM bms: Can not create hazardfree transition. Conflict between %s -> %s and %s -> %s.",
            xbm_GetStNameStr(x, xbm_GetTrSrcStPos(x, tr1)),
            xbm_GetStNameStr(x, xbm_GetTrDestStPos(x, tr1)),
            xbm_GetStNameStr(x, xbm_GetTrSrcStPos(x, tr2)),
            xbm_GetStNameStr(x, xbm_GetTrDestStPos(x, tr2))
            );
          return dclDestroy(cl), 0;
        }
        else
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
          return dclDestroy(cl), 0;
        }
      }
    }
  }

  return dclDestroy(cl), 1;
}

int xbm_IsHFOutOutTransitionPossible(xbm_type x, int n1, int n2)
{
  dcube *s1;
  dcube *e1;
  dcube *s2;
  dcube *e2;
  dcube *trc1 = &(xbm_GetPiIn(x)->tmp[8]);
  dcube *trc2 = &(xbm_GetPiIn(x)->tmp[9]);
  int l1, tr1, l2, tr2;

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
        xbm_Log(x, 0, "XBM: %s/%s t/t  %s -> %s: %s    %s -> %s: %s.",
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
            if ( n1 == n2 )
            {
              /* called from xbmbms. */
              xbm_Error(x, "XBM bms: Can not create hazardfree transition. Conflict between %s -> %s and %s -> %s.",
                xbm_GetStNameStr(x, xbm_GetTrSrcStPos(x, tr1)),
                xbm_GetStNameStr(x, xbm_GetTrDestStPos(x, tr1)),
                xbm_GetStNameStr(x, xbm_GetTrSrcStPos(x, tr2)),
                xbm_GetStNameStr(x, xbm_GetTrDestStPos(x, tr2))
                );
              return 0;
            }
            else
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
              return 0;
            }
          }
        }
      }
    }
  }

  return 1;
}

/*---------------------------------------------------------------------------*/
/* check for unused variables */

static int xbm_check_variables(xbm_type x)
{
  int var_pos;
  
  var_pos = -1;
  while( xbm_LoopVar(x, &var_pos) != 0 )
    if ( xbm_GetVar(x, var_pos)->is_used == 0 )
    {
      xbm_Error(x, "XBM bms: Unused variable '%s'.", 
          xbm_GetVarNameStr(x, var_pos));
      return 0;
    }
  return 1;
}


/*---------------------------------------------------------------------------*/
/* apply directed don't care    */

static int xbm_apply_ddc(xbm_type x)
{
  int tr_pos;
  int i;
  //int s, e;
  xbm_var_type var;
  
  for( i = 0; i < x->inputs; i++ )
  {
    var = xbm_GetVarByIndex(x, XBM_DIRECTION_IN, i);
    assert( var != NULL );
    
    xbm_Log(x, 3, "XBM bms: Variable '%s', edge sensitive: %d, level check: %d.",
      var->name==NULL?"":var->name,
      var->is_edge, var->is_level);
      
    if ( (x->is_auto_ddc_level != 0 && var->is_level != 0 && var->is_edge == 0) ||
         (x->is_auto_ddc_edge != 0  && var->is_level == 0 && var->is_edge != 0) ||
         (x->is_auto_ddc_mix != 0   && var->is_level != 0 && var->is_edge != 0) )
    {
      xbm_Log(x, 3, "XBM bms: Auto apply 'directed don't care' for variable '%s'.",
          var->name==NULL?"":var->name);
          
      tr_pos = -1;
      while( xbm_LoopTr(x, &tr_pos) != 0 )
      {
        if ( dcGetIn(&(xbm_GetTr(x, tr_pos)->level_cond), i) == 0 &&
             dcGetIn(&(xbm_GetTr(x, tr_pos)->edge_cond), i) == 0 )
        {
          dcSetIn(&(xbm_GetTr(x, tr_pos)->level_cond), i, 3);
          xbm_Log(x, 1, "XBM bms: Auto apply directed don't care, variable '%s', transition '%s' -> '%s'.",
            var->name==NULL?"":var->name,
            xbm_GetStNameStr(x, xbm_GetTrSrcStPos(x, tr_pos)), 
            xbm_GetStNameStr(x, xbm_GetTrDestStPos(x, tr_pos)));
        }
      }
    }
  }
  
  return 1;
}

/*---------------------------------------------------------------------------*/
/* copy level/edge to start/end */

static int xbm_copy_tr_level_edge_to_start_end(xbm_type x, int tr_pos)
{
  xbm_transition_type t = xbm_GetTr(x, tr_pos);
  int i;
  int l, e;
  
  dcInSetAll(xbm_GetPiCond(x),  &(t->start_cond), CUBE_IN_MASK_DC);
  dcInSetAll(xbm_GetPiCond(x),  &(t->end_cond), CUBE_IN_MASK_DC);

  for( i = 0; i < x->inputs+x->outputs; i++ )
  {
    dcSetIn(&(t->start_cond), i, 0);
    dcSetIn(&(t->end_cond), i, 0);
  }

  for( i = 0; i < x->inputs+x->outputs; i++ )
  {
    l = dcGetIn(&(t->level_cond), i);
    e = dcGetIn(&(t->edge_cond), i);
    
    assert(e != 3);
    if ( e != 0 )
    {
      dcSetIn(&(t->start_cond), i, 3-e);
      dcSetIn(&(t->end_cond), i, e);
    }
    else if ( l != 0 )
    {
      dcSetIn(&(t->start_cond), i, l);
      dcSetIn(&(t->end_cond), i, l);
    }
  }

  return 1;
}


static int xbm_copy_level_edge_to_start_end(xbm_type x)
{
  int tr_pos;
  
  tr_pos = -1;
  while( xbm_LoopTr(x, &tr_pos) != 0 )
    if ( xbm_copy_tr_level_edge_to_start_end(x, tr_pos) == 0 )
      return 0;
      
  return 1;
}


/*---------------------------------------------------------------------------*/
/* propagate values through the graph */

static int xbm_update_tr_next(xbm_type x, int tr_pos, int *is_changed)
{
  int dest_state_pos = xbm_GetTrDestStPos(x, tr_pos);
  int loop, next_tr_pos;
  xbm_transition_type t1 = xbm_GetTr(x, tr_pos);
  xbm_transition_type t2;
  int i;
  int v1s;
  int v1e;
  int v2s;
  int v2e;
  const char *v;
          
  loop = -1;
  next_tr_pos = -1;

  while( xbm_LoopStOutTr(x, dest_state_pos, &loop, &next_tr_pos) != 0 )
  {
    t2 = xbm_GetTr(x, next_tr_pos);
    for( i = 0; i < x->inputs+x->outputs; i++ )
    {
      v1s = dcGetIn(&(t1->start_cond), i);
      v1e = dcGetIn(&(t1->end_cond), i);

      v2s = dcGetIn(&(t2->start_cond), i);
      v2e = dcGetIn(&(t2->end_cond), i);
      
      /* do some checks... */
      
      
      /*
        start   end    description      
          1      1     constant 1         -->  1  -
          0      0     constant 0         -->  0  -
          0      1     rising edge      
          1      0     falling edge     
          1      -     directed dc down 
          0      -     directed dc up   
      */
      /* rising edge or constant 1 with rising edge */
      if ( v1e == 2 && v2s == 1 && v2e == 2 )
      {
        if ( i < x->inputs )
          v = xbm_GetVarNameStrByIndex(x, XBM_DIRECTION_IN, i);
        else
          v = xbm_GetVarNameStrByIndex(x, XBM_DIRECTION_OUT, i-x->inputs);
        xbm_Error(x, "XBM bms: Logic high '%s' -> '%s' followed by rising edge '%s' -> '%s' on variable '%s'.",
            xbm_GetStNameStr(x, xbm_GetTrSrcStPos(x, tr_pos)),
            xbm_GetStNameStr(x, xbm_GetTrDestStPos(x, tr_pos)),
            xbm_GetStNameStr(x, xbm_GetTrSrcStPos(x, next_tr_pos)),
            xbm_GetStNameStr(x, xbm_GetTrDestStPos(x, next_tr_pos)),
            v);
        return 0;
      }
      /* directed dc down with rising edge */
      if ( v1s == 2 && v1e == 3 && v2s == 1 && v2e == 2 )
      {
        if ( i < x->inputs )
          v = xbm_GetVarNameStrByIndex(x, XBM_DIRECTION_IN, i);
        else
          v = xbm_GetVarNameStrByIndex(x, XBM_DIRECTION_OUT, i-x->inputs);
        xbm_Error(x, "XBM bms: Directed dont't care down '%s' -> '%s' followed by rising edge '%s' -> '%s' on variable '%s'.",
            xbm_GetStNameStr(x, xbm_GetTrSrcStPos(x, tr_pos)),
            xbm_GetStNameStr(x, xbm_GetTrDestStPos(x, tr_pos)),
            xbm_GetStNameStr(x, xbm_GetTrSrcStPos(x, next_tr_pos)),
            xbm_GetStNameStr(x, xbm_GetTrDestStPos(x, next_tr_pos)),
            v);
        return 0;
      }
      /* falling edge or constant 0 with falling edge */
      if ( v1e == 1 && v2s == 2 && v2e == 1 )
      {
        if ( i < x->inputs )
          v = xbm_GetVarNameStrByIndex(x, XBM_DIRECTION_IN, i);
        else
          v = xbm_GetVarNameStrByIndex(x, XBM_DIRECTION_OUT, i-x->inputs);
        xbm_Error(x, "XBM bms: Logic low '%s' -> '%s' followed by falling edge '%s' -> '%s' on variable '%s'.",
            xbm_GetStNameStr(x, xbm_GetTrSrcStPos(x, tr_pos)),
            xbm_GetStNameStr(x, xbm_GetTrDestStPos(x, tr_pos)),
            xbm_GetStNameStr(x, xbm_GetTrSrcStPos(x, next_tr_pos)),
            xbm_GetStNameStr(x, xbm_GetTrDestStPos(x, next_tr_pos)),
            v);
        return 0;
      }
      /* directed dc up with falling edge */
      if ( v1s == 1 && v1e == 3 && v2s == 2 && v2e == 1 )
      {
        if ( i < x->inputs )
          v = xbm_GetVarNameStrByIndex(x, XBM_DIRECTION_IN, i);
        else
          v = xbm_GetVarNameStrByIndex(x, XBM_DIRECTION_OUT, i-x->inputs);
        xbm_Error(x, "XBM bms: Directed dont't care down '%s' -> '%s' followed by falling edge '%s' -> '%s' on variable '%s'.",
            xbm_GetStNameStr(x, xbm_GetTrSrcStPos(x, tr_pos)),
            xbm_GetStNameStr(x, xbm_GetTrDestStPos(x, tr_pos)),
            xbm_GetStNameStr(x, xbm_GetTrSrcStPos(x, next_tr_pos)),
            xbm_GetStNameStr(x, xbm_GetTrDestStPos(x, next_tr_pos)),
            v);
        return 0;
      }
      
      if ( dcGetIn(&(t2->level_cond), i) == 0 && dcGetIn(&(t2->edge_cond), i) == 0 )
      {
        if ( v2s != v2e )
        {
          if ( i < x->inputs )
            v = xbm_GetVarNameStrByIndex(x, XBM_DIRECTION_IN, i);
          else
            v = xbm_GetVarNameStrByIndex(x, XBM_DIRECTION_OUT, i-x->inputs);
          xbm_Error(x, "XBM bms: Condition '%s' -> '%s' is too restrictive for variable '%s'",
            xbm_GetStNameStr(x, xbm_GetTrSrcStPos(x, next_tr_pos)),
            xbm_GetStNameStr(x, xbm_GetTrDestStPos(x, next_tr_pos)),
            v);
          xbm_Error(x, "XBM bms: Variable '%s', state '%s': Entry value '%c', exit value '%c'. ",
            v, 
            xbm_GetStNameStr(x, xbm_GetTrDestStPos(x, next_tr_pos)),
            "x01-"[v2s], 
            "x01-"[v2e]);
          return 0;
        }
        
        if ( (v1e|v2e) != v2e )
        {
          dcSetIn(&(t2->start_cond), i, v2s|v1e);
          dcSetIn(&(t2->end_cond), i, v2e|v1e);
          *is_changed = 1;
        }
      }
      
      /* directed don't care */
      
      if ( v2s == 3 && v2e == 3 )
      {
        if ( v1e == 2 || v1e == 1 )
        {
          /* last transition ends in a valid value */
          dcSetIn(&(t2->start_cond), i, v1e);
          *is_changed = 1;
          xbm_Log(x, 2, "XBM bms: Directed don't care '%c' -> '-', variable '%s', states '%s' -> '%s'.",
            v1s+'0'-1, xbm_GetVarNameStrByIndex(x, XBM_DIRECTION_IN, i),
            xbm_GetStNameStr(x, t2->src_state_pos), 
            xbm_GetStNameStr(x, t2->dest_state_pos));
        }
        else if ( (v1s == 2 || v1s == 1) && v1e == 3 )
        {
          /* last transition already is a directed don't care */
          dcSetIn(&(t2->start_cond), i, v1s);
          xbm_Log(x, 2, "XBM bms: Directed don't care '%c' -> '-', variable '%s', states '%s' -> '%s'.",
            v1s+'0'-1, xbm_GetVarNameStrByIndex(x, XBM_DIRECTION_IN, i),
            xbm_GetStNameStr(x, t2->src_state_pos), 
            xbm_GetStNameStr(x, t2->dest_state_pos));
          *is_changed = 1;
        }
      }
    }
  }
  return 1;
}

static int xbm_update_next(xbm_type x)
{
  int my_is_changed;
  int tr_pos;
  int loop_max = b_set_Cnt(x->transitions)*2;
  int loop_cnt = 0;
  
  do
  {
    my_is_changed = 0;
    tr_pos = -1;
    while( xbm_LoopTr(x, &tr_pos) != 0 )
    {
      if ( xbm_update_tr_next(x, tr_pos, &my_is_changed) == 0 )
        return 0;
    }
    loop_cnt++;
    if ( loop_cnt >= loop_max )
    {
      xbm_Error(x, "XBM: Loop detected.");
      return 0;
    }
  } while( my_is_changed != 0 );
  
  return 1;
}

static int xbm_update_tr_prev(xbm_type x, int tr_pos, int *is_changed)
{
  int src_state_pos = xbm_GetTrSrcStPos(x, tr_pos);
  int loop, prev_tr_pos;
  xbm_transition_type t1;
  xbm_transition_type t2 = xbm_GetTr(x, tr_pos);
  int i;
  int v1s;
  int v1e;
  int v2s;
  int v2e;
  loop = -1;
  prev_tr_pos = -1;

  while( xbm_LoopStInTr(x, src_state_pos, &loop, &prev_tr_pos) != 0 )
  {
    t1 = xbm_GetTr(x, prev_tr_pos);
    for( i = 0; i < x->inputs+x->outputs; i++ )
    {
      v1s = dcGetIn(&(t1->start_cond), i);
      v1e = dcGetIn(&(t1->end_cond), i);

      v2s = dcGetIn(&(t2->start_cond), i);
      v2e = dcGetIn(&(t2->end_cond), i);
      
      if ( dcGetIn(&(t1->level_cond), i) == 0 && dcGetIn(&(t1->edge_cond), i) == 0 )
      {
        assert(v1s == v1e);
        
        if ( v1s == 0 )
        {
          dcSetIn(&(t1->start_cond), i, v2s);
          dcSetIn(&(t1->end_cond), i, v2s);
          *is_changed = 1;
          xbm_Log(x, 1, "XBM bms: Constant transition '%c' -> '%c', variable '%s', states '%s' -> '%s'.",
            "x01-"[v2s], "x01-"[v2s], 
            xbm_GetVarNameStrByIndex(x, i<x->inputs?XBM_DIRECTION_IN:XBM_DIRECTION_OUT, 
                                        i<x->inputs?i:i-x->inputs),
            xbm_GetStNameStr(x, t1->src_state_pos), 
            xbm_GetStNameStr(x, t1->dest_state_pos));
        }
      }
      
      /* directed don't care */
      
      if ( v1s == 3 && v1e == 3 )
      {
        if ( v2e == 2 || v2e == 1 )
        {
          /* last transition ends in a valid value */
          dcSetIn(&(t1->start_cond), i, v2e);
          *is_changed = 1;
          xbm_Log(x, 2, "XBM bms: Directed don't care '%c' -> '-', variable '%s', states '%s' -> '%s'.",
            v2s+'0'-1, xbm_GetVarNameStrByIndex(x, XBM_DIRECTION_IN, i),
            xbm_GetStNameStr(x, t1->src_state_pos), 
            xbm_GetStNameStr(x, t1->dest_state_pos));
        }
        else if ( (v2s == 2 || v2s == 1) && v2e == 3 )
        {
          /* next transition also is a directed don't care */
          dcSetIn(&(t1->start_cond), i, v2s);
          xbm_Log(x, 2, "XBM bms: Directed don't care '%c' -> '-', variable '%s', states '%s' -> '%s'.",
            v2s+'0'-1, xbm_GetVarNameStrByIndex(x, XBM_DIRECTION_IN, i),
            xbm_GetStNameStr(x, t1->src_state_pos), 
            xbm_GetStNameStr(x, t1->dest_state_pos));
          *is_changed = 1;
        }
      }
    }
  }
  return 1;
}

static int xbm_update_prev(xbm_type x)
{
  int my_is_changed;
  int tr_pos;
  int loop_max = b_set_Cnt(x->transitions)*2;
  int loop_cnt = 0;
  
  do
  {
    my_is_changed = 0;
    tr_pos = -1;
    while( xbm_LoopTr(x, &tr_pos) != 0 )
    {
      if ( xbm_update_tr_prev(x, tr_pos, &my_is_changed) == 0 )
        return 0;
    }
    loop_cnt++;
    if ( loop_cnt >= loop_max )
    {
      xbm_Error(x, "XBM: Loop detected.");
      return 0;
    }
  } while( my_is_changed != 0 );
  
  return 1;
}


/*---------------------------------------------------------------------------*/

/*
  start   end    description      
    0      0     constant 0         -->  0  -
    1      1     constant 1         -->  1  -
    0      1     rising edge      
    1      0     falling edge     
    1      -     directed dc down 
    0      -     directed dc up   
*/

#ifdef OLD
static int _old_xbm_apply_ddc(xbm_type x)
{
  int tr_pos;
  int i;
  int s, e;
  xbm_var_type var;
  
  for( i = 0; i < x->inputs; i++ )
  {
    var = xbm_GetVarByIndex(x, XBM_DIRECTION_IN, i);
    assert( var != NULL );
    
    xbm_Log(x, 3, "XBM bms: Variable '%s', edge sensitive: %d, level check: %d.",
      var->name==NULL?"":var->name,
      var->is_edge, var->is_level);
      
    if ( (x->is_auto_ddc_level != 0 && var->is_level != 0 && var->is_edge == 0) ||
         (x->is_auto_ddc_edge != 0  && var->is_level == 0 && var->is_edge != 0) ||
         (x->is_auto_ddc_mix != 0   && var->is_level != 0 && var->is_edge != 0) )
    {
      xbm_Log(x, 3, "XBM bms: Auto apply 'directed don't care' for variable '%s'.",
          var->name==NULL?"":var->name);
          
      tr_pos = -1;
      while( xbm_LoopTr(x, &tr_pos) != 0 )
      {
        if ( dcGetIn(&(xbm_GetTr(x, tr_pos)->level_cond), i) == 0 &&
             dcGetIn(&(xbm_GetTr(x, tr_pos)->edge_cond), i) == 0 )
        {
          s = dcGetIn(&(xbm_GetTr(x, tr_pos)->start_cond), i);
          e = dcGetIn(&(xbm_GetTr(x, tr_pos)->end_cond), i);


          if ( e == s && ( s == 1 || s == 2 ) )
          {
            dcSetIn(&(xbm_GetTr(x, tr_pos)->end_cond), i, 3);
            xbm_Log(x, 1, "XBM bms: Auto apply directed don't care '%c' -> '-', variable '%s', transition '%s' -> '%s'.",
              s+'0'-1, 
              var->name==NULL?"":var->name,
              xbm_GetStNameStr(x, xbm_GetTrSrcStPos(x, tr_pos)), 
              xbm_GetStNameStr(x, xbm_GetTrDestStPos(x, tr_pos)));
          }
        }
      }
    }
  }
  
  return 1;
}
#endif

/*---------------------------------------------------------------------------*/

static int xbm_check_transitions(xbm_type x)
{
  int tr_pos = -1;
  int is_ok = 1;
  int i;
  int s, e;
  int is_edge_signal;
  while( xbm_LoopTr(x, &tr_pos) != 0 )
  {
    if ( dcIsInIllegal(xbm_GetPiCond(x), &(xbm_GetTr(x, tr_pos)->start_cond)) != 0 )
    {
      xbm_Error(x, "XBM bms: Transition '%s' -> '%s', illegal source cube %s",
        xbm_GetStNameStr(x, xbm_GetTrSrcStPos(x, tr_pos)),
        xbm_GetStNameStr(x, xbm_GetTrDestStPos(x, tr_pos)),
        dcToStr(xbm_GetPiCond(x), &(xbm_GetTr(x, tr_pos)->start_cond), "", ""));
      is_ok = 0;
    }
    if ( dcIsInIllegal(xbm_GetPiCond(x), &(xbm_GetTr(x, tr_pos)->end_cond)) != 0 )
    {
      xbm_Error(x, "XBM bms: Transition '%s' -> '%s', illegal destination cube %s",
        xbm_GetStNameStr(x, xbm_GetTrSrcStPos(x, tr_pos)),
        xbm_GetStNameStr(x, xbm_GetTrDestStPos(x, tr_pos)),
        dcToStr(xbm_GetPiCond(x), &(xbm_GetTr(x, tr_pos)->end_cond), "", ""));
      is_ok = 0;
    }
    for( i = x->inputs; i < x->inputs+x->outputs; i++ )
    {
      if ( dcGetIn(&(xbm_GetTr(x, tr_pos)->start_cond), i) == 3 )
      {
        xbm_Error(x, "XBM bms: Transition '%s' -> '%s', ambiguous specification for output-signal '%s'",
          xbm_GetStNameStr(x, xbm_GetTrSrcStPos(x, tr_pos)),
          xbm_GetStNameStr(x, xbm_GetTrDestStPos(x, tr_pos)),
          xbm_GetVarNameStr(x, i)
          );
        is_ok = 0;
      }
    }
    is_edge_signal = 0;
    for( i = 0; i < x->inputs; i++ )
    {
      s = dcGetIn(&(xbm_GetTr(x, tr_pos)->start_cond), i);
      e = dcGetIn(&(xbm_GetTr(x, tr_pos)->end_cond), i);
      if ( s == 3 && 
           e == 3 )
      {
        xbm_Error(x, "XBM bms: Transition '%s' -> '%s', ambiguous specification of directed don't care '%s'",
          xbm_GetStNameStr(x, xbm_GetTrSrcStPos(x, tr_pos)),
          xbm_GetStNameStr(x, xbm_GetTrDestStPos(x, tr_pos)),
          xbm_GetVarNameStr(x, i)
          );
        is_ok = 0;
      }
      if ( s == 1 && e == 2 )
        is_edge_signal = 1;
      if ( s == 2 && e == 1 )
        is_edge_signal = 1;
    }
    if ( is_edge_signal == 0 )
    {
      xbm_Error(x, "XBM bms: Transition '%s' -> '%s' does not contain an edge triggered signal",
        xbm_GetStNameStr(x, xbm_GetTrSrcStPos(x, tr_pos)),
        xbm_GetStNameStr(x, xbm_GetTrDestStPos(x, tr_pos))
        );
      is_ok = 0;
    }
  }
  
  return is_ok;
}

static int xbm_check_in_out_transitions(xbm_type x)
{
  int st_pos;
  int l1, l2;
  int tr1_pos, tr2_pos;
  int is_ok = 1;
  //int i;
  
  st_pos = -1;
  while( xbm_LoopSt(x, &st_pos) != 0 )
  {
    tr1_pos = -1; l1 = -1;
    while( xbm_LoopStInTr(x, st_pos, &l1, &tr1_pos) != 0 )
    {
      tr2_pos = -1; l2 = -1;
      while( xbm_LoopStOutTr(x, st_pos, &l2, &tr2_pos) != 0 )
      {
        if ( dcIsDeltaInNoneZero(xbm_GetPiIn(x), 
                &(xbm_GetTr(x, tr1_pos)->in_end_cond), 
                &(xbm_GetTr(x, tr2_pos)->in_end_cond)) == 0 )   
                /* yes, intersection! */
        {
          is_ok = 0;
          xbm_Error(x, "XBM bms: No stable state for transitions '%s' -> '%s' and '%s' -> '%s'.",
            xbm_GetStNameStr(x, xbm_GetTrSrcStPos(x, tr1_pos)),
            xbm_GetStNameStr(x, xbm_GetTrDestStPos(x, tr1_pos)),
            xbm_GetStNameStr(x, xbm_GetTrSrcStPos(x, tr2_pos)),
            xbm_GetStNameStr(x, xbm_GetTrDestStPos(x, tr2_pos)));
        }
      }
    }
  }
  
  return is_ok;
}

/*---------------------------------------------------------------------------*/
/*
  level start   end    description          start  end   ddc_start ddc_end
    x     0      1     rising edge            0     1        0        1
    x     1      0     falling edge           1     0        1        0
    -     1      -     directed dc down       -     -        1        -
    -     0      -     directed dc up         -     -        0        -
    1     1      1     check 1                1     1        1        -
    0     0      0     check 0                0     0        0        -
*/


static int xbm_in_out_copy(xbm_type x)
{
  int st_pos;
  int tr_pos;
  int loop;
  int is_first;
  int i;
  
  tr_pos = -1;
  while( xbm_LoopTr(x, &tr_pos) != 0 )
  {
    dcInSetAll(xbm_GetPiIn(x), &(xbm_GetTr(x, tr_pos)->in_ddc_start_cond), CUBE_IN_MASK_DC);
    dcCopyInToInRange( xbm_GetPiIn(x), &(xbm_GetTr(x, tr_pos)->in_ddc_start_cond),
       0, xbm_GetPiCond(x), &(xbm_GetTr(x, tr_pos)->start_cond), 0, x->inputs );
       
    dcInSetAll(xbm_GetPiIn(x), &(xbm_GetTr(x, tr_pos)->in_end_cond), CUBE_IN_MASK_DC);
    dcCopyInToInRange( xbm_GetPiIn(x), &(xbm_GetTr(x, tr_pos)->in_end_cond),
       0, xbm_GetPiCond(x), &(xbm_GetTr(x, tr_pos)->end_cond), 0, x->inputs );
       
    dcCopyIn(xbm_GetPiIn(x), &(xbm_GetTr(x, tr_pos)->in_start_cond), 
      &(xbm_GetTr(x, tr_pos)->in_ddc_start_cond));
      
    dcCopyIn(xbm_GetPiIn(x), &(xbm_GetTr(x, tr_pos)->in_ddc_end_cond), 
      &(xbm_GetTr(x, tr_pos)->in_end_cond));


    for( i = 0; i < xbm_GetPiIn(x)->in_cnt; i++ )
    {
      if ( dcGetIn(&(xbm_GetTr(x, tr_pos)->level_cond), i) == 3 )
      {
        dcSetIn(&(xbm_GetTr(x, tr_pos)->in_start_cond), i, 3);
        /*
        dcSetIn(&(xbm_GetTr(x, tr_pos)->in_ddc_end_cond), i, 
          dcGetIn(&(xbm_GetTr(x, tr_pos)->start_cond), i));
        */
      }
      if ( dcGetIn(&(xbm_GetTr(x, tr_pos)->level_cond), i) != 0 )
      {
        dcSetIn(&(xbm_GetTr(x, tr_pos)->in_ddc_end_cond), i, 3);
      }
    }
        
    xbm_Log(x, 0, "XBM bms: '%s' -> '%s'      start %s,     end %s.",
      xbm_GetStNameStr(x, xbm_GetTrSrcStPos(x, tr_pos)),
      xbm_GetStNameStr(x, xbm_GetTrDestStPos(x, tr_pos)),
      dcToStr(xbm_GetPiIn(x), &(xbm_GetTr(x, tr_pos)->in_start_cond), "", ""),
      dcToStr2(xbm_GetPiIn(x), &(xbm_GetTr(x, tr_pos)->in_end_cond), "", ""));
    xbm_Log(x, 0, "XBM bms: '%s' -> '%s'  ddc start %s, ddc end %s.",
      xbm_GetStNameStr(x, xbm_GetTrSrcStPos(x, tr_pos)),
      xbm_GetStNameStr(x, xbm_GetTrDestStPos(x, tr_pos)),
      dcToStr(xbm_GetPiIn(x), &(xbm_GetTr(x, tr_pos)->in_ddc_start_cond), "", ""),
      dcToStr2(xbm_GetPiIn(x), &(xbm_GetTr(x, tr_pos)->in_ddc_end_cond), "", ""));
  }

  st_pos = -1;
  while( xbm_LoopSt(x, &st_pos) != 0 )
  {
    loop = -1;
    tr_pos = -1;
    is_first = 1;
    while ( xbm_LoopStInTr(x, st_pos, &loop, &tr_pos) != 0 )
    {
      if ( is_first != 0 )
      {
        dcOutSetAll(xbm_GetPiOut(x), &(xbm_GetSt(x, st_pos)->out), 0);
        dcCopyInToOutRange(xbm_GetPiOut(x), &(xbm_GetSt(x, st_pos)->out), 0,
          xbm_GetPiCond(x), &(xbm_GetTr(x, tr_pos)->end_cond),
          x->inputs, x->outputs);
        xbm_Log(x, 0, "XBM bms: Output state '%s' is '%s'.",
          xbm_GetStNameStr(x, st_pos),
          dcToStr(xbm_GetPiOut(x), &(xbm_GetSt(x, st_pos)->out), "", ""));
        is_first = 0;
      }
      else
      {
        for( i = 0; i < x->outputs; i++ )
        {
          if (  dcGetOut(&(xbm_GetSt(x, st_pos)->out), i) != 
                dcGetIn(&(xbm_GetTr(x, tr_pos)->end_cond), i+x->inputs)-1 )
          {
            xbm_Error(x, "XBM: Ambiguous output specification for variable '%s'/state '%s'.",
              xbm_GetVarNameStrByIndex(x, XBM_DIRECTION_OUT, i),
              xbm_GetStNameStr(x, st_pos));
            return 0;
          }
        }
      }
    }
  }
  
  return 1;
}

/*---------------------------------------------------------------------------*/

static int xbm_calc_st_in_self_condition(xbm_type x, int st_pos)
{
  int tr1, tr2;
  int l1, l2;
  dcube *c1;
  dcube *c2;
  dcube *cs = &(xbm_GetPiIn(x)->tmp[2]);
  int i;

  dclRealClear(xbm_GetSt(x, st_pos)->in_self_cl);
  
  l1 = -1;
  tr1 = -1;
  while( xbm_LoopStInTr(x, st_pos, &l1, &tr1) != 0 )
  {
    l2 = -1;
    tr2 = -1;  
    c1 = &(xbm_GetTr(x, tr1)->in_end_cond);
    while( xbm_LoopStOutTr(x, st_pos, &l2, &tr2) != 0 )
    {
      c2 = &(xbm_GetTr(x, tr2)->in_start_cond);
      dcOr(xbm_GetPiIn(x), cs, c1, c2);
      if ( dclAdd(xbm_GetPiIn(x), xbm_GetSt(x, st_pos)->in_self_cl, cs) < 0 )
        return 0;
    }
  }

  l2 = -1;
  tr2 = -1;  
  while( xbm_LoopStOutTr(x, st_pos, &l2, &tr2) != 0 )
  {
      c2 = &(xbm_GetTr(x, tr2)->in_end_cond);
      if ( dclSharp(xbm_GetPiIn(x), xbm_GetSt(x, st_pos)->in_self_cl, 
            xbm_GetTrSuper(x, tr2), c2) == 0 )
        return 0;
  }
  
  dclPrimes(xbm_GetPiIn(x), xbm_GetSt(x, st_pos)->in_self_cl);
  
  for( i = 0; i < dclCnt(xbm_GetSt(x, st_pos)->in_self_cl); i++ )
  {
    xbm_Log(x, 0, "XBM bms: Self loop state '%s' %s.", 
      xbm_GetStNameStr(x, st_pos),
      dcToStr(xbm_GetPiIn(x), dclGet(xbm_GetSt(x, st_pos)->in_self_cl, i), "", ""));
  }
  
  return 1;
}

static int xbm_calc_self_condition(xbm_type x)
{
  int st_pos = -1;
  while( xbm_LoopSt(x, &st_pos) != 0 )
    if ( xbm_calc_st_in_self_condition(x, st_pos) == 0 )
      return 0;
  return 1;
}

/*---------------------------------------------------------------------------*/


static void xbm_fill_reset_cube(xbm_type x, dcube *in, dcube *out)
{
  int var_pos;
  xbm_var_type v;
  var_pos = -1;
  dcSetTautology(xbm_GetPiIn(x), in);
  dcSetTautology(xbm_GetPiOut(x), out);
  while( xbm_LoopVar(x, &var_pos) != 0 )
  {
    v = xbm_GetVar(x, var_pos);
    if ( v->direction == XBM_DIRECTION_IN )
    {
      dcSetIn(in, v->index, v->reset_value+1);
    }
    else if ( v->direction == XBM_DIRECTION_OUT )
    {
      dcSetOut(out, v->index, v->reset_value);
    }
  }
  xbm_Log(x, 3, "XBM: Reset value in '%s' out '%s'.", 
    dcToStr(xbm_GetPiIn(x), in, "", ""),dcToStr2(xbm_GetPiOut(x), out, "", ""));
}

static int xbm_is_reset_state(xbm_type x, dcube *in, dcube *out, int st_pos)
{
  xbm_state_type s = xbm_GetSt(x, st_pos);
  if ( dcIsEqualOut(xbm_GetPiOut(x), out, &(s->out)) == 0 )
    return 0;
  if ( dclIsSubSet(xbm_GetPiIn(x), s->in_self_cl, in) == 0 )
    return 0;
  return 1;
}

static int xbm_set_reset_state_pos(xbm_type x)
{
  dcube *in = &(xbm_GetPiIn(x)->tmp[15]);
  dcube *out = &(xbm_GetPiOut(x)->tmp[15]);
  int st_pos;
  
  xbm_fill_reset_cube(x, in, out);
  
  st_pos = -1;
  while( xbm_LoopSt(x, &st_pos) != 0 )
    if ( xbm_is_reset_state(x, in, out, st_pos) != 0 )
    {
      xbm_Log(x, 4, "XBM: Reset state '%s'.", xbm_GetStNameStr(x, st_pos));
      x->reset_st_pos = st_pos;
      return 1;
    }
  
  xbm_Error(x, "XBM: No reset state found.");
  return 0;
}

/*---------------------------------------------------------------------------*/

static int xbm_post_processing(xbm_type x)
{
  if ( xbm_check_variables(x) == 0 )
    return 0;
  if ( xbm_apply_ddc(x) == 0 )
    return 0;
  if ( xbm_copy_level_edge_to_start_end(x) == 0 )
    return 0;
  if ( xbm_update_next(x) == 0 )
    return 0;
  if ( xbm_update_prev(x) == 0 )
    return 0;
/*
  if ( _old_xbm_apply_ddc(x) == 0 )
    return 0;
*/
  if ( xbm_check_transitions(x) == 0 )
    return 0;
  if ( xbm_in_out_copy(x) == 0 )
    return 0;
  if ( xbm_calc_self_condition(x) == 0 )
    return 0;
  if ( xbm_check_in_out_transitions(x) == 0 )
    return 0;
  if ( xbm_set_reset_state_pos(x) == 0 )
    return 0;
    
  return 1;
}


/*---------------------------------------------------------------------------*/
/* bms parser */

int xbm_state = 0;

static void xbm_skipspace(char **s) 
{  
  if ( **(unsigned char **)s >= 128 )
    return;
  while(isspace(**(unsigned char **)s))
    (*s)++;
}

#define XBM_ID_LEN 256

static char *xbm_getid(char **s)
{
   static char t[XBM_ID_LEN];
   int i = 0;
   while( (**s >= '0' || **s == '!' || **s == '(' || **s == ')' || **s == '/') && **s != '~' )
   {
      if ( i < 255 )
        t[i] = **s;
      (*s)++;
      i++;
   }
   t[i] = '\0';
   return t;
}

static int xbm_getval_wd(char **s, int *digits)
{
   int result = 0;
   if ( digits != NULL )
     (*digits) = 0;
   while( **s >= '0' && **s <= '9' ) 
   {
      result = result*10+(**s)-'0';
      (*s)++;
      if ( digits != NULL )
        (*digits)++;
   }
   return result;
}

static int xbm_getval(char **s)
{
  return xbm_getval_wd(s, NULL);
}

static double xbm_getfloat(char **s)
{
  double val;
  int digits;
  double nc;
  val = (double)xbm_getval(s);
  if ( **s == '.' )
  {
    (*s)++;
    nc = (double)xbm_getval_wd(s, &digits);
    while(digits > 0)
    {
      nc /= 10.0;
      digits--;
    }
    val += nc;
  }
  return val;
}

int xbm_AddInputVar(xbm_type x, const char *name, int reset_value)
{
  return xbm_AddVar(x, XBM_DIRECTION_IN, name, reset_value);
}

int xbm_AddOutputVar(xbm_type x, const char *name, int reset_value)
{
  return xbm_AddVar(x, XBM_DIRECTION_OUT, name, reset_value);
}

/*
static int xbm_get_input_cube_pos(xbm_type x, const char *name)
{
  int pos;
  pos = xbm_FindVar(x, XBM_DIRECTION_IN, name);
  if ( pos < 0 )
    return -1;
  return xbm_GetVar(x, pos)->index;
}
*/

/*
static int xbm_get_output_cube_pos(xbm_type x, const char *name)
{
  int pos;
  pos = xbm_FindVar(x, XBM_DIRECTION_OUT, name);
  if ( pos < 0 )
    return -1;
  return xbm_GetVar(x, pos)->index;
}
*/

static int xbm_cmp(xbm_type x, char **s, char *key)
{
  size_t l = strlen(key);
  if ( strncasecmp(*s, key, l) == 0 )
  {
    (*s) += l;
    xbm_skipspace(s);
    return 1;
  }
  return 0;
}

static int xbm_signal(xbm_type x, int tr_pos, int var_pos, int is_alt, char level)
{
  int zero_one;
  int is_level;
  int cube_pos;
  dcube *level_cond = &(xbm_GetTr(x, tr_pos)->level_cond);
  dcube *edge_cond = &(xbm_GetTr(x, tr_pos)->edge_cond);

  xbm_GetVar(x, var_pos)->is_used = 1;
  
  if ( x->is_file_check != 0 )
    return 1;
  
  switch(level)
  {
    case XBM_CHR_RISE_EDGE:
    case XBM_CHR_HIGH_LEVEL:
      zero_one = 2;
      break;
    case XBM_CHR_FALL_EDGE:
    case XBM_CHR_LOW_LEVEL:
      zero_one = 1;
      break;
    case XBM_CHR_DONT_CARE:
    default:
      zero_one = 3;
      break;
  }

  is_level = 0;  
  if ( level == XBM_CHR_HIGH_LEVEL || level == XBM_CHR_LOW_LEVEL )
    is_level = 1;
  if ( is_alt != 0 )
    is_level = 1;
    
  if ( is_level != 0 )
  {
    xbm_GetVar(x, var_pos)->is_level = 1;
  }
  
  if ( level == XBM_CHR_RISE_EDGE || level == XBM_CHR_FALL_EDGE )
    if ( is_level == 0 )
      xbm_GetVar(x, var_pos)->is_edge = 1;

  if ( xbm_GetVar(x, var_pos)->direction == XBM_DIRECTION_OUT )
  {
    if ( zero_one == 3 )
    {
      xbm_Error(x, "XBM: Illegal specification for output variable '%s'.",
        xbm_GetVarNameStr(x, var_pos));
      return 0;
    }
    cube_pos = xbm_GetVar(x, var_pos)->index + x->inputs;
    dcSetIn(level_cond, cube_pos, zero_one);
    /* dcSetIn(edge_cond, cube_pos, zero_one); */
  }
  else if ( xbm_GetVar(x, var_pos)->direction == XBM_DIRECTION_IN )
  {
    cube_pos = xbm_GetVar(x, var_pos)->index;
    assert( cube_pos >= 0 );
    if ( zero_one == 3 )
    {
      dcSetIn(level_cond, cube_pos, zero_one);
      /* dcSetIn(edge_cond, cube_pos, zero_one); */
    }
    else
    {
      if ( is_level != 0 )
      {
        dcSetIn(level_cond, cube_pos, zero_one);
      }
      else
      {
        dcSetIn(edge_cond, cube_pos, zero_one);
      }
    }
  }
  return 1;
}

static int xbm_from_to_line(xbm_type x, char *s)
{
  char *t;
  char from[XBM_ID_LEN];
  char to[XBM_ID_LEN];
  int tr_pos = -1;
  int var_pos = -1;
  int is_alt;
  int direction = XBM_DIRECTION_IN;

  strcpy(from, xbm_getid(&s));
  xbm_skipspace(&s);
  strcpy(to, xbm_getid(&s));
  xbm_skipspace(&s);
  
  if ( from[0] == '\0' )
    return xbm_Error(x, "XBM bms: empty name of source state."), 0;
  if ( to[0] == '\0' )
    return xbm_Error(x, "XBM bms: empty name of target state (source state '%s').", from), 0;

  if ( x->is_file_check == 0 )
  {  
    tr_pos = xbm_FindTrByName(x, from, to);
    if ( tr_pos < 0 )
    {
      tr_pos = xbm_ConnectByName(x, from, to, 1);
      if ( tr_pos < 0 )
        return 0;

    xbm_Log(x, 0, "XBM bms: Connect '%s' -> '%s'.",
        xbm_GetStNameStr(x, xbm_GetTrSrcStPos(x, tr_pos)),
        xbm_GetStNameStr(x, xbm_GetTrDestStPos(x, tr_pos)));
    }
  }

  for(;;)
  {
    xbm_skipspace(&s);
    if ( *s == '\0' )
      break;
      
    if ( *s == '|' )
    {
      direction = XBM_DIRECTION_OUT;
      s++;
      continue;
    }
    
    is_alt = 0;
    if ( *s == '[' )
    {
      s++;
      is_alt = 1;
    }
      
    t = xbm_getid(&s);
    var_pos = xbm_FindVar(x, direction, t);
    if ( var_pos < 0 )
    {
      xbm_Error(x, "XBM: Variable '%s' (%s) not found.", t, 
        direction==XBM_DIRECTION_OUT?"output":"input");
      return 0;
    }
    
    switch(*s)
    {
      case XBM_CHR_RISE_EDGE:
      case XBM_CHR_FALL_EDGE:
      case XBM_CHR_DONT_CARE:
      case XBM_CHR_HIGH_LEVEL:
      case XBM_CHR_LOW_LEVEL:
        if ( xbm_signal(x, tr_pos, var_pos, is_alt, *s) == 0 )
          return 0;
        s++;
        break;
      default:
        xbm_Error(x, "XBM: Illegal level specification for variable '%s' .", t);
        return 0;
    }
    
    if ( is_alt != 0 )
    {
      if ( *s != ']' )
      {
        xbm_Error(x, "XBM: Expected ']' after variable '%s'.", t);
        return 0;
      }
      s++;
    }
    
    
    
  }
  
  return 1;
}

static int xbm_line(xbm_type x, char *s)
{
  xbm_skipspace(&s);
  if ( *s == '\0' )
    return 1;
  
  if ( *s == '#' )
    return 1;

  if ( *s == ';' )
    return 1;

  if ( xbm_state == 0 )
  {
    if ( xbm_cmp(x, &s, "name") != 0 )
    {
      return xbm_SetName(x, xbm_getid(&s));
    }
    else if ( xbm_cmp(x, &s, "input") != 0 )
    {
      char *t;
      int v = 0;
      t = xbm_getid(&s);
      xbm_skipspace(&s);
      if ( *s != '\0' )
        v = xbm_getval(&s);
      if ( xbm_AddInputVar(x, t, v) < 0 )
        return 0;
      return 1;
    }
    else if ( xbm_cmp(x, &s, "output") != 0 )
    {
      char *t;
      int v = 0;
      t = xbm_getid(&s);
      xbm_skipspace(&s);
      if ( *s != '\0' )
        v = xbm_getval(&s);
      if ( xbm_AddOutputVar(x, t, v) < 0 )
        return 0;
      return 1;
    }
    else if ( xbm_cmp(x, &s, "feedback") != 0 )
    {
      char *t;
      int v = 0;
      double delay = 0.0;
      t = xbm_getid(&s);
      xbm_skipspace(&s);
      if ( *s != '\0' )
        v = xbm_getval(&s);
      xbm_skipspace(&s);
      if ( *s != '\0' )
        delay = xbm_getfloat(&s);
      if ( xbm_AddDelayLineVar(x, XBM_DIRECTION_IN, t, v, delay) < 0 )
        return 0;
      if ( xbm_AddDelayLineVar(x, XBM_DIRECTION_OUT, t, v, delay) < 0 )
        return 0;
      xbm_Log(x, 2, "XBM bms: Adding delay line variable '%s' with %dns delay.", t, delay);
      return 1;
    }
    else if ( xbm_cmp(x, &s, "reset") != 0 )
    {
      return xbm_SetResetStateName(x, xbm_getid(&s));
    }
    
    if ( x->is_file_check == 0 )
      if ( xbm_CalcPI(x) == 0 )
        return 0;

    xbm_state = 1;
  }  
  return xbm_from_to_line(x, s);
}

#define XBM_LINE_LEN 1024

static int xbm_read(xbm_type x, FILE *fp)
{
  char line[XBM_LINE_LEN];
  char *s;
  
  xbm_Clear(x);

  xbm_state = 0;
  
  /* read signal transitions into the tmp_cube */
  /* this will also allocate memory for the tmp_cubes */
  
  for(;;)
  {
    s = fgets(line, XBM_LINE_LEN, fp);
    if ( s == NULL )
      break;
    if ( xbm_line(x, s) == 0 )
      return 0;
  }

  if ( x->is_file_check == 0 )
    if ( xbm_post_processing(x) == 0 )
      return 0;
  
  return 1;
}



int xbm_ReadBMS(xbm_type x, const char *name)
{
  int ret = 0;
  FILE *fp;

  xbm_Log(x, 5, "XBM: Read file '%s'.", name);
  
  fp = b_fopen(name, NULL, ".bms", "r");
  if ( fp != NULL )
  {
    ret = xbm_read(x, fp);
    fclose(fp);
  }
  else
  {
    xbm_Error(x, "XBM: Can not read file '%s' (wrong parh or name?).", name);
  }
  
  if ( ret == 0 )
  {
    xbm_Error(x, "XBM: Import error with XBM file '%s'.", name);
  }
  else
  {
    xbm_Log(x, 5, "XBM: File '%s' successfully read (states: %d, inputs: %d, outputs: %d).",
      name, b_set_Cnt(x->states), x->inputs, x->outputs);
  }
  return ret;
}

int IsValidXBMFile(const char *name)
{
  xbm_type x;
  int ret = 0;
  x = xbm_Open();
  if ( x != NULL )
  {
    x->is_file_check = 1;
    ret = xbm_ReadBMS(x, name);
    xbm_Close(x);
  }
  return ret;
}

