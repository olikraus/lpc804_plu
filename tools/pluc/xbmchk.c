/*

  xbmchk.c
  
  check cl_machine of an xbm structure

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
#include <stdio.h>
#include <assert.h>
#include "xbm.h"
#include "dcubehf.h"

int xbm_CheckStrongStateStateTransfers(xbm_type x)
{
  int tr_pos;
  dcube *s = &(xbm_GetPiMachine(x)->tmp[3]);
  dcube *e = &(xbm_GetPiMachine(x)->tmp[4]);
  int in_cnt = x->inputs;
  int out_cnt = x->outputs;
  int code_cnt = xbm_GetPiCode(x)->out_cnt;
  int st_src_pos, st_dest_pos;
  
  assert(in_cnt == xbm_GetPiIn(x)->in_cnt);
  
  tr_pos = -1;
  while( xbm_LoopTr(x, &tr_pos) != 0 )
  {
    st_src_pos = xbm_GetTrSrcStPos(x, tr_pos);
    st_dest_pos = xbm_GetTrDestStPos(x, tr_pos);
    
    dcCopyInToIn( xbm_GetPiMachine(x), s, 0, 
      xbm_GetPiIn(x), &(xbm_GetTr(x, tr_pos)->in_start_cond));
    dcCopyOutToIn( xbm_GetPiMachine(x), s, in_cnt, 
      xbm_GetPiCode(x), &(xbm_GetSt(x, st_src_pos)->code));
    if ( x->is_fbo != 0 )
      dcCopyOutToIn( xbm_GetPiMachine(x), s, in_cnt+code_cnt, 
        xbm_GetPiOut(x), &(xbm_GetSt(x, st_src_pos)->out));
    dcCopyOutToOut( xbm_GetPiMachine(x), s, 0, 
      xbm_GetPiCode(x), &(xbm_GetSt(x, st_src_pos)->code));
    dcCopyOutToOut( xbm_GetPiMachine(x), s, code_cnt, 
      xbm_GetPiOut(x), &(xbm_GetSt(x, st_src_pos)->out));

    dcCopyInToIn( xbm_GetPiMachine(x), e, 0, 
      xbm_GetPiIn(x), &(xbm_GetTr(x, tr_pos)->in_end_cond));
    dcCopyOutToIn( xbm_GetPiMachine(x), e, in_cnt, 
      xbm_GetPiCode(x), &(xbm_GetSt(x, st_src_pos)->code));
    if ( x->is_fbo != 0 )
      dcCopyOutToIn( xbm_GetPiMachine(x), e, in_cnt+code_cnt, 
        xbm_GetPiOut(x), &(xbm_GetSt(x, st_src_pos)->out));
    dcCopyOutToOut( xbm_GetPiMachine(x), e, 0, 
      xbm_GetPiCode(x), &(xbm_GetSt(x, st_dest_pos)->code));
    dcCopyOutToOut( xbm_GetPiMachine(x), e, code_cnt, 
      xbm_GetPiOut(x), &(xbm_GetSt(x, st_dest_pos)->out));

    xbm_Log(x, 0, "XBM:         input change  %s -> %s.", 
      dcToStr(xbm_GetPiMachine(x), s, " ", ""), 
      dcToStr2(xbm_GetPiMachine(x), e, " ", ""));
    if ( dclIsHazardfreeTransition(xbm_GetPiMachine(x), x->cl_machine, s, e) == 0 )
    {
      xbm_Log(x, 3, "XBM warning: input change  %s -> %s (state '%s' -> '%s') might contain a hazard with a non-directed don't care.", 
        dcToStr(xbm_GetPiMachine(x), s, " ", ""), 
        dcToStr2(xbm_GetPiMachine(x), e, " ", ""),
        xbm_GetStNameStr(x, st_src_pos),xbm_GetStNameStr(x, st_dest_pos));
    }

    dcCopyInToIn( xbm_GetPiMachine(x), s, 0, 
      xbm_GetPiIn(x), &(xbm_GetTr(x, tr_pos)->in_end_cond));
    dcCopyOutToIn( xbm_GetPiMachine(x), s, in_cnt, 
      xbm_GetPiCode(x), &(xbm_GetSt(x, st_src_pos)->code));
    if ( x->is_fbo != 0 )
      dcCopyOutToIn( xbm_GetPiMachine(x), s, in_cnt+code_cnt, 
        xbm_GetPiOut(x), &(xbm_GetSt(x, st_src_pos)->out));
    dcCopyOutToOut( xbm_GetPiMachine(x), s, 0, 
      xbm_GetPiCode(x), &(xbm_GetSt(x, st_dest_pos)->code));
    dcCopyOutToOut( xbm_GetPiMachine(x), s, code_cnt, 
      xbm_GetPiOut(x), &(xbm_GetSt(x, st_dest_pos)->out));

    dcCopyInToIn( xbm_GetPiMachine(x), e, 0, 
      xbm_GetPiIn(x), &(xbm_GetTr(x, tr_pos)->in_end_cond));
    dcCopyOutToIn( xbm_GetPiMachine(x), e, in_cnt, 
      xbm_GetPiCode(x), &(xbm_GetSt(x, st_dest_pos)->code));
    if ( x->is_fbo != 0 )
      dcCopyOutToIn( xbm_GetPiMachine(x), e, in_cnt+code_cnt, 
        xbm_GetPiOut(x), &(xbm_GetSt(x, st_dest_pos)->out));
    dcCopyOutToOut( xbm_GetPiMachine(x), e, 0, 
      xbm_GetPiCode(x), &(xbm_GetSt(x, st_dest_pos)->code));
    dcCopyOutToOut( xbm_GetPiMachine(x), e, code_cnt, 
      xbm_GetPiOut(x), &(xbm_GetSt(x, st_dest_pos)->out));
    
    xbm_Log(x, 0, "XBM:         code change   %s -> %s.", 
      dcToStr(xbm_GetPiMachine(x), s, " ", ""), 
      dcToStr2(xbm_GetPiMachine(x), e, " ", ""));
    if ( dclIsHazardfreeTransition(xbm_GetPiMachine(x), x->cl_machine, s, e) == 0 )
    {
      xbm_Log(x, 3, "XBM warning: code change   %s -> %s (state '%s' -> '%s') might contain a hazard with a non-directed don't care.", 
        dcToStr(xbm_GetPiMachine(x), s, " ", ""), 
        dcToStr2(xbm_GetPiMachine(x), e, " ", ""),
        xbm_GetStNameStr(x, st_src_pos),xbm_GetStNameStr(x, st_dest_pos));
      {
        int i;
        for( i = 0; i < xbm_GetPiMachine(x)->out_cnt; i++ )
          if ( dclIsHazardfreeFunction(xbm_GetPiMachine(x), x->cl_machine, s, e, i) == 0 )
          {
            xbm_Log(x, 3, "XBM warning: hazard on output %d.", i);
          }
      }
    }
  }
  return 1;
}

int xbm_CheckStrongStateSelfTransfers(xbm_type x)
{
  int st_pos;
  int tr1, tr2;
  int l1, l2;
  dcube *s = &(xbm_GetPiMachine(x)->tmp[3]);
  dcube *e = &(xbm_GetPiMachine(x)->tmp[4]);
  int in_cnt = x->inputs;
  int out_cnt = x->outputs;
  int code_cnt = xbm_GetPiCode(x)->out_cnt;
  
  st_pos = -1;
  while( xbm_LoopSt(x, &st_pos) != 0 )
  {
    l1 = -1;
    while( xbm_LoopStInTr(x, st_pos, &l1, &tr1) != 0 )
    {

      dcCopyInToIn( xbm_GetPiMachine(x), s, 0, 
        xbm_GetPiIn(x), &(xbm_GetTr(x, tr1)->in_end_cond));
      dcCopyOutToIn( xbm_GetPiMachine(x), s, in_cnt, 
        xbm_GetPiCode(x), &(xbm_GetSt(x, st_pos)->code));
      if ( x->is_fbo != 0 )
        dcCopyOutToIn( xbm_GetPiMachine(x), s, in_cnt+code_cnt, 
          xbm_GetPiOut(x), &(xbm_GetSt(x, st_pos)->out));
      dcCopyOutToOut( xbm_GetPiMachine(x), s, 0, 
        xbm_GetPiCode(x), &(xbm_GetSt(x, st_pos)->code));
      dcCopyOutToOut( xbm_GetPiMachine(x), s, code_cnt, 
        xbm_GetPiOut(x), &(xbm_GetSt(x, st_pos)->out));

      l2 = -1;
      while( xbm_LoopStOutTr(x, st_pos, &l2, &tr2) != 0 )
      {
        
        dcCopyInToIn( xbm_GetPiMachine(x), e, 0, 
          xbm_GetPiIn(x), &(xbm_GetTr(x, tr2)->in_start_cond));
        dcCopyOutToIn( xbm_GetPiMachine(x), e, in_cnt, 
          xbm_GetPiCode(x), &(xbm_GetSt(x, st_pos)->code));
        if ( x->is_fbo != 0 )
          dcCopyOutToIn( xbm_GetPiMachine(x), e, in_cnt+code_cnt, 
            xbm_GetPiOut(x), &(xbm_GetSt(x, st_pos)->out));
        dcCopyOutToOut( xbm_GetPiMachine(x), e, 0, 
          xbm_GetPiCode(x), &(xbm_GetSt(x, st_pos)->code));
        dcCopyOutToOut( xbm_GetPiMachine(x), e, code_cnt, 
          xbm_GetPiOut(x), &(xbm_GetSt(x, st_pos)->out));
        
        xbm_Log(x, 0, "XBM:         self loop     %s -> %s.", 
          dcToStr(xbm_GetPiMachine(x), s, " ", ""), 
          dcToStr2(xbm_GetPiMachine(x), e, " ", ""));
        if ( dclIsHazardfreeTransition(xbm_GetPiMachine(x), x->cl_machine, s, e) == 0 )
        {
          xbm_Log(x, 4, "XBM warning: self loop     %s -> %s (state '%s') might contain a hazard.", 
            dcToStr(xbm_GetPiMachine(x), s, " ", ""), 
            dcToStr2(xbm_GetPiMachine(x), e, " ", ""),
            xbm_GetStNameStr(x, st_pos));
        }        
      }
    }
  }
  return 1;
}

/*---------------------------------------------------------------------------*/

static int check_cb(void *data, dcube *s, dcube *e)
{
  xbm_type x = (xbm_type)data;
  if ( dclIsHazardfreeTransition(xbm_GetPiMachine(x), x->cl_machine, s, e) == 0 )
  {
    xbm_Error(x, "XBM: Transition %s -> %s contains hazard.", 
      dcToStr(xbm_GetPiMachine(x), s, " ", ""), 
      dcToStr(xbm_GetPiMachine(x), s, " ", ""));
    return 0;
  }
  return 1;
}

int xbm_CheckStateStateTransfers(xbm_type x)
{
  /* xbmfn.c */
  return xbm_DoStateStateTransfers(x, check_cb, x);
}

int xbm_CheckStateSelfTransfers(xbm_type x)
{
  /* xbmfn.c */
  return xbm_DoStateSelfTransfers(x, check_cb, x);
}

/*---------------------------------------------------------------------------*/

int xbm_CheckStaticTransferValues(xbm_type x)
{
  int tr_pos;
  int st_src_pos, st_dest_pos;
  dcube *c = &(xbm_GetPiMachine(x)->tmp[3]);
  dclist cl;
  int i, cnt;
  
  int in_cnt = x->inputs;
  int out_cnt = x->outputs;
  int code_cnt = xbm_GetPiCode(x)->out_cnt;

  if ( dclInit(&cl) == 0 )
    return 0;
  
  tr_pos = -1;
  while( xbm_LoopTr(x, &tr_pos) != 0 )
  {
    st_src_pos = xbm_GetTrSrcStPos(x, tr_pos);
    st_dest_pos = xbm_GetTrDestStPos(x, tr_pos);
    
    dcCopyInToIn( xbm_GetPiMachine(x), c, 0, 
      xbm_GetPiIn(x), &(xbm_GetTr(x, tr_pos)->in_end_cond));
    dcCopyOutToIn( xbm_GetPiMachine(x), c, in_cnt, 
      xbm_GetPiCode(x), &(xbm_GetSt(x, st_src_pos)->code));
    if ( x->is_fbo != 0 )
      dcCopyOutToIn( xbm_GetPiMachine(x), c, in_cnt+code_cnt, 
        xbm_GetPiOut(x), &(xbm_GetSt(x, st_src_pos)->out));
        
    dcOutSetAll(xbm_GetPiMachine(x), c, 0);
    dclClear(cl);
    if ( dclAdd(xbm_GetPiMachine(x), cl, c) < 0 )
      return dclDestroy(cl), 0;
    
    dclResultList(xbm_GetPiMachine(x), cl, x->cl_machine);

    cnt = dclCnt(cl);
    for( i = 0; i < cnt; i++ )
    {
      if ( dcIsEqualOutRange(dclGet(cl, i), 0, 
                           &(xbm_GetSt(x, st_dest_pos)->code), 0, 
                           code_cnt) == 0 )
      {
        xbm_Error(x, "XBM: Illegal destination code '%s'->'%s' %d/%d %s (%s) expected %s.",
          xbm_GetStNameStr(x, st_src_pos),
          xbm_GetStNameStr(x, st_dest_pos),
          i+1, cnt,
          dcToStr(xbm_GetPiMachine(x), dclGet(cl, i), " ", ""),
          dcToStr2(xbm_GetPiMachine(x), c, " ", ""),
          dcToStr3(xbm_GetPiCode(x), &(xbm_GetSt(x, st_dest_pos)->code), "", ""));
        return dclDestroy(cl), 0;
      }
      
      if ( dcIsEqualOutRange(dclGet(cl, i), code_cnt, 
                           &(xbm_GetSt(x, st_dest_pos)->out), 0, 
                           out_cnt) == 0 )
      {
        xbm_Error(x, "XBM: Illegal output code '%s'->'%s' %d/%d %s (%s) expected %s.",
          xbm_GetStNameStr(x, st_src_pos),
          xbm_GetStNameStr(x, st_dest_pos),
          i+1, cnt,
          dcToStr(xbm_GetPiMachine(x), dclGet(cl, i), " ", ""),
          dcToStr2(xbm_GetPiMachine(x), c, " ", ""),
          dcToStr3(xbm_GetPiOut(x), &(xbm_GetSt(x, st_dest_pos)->out), "", ""));
        return dclDestroy(cl), 0;
      }
    }
  }
  
  return dclDestroy(cl), 1;
}



/*---------------------------------------------------------------------------*/

int xbm_CheckTransferFunction(xbm_type x)
{ 
  if ( x->cl_machine == NULL || x->pi_machine == NULL )
    return 0;
    
  if ( x->is_sync == 0 )
  {
    /* asynchronous checks */
    
    xbm_Log(x, 2, "XBM: State-state transfers (chk).");
    if ( xbm_CheckStateStateTransfers(x) == 0 )
      return 0;

    xbm_Log(x, 2, "XBM: Self-state transfers (chk).");
    if ( xbm_CheckStateSelfTransfers(x) == 0 )
      return 0;

    xbm_Log(x, 2, "XBM: State-state transfers (strong chk).");
    if ( xbm_CheckStrongStateStateTransfers(x) == 0 )
      return 0;

    xbm_Log(x, 2, "XBM: Self-state transfers (strong chk).");
    if ( xbm_CheckStrongStateSelfTransfers(x) == 0 )
      return 0;
  }

  /* synchronous + asynchronous checks */

  xbm_Log(x, 2, "XBM: Static transfers (chk).");
  if( xbm_CheckStaticTransferValues(x) == 0 )
    return 0;
    
  xbm_Log(x, 4, "XBM: Transfer function is valid.");
  return 1;
}

