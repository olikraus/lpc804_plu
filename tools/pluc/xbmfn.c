/*

  xbmfn.c
  
  build boolean function from an encoded xbm specification

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
#include <stdarg.h>
#include <assert.h>
#include "xbm.h"
#include "dcubehf.h"

/*
  directed don't care     transitions   
   s      e               ds     de
  '1' -> '-'              '1' -> '-'
                          '0' -> '0'

  '0' -> '-'              '0' -> '-'
                          '1' -> '1'
*/

static int next_value(pinfo *pi, dcube *ds, dcube *de, dcube *s, dcube *e)
{
  int i = 0;
  for(;;)
  {
    if ( dcGetIn(e, i) == 3 )   /* only directed don't cares */
    {
      if ( dcGetIn(de, i) == 3 )
      {
        dcSetIn(de, i, 3-dcGetIn(s, i));
        dcSetIn(ds, i, 3-dcGetIn(s, i));
        break;
      }
      else
      {
        dcSetIn(de, i, 3);
        dcSetIn(ds, i, dcGetIn(s, i));
        i++;
      }
    }
    else
    {
      i++;
    }
    if ( i >= pi->in_cnt )
      return 0;
  }
  return 1;
}

/*
int xbm_prepare_dc_cnt(xbm_type x, dcube *cnt, dcube *mask, dcube *s, dcube *e)
{
  int i;
  int sv, ev;
  dcSetOutTautology(x->pi_machine, mask);
  dcSetOutTautology(x->pi_machine, cnt);
  for( i = 0; i < x->pi_machine->in_cnt; i++ )
  { 
    sv = dcGetIn(s, i);
    ev = dcGetIn(e, i);
    dcSetIn(cnt, i, 3);
    dcSetIn(mask, i, 0);
    if ( sv == 3 || ev == 3 )
    {
      if ( sv != 3 || ev != 3 )
      {
        xbm_Error(x, "XBM fn: DC mismatch");
        return 0;
      }
      dcSetIn(cnt, i, 1);
      dcSetIn(mask, i, 3);
    }
  }
  return 1;
}
*/

void xbm_DestroyMachine(xbm_type x)
{
  if ( x->pi_machine != NULL )
    pinfoClose(x->pi_machine);
  x->pi_machine = NULL;
  if ( x->cl_machine != NULL )
    dclDestroy(x->cl_machine);
  x->cl_machine = NULL;
}

int xbm_InitMachine(xbm_type x)
{
  int in, out;
  
  if ( x->is_fbo == 0 )
    in = x->inputs + xbm_GetPiCode(x)->out_cnt;
  else
    in = x->inputs + xbm_GetPiCode(x)->out_cnt + x->outputs;
  out = xbm_GetPiCode(x)->out_cnt + x->outputs;
  
  xbm_DestroyAsync(x);
  x->pi_machine = pinfoOpenInOut(in, out);
  if ( x->pi_machine == NULL )
    return 0;
  /* pinfoInitProgress(x->pi_machine); */
  if ( dclInit(&(x->cl_machine)) == 0 )
    return 0;
  return 1;
}

/*
  directed don't care     transitions   
  '1' -> '-'              '1' -> '-'
                          '0' -> '0'

  '0' -> '-'              '0' -> '-'
                          '1' -> '1'
*/


int xbm_ImportTransferFunction(xbm_type x, const char *name)
{
  pinfo pi; 
  dclist cl;
  if ( pinfoInit(&pi) == 0 )
    return 0;
  if ( dclInit(&cl) == 0 )
    return pinfoDestroy(&pi), 0;
  if ( dclImport(&pi, cl, NULL, name) == 0 )
  {
    xbm_Error(x, "XBM: Can not import transfer function '%s'.", name);
    return dclDestroy(cl), pinfoDestroy(&pi), 0;
  }
  if ( x->pi_machine->in_cnt != pi.in_cnt || 
       x->pi_machine->out_cnt != pi.out_cnt )
  {
    xbm_Error(x, "XBM: Mismatch of imported transfer function '%s'.", name);
    return dclDestroy(cl), pinfoDestroy(&pi), 0;
  }
  if ( dclCopy(x->pi_machine, x->cl_machine, cl) == 0 )
    return 0;
    
  xbm_Log(x, 4, "XBM: Import of transfer function '%s' done.", name);
  return dclDestroy(cl), pinfoDestroy(&pi), 1;
  
}

/*---------------------------------------------------------------------------*/

int xbm_DoStateStateTransfers(xbm_type x, int (*fn)(void *data, dcube *s, dcube *e), void *data)
{
  int tr_pos;
  dcube *s = &(xbm_GetPiMachine(x)->tmp[3]);
  dcube *e = &(xbm_GetPiMachine(x)->tmp[4]);
  dcube *ds = &(xbm_GetPiMachine(x)->tmp[11]);
  dcube *de = &(xbm_GetPiMachine(x)->tmp[14]);
  dcube *end;
  //dclist in_self_cl;
  int in_cnt = x->inputs;
  //int out_cnt = x->outputs;
  int code_cnt = xbm_GetPiCode(x)->out_cnt;
  int st_src_pos, st_dest_pos;
  //int i;
  
  assert(in_cnt == xbm_GetPiIn(x)->in_cnt);
  
  tr_pos = -1;
  while( xbm_LoopTr(x, &tr_pos) != 0 )
  {
    st_src_pos = xbm_GetTrSrcStPos(x, tr_pos);
    st_dest_pos = xbm_GetTrDestStPos(x, tr_pos);

    xbm_Log(x, 1, "XBM: transition %s -> %s.", 
      xbm_GetStNameStr(x, st_src_pos), xbm_GetStNameStr(x, st_dest_pos));
    
    dcCopyInToIn( xbm_GetPiMachine(x), s, 0, 
      xbm_GetPiIn(x), &(xbm_GetTr(x, tr_pos)->in_ddc_start_cond));
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

    dcCopy(xbm_GetPiMachine(x), ds, s);
    dcCopy(xbm_GetPiMachine(x), de, e);
    do
    {
      xbm_Log(x, 1, "XBM: input change  %s -> %s.", 
        dcToStr(xbm_GetPiMachine(x), ds, " ", ""), 
        dcToStr2(xbm_GetPiMachine(x), de, " ", ""));
      if ( fn(data, ds, de) == 0 )
        return 0;
    } while(next_value(xbm_GetPiMachine(x), ds, de, s, e) != 0);


    /* end = &(xbm_GetTr(x, tr_pos)->in_ddc_end_cond); */
    end = &(xbm_GetTr(x, tr_pos)->in_end_cond);
    
    /*
    in_self_cl = xbm_GetSt(x, st_dest_pos)->in_self_cl;
    for( i = 0; i < dclCnt(in_self_cl); i++ )
      if ( dcIsInSubSet(xbm_GetPiMachine(x), dclGet(in_self_cl, i), end) != 0 )
        end = dclGet(in_self_cl, i);
    */

    dcCopyInToIn( xbm_GetPiMachine(x), s, 0, 
      xbm_GetPiIn(x), end);
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
      xbm_GetPiIn(x), end);
    dcCopyOutToIn( xbm_GetPiMachine(x), e, in_cnt, 
      xbm_GetPiCode(x), &(xbm_GetSt(x, st_dest_pos)->code));
    if ( x->is_fbo != 0 )
      dcCopyOutToIn( xbm_GetPiMachine(x), e, in_cnt+code_cnt, 
        xbm_GetPiOut(x), &(xbm_GetSt(x, st_dest_pos)->out));
    dcCopyOutToOut( xbm_GetPiMachine(x), e, 0, 
      xbm_GetPiCode(x), &(xbm_GetSt(x, st_dest_pos)->code));
    dcCopyOutToOut( xbm_GetPiMachine(x), e, code_cnt, 
      xbm_GetPiOut(x), &(xbm_GetSt(x, st_dest_pos)->out));

    xbm_Log(x, 1, "XBM: code change   %s -> %s.", 
      dcToStr(xbm_GetPiMachine(x), s, " ", ""), 
      dcToStr2(xbm_GetPiMachine(x), e, " ", ""));
    if ( fn(data, s, e) == 0 )
      return 0;

    /*    
    dcCopy(xbm_GetPiMachine(x), ds, s);
    dcCopy(xbm_GetPiMachine(x), de, e);
    do
    {
      xbm_Log(x, 1, "XBM: code change   %s -> %s.", 
        dcToStr(xbm_GetPiMachine(x), ds, " ", ""), 
        dcToStr2(xbm_GetPiMachine(x), de, " ", ""));
      if ( fn(data, ds, de) == 0 )
        return 0;
    } while(next_value(xbm_GetPiMachine(x), ds, de, s, e) != 0);
    */
  }
  return 1;
}

int xbm_DoStateSelfTransfers(xbm_type x, int (*fn)(void *data, dcube *s, dcube *e), void *data)
{
  int st_pos;
  int tr1, tr2;
  int l1, l2;
  dcube *s = &(xbm_GetPiMachine(x)->tmp[3]);
  dcube *e = &(xbm_GetPiMachine(x)->tmp[4]);
  //dcube *cnt = &(xbm_GetPiMachine(x)->tmp[11]);
  //dcube *mask = &(xbm_GetPiMachine(x)->tmp[14]);
  int in_cnt = x->inputs;
  //int out_cnt = x->outputs;
  int code_cnt = xbm_GetPiCode(x)->out_cnt;
  
  st_pos = -1;
  while( xbm_LoopSt(x, &st_pos) != 0 )
  {
    l1 = -1;
    while( xbm_LoopStInTr(x, st_pos, &l1, &tr1) != 0 )
    {
    xbm_Log(x, 1, "XBM: state %s.", 
      xbm_GetStNameStr(x, st_pos));

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
        
        xbm_Log(x, 1, "XBM: self loop     %s -> %s.", 
          dcToStr(xbm_GetPiMachine(x), s, " ", ""), 
          dcToStr2(xbm_GetPiMachine(x), e, " ", ""));
        if ( fn(data, s, e) == 0 )
          return 0;
        
      }
    }
  }
  return 1;
}

static int build_cb(void *data, dcube *s, dcube *e)
{
  hfp_type hfp = (hfp_type)data;
  return hfp_AddFromToTransition(hfp, s, e);
}

int xbm_BuildStateStateTransfers(xbm_type x, hfp_type hfp)
{
  return xbm_DoStateStateTransfers(x, build_cb, hfp);
}

int xbm_BuildStateSelfTransfers(xbm_type x, hfp_type hfp)
{
  return xbm_DoStateSelfTransfers(x, build_cb, hfp);
}

static void xbm_hfp_log_cb(void *data, const char *fmt, va_list va)
{
  xbm_type x = (xbm_type)data;
  xbm_LogVA(x, 0, fmt, va);
}

static void xbm_hfp_err_cb(void *data, const char *fmt, va_list va)
{
  xbm_type x = (xbm_type)data;
  xbm_ErrorVA(x, fmt, va);
}


int xbm_BuildAsynchronousTransferFunction(xbm_type x)
{
  hfp_type hfp;

  if ( xbm_InitMachine(x) == 0 )
    return 0;


  /* file import */

  if ( x->import_file_pla_machine != NULL )
    return xbm_ImportTransferFunction(x, x->import_file_pla_machine);


  /* calculate transfer function */

  hfp = hfp_Open(x->pi_machine->in_cnt, x->pi_machine->out_cnt);
  if ( hfp == NULL )
    return 0;

  hfp_SetLogCB(hfp, xbm_hfp_log_cb, (void *)x);
  hfp_SetErrorCB(hfp, xbm_hfp_err_cb, (void *)x);

  xbm_Log(x, 2, "XBM: State-state transfers (fn).");
  if ( xbm_BuildStateStateTransfers(x, hfp) == 0 )
    return hfp_Close(hfp), 0;

  xbm_Log(x, 2, "XBM: Self-state transfers (fn).");
  if ( xbm_BuildStateSelfTransfers(x, hfp) == 0 )
    return hfp_Close(hfp), 0;
    
  xbm_Log(x, 2, "XBM: Minimize.");
  if ( hfp_Minimize(hfp, 0, 1) == 0 )
    return hfp_Close(hfp), 0;
    
  xbm_Log(x, 2, "XBM: Transfer function finished.");
  
  if ( dclCopy(x->pi_machine, x->cl_machine, hfp->cl_on) == 0 )
    return hfp_Close(hfp), 0;

  /* dclShow(x->pi_machine, x->cl_machine); */

  return hfp_Close(hfp), 1;
}

/*---------------------------------------------------------------------------*/

int xbm_build_sync_on_off_set(xbm_type x, dclist cl_on, dclist cl_off)
{
  dcube *c = &(xbm_GetPiMachine(x)->tmp[3]);
  int st_src_pos, st_dest_pos;
  int tr_pos;
  int st_pos;
  int in_cnt = x->inputs;
  //int out_cnt = x->outputs;
  int code_cnt = xbm_GetPiCode(x)->out_cnt;
  int i, cnt;
  
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
    dcCopyOutToOut( xbm_GetPiMachine(x), c, 0, 
      xbm_GetPiCode(x), &(xbm_GetSt(x, st_dest_pos)->code));
    dcCopyOutToOut( xbm_GetPiMachine(x), c, code_cnt, 
      xbm_GetPiOut(x), &(xbm_GetSt(x, st_dest_pos)->out));

    if ( dcIsIllegal(xbm_GetPiMachine(x), c) == 0 )
      if ( dclAdd(xbm_GetPiMachine(x), cl_on, c) < 0 )
        return 0;
      
    dcInvOut(xbm_GetPiMachine(x), c);

    if ( dcIsIllegal(xbm_GetPiMachine(x), c) == 0 )
      if ( dclAdd(xbm_GetPiMachine(x), cl_off, c) < 0 )
        return 0;
  }

  dclSCC(xbm_GetPiMachine(x), cl_on);
  dclSCC(xbm_GetPiMachine(x), cl_off);

  if ( dclIsIntersection(xbm_GetPiMachine(x), cl_on, cl_off) != 0 )
  {
    xbm_Error(x, "XBM: Illegal on/off-set intersection (transitions).");
    return 0;
  }

  st_pos = -1;
  while( xbm_LoopSt(x, &st_pos) != 0 )
  {
    cnt = dclCnt(xbm_GetSt(x, st_pos)->in_self_cl);
    for( i = 0; i < cnt; i++ )
    {
      dcCopyInToIn( xbm_GetPiMachine(x), c, 0, 
        xbm_GetPiIn(x), dclGet(xbm_GetSt(x, st_pos)->in_self_cl, i));
      dcCopyOutToIn( xbm_GetPiMachine(x), c, in_cnt, 
        xbm_GetPiCode(x), &(xbm_GetSt(x, st_pos)->code));
      if ( x->is_fbo != 0 )
        dcCopyOutToIn( xbm_GetPiMachine(x), c, in_cnt+code_cnt, 
          xbm_GetPiOut(x), &(xbm_GetSt(x, st_pos)->out));
      dcCopyOutToOut( xbm_GetPiMachine(x), c, 0, 
        xbm_GetPiCode(x), &(xbm_GetSt(x, st_pos)->code));
      dcCopyOutToOut( xbm_GetPiMachine(x), c, code_cnt, 
        xbm_GetPiOut(x), &(xbm_GetSt(x, st_pos)->out));
        
      if ( dcIsIllegal(xbm_GetPiMachine(x), c) == 0 )
        if ( dclAdd(xbm_GetPiMachine(x), cl_on, c) < 0 )
          return 0;

      dcInvOut(xbm_GetPiMachine(x), c);

      if ( dcIsIllegal(xbm_GetPiMachine(x), c) == 0 )
        if ( dclAdd(xbm_GetPiMachine(x), cl_off, c) < 0 )
          return 0;
    }
  }
  
  dclSCC(xbm_GetPiMachine(x), cl_on);
  dclSCC(xbm_GetPiMachine(x), cl_off);

  
  if ( dclIsIntersection(xbm_GetPiMachine(x), cl_on, cl_off) != 0 )
  {
    xbm_Error(x, "XBM: Illegal on/off-set intersection (states).");
    return 0;
  }
  
  return 1;
}


int xbm_BuildSynchronousTransferFunction(xbm_type x)
{
  dclist cl_on, cl_off, cl_dc;
  
  if ( xbm_InitMachine(x) == 0 )
    return 0;

  /* file import */

  if ( x->import_file_pla_machine != NULL )
    return xbm_ImportTransferFunction(x, x->import_file_pla_machine);

  if ( dclInitVA(3, &cl_on, &cl_off, &cl_dc) == 0 )
    return 0;
    
  if ( dclAdd(xbm_GetPiMachine(x), cl_dc, &(xbm_GetPiMachine(x)->tmp[0])) < 0 )
    return dclDestroyVA(3, cl_on, cl_off, cl_dc), 0;

  /* calculate transfer function */

  xbm_Log(x, 2, "XBM: Build synchronous transfer function.");

  if ( xbm_build_sync_on_off_set(x, cl_on, cl_off) == 0 )
    return dclDestroyVA(3, cl_on, cl_off, cl_dc), 0;

  if ( dclSubtract(xbm_GetPiMachine(x), cl_dc, cl_on) == 0 )
    return dclDestroyVA(3, cl_on, cl_off, cl_dc), 0;

  if ( dclSubtract(xbm_GetPiMachine(x), cl_dc, cl_off) == 0 )
    return dclDestroyVA(3, cl_on, cl_off, cl_dc), 0;

  xbm_Log(x, 2, "XBM: Minimize synchronous transfer function.");

  /* using minimize tobias' functions would be preferable, but it seems, 
     that there are some strange behaviours....
  if ( dclMinimizeDC(xbm_GetPiMachine(x), cl_on, cl_dc, 0, 1) == 0 )
    return dclDestroyVA(3, cl_on, cl_off, cl_dc), 0;
  */

  /* use the binate cover procedure instead... */    
  if ( dclMinimizeDCWithBCP(xbm_GetPiMachine(x), cl_on, cl_dc) == 0 )
    return dclDestroyVA(3, cl_on, cl_off, cl_dc), 0;
  
  if ( dclCopy(x->pi_machine, x->cl_machine, cl_on) == 0 )
    return dclDestroyVA(3, cl_on, cl_off, cl_dc), 0;
    
  return dclDestroyVA(3, cl_on, cl_off, cl_dc), 1;
}

/*---------------------------------------------------------------------------*/

int xbm_BuildTransferFunction(xbm_type x)
{
  if ( x->is_sync == 0 )
    return xbm_BuildAsynchronousTransferFunction(x);
  return xbm_BuildSynchronousTransferFunction(x);
}

/*---------------------------------------------------------------------------*/

int xbm_WritePLA(xbm_type x, const char *name)
{
  if ( x->cl_machine == NULL )
    return 0;
  if ( x->pi_machine == NULL )
    return 0;
  xbm_Log(x, 5, "XBM: Write PLA file '%s'.", name);
  if ( dclWritePLA(x->pi_machine, x->cl_machine, name) == 0 )
    return 0;
  return 1;
}

int xbm_WriteBEX(xbm_type x, const char *name)
{
  if ( x->cl_machine == NULL )
    return 0;
  if ( x->pi_machine == NULL )
    return 0;
  xbm_Log(x, 5, "XBM: Write BEX file '%s'.", name);
  if ( dclWriteBEX(x->pi_machine, x->cl_machine, name) == 0 )
    return 0;
  return 1;
}

int xbm_Write3DEQN(xbm_type x, const char *name)
{
  if ( x->cl_machine == NULL )
    return 0;
  if ( x->pi_machine == NULL )
    return 0;
  xbm_Log(x, 5, "XBM: Write EQN (3D) file '%s'.", name);
  if ( dclWrite3DEQN(x->pi_machine, x->cl_machine, name) == 0 )
    return 0;
  return 1;
}

