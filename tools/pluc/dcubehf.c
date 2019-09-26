/*

  dcubehf.c

  Hazard detection
  Hazardfree cover
  
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

#include <stdlib.h>
#include <stdarg.h>
#include <assert.h>
#include "dcube.h"
#include "dcubehf.h"
#include "matrix.h"

#include "mwc.h"

/*-- dclHazardAnalysisOut ---------------------------------------------------*/

int dclHazardAnalysisOut(pinfo *pi, dclist cl, dcube *start, dcube *end, int out, int (*fn)(void *data, dchazard *dch), void *data)
{
  dcube *r = &(pi->tmp[5]);
  dcube *m = &(pi->tmp[6]);
  int i, cnt = dclCnt(cl);
  
  dchazard dch;
  
  dch.inter     = r; 
  dch.super     = m; 
  dch.cl        = cl;
  dch.start     = start;
  dch.end       = end;
  dch.out       = out;
  dch.is_ok     = 0;
  if ( dclInitCached(pi, &dch.cl_local) == 0 )
    return 0;

  /* calculate a bounding box of the transition */  
  dcOr(pi, m, start, end);
  dcOutSetAll(pi, m, 0);
  
  /* consider the specified output line */
  dcSetOut(m, out, 1);

  /* calculate the local (with respect to m) intersection of all cubes */
  if ( dclIntersectionCube(pi, dch.cl_local, cl, m) == 0 )
    return dclDestroyCached(pi, dch.cl_local), 0;
    
  /* r is the intersection of all these elements */
  dclAndElements(pi, r, dch.cl_local);
  
  /* calculate the function value of start and end inside */
  /* the current output function. */
  /* To be free of hazards the start and/or end cube */
  /* must be covered by an implikant, so the calculation */
  /* of the function value is simplified to a containment */
  /* test. */
  dch.f_start = 0; 
  dch.f_end = 0;
  for( i = 0; i < cnt; i++ )
  {
    if ( dcGetOut(dclGet(cl, i), out) != 0 )
    {
      if ( dcIsInSubSet(pi, dclGet(cl, i), start) != 0 )
        dch.f_start = 1;
      if ( dcIsInSubSet(pi, dclGet(cl, i), end) != 0 )
        dch.f_end = 1;
    }
  }

  dch.type = (dch.f_start<<1)|dch.f_end;

  if ( dch.f_start != 0 && dch.f_end != 0 )
  {
    if ( dcIsIllegal(pi, r) != 0 )
    {
      dch.msg = DCH_MSG_ERROR_NOT_UNATE;
    }
    else
    {
      for( i = 0; i < cnt; i++ )
        if ( dcIsSubSet(pi, dclGet(cl, i), m) != 0 )
          break;
      if ( i >= cnt )
      {
        dch.msg = DCH_MSG_ERROR_SUPER_NOT_SUBSET;
      }
      else
      {
        dch.is_ok = 1;
        dch.msg = DCH_MSG_OK_SUPER_IS_SUBSET;
      }
    }
    return dclDestroyCached(pi, dch.cl_local), fn(data, &dch);
  }
  else if ( dch.f_start != 0 )
  {
    if ( dcIsIllegal(pi, r) != 0 )
    {
      dch.msg = DCH_MSG_ERROR_NOT_UNATE;
    }
    else if ( dcIsInSubSet(pi, r, start) == 0 )
    {
      dch.msg = DCH_MSG_ERROR_START_NOT_COVERED;
    }
    else 
    {
      dch.is_ok = 1;
      dch.msg = DCH_MSG_OK_START_IS_COVERED;
    }
    return dclDestroyCached(pi, dch.cl_local), fn(data, &dch);
  }
  else if ( dch.f_end != 0 )
  {
    if ( dcIsIllegal(pi, r) != 0 )
    {
      dch.msg = DCH_MSG_ERROR_NOT_UNATE;
    }
    else if ( dcIsInSubSet(pi, r, end) == 0 )
    {
      dch.msg = DCH_MSG_ERROR_END_NOT_COVERED;
    }
    else 
    {
      dch.is_ok = 1;
      dch.msg = DCH_MSG_OK_END_IS_COVERED;
    }
    return dclDestroyCached(pi, dch.cl_local), fn(data, &dch);
  }
  else
  {
    /* both output values are zero, there must be no intersection with m */
    for( i = 0; i < cnt; i++ )
      if ( dcIsDeltaNoneZero(pi, m, dclGet(cl, i)) == 0 )
      {
        dch.msg = DCH_MSG_ERROR_SUPER_HAS_INTERSECTION;
        return dclDestroyCached(pi, dch.cl_local), fn(data, &dch);
      }
    dch.is_ok = 1;
    dch.msg = DCH_MSG_OK_SUPER_HAS_NO_INTERSECTION;
    return dclDestroyCached(pi, dch.cl_local), fn(data, &dch);
  }
}

/*-- dclHazardAnalysis ------------------------------------------------------*/

int dclHazardAnalysis(pinfo *pi, dclist cl, dcube *start, dcube *end, int out, int (*fn)(void *data, dchazard *dch), void *data)
{
  int i;
  for( i = 0; i < pi->out_cnt; i++ )
    if ( dclHazardAnalysisOut(pi, cl, start, end, i, fn, data) == 0 )
      return 0;
  return 1;
}



/*-- dclIsHazardfreeTransition ----------------------------------------------*/

/* 
  Checks if the transition from
  in(start) to in(end) within the
  output 'out' is free of potential
  hazards.
*/

int dclIsHazardfreeFunction(pinfo *pi, dclist cl, dcube *start, dcube *end, int out)
{
  dcube *r = &(pi->tmp[5]);
  dcube *m = &(pi->tmp[6]);
  int i, cnt = dclCnt(cl);
  int f_start, f_end;

  /* calculate a bounding box of the transition */  
  dcOr(pi, m, start, end);
  dcOutSetAll(pi, m, 0);
  
  /* consider the specified output line */
  dcSetOut(m, out, 1);

  /* calculate the local (with respect to m) intersection of all cubes */
  dclAndLocalElements(pi, r, m, cl);
  
  /* calculate the function value of start and end inside */
  /* the current output function. */
  /* To be free of hazards the start and/or end cube */
  /* must be covered by an implikant, so the calculation */
  /* of the function value is simplified to a containment */
  /* test. */
  f_start = 0; 
  f_end = 0;
  for( i = 0; i < cnt; i++ )
  {
    if ( dcGetOut(dclGet(cl, i), out) != 0 )
    {
      if ( dcIsInSubSet(pi, dclGet(cl, i), start) != 0 )
        f_start = 1;
      if ( dcIsInSubSet(pi, dclGet(cl, i), end) != 0 )
        f_end = 1;
    }
  }

  if ( f_start != 0 && f_end != 0 )
  {
    for( i = 0; i < cnt; i++ )
      if ( dcIsSubSet(pi, dclGet(cl, i), m) != 0 )
        break;
    if ( i >= cnt )
      return /* printf("mask not subset (out %d)\n", out), */ 0;
    return 1;
  }
  else if ( f_start != 0 )
  {
    if ( dcIsIllegal(pi, r) != 0 )
    {
      /* not unate */
      /*
      printf("not unate (out %d, area %s)\n", out, dcToStr(pi, m, " ", ""));  
      {
        int i, cnt = dclCnt(cl);
        for( i = 0; i < cnt; i++ )
          if ( dcIsDeltaNoneZero(pi, m, dclGet(cl, i)) == 0 )
            puts(dcToStr(pi, dclGet(cl, i), " ", ""));
      }
      */
      return 0;
    }
    if ( dcIsInSubSet(pi, r, start) == 0 )
      return /* printf("start not covered (out %d, area %s)\n", out, dcToStr(pi, m, " ", "")), */ 0;
    return 1;
  }
  else if ( f_end != 0 )
  {
    if ( dcIsIllegal(pi, r) != 0 )
    {
      /* not unate */
      /*
      printf("not unate (out %d, area %s)\n", out, dcToStr(pi, m, " ", ""));  
      {
        int i, cnt = dclCnt(cl);
        for( i = 0; i < cnt; i++ )
          if ( dcIsDeltaNoneZero(pi, m, dclGet(cl, i)) == 0 )
            puts(dcToStr(pi, dclGet(cl, i), " ", ""));
      }
      */
      return 0;
    }
    if ( dcIsInSubSet(pi, r, end) == 0 )
      return /* printf("end not covered (out %d)\n", out), */ 0;
    return 1;
  }
  
  /* both output values are zero, there must be no intersection with m */
  /*
  for( i = 0; i < cnt; i++ )
    if ( dcIsDeltaNoneZero(pi, m, dclGet(cl, i)) == 0 )
      return 0;
  */
  return 1;
}

int dclIsHazardfreeTransition(pinfo *pi, dclist cl, dcube *start, dcube *end)
{
  int i;
  for( i = 0; i < pi->out_cnt; i++ )
    if ( dclIsHazardfreeFunction(pi, cl, start, end, i) == 0 )
      return 0;
  return 1;
}


/*---------------------------------------------------------------------------*/

pcube *pcOpen(pinfo *pi)
{
  pcube *p;
  p = (pcube *)malloc(sizeof(struct _pcube_struct));
  if ( p != NULL )
  {
    if ( dcInit(pi, &(p->sc)) != 0 )
    {
      if ( dcInit(pi, &(p->tc)) != 0 )
      {
        return p;
      }
      dcDestroy(&(p->sc));
    }
    free(p);
  }
  return NULL;
}

void pcClose(pcube *p)
{
  dcDestroy(&(p->sc));
  dcDestroy(&(p->tc));
  free(p);
}

static void hfp_dummy_cb(void *data, const char *fmt, va_list va)
{
}

hfp_type hfp_Open(int in, int out)
{
  hfp_type hfp;
  hfp = (hfp_type)malloc(sizeof(struct _hfp_struct));
  if ( hfp != NULL )
  {
    hfp->log_data = NULL;
    hfp->log_fn = hfp_dummy_cb;
    hfp->err_data = NULL;
    hfp->err_fn = hfp_dummy_cb;
    hfp->pi = pinfoOpenInOut(in, out);
    if ( hfp->pi != NULL )
    {
      hfp->pclist = b_set_Open();
      if ( hfp->pclist != NULL )
      {
        if ( dclInitVA(4, &(hfp->cl_req_on), &(hfp->cl_req_off), &(hfp->cl_on), &(hfp->cl_dc)) != 0 )
        {
          if ( dclClearFlags(hfp->cl_dc) != 0 )
          {
            if ( dclAdd(hfp->pi, hfp->cl_dc, &(hfp->pi->tmp[0])) >= 0 )
            {
              return hfp;
            }
          }
          dclDestroyVA(4, hfp->cl_req_on, hfp->cl_req_off, hfp->cl_on, hfp->cl_dc);
        }
        b_set_Close(hfp->pclist);
      }
      pinfoClose(hfp->pi);
    }
    free(hfp);
  }
  return NULL;
}

void hfp_Close(hfp_type hfp)
{
  int i = -1;
  /*
  puts("--- cl_req_on ---");
  dclShow(hfp->pi, hfp->cl_req_on);
  puts("--- cl_req_off ---");
  dclShow(hfp->pi, hfp->cl_req_off);
  puts("--- cl_dc ---");
  dclShow(hfp->pi, hfp->cl_dc);
  i = -1;
  while( b_set_WhileLoop(hfp->pclist, &i) != 0 )
  {
    printf("%s  - ", dcToStr(hfp->pi, &(hfp_GetPC(hfp, i)->sc), " ", ""));
    printf("%s\n", dcToStr(hfp->pi, &(hfp_GetPC(hfp, i)->tc), " ", ""));
  }
  */
  i = -1;
  while( b_set_WhileLoop(hfp->pclist, &i) != 0 )
    pcClose(hfp_GetPC(hfp, i));
  
  dclDestroyVA(4, hfp->cl_req_on, hfp->cl_req_off, hfp->cl_on, hfp->cl_dc);
  b_set_Close(hfp->pclist);
  pinfoClose(hfp->pi);
  free(hfp);
}

void hfp_SetLogCB(hfp_type hfp, void log_cb(void *log_data, const char *fmt, va_list va), void *log_data)
{
  hfp->log_fn = log_cb;
  hfp->log_data = log_data;
}

void hfp_SetErrorCB(hfp_type hfp, void err_cb(void *err_data, const char *fmt, va_list va), void *err_data)
{
  hfp->err_fn = err_cb;
  hfp->err_data = err_data;
}

void hfp_LogVA(hfp_type hfp, const char *fmt, va_list va)
{
  hfp->log_fn(hfp->log_data, fmt, va);
}

void hfp_Log(hfp_type hfp, const char *fmt, ...)
{
  va_list va;
  va_start(va, fmt);
  hfp_LogVA(hfp, fmt, va);
  va_end(va);
}

void hfp_ErrorVA(hfp_type hfp, const char *fmt, va_list va)
{
  hfp->err_fn(hfp->err_data, fmt, va);
}

void hfp_Error(hfp_type hfp, const char *fmt, ...)
{
  va_list va;
  va_start(va, fmt);
  hfp_ErrorVA(hfp, fmt, va);
  va_end(va);
}

int hfp_AddPC(hfp_type hfp, pcube *pc)
{
  return b_set_Add(hfp->pclist, pc);
}

pcube *hfp_GetPC(hfp_type hfp, int id)
{
  return (pcube *)b_set_Get(hfp->pclist, id);
}

int hfp_AddStartTransitionCube(hfp_type hfp, dcube *sc, dcube *tc)
{
  pcube *pc;
  int pos;
  
  if ( dcInDCCnt(hfp->pi, tc) == 0 )
    return 1;

  pc = pcOpen(hfp->pi);
  if ( pc == NULL )
    return 0;
  dcCopy(hfp->pi, &(pc->sc), sc);
  dcCopy(hfp->pi, &(pc->tc), tc);

  hfp_Log(hfp, "HFP: Trans/Start   %s/%s", 
    dcToStr(hfp->pi, tc , " ",""),
    dcToStr2(hfp->pi, sc , " ",""));

  /*
  dcSetOutTautology(hfp->pi, &(pc->sc));
  dcSetOutTautology(hfp->pi, &(pc->tc));
  */
  
  pos = hfp_AddPC(hfp, pc);
  if ( pos >= 0 )
    return 1;

  pcClose(pc);
  return 0;
}

int hfp_AddStartEndCube(hfp_type hfp, dcube *sc, dcube *ec)
{
  pcube *pc = pcOpen(hfp->pi);  
  int pos;
  
  if ( pc == NULL )
    return 0;
  
  dcCopy(hfp->pi, &(pc->sc), sc);
  dcOr(hfp->pi, &(pc->tc), sc, ec);

  if ( dcInDCCnt(hfp->pi, &(pc->tc)) == 0 )
    return pcClose(pc), 1;
  
  /*
  dcSetOutTautology(hfp->pi, &(pc->sc));
  dcSetOutTautology(hfp->pi, &(pc->tc));
  */
  
  pos = hfp_AddPC(hfp, pc);
  if ( pos >= 0 )
    return 1;
    
  pcClose(pc);
  return 0;
}

static int hfp_IsIntersectionCube(hfp_type hfp, pinfo *pi, dclist cl, dcube *c, const char *setname)
{
  int i, cnt = dclCnt(cl);
  for( i = 0; i < cnt; i++ )
    if ( dcIsDeltaNoneZero(pi, dclGet(cl, i), c) == 0 )
    {
      hfp_Log(hfp, "HFP: Illegal intersection of cube %s with %scube %s.",
        dcToStr(pi, c, " ", ""), 
        setname,
        dcToStr2(pi, dclGet(cl, i), " ", ""));
      return 1;
    }
  return 0;
}


int hfp_AddRequiredOnCube(hfp_type hfp, dcube *c)
{
  hfp_Log(hfp, "HFP: On-set    <-- %s", dcToStr(hfp->pi, c , " ",""));
  if ( hfp_IsIntersectionCube(hfp, hfp->pi, hfp->cl_req_off, c, "off-set ") != 0)
    return 0;
  if ( dclAdd(hfp->pi, hfp->cl_req_on, c) < 0 )
    return 0;
  /*
  if ( dclSCCSubtractCube(hfp->pi, hfp->cl_dc, c) == 0 )
    return 0;
  */
  return 1;
}

int hfp_AddRequiredOffCube(hfp_type hfp, dcube *c)
{
  hfp_Log(hfp, "HFP: Off-set   <-- %s", dcToStr(hfp->pi, c , " ",""));
  if ( hfp_IsIntersectionCube(hfp, hfp->pi, hfp->cl_req_on, c, "on-set ") != 0) 
    return 0;
  if ( dclAdd(hfp->pi, hfp->cl_req_off, c) < 0 )
    return 0;
  /*
  if ( dclSCCSubtractCube(hfp->pi, hfp->cl_dc, c) == 0 )
    return 0;
  */
  return 1;
}

int hfp_AddRequiredOnCubeList(hfp_type hfp, dclist cl)
{
  int i, cnt = dclCnt(cl);
  for( i = 0; i < cnt; i++ )
    if ( hfp_AddRequiredOnCube(hfp, dclGet(cl, i)) == 0 )
      return 0;
  return 1;

  /*
  if ( dclSCCUnion(hfp->pi, hfp->cl_req_on, cl) == 0 )
    return 0;
  return 1;
  */
}

int hfp_AddRequiredOffCubeList(hfp_type hfp, dclist cl)
{
  int i, cnt = dclCnt(cl);
  for( i = 0; i < cnt; i++ )
    if ( hfp_AddRequiredOffCube(hfp, dclGet(cl, i)) == 0 )
      return 0;
  return 1;
  /*
  if ( dclSCCUnion(hfp->pi, hfp->cl_req_off, cl) == 0 )
    return 0;
  return 1;
  */
}

int hfp_AddStartTransitionCubes(hfp_type hfp, dcube *c, dclist cl_all)
{
  pcube *pc;
  int pos;
  int i, cnt;
  if ( dcIsIllegal(hfp->pi, c) != 0 )
    return 0;
    
  cnt = dclCnt(cl_all);
  for( i = 0; i < cnt; i++ )
  {
    if ( dcIsSubSet(hfp->pi, dclGet(cl_all, i), c) == 0 )
      return 0;
    pc = pcOpen(hfp->pi);
    if ( pc == NULL )
      return 0;
    dcCopy(hfp->pi, &(pc->sc), c);
    dcCopy(hfp->pi, &(pc->tc), dclGet(cl_all, i));
    pos = hfp_AddPC(hfp, pc);
    if ( pos < 0 )
    {
      pcClose(pc);
      return 0;
    }  
  }
  
  return 1;
}

static int hfp_add_from_to_transition_out(hfp_type hfp, dcube *sc, dcube *dc, int o)
{
  int sov = dcGetOut(sc, o);
  int dov = dcGetOut(dc, o);
  dcube *c = &(hfp->pi->tmp[13]);
  
  dcOr(hfp->pi, c, sc, dc);
  dcSetOut(c, o, 1);

  if ( sov == dov )
  {
    if ( sov != 0 )
    {
      /* printf("on  %s\n", dcToStr(hfp->pi, c , " ","")); */
      if (hfp_AddRequiredOnCube(hfp, c) == 0 )
        return 0;
    }
    else
    {
      /* printf("off %s\n", dcToStr(hfp->pi, c , " ","")); */
      if (hfp_AddRequiredOffCube(hfp, c) == 0 )
        return 0;
    }
  }
  else 
  {
    dclist cl;
    if ( dclInitCached(hfp->pi, &cl) == 0 )
      return 0;
    if ( dclAdd(hfp->pi, cl, c) < 0 )
      return dclDestroyCached(hfp->pi, cl), 0;
    if ( sov != 0 )
    {
      /* 1 -> 0 transition */
      dcSetOut(sc, o, 1);
      dcSetOut(dc, o, 1);
      if ( dclSubtractCube(hfp->pi, cl, dc) == 0 )
        return dclDestroyCached(hfp->pi, cl), 0;
      if ( hfp_AddStartTransitionCube(hfp, sc, c) == 0 )
        return dclDestroyCached(hfp->pi, cl), 0;
      if ( hfp_AddRequiredOffCube(hfp, dc) == 0 )
        return dclDestroyCached(hfp->pi, cl), 0;
      if ( hfp_AddRequiredOnCubeList(hfp, cl) == 0 )
        return dclDestroyCached(hfp->pi, cl), 0;
      dcSetOut(sc, o, 0);
      dcSetOut(dc, o, 0);
    }
    else
    {
      /* 0 -> 1 transition */
      dcSetOut(sc, o, 1);
      dcSetOut(dc, o, 1);
      if ( dclSubtractCube(hfp->pi, cl, dc) == 0 )
        return dclDestroyCached(hfp->pi, cl), 0;
      if ( hfp_AddStartTransitionCube(hfp, dc, c) == 0 )
        return dclDestroyCached(hfp->pi, cl), 0;
      if ( hfp_AddRequiredOnCube(hfp, dc) == 0 )
        return dclDestroyCached(hfp->pi, cl), 0;
      if ( hfp_AddRequiredOffCubeList(hfp, cl) == 0 )
        return dclDestroyCached(hfp->pi, cl), 0;
      dcSetOut(sc, o, 0);
      dcSetOut(dc, o, 0);
    }
    dclDestroyCached(hfp->pi, cl); 
  }
  return 1;
}


int hfp_AddFromToTransition(hfp_type hfp, dcube *from, dcube *to)
{
  dcube *sc = &(hfp->pi->tmp[16]);
  dcube *dc = &(hfp->pi->tmp[17]);
  int i;
  for( i = 0; i < hfp->pi->out_cnt; i++ )
  {
    dcCopyIn(   hfp->pi, sc, from);
    dcOutSetAll(hfp->pi, sc, 0);
    dcSetOut(            sc, i, dcGetOut(from, i));
    dcCopyIn(   hfp->pi, dc, to);
    dcOutSetAll(hfp->pi, dc, 0);
    dcSetOut(            dc, i, dcGetOut(to, i));
    if ( hfp_add_from_to_transition_out(hfp, sc, dc, i) == 0 )
      return 0;
  }
  return 1;
}


int hfp_IsIllegalIntersection(hfp_type hfp, dcube *c, int pc_id)
{
  pcube *pc;
  pc = hfp_GetPC(hfp, pc_id);
  
  if ( dcIsDeltaNoneZero(hfp->pi, c, &(pc->tc)) > 0 )
    return 0; /* no intersection */
  
  if ( dcIsSubSet(hfp->pi, c, &(pc->sc)) != 0 )
    return 0; /* start value is covered, no illegal intersection */
  
  /* ahh... yes, thats an illegal intersection */
  return 1;
}

int hfp_CheckReqCube(hfp_type hfp)
{
  int i, j;
  for(i = 0; i < dclCnt(hfp->cl_req_on); i++ )
  {
    j = -1;
    while( b_set_WhileLoop(hfp->pclist, &j) != 0 )
      if ( hfp_IsIllegalIntersection(hfp, dclGet(hfp->cl_req_on, i), j) != 0 )
      {
        hfp_Log(hfp, "HFP: req on  %s", dcToStr(hfp->pi, dclGet(hfp->cl_req_on, i) , " ",""));
        hfp_Log(hfp, "HFP: tc      %s", dcToStr(hfp->pi, &(hfp_GetPC(hfp, j)->tc) , " ",""));
        hfp_Log(hfp, "HFP: sc      %s", dcToStr(hfp->pi, &(hfp_GetPC(hfp, j)->sc) , " ",""));
        return 0;
      }
  }  
  for(i = 0; i < dclCnt(hfp->cl_req_off); i++ )
  {
    j = -1;
    while( b_set_WhileLoop(hfp->pclist, &j) != 0 )
      if ( hfp_IsIllegalIntersection(hfp, dclGet(hfp->cl_req_off, i), j) != 0 )
      {
        hfp_Log(hfp, "HFP: req off %s", 
          dcToStr(hfp->pi, dclGet(hfp->cl_req_off, i) , " ",""));
        hfp_Log(hfp, "HFP: tc      %s", dcToStr(hfp->pi, &(hfp_GetPC(hfp, j)->tc) , " ",""));
        hfp_Log(hfp, "HFP: sc      %s", dcToStr(hfp->pi, &(hfp_GetPC(hfp, j)->sc) , " ",""));
/*        
        return 0;
*/
      }
  }  
  return 1;
}

int hfp_ToDHF(hfp_type hfp)
{
  int j;
  int is_any_illegal_intersection;
  dcube *c = &(hfp->pi->tmp[10]);
  pcube *pc;
  
  dclist cl; /* result */
  
  if ( dclInit(&cl) == 0 )
    return 0;
  
  if ( dclClearFlags(hfp->cl_on) == 0 )
    return dclDestroy(cl), 0;

  if ( dclClearFlags(cl) == 0 )
    return dclDestroy(cl), 0;

  hfp_Log(hfp, "HFP: Privileged cubes: %d", b_set_Cnt(hfp->pclist));

  
  while( dclCnt(hfp->cl_on) > 0 )
  {
    /* get the last cube and delete it from hfp->cl_on */
    dcCopy(hfp->pi, c, dclGet(hfp->cl_on, dclCnt(hfp->cl_on)-1));
    dclDeleteCube(hfp->pi, hfp->cl_on, dclCnt(hfp->cl_on)-1);
    
    is_any_illegal_intersection = 0;
    j = -1;
    while( b_set_WhileLoop(hfp->pclist, &j) != 0 )
    { 
      if ( hfp_IsIllegalIntersection(hfp, c, j) != 0 )
      {
        pc = hfp_GetPC(hfp, j);
        if ( dclSCCSharpAndSetFlag(hfp->pi, hfp->cl_on, c, &(pc->tc)) == 0 )
          return dclDestroy(cl), 0;
        dclDeleteCubesWithFlag(hfp->pi, hfp->cl_on);
        is_any_illegal_intersection = 1;
      }        
    }
    if ( is_any_illegal_intersection == 0 )
    {
      if ( dclSCCAddAndSetFlag(hfp->pi, cl, c) == 0 )
        return dclDestroy(cl), 0;
      dclDeleteCubesWithFlag(hfp->pi, cl);
    }
  }
  
  if ( dclCopy(hfp->pi, hfp->cl_on, cl) == 0 )
    return dclDestroy(cl), 0;
    
  return dclDestroy(cl), 1;
}

static int dclMergeEqualIn(pinfo *pi, dclist cl)
{
  int i, j, cnt;
  cnt = dclCnt(cl);
  if ( dclClearFlags(cl) == 0 )
    return 0;
  for( i = 0; i < cnt; i++ )
  {
    for( j = i+1; j < cnt; j++ )
    {
      if ( dcIsEqualIn(pi, dclGet(cl, i), dclGet(cl, j)) != 0 )
      {
        dcOrOut(pi, dclGet(cl, i), dclGet(cl, i), dclGet(cl, j));
        dclSetFlag(cl, j);
      }
    }
  }
  dclDeleteCubesWithFlag(pi, cl);
  return 1;
}

int hfp_Check(hfp_type hfp)
{
  int i;
  for( i = 0; i < dclCnt(hfp->cl_req_on); i++ )
    if ( dclIsSingleSubSet(hfp->pi, hfp->cl_on, dclGet(hfp->cl_req_on, i)) == 0 )
      return 0;
  
  return 1;
}

int hfp_Minimize(hfp_type hfp, int is_greedy, int is_literal)
{
  pinfo *pi = hfp->pi;
  dclist cl_es, cl_fr, cl_pr, cl_on;
  dclInitVA(4, &cl_es, &cl_fr, &cl_pr, &cl_on);
  /* pinfoInitProgress(pi); */

  dclSCC(hfp->pi, hfp->cl_req_on);
  
  dclSCC(hfp->pi, hfp->cl_req_off);

  if ( hfp_CheckReqCube(hfp) == 0 )
    return 0;

  hfp_Log(hfp, "HFP: Required on cubes: %d", dclCnt(hfp->cl_req_on));
  hfp_Log(hfp, "HFP: Required off cubes: %d", dclCnt(hfp->cl_req_off));


  dclSCCUnion(hfp->pi, hfp->cl_on, hfp->cl_req_on);

  if ( dclCopy(hfp->pi, hfp->cl_dc, hfp->cl_on) == 0 )
    return dclDestroyVA(4, cl_es, cl_fr, cl_pr, cl_on), 0;   
  
  if ( dclSCCUnion(hfp->pi, hfp->cl_dc, hfp->cl_req_off) == 0 )
    return dclDestroyVA(4, cl_es, cl_fr, cl_pr, cl_on), 0;   

  dclMergeEqualIn(hfp->pi, hfp->cl_dc);

  if ( dclComplement(hfp->pi, hfp->cl_dc) == 0 )
    return dclDestroyVA(4, cl_es, cl_fr, cl_pr, cl_on), 0;   

  /*
  assert( dclIsIntersection(hfp->pi, hfp->cl_on, hfp->cl_req_off) == 0 );
  assert( dclIsIntersection(hfp->pi, hfp->cl_dc, hfp->cl_req_off) == 0 );
  */
    
  if ( dclCopy(pi, cl_on, hfp->cl_on) == 0 )
    return dclDestroyVA(4, cl_es, cl_fr, cl_pr, cl_on), 0;   


  if ( dclPrimesDC(pi, hfp->cl_on, hfp->cl_dc) == 0 )
    return dclDestroyVA(4, cl_es, cl_fr, cl_pr, cl_on), 0;   

  hfp_Log(hfp, "HFP: Boolean primes: %d", dclCnt(hfp->cl_on));


  /*
  if ( hfp_Check(hfp) == 0 )
    return dclDestroyVA(4, cl_es, cl_fr, cl_pr, cl_on), 0;   
  */

  /*
  printf("--- primes %d ---\n", dclCnt(hfp->cl_on));
  dclShow(hfp->pi, hfp->cl_on);
  */
  
  if ( hfp_ToDHF(hfp) == 0 )
    return dclDestroyVA(4, cl_es, cl_fr, cl_pr, cl_on), 0;   

  /*
  if ( hfp_Check(hfp) == 0 )
    return dclDestroyVA(4, cl_es, cl_fr, cl_pr, cl_on), 0;   
  */

  /*
  printf("--- hf primes %d ---\n", dclCnt(hfp->cl_on));
  dclShow(hfp->pi, hfp->cl_on);
  */

  if ( dclSCC(pi, hfp->cl_on) == 0 )
    return dclDestroyVA(4, cl_es, cl_fr, cl_pr, cl_on), 0;   
  /*
  printf("--- reduced primes %d ---\n", dclCnt(hfp->cl_on));
  dclShow(hfp->pi, hfp->cl_on);
  */

  hfp_Log(hfp, "HFP: DHF primes: %d", dclCnt(hfp->cl_on));

  if ( dclSplitRelativeEssential(pi, cl_es, cl_fr, cl_pr, hfp->cl_on, hfp->cl_dc, hfp->cl_req_on) == 0 )
    return dclDestroyVA(4, cl_es, cl_fr, cl_pr, cl_on), 0;


  /* a fully redundant cube might be required by an element of cl_req_on (?) */
  /* problem: one can not add cl_fr without essential cubes !!!! */
  /* There should be an additional procedure */
  /*   with all required cubes */
  /*     if the req cube is not satisfied */
  /*        try to find an element from cl_fr which satisfies the req cube */
  /*        add this element  */
  /*        error of no element from cl_fr was found */
  
  /*  
  if ( dclJoin(pi, cl_pr, cl_fr) == 0 )
    return dclDestroyVA(4, cl_es, cl_fr, cl_pr, cl_on), 0;   
  */

  if ( is_literal != 0 )
  {
    if ( maMatrixIrredundant(pi, cl_es, cl_pr, hfp->cl_dc, hfp->cl_req_on, is_greedy, MA_LIT_SOP) == 0 )
      return dclDestroyVA(4, cl_es, cl_fr, cl_pr, cl_on), 0;
  }
  else
  {
    if ( maMatrixIrredundant(pi, cl_es, cl_pr, hfp->cl_dc, hfp->cl_req_on, is_greedy, MA_LIT_NONE) == 0 )
      return dclDestroyVA(4, cl_es, cl_fr, cl_pr, cl_on), 0;
  }

  /*  
  if ( dclIrredundantGreedy(pi, cl_es, cl_pr, hfp->cl_dc, hfp->cl_req_on) == 0 )
    return dclDestroyVA(4, cl_es, cl_fr, cl_pr, cl_on), 0;
  */

  if ( dclJoin(pi, cl_pr, cl_es) == 0 )
    return dclDestroyVA(4, cl_es, cl_fr, cl_pr, cl_on), 0;
    
  dclRestrictOutput(pi, cl_pr);
  
  if( dclIsEquivalentDC(pi, cl_pr, cl_on, hfp->cl_dc) == 0 )
    return dclDestroyVA(4, cl_es, cl_fr, cl_pr, cl_on), 0;
    
  if ( dclCopy(pi, hfp->cl_on, cl_pr) == 0 )
    return dclDestroyVA(4, cl_es, cl_fr, cl_pr, cl_on), 0;   

  /*
  printf("--- minimal hazardfree solution %d ---\n", dclCnt(hfp->cl_on));
  dclShow(hfp->pi, hfp->cl_on);
  */
  
  
  /*
  if ( hfp_Check(hfp) == 0 )
    return dclDestroyVA(4, cl_es, cl_fr, cl_pr, cl_on), 0;   
  */

  hfp_Log(hfp, "HFP: Selected primes: %d", dclCnt(hfp->cl_on));
  
  
  dclDestroyVA(4, cl_es, cl_fr, cl_pr, cl_on);
  return 1;
}




