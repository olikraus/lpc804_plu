/*

  dcubeustt.c
  
  minimize a ustt encoding

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
  minimization for an asynchronous FSM.
  
  Reference:
    well... I would say I invented some of the functions below.., :-)
  
*/

#include <stdarg.h>
#include <assert.h>
#include "dcube.h"
#include "matrix.h"


/* comments from fsmustt.c ....
  The function dclPrimesInv() will calculate the primes, but we must
  consider the fact, that also the "inverted" cube is a valid solution.

     Invert all elementes, use dclPrimesInv(). This will generate all
     primes, inverted and none-inverted. This means, that the solution
     must be cut down to the half number of terms.
     
  This idea does not really work:
     Skip the toplevel step of the URP inside dclPrimesInv(). 
     This would generate only half of the solution, which
     is exactly what we want. To do this, we must make the the input cover
     unate in one variable and invert and add all entries that are don't care
     in that variable.
  
  There was a bug in dclPrimesInv() that could have caused the problems above.
  dclPrimesInv() has been fixed on 31. may 2001, ok
  
  01. april 2002, again tried the second idea... hmmm generates even more
  prime partitions, indeed some results are larger... see freq_3_1.bms
  (with exact cover!)
  
*/

#include "dcube.h"
#include <assert.h>

int dclPrimesUSTT(pinfo *pi, dclist cl)
{ 
  dcube *cp = &(pi->tmp[1]);
  int i, cnt = dclCnt(cl);
  int j;
  dclist clt;

  /* add inverted cubes */
  
  for( i = 0; i < cnt; i++ )
  {
    dcCopy(pi, cp, dclGet(cl, i));
    dcInvIn(pi, cp);
    if ( dclAdd(pi, cl, cp) < 0 )
      return 0;
  }
  
  /* calculate prime partitions/dichotimies  */

  if ( dclInit(&clt) == 0 )
    return 0;
  if ( dclCopy(pi, clt, cl) == 0 )
    return dclDestroy(clt), 0;
  if ( dclPrimesInv(pi, cl) == 0 )
    return dclDestroy(clt), 0;

  /* do some verification of the result */

  for( i = 0; i < dclCnt(clt); i++ )
  {
    for( j = 0; j < dclCnt(cl); j++ )
    {
      if ( dcIsSubSet(pi, dclGet(clt, i), dclGet(cl, j)) != 0 )
        break;
    }
    if ( j >= dclCnt(cl) )
    {
      assert(0);
    }
  }
  dclDestroy(clt);

  /* reduce to a unique solution */

  dclClearFlags(cl);
  for( i = 0; i < dclCnt(cl); i++ )
  {
    if ( dclIsFlag(cl, i) == 0 )
    {
      dcCopy(pi, cp, dclGet(cl, i));
      dcInvIn(pi, cp);
      for( j = i+1; j < dclCnt(cl); j++ )
      {
        if ( dclIsFlag(cl, i) == 0 )
        {
          if ( dcIsEqual(pi, cp, dclGet(cl, j)) != 0 )
          {
            dclSetFlag(cl, j);
          }
        }
      }
    }
  }
  dclDeleteCubesWithFlag(pi, cl);
 
  return 1;
}

#include "fsm.h"

static int async_PartitionCoverElement(pinfo *pi_m, dclist cl_m, pinfo *pi, dcube *c, dclist cl_i)
{
  int i, cnt = dclCnt(cl_i);
  dcube *m = &(pi_m->tmp[9]);
  dcube *cp = &(pi->tmp[1]);
  dcOutSetAll(pi_m, m, 0);
  dcInSetAll(pi_m, m, CUBE_IN_MASK_DC);
  dcCopy(pi, cp, c);
  dcInvIn(pi, cp);
  
  for( i = 0; i < cnt; i++ )
  {
    if ( dcIsSubSet(pi, dclGet(cl_i, i), c) != 0 )
      dcSetOut(m, i, 1);
    if ( dcIsSubSet(pi, dclGet(cl_i, i), cp) != 0 )
      dcSetOut(m, i, 1);
  }
      
  if ( dclAdd(pi_m, cl_m, m) < 0 )
    return 0;
  return 1;
}

/* no need to transpose! */
static int async_PartitionDCubeCoverMatrix(pinfo *pi_m, dclist cl_m, pinfo *pi, dclist cl_p, dclist cl_i)
{
  int i, cnt = dclCnt(cl_p);

  if ( dclSetPinfoByLength(pi_m, cl_i) == 0 )
    return 0;

  dclClear(cl_m);
  dclClearFlags(cl_m);

  for(i = 0; i < cnt; i++ )
  {
    if ( async_PartitionCoverElement(pi_m, cl_m, pi, dclGet(cl_p, i), cl_i) == 0 )
      return 0;
  }
  assert(dclCnt(cl_p) == dclCnt(cl_m));
  return 1;
}

static void ustt_do_msg(void (*msg)(void *data, char *fmt, va_list va), void *data, char *fmt, ...)
{
  va_list va;
  va_start(va, fmt);
  msg(data, fmt, va);
  va_end(va);
}

int dclMinimizeUSTT(pinfo *pi, dclist cl, void (*msg)(void *data, char *fmt, va_list va), void *data, const char *pre, const char *primes_file)
{
  dclist cl_p;
  dclist cl_m;
  pinfo pi_m;
  int i, j;
  dcube *cp = &(pi->tmp[1]);

  if ( dclInitVA(2, &cl_p, &cl_m) == 0 )
    return 0;
  if ( pinfoInit(&pi_m) == 0 )
    return dclDestroyVA(2, cl_p, cl_m), 0;
    
  if ( dclCopy(pi, cl_p, cl) == 0 )
    return dclDestroyVA(2, cl_p, cl_m), pinfoDestroy(&pi_m), 0;

  ustt_do_msg(msg, data, "%sCalculating prime partitions.", pre);
    
  if ( dclPrimesUSTT(pi, cl_p) == 0 )
    return dclDestroyVA(2, cl_p, cl_m), pinfoDestroy(&pi_m), 0;
    
  if ( primes_file != NULL )
  {
    ustt_do_msg(msg, data, "%sWriting prime partitions to PLA file '%s'.", 
      pre, primes_file);
    dclWritePLA(pi, cl_p, primes_file);
  }
    

  ustt_do_msg(msg, data, "%s%d prime partition%s.", 
    pre, dclCnt(cl_p), dclCnt(cl_p)==1?"":"s");

  /*
  if ( fsm != NULL )    
    fsm_LogDCLLev(fsm, 2, "FSM: ", 1, "Partition table", pi, cl_p);
  */

  ustt_do_msg(msg, data, "%sMinimal cover for prime partitions.", pre);
    
  if ( async_PartitionDCubeCoverMatrix(&pi_m, cl_m, pi, cl_p, cl) == 0 )
    return dclDestroyVA(2, cl_p, cl_m), pinfoDestroy(&pi_m), 0;

  ustt_do_msg(msg, data, "%sPartition cover matrix contains %d conditions.", pre, dclCnt(cl_m));

  /*
  if ( fsm != NULL )    
    fsm_LogDCLLev(fsm, 1, "FSM: ", 1, "Partition cover matrix", &pi_m, cl_m);
  */

  if ( maMatrixDCL(&pi_m, cl_m, 1, MA_LIT_NONE) == 0 )
    return dclDestroyVA(2, cl_p, cl_m), pinfoDestroy(&pi_m), 0;

  dclClearFlags(cl_p);
  for(i = 0; i < dclCnt(cl_m); i++)
  {
    if ( dclIsFlag(cl_m, i) == 0 )
      dclSetFlag(cl_p, i);
  }
  dclDeleteCubesWithFlag(pi, cl_p);

  /* verify the result: each condition must be satisfied */

  ustt_do_msg(msg, data, "%sChecking cover for prime partitions.", pre);
  for( i = 0; i < dclCnt(cl); i++ )
  {
    dcCopy(pi, cp, dclGet(cl, i));
    dcInvIn(pi, cp);
    for( j = 0; j < dclCnt(cl_p); j++ )
    {
      if ( dcIsSubSet(pi, dclGet(cl, i), dclGet(cl_p, j)) != 0 )
      {
        /*
        fsm_LogLev(fsm, 0, "FSM: Partition %3d (%s) satisfied by %d (%s).", 
          i, 
          dcToStr(fsm->pi_async, dclGet(fsm->cl_async, i), "", ""),
          j,
          dcToStr2(fsm->pi_async, dclGet(cl_p, j), "", "")
          );
        */
        break;
      }
      if ( dcIsSubSet(pi, cp, dclGet(cl_p, j)) != 0 )
      {
        /*
        fsm_LogLev(fsm, 0, "FSM: Partition %3d (%s) satisfied by %d (%s).", 
          i, 
          dcToStr(fsm->pi_async, cp, "", ""),
          j,
          dcToStr2(fsm->pi_async, dclGet(cl_p, j), "", "")
          );
        */
        break;
      }
    }
    if ( j >= dclCnt(cl_p) )
    {
      /*
      fsm_LogDCL(fsm, "FSM: ", 1, "Minimized partition table", fsm->pi_async, cl_p);
      */
      ustt_do_msg(msg, data, "%sInvald encoding (internal error, %d:%s not satisfied).", 
        pre, i, dcToStr(pi, dclGet(cl, i), " ", ""));
      return dclDestroyVA(2, cl_p, cl_m), pinfoDestroy(&pi_m), 0;
    }
  }


  if ( dclCopy(pi, cl, cl_p) == 0 )
    return dclDestroyVA(2, cl_p, cl_m), pinfoDestroy(&pi_m), 0;

  ustt_do_msg(msg, data, "%sSelection of prime partitions is valid (old: %d, new: %d).", 
    pre, dclCnt(cl), dclCnt(cl_p));

  return dclDestroyVA(2, cl_p, cl_m), pinfoDestroy(&pi_m), 1;
}


