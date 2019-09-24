/*

  mcov.c

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
#include "mcov.h"
#include <stdlib.h>
#include <assert.h>
#include "mwc.h"

int mcovInit(mcov *mc, pinfo *pi)
{
  *mc = malloc(sizeof(struct _mcov_struct));
  if ( *mc != NULL )
  {
    if ( dclInitVA(2, &((*mc)->cl_pr), &((*mc)->cl_matrix)) != 0 )
    {
      if ( pinfoInit(&((*mc)->pi_matrix)) != 0 )
      {
        if ( pinfoInit(&((*mc)->pi_select)) != 0 )
        {
          (*mc)->pi_pr = pi;
          (*mc)->is_select_init = 0;
          (*mc)->is_matrix_init = 0;
          (*mc)->cost_curr = 0;
          return 1;
        }
        pinfoDestroy(&((*mc)->pi_matrix));
      }
      dclDestroyVA(2, (*mc)->cl_pr, (*mc)->cl_matrix);
    }
    free(*mc);
  }
  return 0;
}

void mcovDestroyMatrix(mcov mc)
{
  if ( mc->is_matrix_init != 0 )
  {
    dclClear(mc->cl_matrix);
    dcDestroy(&(mc->matrix_or));
    mc->is_matrix_init = 0;
  }
} 


void mcovDestroySelect(mcov mc)
{
  if ( mc->is_select_init != 0 )
  {
    dcDestroy(&(mc->select_curr));
    dcDestroy(&(mc->select_opt));
    mc->is_select_init = 0;
  }
} 

void mcovDestroy(mcov mc)
{
  mcovDestroyMatrix(mc);
  mcovDestroySelect(mc);
  dclDestroyVA(2, mc->cl_pr, mc->cl_matrix);
  pinfoDestroy(&(mc->pi_select));
  pinfoDestroy(&(mc->pi_matrix));
  free(mc);
}

int mcovInitMatrix(mcov mc)
{
  mcovDestroyMatrix(mc);
  assert(mc->is_matrix_init == 0);
  if ( dcInit(&(mc->pi_matrix), &(mc->matrix_or)) != 0 )
  {
    mc->is_matrix_init = 1;
    return 1;
  }
  return 0;
  
}

int mcovInitSelect(mcov mc)
{
  mcovDestroySelect(mc);
  assert(mc->is_select_init == 0);
  if ( dclSetPinfoByLength(&(mc->pi_select), mc->cl_pr ) != 0 )
  {
    if ( dcInit(&(mc->pi_select), &(mc->select_curr)) != 0 )
    {
      if ( dcInit(&(mc->pi_select), &(mc->select_opt)) != 0 )
      {
        mc->is_select_init = 1;
        return 1;
      }
      dcDestroy(&(mc->select_curr));
    }
  }
  return 0;
}

void mcovInvertMatrix(mcov mc)
{
  int i, j, cnt = mc->pi_matrix.out_cnt;
  int v1, v2;
  /*
  puts("- 1 -");
  dclShow(&(mc->pi_matrix), mc->cl_matrix);
  */
  for( i = 0; i < cnt; i++ )
    for( j = 0; j < i; j++ )
    { 
      v1 = dcGetOut(dclGet(mc->cl_matrix, i), j);
      v2 = dcGetOut(dclGet(mc->cl_matrix, j), i);
      dcSetOut(dclGet(mc->cl_matrix, i), j, v2);
      dcSetOut(dclGet(mc->cl_matrix, j), i, v1);
    }
  /*
  puts("- 2 -");
  dclShow(&(mc->pi_matrix), mc->cl_matrix);
  */
}

int mcovIrredundantMatrix(mcov mc, dclist cl_es, dclist cl_pr, dclist cl_dc, dclist cl_rc)
{
  if ( dclIrredundantDCubeTMatrixRC(&(mc->pi_matrix), mc->cl_matrix, mc->pi_pr, cl_es, cl_pr, cl_dc, cl_rc) == 0 )
    return 0;
  if ( dclReduceDCubeTMatrix(&(mc->pi_matrix), mc->cl_matrix, mc->pi_pr, cl_pr) == 0 )
    return 0;
  if ( dclCopy(mc->pi_pr, mc->cl_pr, cl_pr) == 0 )
    return 0;
  if ( mcovInitMatrix(mc) == 0 )
    return 0;
  if ( mcovInitSelect(mc) == 0 )
    return 0;
  return 1;
}
/* ergibt 1, wenn alle cubes mindestens einmal ueberdeckt sind */
int mcovCalcOr(mcov mc, dcube *flags)
{
  int i;
  dcInSetAll(&(mc->pi_matrix), &(mc->matrix_or), CUBE_IN_MASK_DC);
  dcOutSetAll(&(mc->pi_matrix), &(mc->matrix_or), 0);
  for( i = 0; i < mc->pi_select.out_cnt; i++ )
  {
    if ( dcGetOut(flags, i) != 0 )
    {
      dcOr(&(mc->pi_matrix), &(mc->matrix_or), &(mc->matrix_or), dclGet(mc->cl_matrix, i));
    }
  }
  return dcIsTautology(&(mc->pi_matrix), &(mc->matrix_or));
}


int mcovReducePartialRedundantList(mcov mc, dclist cl_pr, dcube *flags)
{
  int i;
  if ( dclClearFlags(cl_pr) == 0 )
    return 0;
  for( i = 0; i < mc->pi_select.out_cnt; i++ )
  {
    if ( dcGetOut(flags, i) == 0 )
    {
      dclSetFlag(cl_pr, i);
    }
  }
  dclDeleteCubesWithFlag(mc->pi_pr, cl_pr);
  return 1;
}

static void mcovPushCurr(mcov mc, int pos)
{
  dcSetOut(&(mc->select_curr), pos, 1);
  mc->cost_curr += dclGet(mc->cl_pr, pos)->n;
}

static void mcovPopCurr(mcov mc, int pos)
{
  dcSetOut(&(mc->select_curr), pos, 0);
  mc->cost_curr -= dclGet(mc->cl_pr, pos)->n;
}

void mcovCopyCurrToOpt(mcov mc)
{
  dcCopy(&(mc->pi_select), &(mc->select_opt), &(mc->select_curr));
  mc->cost_opt = mc->cost_curr;
  /*
  printf("%7d %3d  %s\n", clock()/1000, mc->cost_curr, 
    dcToStr(&(mc->pi_select), &(mc->select_opt), " ", ""));
  */
}

int mcovExactSub(mcov mc, int pos, int depth, dcube *prev_or)
{ 
  dcube *curr_or = &(mc->pi_matrix.stack1[depth]);
  dcube *curr;
  int i;
  int is_tautology;
  int out_words = mc->pi_matrix.out_words;

  dcInSetAll(&(mc->pi_matrix), curr_or, CUBE_IN_MASK_DC);

  while(pos < mc->pi_select.out_cnt)
  {
    mc->cost_curr += dclGet(mc->cl_pr, pos)->n;
    if ( mc->cost_curr < mc->cost_opt  )
    {
      curr = dclGet(mc->cl_matrix, pos);

      for( i = 0; i < out_words; i++ )
      {
        if ( (curr->out[i] | prev_or->out[i]) != prev_or->out[i] )
          break;
      }

      /* nur wenn sich etwas aendern wuerde */
      if ( i < out_words )
      {
        is_tautology = 1; 
        for( i = 0; i < out_words; i++ )
        {
          curr_or->out[i] = curr->out[i] | prev_or->out[i];
          if ( curr_or->out[i] != CUBE_OUT_MASK )
            is_tautology = 0;
        }
        /* Wenn tautologie (also wenn alle zeilen mindestens eine 1 haben) */
        if ( is_tautology != 0 )
        {
          if ( mc->cost_opt > mc->cost_curr )
          {
            dcSetOut(&(mc->select_curr), pos, 1);
            mcovCopyCurrToOpt(mc);
            dcSetOut(&(mc->select_curr), pos, 0);
          }
        }
        else
        {
          dcSetOut(&(mc->select_curr), pos, 1);
          if ( mcovExactSub(mc, pos+1, depth+1, curr_or) == 0 )
            return 0;
          dcSetOut(&(mc->select_curr), pos, 0);
        }
      }
    }

    mc->cost_curr -= dclGet(mc->cl_pr, pos)->n;
    pos++;
  }
  return 1;
}

int mcovExactStart(mcov mc)
{
  int i;
  dcube *curr_or = &(mc->pi_matrix.stack1[0]);
  dcInSetAll(&(mc->pi_matrix), curr_or, CUBE_IN_MASK_DC);
  dcOutSetAll(&(mc->pi_matrix), curr_or, 0);
  curr_or->out[mc->pi_matrix.out_words-1] |= ~mc->pi_matrix.out_last_mask;
  for( i = 0; i < dclCnt(mc->cl_pr); i++ )
  {
    dclGet(mc->cl_pr, i)->n = mc->pi_pr->in_cnt-dcInDCCnt(mc->pi_pr, dclGet(mc->cl_pr, i));
    /*
    if ( dcOutCnt(&(mc->pi_matrix), dclGet(mc->cl_matrix, i)) == 0 )
      dclGet(mc->cl_pr, i)->n = 0;
    */
  }
  mc->cost_curr = 0;
  mc->cost_opt = ~(unsigned int)0;
  if ( mcovExactSub(mc, 0, 1, curr_or) == 0 )
    return 0;
  if ( mc->cost_opt == ~(unsigned int)0 )
    return 0;
  return 1;
}


int mcovExact(mcov mc, dclist cl_es, dclist cl_pr, dclist cl_dc, dclist cl_rc)
{
  if ( dclCnt(cl_pr) <= 1 )
    return 1; /* es gibt nix zu tun */
  if ( mcovIrredundantMatrix(mc, cl_es, cl_pr, cl_dc, cl_rc) == 0 )
    return 0;
  if ( mcovExactStart(mc) == 0 )
    return 0;
  return mcovReducePartialRedundantList(mc, cl_pr, &(mc->select_opt));
}


/* diese routine erfordert vorher ZWINGEND den aufruf von dclSplitRelativeEssential */
/* cl_pr darf keine relativ essentiellen cubes enthalten */
int dclIrredundantExact(pinfo *pi, dclist cl_es, dclist cl_pr, dclist cl_dc, dclist cl_rc)
{
  mcov mc;
  if ( dclCnt(cl_pr) <= 1 )
    return 1; /* es gibt nix zu tun */
  if ( mcovInit(&mc, pi) == 0 )
    return 0;
  if ( mcovExact(mc, cl_es, cl_pr, cl_dc, cl_rc) ==0 )
    return mcovDestroy(mc), 0;
  return mcovDestroy(mc), 1;
}



/* findet eine spalte, die maximal viele 1'er hinzufuegt */
int mcovFindMaxAdditionalCol(mcov mc, dcube *prev_or)
{
  int i;
  int out_cnt_opt = 0;
  int position_opt = -1;
  int out_cnt_curr;
  dcube *or = &(mc->pi_matrix.tmp[1]);
  for( i = 0; i < mc->pi_select.out_cnt; i++ )
  {
    if ( dcGetOut(&(mc->select_curr), i) == 0 )
    {
      dcOr(&(mc->pi_matrix), or, dclGet(mc->cl_matrix, i), prev_or);
      out_cnt_curr = dcOutCnt(&(mc->pi_matrix), or);
      if ( out_cnt_opt < out_cnt_curr )
      {
        out_cnt_opt = out_cnt_curr;
        position_opt = i;
      }
    }
  }
  return position_opt;
}

/* --> section 37.3, p974 und sec 17, p329 im Corman, Leiserson, Rivest */
int mcovGreedy(mcov mc, dclist cl_es, dclist cl_pr, dclist cl_dc, dclist cl_rc)
{
  int pos;
  dcube *or = &(mc->pi_matrix.tmp[2]);
  
  if ( dclCnt(cl_pr) <= 1 )
    return 1; /* es gibt nix zu tun */
  if ( mcovIrredundantMatrix(mc, cl_es, cl_pr, cl_dc, cl_rc) == 0 )
    return 0;
  
  dcInSetAll(&(mc->pi_matrix), or, CUBE_IN_MASK_DC);
  dcOutSetAll(&(mc->pi_matrix), or, 0);

  dcInSetAll(&(mc->pi_select), &(mc->select_curr), CUBE_IN_MASK_DC);
  dcOutSetAll(&(mc->pi_select), &(mc->select_curr), 0);
  
  for(;;)
  {
    pos = mcovFindMaxAdditionalCol(mc, or);
    if ( pos < 0 )
      break;
    dcSetOut(&(mc->select_curr), pos, 1);
    dcOr(&(mc->pi_matrix), or, or, dclGet(mc->cl_matrix, pos));
    if ( dcIsTautology(&(mc->pi_matrix), or) != 0 )
      break;
  }
  
  if ( pos < 0 )
    return 1;
 
  return mcovReducePartialRedundantList(mc, cl_pr, &(mc->select_curr));
}

/* diese routine erfordert vorher ZWINGEND den aufruf von dclSplitRelativeEssential */
/* cl_pr darf keine relativ essentiellen cubes enthalten */
int dclIrredundantGreedy(pinfo *pi, dclist cl_es, dclist cl_pr, dclist cl_dc, dclist cl_rc)
{
  mcov mc;
  if ( dclCnt(cl_pr) <= 1 )
    return 1; /* es gibt nix zu tun */
  if ( mcovInit(&mc, pi) == 0 )
    return 0;
  if ( mcovGreedy(mc, cl_es, cl_pr, cl_dc, cl_rc) ==0 )
    return mcovDestroy(mc), 0;
  return mcovDestroy(mc), 1;
}

