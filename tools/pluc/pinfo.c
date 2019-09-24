/*

  pinfo.c
  
  problem information
  
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


#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include "pinfo.h"
#include "dcube.h"
#include "mwc.h"
#include "b_io.h"

/*-- bitcnt -----------------------------------------------------------------*/


/*
int count_bit01(unsigned x)
{
  if ( x == 0 ) return 0;
  if ( (x & (x-1)) == 0 ) return 1;
  return 2;
}
#define count_bit01(x) ((x==0)?(0):(((x & (x-1))==0)?1:2))
*/


/* BEGIN_LATEX bitcount.tex */
int bitcount(c_int x)
{
  register int cnt1 = 0;
  register int cnt2 = 0;
  register int i = SIZEOF_INT*8/4;
  register unsigned a = x;
  register unsigned b = x>>(SIZEOF_INT*8/2);
  register unsigned mask;
  
  mask = 1 + (1<<(SIZEOF_INT*8/4));
  
  while( i > 0 )
  {  
    cnt1 += a&mask;
    cnt2 += b&mask;
    a>>=1;
    b>>=1;
    i--;
  }
  return (cnt1&255)+(cnt2&255)+(cnt1>>(SIZEOF_INT*8/4))+(cnt2>>(SIZEOF_INT*8/4));
}
/* END_LATEX */

/*-- pinfoInit --------------------------------------------------------------*/

int pinfoInitInOut(pinfo *pi, int in, int out)
{
  int i;
  pi->split = NULL;
  pi->progress = NULL;
  
  pi->in_cnt = 0;
  pi->out_cnt = 0;
  pi->in_words = 0;
  pi->out_words = 0;
  pi->in_out_words_min = 0;
  pi->in_last_mask = 0;
  pi->out_last_mask = CUBE_OUT_MASK;
  pi->cache_cnt = 0;
  for( i = 0; i < PINFO_CACHE_CL; i++ )
  {
    pi->cache_cl[i] = NULL;
  }
  
  if ( dclInit(&(pi->cl_u)) == 0 )
    return 0;

  pi->in_cnt = in;
  pi->in_words = (in+CUBE_SIGNALS_PER_IN_WORD-1)/CUBE_SIGNALS_PER_IN_WORD;
  pi->in_last_mask = (((c_int)1)<<((in%CUBE_SIGNALS_PER_IN_WORD)*2))-1;

  pi->out_cnt = out;
  pi->out_words = (out+CUBE_SIGNALS_PER_OUT_WORD-1)/CUBE_SIGNALS_PER_OUT_WORD;
  if ( (out%CUBE_SIGNALS_PER_OUT_WORD) != 0 )
    pi->out_last_mask = (((c_int)1)<<((out%CUBE_SIGNALS_PER_OUT_WORD)))-1;
  else
    pi->out_last_mask = CUBE_OUT_MASK;

  if ( pi->in_words < pi->out_words )
    pi->in_out_words_min = pi->in_words;
  else
    pi->in_out_words_min = pi->out_words;


  for( i = 0; i < PINFO_TMP_CUBES; i++ )
    if ( dcInit(pi, &(pi->tmp[i])) == 0 )
    {
      while( i > 0 )
      {
        i--;
        dcDestroy(&(pi->tmp[i]));
      }
      dclDestroy(pi->cl_u);
      return 0;
    }

  dcSetTautology(pi, &(pi->tmp[0]));
    
  pi->stack1 = pi->tmp+PINFO_USER_CUBES;
  pi->stack2 = pi->tmp+PINFO_USER_CUBES+PINFO_STACK_CUBES;
  pi->in_sl = NULL;
  pi->out_sl = NULL;
  return 1;
}

int pinfoInit(pinfo *pi)
{
  return pinfoInitInOut(pi, 0, 0);
}

/*
  init a pinfo structure with a different (source) pinfo structure
  Only the selected output var is taken over
  Additionally a cl_dest_ptr is init and updated
  
*/
int pinfoInitFromOutVar(pinfo *pi_dest, dclist *cl_dest_ptr, pinfo *pi_src, dclist cl_src, int out_var)
{
  int i, cnt;
  dclist cl_dest;
  dcube *c;
  
  /* init cube list and pinfo */
  
  if ( dclInit(cl_dest_ptr) == 0 )
    return 0;
  cl_dest = *cl_dest_ptr;
  if ( pinfoInitInOut(pi_dest, pi_src->in_cnt, 0) == 0 )
    return dclDestroy(cl_dest), 0;
  
  /* assign in and out labels */
  
  if ( pinfoCopyInLabels(pi_dest, pinfoGetInLabelList(pi_src)) == 0 )
    return pinfoDestroy(pi_dest), dclDestroy(cl_dest), 0;
  if ( pinfoAddOutLabel(pi_dest, pinfoGetOutLabel(pi_src, out_var))  < 0 )
    return pinfoDestroy(pi_dest), dclDestroy(cl_dest), 0;

  /* copy the cubes and adjust the output */
  
  cnt = dclCnt(cl_src);
  for( i = 0; i < cnt; i++ )
  {
    if ( dcGetOut(dclGet(cl_src, i), out_var) != 0 )
    {
      c =dclAddEmptyCube(pi_dest, cl_dest);
      if ( c == NULL )
	return pinfoDestroy(pi_dest), dclDestroy(cl_dest), 0;
      dcCopyIn(pi_dest, c, dclGet(cl_src, i));
      dcSetOut( c, 0, 1);
    }
  }
  
  return 1;
}



/*-- pinfoDestroySplit ------------------------------------------------------*/
void pinfoDestroySplit(pinfo *pi)
{
  int i;

  if ( pi->split == NULL )
    return;

  if ( pi->split->in_half_bit_cnt != NULL )
    free(pi->split->in_half_bit_cnt);

  if ( pi->split->out_bit_cnt != NULL )
    free(pi->split->out_bit_cnt);

  if ( pi->split->out_cur_select != NULL )
    free(pi->split->out_cur_select);

  pi->split->in_half_bit_cnt = NULL;
  pi->split->out_bit_cnt = NULL;
  pi->split->out_cur_select = NULL;
  
  for( i = 0; i < PINFO_OUT_SOL_CNT; i++ )
  {
    if ( pi->split->out_opt_select[i] != NULL )
      free(pi->split->out_opt_select[i]);
    pi->split->out_opt_select[i] = NULL;
  }
  
  free(pi->split);
  pi->split = NULL;
}


/*-- pinfoDestroyProgress ---------------------------------------------------*/
void pinfoDestroyProgress(pinfo *pi)
{
  if ( pi->progress == NULL )
    return;
  free(pi->progress);
  pi->progress = NULL;
  return;
}


/*-- pinfoInitSplit ---------------------------------------------------------*/
int pinfoInitSplit(pinfo *pi)
{
  int i;
  if ( pi->split != NULL )
    return 1;
    
  pi->split = (pinfo_split_struct *)malloc(sizeof(pinfo_split_struct));
  if ( pi->split == NULL )
    return 0;
  
  pi->split->in_half_bit_cnt = NULL;
  pi->split->out_bit_cnt = NULL;
  pi->split->out_cur_select = NULL;

  for( i = 0; i < PINFO_OUT_SOL_CNT; i++ )
    pi->split->out_opt_select[i] = NULL;

  pi->split->in_half_bit_cnt = malloc(sizeof(unsigned)*pi->in_words*CUBE_SIGNALS_PER_IN_WORD*2);
  if ( pi->split->in_half_bit_cnt == NULL )
  {
    pinfoDestroySplit(pi);
    return 0;
  }

  
  if (  pi->out_cnt != 0 )
  {
    pi->split->out_bit_cnt = malloc(sizeof(unsigned)*pi->out_cnt);
    pi->split->out_cur_select = malloc(sizeof(int)*pi->out_cnt);
    if ( pi->split->out_bit_cnt == NULL || 
         pi->split->out_cur_select == NULL )
    {
      pinfoDestroySplit(pi);
      return 0;
    }

    for( i = 0; i < PINFO_OUT_SOL_CNT; i++ )
    {
      pi->split->out_opt_select[i] = malloc(sizeof(int)*pi->out_cnt);
      if ( pi->split->out_opt_select[i] == NULL )
      {
        pinfoDestroySplit(pi);
        return 0;
      }
    }
  }

  return 1;
}

/*-- pinfoInitProgress ------------------------------------------------------*/
int pinfoInitProgress(pinfo *pi)
{ 
  if ( pi->progress != NULL )
    return 1;
  pi->progress = malloc(sizeof(pinfo_progress_struct));
  if ( pi->progress == NULL ) 
    return 0;
  pi->progress->proc_depth = 0;
  return 1;
}

void pinfoDestroyCache(pinfo *pi)
{
  int i;
  for( i = 0; i < pi->cache_cnt; i++ )
  {
    if ( pi->cache_cl[i] != NULL )
      dclDestroy(pi->cache_cl[i]);
  }
  pi->cache_cnt = 0;
}


/*-- pinfoDestroy -----------------------------------------------------------*/

void pinfoDestroy(pinfo *pi)
{
  int i;
  pinfoDestroySplit(pi);
  pinfoDestroyProgress(pi);
  pinfoDestroyCache(pi);
  pi->in_cnt = 0;
  pi->out_cnt = 0;
  for( i = 0; i < PINFO_TMP_CUBES; i++ )
    dcDestroy(&(pi->tmp[i]));
  dclDestroy(pi->cl_u);
  if ( pi->in_sl != NULL )
    b_sl_Close(pi->in_sl);
  if ( pi->out_sl != NULL )
    b_sl_Close(pi->out_sl);
    
}

/*-- pinfoOpen --------------------------------------------------------------*/

pinfo *pinfoOpen()
{
  pinfo *pi;
  pi = malloc(sizeof(struct _pinfo_struct));
  if ( pi != NULL )
  {
    if ( pinfoInit(pi) != 0 )
    {
      return pi;
    }
    free(pi);
  }
  return NULL;
}

pinfo *pinfoOpenInOut(int in, int out)
{
  pinfo *pi;
  pi = malloc(sizeof(struct _pinfo_struct));
  if ( pi != NULL )
  {
    if ( pinfoInitInOut(pi, in, out) != 0 )
    {
      return pi;
    }
    free(pi);
  }
  return NULL;
}

/*-- pinfoClose -------------------------------------------------------------*/

void pinfoClose(pinfo *pi)
{
  pinfoDestroy(pi);
  free(pi);
}


/*-- pinfoSetInCnt ----------------------------------------------------------*/

int pinfoSetInCnt(pinfo *pi, int in)
{
  int i;
  pinfoDestroySplit(pi);
  pinfoDestroyCache(pi);

  if ( pi->in_cnt == in )
    return 1;
    
  pi->in_cnt = in;
  pi->in_words = (in+CUBE_SIGNALS_PER_IN_WORD-1)/CUBE_SIGNALS_PER_IN_WORD;
  pi->in_last_mask = (((c_int)1)<<((in%CUBE_SIGNALS_PER_IN_WORD)*2))-1;
  for( i = 0; i < PINFO_TMP_CUBES; i++ )
    if ( dcAdjustByPinfo(pi, &(pi->tmp[i])) == 0 )
      return 0;  

  if ( pi->in_words < pi->out_words )
    pi->in_out_words_min = pi->in_words;
  else
    pi->in_out_words_min = pi->out_words;

  dcSetTautology(pi, &(pi->tmp[0]));
  dclRealClear(pi->cl_u);
  if ( dclAdd(pi, pi->cl_u, &(pi->tmp[0])) < 0 )
    return 0;
  

  return 1;
}


/*-- pinfoSetOutCnt ---------------------------------------------------------*/

int pinfoSetOutCnt(pinfo *pi, int out)
{
  int i;
  pinfoDestroySplit(pi);
  pinfoDestroyCache(pi);

  if ( pi->out_cnt == out )
    return 1;
  
  pi->out_cnt = out;
  pi->out_words = (out+CUBE_SIGNALS_PER_OUT_WORD-1)/CUBE_SIGNALS_PER_OUT_WORD;
  if ( (out%CUBE_SIGNALS_PER_OUT_WORD) != 0 )
    pi->out_last_mask = (((c_int)1)<<((out%CUBE_SIGNALS_PER_OUT_WORD)))-1;
  else
    pi->out_last_mask = CUBE_OUT_MASK;
  for( i = 0; i < PINFO_TMP_CUBES; i++ )
    if ( dcAdjustByPinfo(pi, &(pi->tmp[i])) == 0 )
      return 0;
  dcSetTautology(pi, &(pi->tmp[0]));
  
  dclRealClear(pi->cl_u);
  if ( dclAdd(pi, pi->cl_u, &(pi->tmp[0])) < 0 )
    return 0;
    
  if ( pi->in_words < pi->out_words )
    pi->in_out_words_min = pi->in_words;
  else
    pi->in_out_words_min = pi->out_words;
  return 1;
}

/*-- pinfoAddInLabel ---------------------------------------------------------*/
/* add a in label, if it does not yet exist. return index to it */
int pinfoAddInLabel(pinfo *pi, const char *s)
{
  int pos = -1;

  if ( pi->in_sl == NULL )
    pi->in_sl = b_sl_Open();
  
  /* check whether the label already exists */
  pos = b_sl_Find(pi->in_sl, s);
  if ( pos >= 0 )
    return pos;		/* yes, it exists, return the position */
  
  /* define the new index for the label */
  pos = pi->in_cnt;
  
  /* add signal to the pinfo structure */  
  if ( pinfoSetInCnt(pi, pos + 1) ==  0 )
    return -1;
  
  /* append the new label to the label list */
  pos = b_sl_Add(pi->in_sl, s);
  if ( pos < 0 )
    return pinfoSetInCnt(pi, pos), -1;	/* alloc error */
  
  return pos;
}
/*-- pinfoDeleteInLabel ---------------------------------------------------------*/
void pinfoDeleteInLabel(pinfo *pi, int pos)
{
  if ( pi->in_sl == NULL )
    return;		/* do nothing */
  if ( pi->in_cnt <= pos )
    return;		/* do nothing */
  b_sl_Del(pi->in_sl, pos);
  pinfoSetInCnt(pi, pi->in_cnt - 1);
}


/*-- pinfoAddOutLabel ---------------------------------------------------------*/
/* add a out label, if it does not yet exist. return index to it */
int pinfoAddOutLabel(pinfo *pi, const char *s)
{
  int pos = -1;

  if ( pi->out_sl == NULL )
    pi->out_sl = b_sl_Open();
  
  /* check whether the label already exists */
  pos = b_sl_Find(pi->out_sl, s);
  if ( pos >= 0 )
    return pos;		/* yes, it exists, return the position */
  
  /* define the new index for the label */
  pos = pi->out_cnt;
  
  /* add signal to the pinfo structure */  
  if ( pinfoSetOutCnt(pi, pos + 1) ==  0 )
    return -1;
  
  /* append the new label to the label list */
  pos = b_sl_Add(pi->out_sl, s);
  if ( pos < 0 )
    return pinfoSetOutCnt(pi, pos), -1;	/* alloc error */
  
  return pos;
}


/*-- pinfoGetInLabelList ----------------------------------------------------*/

b_sl_type pinfoGetInLabelList(pinfo *pi)
{
  if ( pi->in_sl == NULL )
  {
    pi->in_sl = b_sl_Open();
  }
  return pi->in_sl;
}

/*-- pinfoGetOutLabelList ---------------------------------------------------*/

b_sl_type pinfoGetOutLabelList(pinfo *pi)
{
  if ( pi->out_sl == NULL )
  {
    pi->out_sl = b_sl_Open();
  }
  return pi->out_sl;
}

/*-- pinfoImportInLabels ----------------------------------------------------*/

int pinfoImportInLabels(pinfo *pi, const char *s, const char *delim)
{
  b_sl_type sl;
  sl = pinfoGetInLabelList(pi);
  if ( sl == NULL )
    return 0;
  b_sl_Clear(sl);
  if ( b_sl_ImportByStr(sl, s, delim, "") < 0 )
    return 0;
  return 1;
}

/*-- pinfoImportOutLabels ---------------------------------------------------*/

int pinfoImportOutLabels(pinfo *pi, const char *s, const char *delim)
{
  b_sl_type sl;
  sl = pinfoGetOutLabelList(pi);
  if ( sl == NULL )
    return 0;
  b_sl_Clear(sl);
  if ( b_sl_ImportByStr(sl, s, delim, "") < 0 )
    return 0;
  return 1;
}

/*-- pinfoCopyInLabels ------------------------------------------------------*/

int pinfoCopyInLabels(pinfo *pi, b_sl_type sl)
{ 
  b_sl_type dest;
  if ( sl == NULL )
  {
    if ( pi->in_sl != NULL )
      b_sl_Clear(pi->in_sl);
    return 1;
  }
  dest = pinfoGetInLabelList(pi);
  if ( dest == NULL )
    return 0;
  if ( b_sl_Copy(dest, sl) == 0 )
    return 0;
  return 1;
}

/*-- pinfoCopyOutLabels -----------------------------------------------------*/

int pinfoCopyOutLabels(pinfo *pi, b_sl_type sl)
{ 
  b_sl_type dest;
  if ( sl == NULL )
  {
    if ( pi->out_sl != NULL )
      b_sl_Clear(pi->out_sl);
    return 1;
  }
  dest = pinfoGetOutLabelList(pi);
  if ( dest == NULL )
    return 0;
  if ( b_sl_Copy(dest, sl) == 0 )
    return 0;
  return 1;
}

/*-- pinfoCopyLabels --------------------------------------------------------*/

int pinfoCopyLabels(pinfo *dest_pi, pinfo *src_pi)
{
  if ( pinfoCopyInLabels(dest_pi, src_pi->in_sl) == 0 )
    return 0;
  if ( pinfoCopyOutLabels(dest_pi, src_pi->out_sl) == 0 )
    return 0;
  return 1;
}

/*-- pinfoGetInLabel --------------------------------------------------------*/

const char *pinfoGetInLabel(pinfo *pi, int pos)
{
  if ( pi->in_sl == NULL )
    return NULL;
  if ( pos < 0 || pos >= b_sl_GetCnt(pi->in_sl) )
    return NULL;
  return b_sl_GetVal(pi->in_sl, pos);
}

/*-- pinfoSetInLabel --------------------------------------------------------*/

/* returns 0 in case of an error */
int pinfoSetInLabel(pinfo *pi, int pos, const char *s)
{
  if ( pi->in_sl == NULL )
    return 0;
  if ( pos < 0 || pos >= b_sl_GetCnt(pi->in_sl) )
    return 0;
  return b_sl_Set(pi->in_sl, pos, s);
}

/*-- pinfoGetOutLabel -------------------------------------------------------*/

const char *pinfoGetOutLabel(pinfo *pi, int pos)
{
  if ( pi->out_sl == NULL )
    return NULL;
  if ( pos < 0 || pos >= b_sl_GetCnt(pi->out_sl) )
    return NULL;
  return b_sl_GetVal(pi->out_sl, pos);
}

/*-- pinfoSetOutLabel --------------------------------------------------------*/

/* returns 0 in case of an error */
int pinfoSetOutLabel(pinfo *pi, int pos, const char *s)
{
  if ( pi->out_sl == NULL )
    return 0;
  if ( pos < 0 || pos >= b_sl_GetCnt(pi->out_sl) )
    return 0;
  return b_sl_Set(pi->out_sl, pos, s);
}

/*-- pinfoFindInLabelPos ----------------------------------------------------*/

int pinfoFindInLabelPos(pinfo *pi, const char *s)
{
 if ( pi->in_sl == NULL )
   return -1;
 return b_sl_Find(pi->in_sl, s);
}

/*-- pinfoFindOutLabelPos ---------------------------------------------------*/

int pinfoFindOutLabelPos(pinfo *pi, const char *s)
{
  if ( pi->out_sl == NULL )
    return -1;
  return b_sl_Find(pi->out_sl, s);
}

/*-- pinfoCopy --------------------------------------------------------*/
/* copy the problem info to the destiion info */
int pinfoCopy(pinfo *dest, pinfo *src)
{
  if ( pinfoSetInCnt(dest, pinfoGetInCnt(src)) == 0 )
    return 0;
  if ( pinfoSetOutCnt(dest, pinfoGetOutCnt(src)) == 0 )
    return 0;
  if ( pinfoCopyLabels(dest, src) == 0 )
    return 0;
  return 1;
}


/*-- pinfoProcessOut --------------------------------------------------------*/
void pinfoProcessOut(pinfo *pi)
{
  int i;
  if ( pi->progress == NULL )
    return;
  for( i = 0; i < pi->progress->proc_depth; i++ )
    fprintf(stderr, "%s ", pi->progress->proc_name[i]);
  fprintf(stderr, "                                 \r"); fflush(stderr);
}

/*-- pinfoProcedureInit -----------------------------------------------------*/
void pinfoProcedureInit(pinfo *pi, char *fn_name, int max)
{
  if ( pi->progress == NULL )
    return;
  strncpy(pi->progress->procedure_fn_name, fn_name, 22);
  pi->progress->procedure_fn_name[21] = '\0';
  pi->progress->procedure_max = max;
  pi->progress->proc_depth++;
}

/*-- pinfoProcedureFinish ---------------------------------------------------*/
void pinfoProcedureFinish(pinfo *pi)
{
  if ( pi->progress == NULL )
    return;
  pi->progress->proc_depth--;
}

/*-- pinfoProcedureDo -------------------------------------------------------*/

int pinfoProcedureDo(pinfo *pi, int curr)
{
  if ( pi->progress == NULL )
    return 1;
  sprintf(pi->progress->proc_name[pi->progress->proc_depth-1], 
    "%20s %6d/%6d", pi->progress->procedure_fn_name, curr, pi->progress->procedure_max); 
  pinfoProcessOut(pi);
  fflush(stderr);
  return 1;
}

/*-- pinfoDepth -------------------------------------------------------------*/

int pinfoDepth(pinfo *pi, char *fn_name, int depth)
{
  if ( pi->progress == NULL )
    return 1;
  fprintf(stderr, "%20s %6d                  \r", fn_name, depth); fflush(stderr);
  return 1;
}

/*-- pinfoBTreeInit ---------------------------------------------------------*/

#define PINFO_BTREE_CALC_DEPTH 12

void pinfoBTreeInit(pinfo *pi, char *fn_name)
{
  if ( pi->progress == NULL )
    return;
  strncpy(pi->progress->btree_fn_name, fn_name, 22);
  pi->progress->btree_fn_name[21] = '\0';
  pi->progress->btree_cnt = (1<<PINFO_BTREE_CALC_DEPTH)-1;
  pi->progress->btree_depth = 0;
  pi->progress->proc_depth++;

  sprintf(pi->progress->proc_name[pi->progress->proc_depth-1], 
    "%20s %3d.%1d%%", pi->progress->btree_fn_name, 0, 0);
  pinfoProcessOut(pi);
}

/*-- pinfoBTreeFinish -------------------------------------------------------*/

void pinfoBTreeFinish(pinfo *pi)
{
  if ( pi->progress == NULL )
    return;
  pi->progress->proc_depth--;
}

/*-- pinfoBTreeStart --------------------------------------------------------*/

int pinfoBTreeStart(pinfo *pi)
{
  if ( pi->progress == NULL )
    return 1;
  if ( pi->progress->btree_depth < PINFO_BTREE_CALC_DEPTH && pi->progress->btree_depth > 0)
    pi->progress->btree_cnt ^= ((unsigned int)1)<<(PINFO_BTREE_CALC_DEPTH-pi->progress->btree_depth);
  pi->progress->btree_depth++;
  return 1;
}

/*-- pinfoBTreeEnd ----------------------------------------------------------*/

void pinfoBTreeEnd(pinfo *pi)
{
  int pm;
  if ( pi->progress == NULL )
    return;
  pm = (pi->progress->btree_cnt*1000)>>PINFO_BTREE_CALC_DEPTH;
  pi->progress->btree_depth--;
  if ( pi->progress->btree_depth < PINFO_BTREE_CALC_DEPTH )
  {
    sprintf(pi->progress->proc_name[pi->progress->proc_depth-1], 
      "%20s %3d.%1d%%", pi->progress->btree_fn_name, pm/10, pm%10);
    
    pinfoProcessOut(pi);
  }
}


/*-- pinfoClearCnt ----------------------------------------------------------*/

int pinfoClearCnt(pinfo *pi)
{
  int i, j, cnt;
  if ( pinfoInitSplit(pi) == 0 )
    return 0;
  cnt = pi->in_words*CUBE_SIGNALS_PER_IN_WORD*2;
  for( i = 0; i < cnt-8; i+=8 )
  {
    pi->split->in_half_bit_cnt[i+0] = 0;
    pi->split->in_half_bit_cnt[i+1] = 0;
    pi->split->in_half_bit_cnt[i+2] = 0;
    pi->split->in_half_bit_cnt[i+3] = 0;
    pi->split->in_half_bit_cnt[i+4] = 0;
    pi->split->in_half_bit_cnt[i+5] = 0;
    pi->split->in_half_bit_cnt[i+6] = 0;
    pi->split->in_half_bit_cnt[i+7] = 0;
  }
  for( ; i < cnt; i++ )
    pi->split->in_half_bit_cnt[i] = 0;
    
  for( i = 0; i < pi->out_cnt; i++ )
  {
    pi->split->out_bit_cnt[i] = 0;
    pi->split->out_cur_select[i] = 0;
  }
  for( j = 0; j < PINFO_OUT_SOL_CNT; j++ )
  {
    pi->split->out_opt_select_cnt[j] = 0;
    for( i = 0; i < pi->out_cnt; i++ )
    {
      pi->split->out_opt_select[j][i] = 0;
    }
  }
  return 1;
}

/*-- pinfoDCLCnt ------------------------------------------------------------*/

void pinfoCntDCube(pinfo *pi, dcube *c)
{
  int i;
  c_int mask;
  unsigned *ptr;
  ptr = pi->split->in_half_bit_cnt;
  for( i = 0; i < pi->in_words; i++ )
  {
    mask = 1;
    while( mask != 0 )
    {
      if ( (c->in[i] & mask) != 0 )
        (*ptr)++;
      ptr++;
      mask<<=1;
    }
  }
}

void pinfoCntDCube8(pinfo *pi, dcube *c1, dcube *c2, dcube *c3, dcube *c4, dcube *c5, dcube *c6, dcube *c7, dcube *c8)
{
  register int i, j;
  register c_int mask;
  register c_int sum;
  register c_int i1, i2, i3, i4, i5, i6, i7, i8;
  unsigned *ptr;
  ptr = pi->split->in_half_bit_cnt;
  for( i = 0; i < pi->in_words; i++ )
  {
    mask = 1;
    i1 = c1->in[i];
    i2 = c2->in[i];
    i3 = c3->in[i];
    i4 = c4->in[i];
    i5 = c5->in[i];
    i6 = c6->in[i];
    i7 = c7->in[i];
    i8 = c8->in[i];
    for( j = 0; j < CUBE_SIGNALS_PER_OUT_WORD; j++ )
    {
      sum = 0;
      sum += i1 & mask;
      i1 >>= 1;
      sum += i2 & mask;
      i2 >>= 1;
      sum += i3 & mask;
      i3 >>= 1;
      sum += i4 & mask;
      i4 >>= 1;
      sum += i5 & mask;
      i5 >>= 1;
      sum += i6 & mask;
      i6 >>= 1;
      sum += i7 & mask;
      i7 >>= 1;
      sum += i8 & mask;
      i8 >>= 1;
      (*ptr) += sum;
      ptr++;
    }
  }
}

/*-- pinfoCntDCList ---------------------------------------------------------*/

/* 
  Creates a distribution on the values for each variable over all cubes.
  The distribution is stored in pi
  Use pinfoGetZeroCnt, pinfoGetOneCnt and pinfoGetDCCnt after calling pinfoCntDCList
  pinfoIsBinateInVar, pinfoGetMinDCCntVar
*/
int pinfoCntDCList(pinfo *pi, dclist cl, dcube *cof)
{
  int j, i, cnt = dclCnt(cl);
  unsigned *ptr;
  c_int mask;
  
  if ( pinfoClearCnt(pi) == 0 )
    return 0;

  for( i = 0; i < cnt-8; i+=8 )
  {
    pinfoCntDCube8(pi, dclGet(cl, i+0), dclGet(cl, i+1), dclGet(cl, i+2), dclGet(cl, i+3), 
                       dclGet(cl, i+4), dclGet(cl, i+5), dclGet(cl, i+6), dclGet(cl, i+7));
  }
  
  for( ; i < cnt; i++ )
    pinfoCntDCube(pi, dclGet(cl, i));

  ptr = pi->split->out_bit_cnt;
  for( j = 0; j < pi->out_words; j++ )
  {
    mask = 1;
    while( mask != 0 )
    {
      if ( (cof->out[j] & mask) != 0 )
      {
        for( i = 0; i < cnt-4; i+=4 )
        {
          if ( (dclGet(cl, i+0)->out[j] & mask) != 0 )
            (*ptr)++;
          if ( (dclGet(cl, i+1)->out[j] & mask) != 0 )
            (*ptr)++;
          if ( (dclGet(cl, i+2)->out[j] & mask) != 0 )
            (*ptr)++;
          if ( (dclGet(cl, i+3)->out[j] & mask) != 0 )
            (*ptr)++;
        }
        for( ; i < cnt; i++ )
        {
          if ( (dclGet(cl, i)->out[j] & mask) != 0 )
            (*ptr)++;
        }
      }
      ptr++;
      mask<<=1;
    }
  }
  
  pi->split->cube_cnt = cnt;
  return 1;
}

/*
  requires a call to pinfoCntDCList()
*/
unsigned pinfoGetZeroCnt(pinfo *pi, int var)
{
  return pi->split->cube_cnt - pi->split->in_half_bit_cnt[var*2+1];
}

/*
  requires a call to pinfoCntDCList()
*/
unsigned pinfoGetOneCnt(pinfo *pi, int var)
{
  return pi->split->cube_cnt - pi->split->in_half_bit_cnt[var*2];
}

/*
  requires a call to pinfoCntDCList()
*/
unsigned pinfoGetDCCnt(pinfo *pi, int var)
{
  return pi->split->in_half_bit_cnt[var*2+1] + pi->split->in_half_bit_cnt[var*2] - pi->split->cube_cnt;
}

/*
  requires a call to pinfoCntDCList()
*/
unsigned pinfoGetOneZeroDiff(pinfo *pi, int var)
{
  unsigned o, z;
  o = pinfoGetOneCnt(pi, var);
  z = pinfoGetZeroCnt(pi, var);
  if ( o > z )
    return o-z;
  return z-o;
}

/*
  requires a call to pinfoCntDCList()
*/
int pinfoIsBinateInVar(pinfo *pi, int var)
{
  if ( (pinfoGetOneCnt(pi, var) > 0) && (pinfoGetZeroCnt(pi, var) > 0) )
    return 1;
  return 0;
}

/*
  returns the number of input vars which are not fully DC 
  requires a call to pinfoCntDCList()
*/
int pinfoGetNoneDCInVarCnt(pinfo *pi)
{
  int i;
  int none_dc_in_var_cnt = 0;
  for( i = 0; i < pi->in_cnt; i++ )
  {
    if ( pinfoGetDCCnt(pi, i) != pi->split->cube_cnt )
      none_dc_in_var_cnt++;
  }
  return none_dc_in_var_cnt;
}

int pinfoGetMinDCCntVar(pinfo *pi)
{
  int i;
  unsigned min_dc_cnt = 2*pi->split->cube_cnt+1;
  pi->split->in_best_var = -1;
  for( i = 0; i < pi->in_cnt; i++ )
  {
    if ( pinfoIsBinateInVar(pi, i) != 0 )
    {
      if ( pi->split->in_best_var < 0 )
      {
        min_dc_cnt = pinfoGetDCCnt(pi, i);
        pi->split->in_best_left_cnt = pi->split->in_half_bit_cnt[i*2];
        pi->split->in_best_right_cnt = pi->split->in_half_bit_cnt[i*2+1];
        pi->split->in_best_total_cnt = pi->split->cube_cnt;
        pi->split->in_best_var = i;
      }
      else if ( min_dc_cnt > pinfoGetDCCnt(pi, i)  )
      {
        min_dc_cnt = pinfoGetDCCnt(pi, i);
        pi->split->in_best_left_cnt = pi->split->in_half_bit_cnt[i*2];
        pi->split->in_best_right_cnt = pi->split->in_half_bit_cnt[i*2+1];
        pi->split->in_best_total_cnt = pi->split->cube_cnt;
        pi->split->in_best_var = i;
      }
    }
  }
  return pi->split->in_best_var;
}

int pinfoGetMinZeroOneDiffVar(pinfo *pi)
{
  int i;
  unsigned min_diff = pi->split->cube_cnt+1;
  pi->split->in_best_var = -1;
  for( i = 0; i < pi->in_cnt; i++ )
  {
    if ( pinfoIsBinateInVar(pi, i) != 0 )
    {
      if ( min_diff > pinfoGetOneZeroDiff(pi, i)  )
      {
        min_diff = pinfoGetOneZeroDiff(pi, i);
        pi->split->in_best_left_cnt = pi->split->in_half_bit_cnt[i*2];
        pi->split->in_best_right_cnt = pi->split->in_half_bit_cnt[i*2+1];
        pi->split->in_best_total_cnt = pi->split->cube_cnt;
        pi->split->in_best_var = i;
      }
    }
  }
  return pi->split->in_best_var;
}

void pinfo_out_calc_cur_cost(pinfo *pi)
{
  unsigned x;
  if ( pi->split->out_cur_sum > pi->split->out_dest_sum )
    x = pi->split->out_cur_sum - pi->split->out_dest_sum;
  else
    x = pi->split->out_dest_sum - pi->split->out_cur_sum;
  pi->split->out_cur_cost = x;
}


void pinfo_out_update_grp_max(pinfo *pi)
{
  int i;
  pi->split->out_opt_grp_max_cost = 0;
  pi->split->out_opt_grp_max_pos = -1;
  for( i = 0; i < pi->split->out_opt_grp_cnt; i++ )
  {
    if ( pi->split->out_opt_grp_max_cost <= pi->split->out_opt_cost[i] )
    {
      pi->split->out_opt_grp_max_pos = i;
      pi->split->out_opt_grp_max_cost = pi->split->out_opt_cost[i];
    }
  }
}

void pinfo_out_copy_cur_to_opt(pinfo *pi, int pos)
{
  int i;
  for( i = 0; i < pi->out_cnt; i++ )
    pi->split->out_opt_select[pos][i] = pi->split->out_cur_select[i];
  pi->split->out_opt_select_cnt[pos] = pi->split->out_cur_select_cnt;
  pi->split->out_opt_sum[pos] = pi->split->out_cur_sum;
  pi->split->out_opt_cost[pos] = pi->split->out_cur_cost;
  assert(pi->split->out_bit_cnt[pi->split->out_opt_select[pos][0]] != 0);
  pinfo_out_update_grp_max(pi);
}

void pinfo_out_copy_cur_to_grp(pinfo *pi)
{
  if ( pi->split->out_opt_grp_cnt < PINFO_OUT_SOL_CNT )
  {
    pi->split->out_opt_grp_cnt++;
    pinfo_out_copy_cur_to_opt(pi, pi->split->out_opt_grp_cnt-1);
    assert(pi->split->out_bit_cnt[pi->split->out_opt_select[pi->split->out_opt_grp_cnt-1][0]] != 0);
  }
  else
  {
    assert(pi->split->out_opt_grp_max_pos >= 0);
    assert(pi->split->out_opt_grp_max_pos < pi->split->out_opt_grp_cnt );
    pinfo_out_copy_cur_to_opt(pi, pi->split->out_opt_grp_max_pos);
    assert(pi->split->out_bit_cnt[pi->split->out_opt_select[pi->split->out_opt_grp_max_pos][0]] != 0);
  }
}

void pinfo_out_add_column(pinfo *pi, int var)
{
  pi->split->out_cur_select[pi->split->out_cur_select_cnt] = var;
  pi->split->out_cur_sum += pi->split->out_bit_cnt[var];
  pi->split->out_cur_select_cnt++;
  pinfo_out_calc_cur_cost(pi);
}

void pinfo_out_pop_column(pinfo *pi)
{
  int var;
  pi->split->out_cur_select_cnt--;
  var = pi->split->out_cur_select[pi->split->out_cur_select_cnt];
  pi->split->out_cur_sum -= pi->split->out_bit_cnt[var];
  pinfo_out_calc_cur_cost(pi);
}

int pinfo_out_find_next(pinfo *pi, int start)
{ 
  while(pi->split->out_bit_cnt[start] == 0)
  {
    if ( start >= pi->out_cnt )
      return -1;
    start++;
  }
   
  return start;
}

int pinfo_out_select_sub(pinfo *pi, int start)
{
  int var;
  if ( pi->split->out_cur_select_cnt+1 >= pi->split->out_active_cnt )
    return 1;
  var = pinfo_out_find_next(pi, start);
  if ( var < 0 )
    return 1;
  pinfo_out_add_column(pi, var);
  if ( pi->split->out_cur_cost <= pi->split->out_opt_grp_max_cost )
  {
    pinfo_out_copy_cur_to_grp(pi);
    /*
    if ( pi->split->out_opt_grp_max_cost == 0 )
      return 1;
    */
  }
  if ( pinfo_out_select_sub(pi, var+1) != 0 )
    return 1;
  pinfo_out_pop_column(pi);
  return pinfo_out_select_sub(pi, var+1);
}

int pinfo_out_select(pinfo *pi)
{
  int i;
  int start;
  pi->split->out_cur_select_cnt = 0;
  pi->split->out_cur_sum = 0;
  pi->split->out_dest_sum = 0;
  
  pi->split->out_active_cnt = 0;
  for( i = 0; i < pi->out_cnt; i++ )
    if ( pi->split->out_bit_cnt[i] > 0 )
    {
      pi->split->out_active_cnt++;
      pi->split->out_dest_sum += pi->split->out_bit_cnt[i];
    }
    
  if ( pi->split->out_active_cnt <= 1 )
    return 0;

  if ( pi->split->out_active_cnt == 2 )
  {
    pinfo_out_add_column(pi, pinfo_out_find_next(pi, 0));
    pi->split->out_opt_grp_cnt = 1;
    pinfo_out_copy_cur_to_opt(pi, 0);
    return 1;
  }
  
  pi->split->out_cur_cost = pi->split->out_dest_sum+1;

  for( i = 0; i < PINFO_OUT_SOL_CNT; i++ )
  {
    pi->split->out_opt_sum[i] = pi->split->out_dest_sum+1;
    pi->split->out_opt_cost[i] = pi->split->out_dest_sum+1;
    pi->split->out_opt_select_cnt[i] = 0;
  }
  
  pi->split->out_opt_grp_max_pos = -1;
  pi->split->out_opt_grp_max_cost = pi->split->out_dest_sum+1;
  pi->split->out_opt_grp_cnt = 0;
  
  pi->split->out_dest_sum/=2;
  
  start = pinfo_out_find_next(pi, 0);
  pinfo_out_add_column(pi, start);
  if ( pinfo_out_select_sub(pi, start+1) == 0 )
    return 0;
  for( i = 0; i < pi->split->out_opt_grp_cnt; i++ )
  {
    assert(pi->split->out_bit_cnt[pi->split->out_opt_select[i][0]] != 0);
  }
  return 1;
}

int pinfo_out_opt_to_dcube(pinfo *pi, int pos, dcube *cleft, dcube *cright)
{
  int i, j, aktive_cnt;
  
  
  dcInSetAll(pi, cleft, CUBE_IN_MASK_DC);
  dcOutSetAll(pi, cleft, 0);
  dcInSetAll(pi, cright, CUBE_IN_MASK_DC);
  dcOutSetAll(pi, cright, 0);

  if ( pi->split->out_active_cnt <= 1 )
    return 0;
  if ( pi->split->out_opt_grp_cnt <= 0 )
    return 0;

  assert( pos >= 0 );
  assert( pos < pi->split->out_opt_grp_cnt );
  i = pi->split->out_opt_select[pos][0];
  assert(pi->split->out_bit_cnt[pi->split->out_opt_select[pos][0]] != 0);
  j = 0;
  aktive_cnt = 0;
  while( aktive_cnt < pi->split->out_active_cnt && i < pi->out_cnt )
  {
    if ( pi->split->out_bit_cnt[i] != 0 )
    {
      if ( pi->split->out_opt_select[pos][j] == i && j < pi->split->out_opt_select_cnt[pos] )
      {
        dcSetOut(cleft, i, 1);
        j++;
      }
      else
      {
        dcSetOut(cright, i, 1);
      }
      aktive_cnt++;
    }
    i++;
  }
  assert(j == pi->split->out_opt_select_cnt[pos]);
  
  return 1;
}

int pinfo_out_best_solution(pinfo *pi, dclist cl)
{
  int i;
  dcube *cleft = pi->tmp+3;
  dcube *cright = pi->tmp+4;
  int cnt_left;
  int cnt_right;
  int cnt_total = dclCnt(cl);
  int diff;
  int min;
  pi->split->out_best_pos = -1;
  
  min = 2*cnt_total+3;
  for( i = 0; i < pi->split->out_opt_grp_cnt; i++ )
  {
    assert(pi->split->out_bit_cnt[pi->split->out_opt_select[i][0]] != 0);
    if ( pinfo_out_opt_to_dcube(pi, i, cleft, cright) == 0 )
      return -1;
    cnt_left = dclGetCofactorCnt(pi, cl, cleft);
    cnt_right = dclGetCofactorCnt(pi, cl, cright);
    diff = cnt_left - cnt_right;
    diff = cnt_total - cnt_left + cnt_total - cnt_right;
    if ( diff < 0 )
      diff = -diff;
    if ( min > diff )
    {
      pi->split->out_best_left_cnt = cnt_left;
      pi->split->out_best_right_cnt = cnt_right;
      pi->split->out_best_total_cnt = cnt_total;
      pi->split->out_best_pos = i;
      min = diff;
    }
  }
  return pi->split->out_best_pos;
}


/*-- pinfoGetSplittingInVar -------------------------------------------------*/

int pinfoGetSplittingInVar(pinfo *pi)
{
  return pinfoGetMinDCCntVar(pi);	/* side effect, result is also stored in pi->split->in_best_var */
  /* var_diff = pinfoGetMinZeroOneDiffVar(pi); */
}

int pinfo_in_var_to_dcube(pinfo *pi, int var, dcube *r, dcube *rinv, dcube *cof)
{
  dcCopy(pi, r, cof);
  dcCopy(pi, rinv, cof);
  dcInSetAll(pi, r, CUBE_IN_MASK_DC);
  dcInSetAll(pi, rinv, CUBE_IN_MASK_DC);
  dcSetIn(r, var, 2);
  dcSetIn(rinv, var, 1);
  return 1;
}

int pinfoGetInVarDCubeCofactor(pinfo *pi, dcube *r, dcube *rinv, dclist cl, dcube *cof)
{
  int var;
  if ( pinfoCntDCList(pi, cl, cof) == 0 )
    return 0;		/* memory issue */
  var = pinfoGetSplittingInVar(pi);
  if ( var >= 0 )
    return pinfo_in_var_to_dcube(pi, var, r, rinv, cof);
  return 0;	/* no split possible, function is unate (?) */
}

int pinfoGetOutVarDCubeCofactor(pinfo *pi, dcube *r, dcube *rinv, dclist cl, dcube *cof)
{
  int best_pos;
  if ( pinfoCntDCList(pi, cl, cof) == 0 )
    return 0;
  if ( pinfo_out_select(pi) == 0 )
    return 0;
  best_pos = pinfo_out_best_solution(pi, cl);
  return pinfo_out_opt_to_dcube(pi, best_pos, r, rinv);
}

/*
pinfoGetDCubeCofactorForSplitting is called by
  int dcGetCofactorForSplit(pinfo *pi, dcube *l, dcube *r, dclist cl, dcube *cof)

*/
int pinfoGetDCubeCofactorForSplitting(pinfo *pi, dcube *r, dcube *rinv, dclist cl, dcube *cof)
{
  int in;
  int out;
  if ( pinfoCntDCList(pi, cl, cof) == 0 )
    return 0;
  pinfoGetSplittingInVar(pi);	/* result is stored in pi->split->in_best_var */
  if ( pinfo_out_select(pi) == 0 )
    pi->split->out_best_pos = -1;
  else
    pinfo_out_best_solution(pi, cl);
    
  if ( pi->split->out_best_pos < 0 && pi->split->in_best_var < 0 )
    return 0;
    
  if ( pi->split->out_best_pos >= 0 && pi->split->in_best_var >= 0 )
  {
    in = pi->split->in_best_left_cnt+pi->split->in_best_right_cnt;
    out = pi->split->out_best_left_cnt+pi->split->out_best_right_cnt;
    if ( in < out )
      return pinfo_in_var_to_dcube(pi, pi->split->in_best_var, r, rinv, cof);
    return pinfo_out_opt_to_dcube(pi, pi->split->out_best_pos, r, rinv);
  }
  
  if ( pi->split->in_best_var >= 0 )
    return pinfo_in_var_to_dcube(pi, pi->split->in_best_var, r, rinv, cof);
  
  return pinfo_out_opt_to_dcube(pi, pi->split->out_best_pos, r, rinv);
}

/*---------------------------------------------------------------------------*/

/* merge function cl_src into cl_dest, update pi_dest with all new signals of pi_src */
/* returns 1 if successful, returns 0 for any error */
int pinfoMerge(pinfo *pi_dest, dclist cl_dest, pinfo *pi_src, dclist cl_src)
{
  int i, j, k, m;
  int in_pos;
  int out_pos;
  int *in_map;
  int val;
  dclist src_on_cl, src_off_cl;


  if ( dclInitVA(2, &src_on_cl, &src_off_cl) == 0 )
    return 0;
  
  /* allocate a mapping list for the input signals of the source function */
  /* this will return the signal position in the dest list, based on the index of the source list */
  in_map = (int *)malloc(sizeof(int)*(pi_src->in_cnt));
  if ( in_map == NULL )
    return dclDestroyVA(2, src_on_cl, src_off_cl), 0;

  /* add the inputs from src to dest, consider existing signals and fill the in_map table */
  for( i = 0; i <  b_sl_GetCnt(pi_src->in_sl); i++ )
  {
    in_pos = pinfoAddInLabel( pi_dest, b_sl_GetVal(pi_src->in_sl, i) );
    if ( in_pos < 0 )
      return free(in_map), dclDestroyVA(2, src_on_cl, src_off_cl), 0;
    in_map[i] = in_pos;
  }  
  
  /* loop over the out variables of cl_src */
  /* there are three cases: */
  /* 1. Out signal exists as an input in dest: */
  /*     The dest input is replaced by the current output function, no new output is created */
  /* 2. Out signal does not exist as input or output in dest */
  /*     New out signal is created in dest, the src function is added to the dest function */
  /* 3. Out signal exists as dest out function: */
  /* 	 This shouldn't happen from an electronics point of view (two functions driving the same line) */
  /*    but it is assumed, that this is just a OR connection and treated like in 2. */
  /*    maybe a warning should be generated */
  /* The fourth case, where the out signal exists as input and output in dest is ignored and */
  /* treated like in case 1. This case is an async circuit... hopefully the user knows what he does */

  for( j = 0; j <  b_sl_GetCnt(pi_src->out_sl); j++ )
  {
    /* check, whether the output variable exists as an input variable in dest */
    in_pos = b_sl_Find(pi_dest->in_sl, b_sl_GetVal(pi_src->out_sl, j));
    if ( in_pos >= 0 )
    {
      /* case 1: output exists as input in the destination cube list */
      
      /* extract the on set out of the source function */
      if ( dclCopyByOut(pi_src, src_on_cl, cl_src, j) == 0 )
	return free(in_map), dclDestroyVA(2, src_on_cl, src_off_cl), 0;
	
      /* then, take this on set and create the complement */
      if ( dclCopy(pi_src, src_off_cl, src_on_cl) == 0 )
	return free(in_map), dclDestroyVA(2, src_on_cl, src_off_cl), 0;
      if ( dclComplement(pi_src, src_off_cl) == 0 )
	return free(in_map), dclDestroyVA(2, src_on_cl, src_off_cl), 0;
      
      /* on set and its complement will be used to replace the input in the destination cube list */

      /* loop over all cubes of the target function and replace the input variable */
      dclClearFlags(cl_dest);
      for( k = 0; k < dclCnt(cl_dest); k++ )
      {
	switch( dcGetIn( dclGet(cl_dest, k), in_pos ) )
	{
	  case 1:		/* input variable appears negative: use src_off_cl */
	    /* mark the orignal cube for deletion */
	    dclSetFlag(cl_dest, k);		/* the illegal block is marked and will be deleted below */	    
	  
	    /* loop over the source cubes */
	    for( m = 0; m < dclCnt(src_off_cl); m++ )
	    {	
	      /* constuct the new cube for the target list */
	      dcCopy( pi_dest, pi_dest->tmp+16, dclGet(cl_dest, k) );	/* get the current target cube, use tmp store place 16 */
	      dcSetIn( pi_dest->tmp+16, in_pos, 3);		/* make the variable (which should be replaced) a don't care */
	      for( i = 0; i <  b_sl_GetCnt(pi_src->in_sl); i++ )
	      {
		val = dcGetIn(dclGet(src_off_cl, m), i);			/* get the in value of the source cube */
		dcSetIn( pi_dest->tmp+16, in_map[i], val );		/* assign this value to the correct pos in the target cube */
	      }	      
	      /* add the new cube */
	      if ( dclAdd(pi_dest, cl_dest, pi_dest->tmp+16) < 0 )
		return free(in_map), dclDestroyVA(2, src_on_cl, src_off_cl), 0;
	    }
	    
	    break;
	  case 2:		/* input variable appears positive: use src_on_cl */
	    /* mark the orignal cube for deletion */
	    dclSetFlag(cl_dest, k);		/* the illegal block is marked and will be deleted below */	    
	  
	    /* loop over the source cubes */
	    for( m = 0; m < dclCnt(src_on_cl); m++ )
	    {	
	      /* constuct the new cube for the target list */
	      dcCopy( pi_dest, pi_dest->tmp+16, dclGet(cl_dest, k) );	/* get the current target cube, use tmp store place 16 */
	      dcSetIn( pi_dest->tmp+16, in_pos, 3);		/* make the variable (which should be replaced) a don't care */
	      for( i = 0; i <  b_sl_GetCnt(pi_src->in_sl); i++ )
	      {
		val = dcGetIn(dclGet(src_on_cl, m), i);			/* get the in value of the source cube */
		dcSetIn( pi_dest->tmp+16, in_map[i], val );		/* assign this value to the correct pos in the target cube */
	      }	      
	      /* add the new cube */
	      if ( dclAdd(pi_dest, cl_dest, pi_dest->tmp+16) < 0 )
		return free(in_map), dclDestroyVA(2, src_on_cl, src_off_cl), 0;
	    }
	    
	    break;
	  default:	/* input variable is don't care or illegal */
	    
	    break;
	}	/* switch */
      } /* loop over desintation cubes */
      dclDeleteCubesWithFlag(pi_dest, cl_dest);	/* delete all cubes, which hab been replaced above */
      
    }
    else
    {
      /* case 2. or 3. */
      /* output signal does not exist in the destintion function as input */
      //printf("Read: Merge true output %s\n", b_sl_GetVal(pi_src->out_sl, j));
      
      /* add the out label, return pos to existing or new label */
      //out_pos = b_sl_Find(pi_dest->out_sl, b_sl_GetVal(pi_src->out_sl, j));
      out_pos = pinfoAddOutLabel(pi_dest, b_sl_GetVal(pi_src->out_sl, j));
      
      if ( out_pos < 0 )
	return free(in_map), dclDestroyVA(2, src_on_cl, src_off_cl), 0;
      
      /* extract the on set out of the source function */
      if ( dclCopyByOut(pi_src, src_on_cl, cl_src, j) == 0 )
	return free(in_map), dclDestroyVA(2, src_on_cl, src_off_cl), 0;
	
      /* then, take this on set and create the complement */
      /*
      if ( dclCopy(pi_src, src_off_cl, src_on_cl) == 0 )
	return free(in_map), dclDestroyVA(2, src_on_cl, src_off_cl), 0;
      if ( dclComplement(pi_src, src_off_cl) == 0 )
	return free(in_map), dclDestroyVA(2, src_on_cl, src_off_cl), 0;
      */
      
      /* loop over the source cubes */
      for( m = 0; m < dclCnt(src_on_cl); m++ )
      {	
	/* constuct the new cube for the target list */
	dcCopy( pi_dest, pi_dest->tmp+16, pi_dest->tmp+0 );	/* derive the new cube from the tautology block  */
	dcOutSetAll(pi_dest, pi_dest->tmp+16, 0);
	dcSetOut( pi_dest->tmp+16, out_pos, 1);		/* set the out variable */
	for( i = 0; i <  b_sl_GetCnt(pi_src->in_sl); i++ )
	{
	  val = dcGetIn(dclGet(src_on_cl, m), i);			/* get the in value of the source cube */
	  dcSetIn( pi_dest->tmp+16, in_map[i], val );		/* assign this value to the correct pos in the target cube */
	}
	/* add the new cube */
	if ( dclAdd(pi_dest, cl_dest, pi_dest->tmp+16) < 0 )
	  return free(in_map), dclDestroyVA(2, src_on_cl, src_off_cl), 0;
      }
      
    }
  }
  return free(in_map), dclDestroyVA(2, src_on_cl, src_off_cl), 1;  
}



/*---------------------------------------------------------------------------*/

int pinfoWrite(pinfo *pi, FILE *fp)
{
  if ( pi == NULL )
  {
    if ( b_io_WriteInt(fp, 0) == 0 )            return 0;
    return 1;
  }
  
  if ( b_io_WriteInt(fp, 1) == 0 )              return 0;
  if ( b_io_WriteInt(fp, pi->in_cnt) == 0 )     return 0;
  if ( b_io_WriteInt(fp, pi->out_cnt) == 0 )    return 0;
  if ( pi->progress == NULL )
  {
    if ( b_io_WriteInt(fp, 0) == 0 )            return 0;
  }
  else
  {
    if ( b_io_WriteInt(fp, 1) == 0 )            return 0;
  }
  return 1;
}

int pinfoRead(pinfo **pi, FILE *fp)
{
  int in, out;
  if ( *pi != NULL )
    pinfoClose(*pi);
  *pi = NULL;

  if ( b_io_ReadInt(fp, &in) == 0 )     return 0;
  if ( in == 0 )
    return 1;


  if ( b_io_ReadInt(fp, &in) == 0 )     return 0;
  if ( b_io_ReadInt(fp, &out) == 0 )    return 0;
  
  *pi = pinfoOpenInOut(in, out);
  if ( *pi == NULL )
    return 0;
  /*
  if ( pinfoSetInCnt(*pi, in) == 0 )    return 0;
  if ( pinfoSetOutCnt(*pi, out) == 0 )  return 0;
  */

  if ( b_io_ReadInt(fp, &in) == 0 )     return 0;
    if ( in != 0 )
      pinfoInitProgress(*pi);
  return 1;
}


/*---------------------------------------------------------------------------*/

