/*
  
  dynamic cube
  
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

/*! \defgroup dclist Sum of Product Management */


#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include "b_io.h"
#include "b_ff.h"
#include "dcube.h"
#include "mcov.h"
#include "mwc.h"
#include "matrix.h"

/*-- dcInSetAll -------------------------------------------------------------*/
void dcInSetAll(pinfo *pi, dcube *c, c_int v)
{
  int i;
  for( i = 0; i < pi->in_words; i++ )
    c->in[i] = v;
}

/*-- dcCopy -----------------------------------------------------------------*/
void dcCopy(pinfo *pi, dcube *dest, dcube *src)
{
  int i;
    
  for( i = 0; i < pi->in_out_words_min; i++ )
  {
    dest->in[i] = src->in[i];
    dest->out[i] = src->out[i];
  }
  for( i = pi->in_out_words_min; i < pi->in_words; i++ )
    dest->in[i] = src->in[i];
  for( i = pi->in_out_words_min; i < pi->out_words; i++ )
    dest->out[i] = src->out[i];
  dest->n = src->n;
}

/*-- dcCopyOut --------------------------------------------------------------*/
void dcCopyOut(pinfo *pi, dcube *dest, dcube *src)
{
  int i;
  for( i = 0; i < pi->out_words; i++ )
    dest->out[i] = src->out[i];
}

/*-- dcCopyIn ---------------------------------------------------------------*/
void dcCopyIn(pinfo *pi, dcube *dest, dcube *src)
{
  int i;
  for( i = 0; i < pi->in_words; i++ )
    dest->in[i] = src->in[i];
}

/*-- dcCopyInToIn -----------------------------------------------------------*/
void dcCopyInToIn(pinfo *pi_dest, dcube *dest, int dest_offset, pinfo *pi_src, dcube *src)
{
  int i;
  int s;
  for( i = 0; i < pi_src->in_cnt; i++ )
  {
    s = dcGetIn(src, i);
    dcSetIn(dest, i+dest_offset, s);
  }
}

/*-- dcCopyInToInRange ------------------------------------------------------*/
void dcCopyInToInRange(pinfo *pi_dest, dcube *dest, int dest_offset, 
                       pinfo *pi_src, dcube *src, int src_offset, int src_cnt)
{
  int i;
  int s;
  assert(src_offset+src_cnt <= pi_src->in_cnt);
  for( i = src_offset; i < src_offset+src_cnt; i++ )
  {
    s = dcGetIn(src, i);
    dcSetIn(dest, i+dest_offset-src_offset, s);
  }
}


/*-- dcCopyOutToIn ----------------------------------------------------------*/
void dcCopyOutToIn(pinfo *pi_dest, dcube *dest, int dest_offset, pinfo *pi_src, dcube *src)
{
  int i;
  int s;
  for( i = 0; i < pi_src->out_cnt; i++ )
  {
    s = dcGetOut(src, i) + 1;
    dcSetIn(dest, i+dest_offset, s);
  }
}

/*-- dcCopyOutToInRange -----------------------------------------------------*/
void dcCopyOutToInRange(pinfo *pi_dest, dcube *dest, int dest_offset, 
                        pinfo *pi_src, dcube *src, int src_offset, int src_cnt)
{
  int i;
  int s;
  assert(src_offset+src_cnt <= pi_src->in_cnt);
  for( i = src_offset; i < src_offset+src_cnt; i++ )
  {
    s = dcGetOut(src, i) + 1;
    dcSetIn(dest, i+dest_offset-src_offset, s);
  }
}

/*-- dcCopyInToOut ----------------------------------------------------------*/
void dcCopyInToOut(pinfo *pi_dest, dcube *dest, int dest_offset, pinfo *pi_src, dcube *src)
{
  int i;
  int s;
  for( i = 0; i < pi_src->in_cnt; i++ )
  {
    s = dcGetIn(src, i) - 1;
    dcSetOut(dest, i+dest_offset, s);
  }
}

/*-- dcCopyInToOutRange -----------------------------------------------------*/
void dcCopyInToOutRange(pinfo *pi_dest, dcube *dest, int dest_offset, 
                        pinfo *pi_src, dcube *src, int src_offset, int src_cnt)
{
  int i;
  int s;
  assert(src_offset+src_cnt <= pi_src->in_cnt);
  for( i = src_offset; i < src_offset+src_cnt; i++ )
  {
    s = dcGetIn(src, i) - 1;
    dcSetOut(dest, i+dest_offset-src_offset, s);
  }
}

/*-- dcCopyOutToOut ---------------------------------------------------------*/
void dcCopyOutToOut(pinfo *pi_dest, dcube *dest, int dest_offset, 
                    pinfo *pi_src, dcube *src)
{
  int i, cnt;
  int s;
  cnt = pi_src->out_cnt;
  if ( cnt > pi_dest->out_cnt-dest_offset )
    cnt = pi_dest->out_cnt-dest_offset;
  for( i = 0; i < cnt; i++ )
  {
    s = dcGetOut(src, i);
    dcSetOut(dest, i+dest_offset, s);
  }
}

/*-- dcCopyOutToOutRange ----------------------------------------------------*/
void dcCopyOutToOutRange(pinfo *pi_dest, dcube *dest, int dest_offset, 
                         pinfo *pi_src, dcube *src, int src_offset, int src_cnt)
{
  int i;
  int s;
  assert(src_offset+src_cnt <= pi_src->out_cnt);
  assert(dest_offset+src_cnt < pi_dest->out_cnt);
  for( i = src_offset; i < src_offset+src_cnt; i++ )
  {
    s = dcGetOut(src, i);
    dcSetOut(dest, i+dest_offset-src_offset, s);
  }
}

/*-- dcDeleteIn -----------------------------------------------------------*/
/* remove a variable at 'pos', shift all other variables one position to the left, fill last var with DC */
void dcDeleteIn(pinfo *pi, dcube *c, int pos)
{
  int i;
  int s;
  if ( pi->in_cnt > 0 )
  {
    for( i = pos+1; i < pi->in_cnt; i++ )
    {
      s = dcGetIn(c, i);
      //printf("%d:%d ", i, s);
      dcSetIn(c, i-1, s);
    }
    //printf("\n");
    dcSetIn(c, pi->in_cnt-1, 3);  
  }
}


/*-- dcOutSetAll ------------------------------------------------------------*/
void dcOutSetAll(pinfo *pi, dcube *c, c_int v)
{
  int i;
  for( i = 0; i < pi->out_words; i++ )
    c->out[i] = v;
}

/*-- dcAllClear -------------------------------------------------------------*/
void dcAllClear(pinfo *pi, dcube *c)
{ 
  dcInSetAll(pi, c, 0);
  dcOutSetAll(pi, c, 0);
}

/*-- dcSetTautology ---------------------------------------------------------*/
void dcSetTautology(pinfo *pi, dcube *c)
{ 
  dcInSetAll(pi, c, CUBE_IN_MASK_DC);
  dcOutSetAll(pi, c, CUBE_OUT_MASK);
  if ( pi->out_words > 0 )
    c->out[pi->out_words-1] = pi->out_last_mask;
}

/*-- dcSetOutTautology ------------------------------------------------------*/
void dcSetOutTautology(pinfo *pi, dcube *c)
{ 
  dcOutSetAll(pi, c, CUBE_OUT_MASK);
  if ( pi->out_words > 0 )
    c->out[pi->out_words-1] = pi->out_last_mask;
}

/*-- dcInitMem --------------------------------------------------------------*/

int dcInitMem(pinfo *pi, dcube *c)
{
  c->in = NULL;
  c->out = NULL;
  return dcAdjustByPinfo(pi, c);
}

/*-- dcInit -----------------------------------------------------------------*/

int dcInit(pinfo *pi, dcube *c)
{
  c->in = NULL;
  c->out = NULL;
  if ( dcAdjustByPinfo(pi, c) == 0 )
    return 0;
  dcInSetAll(pi, c, CUBE_IN_MASK_DC);
  dcOutSetAll(pi, c, 0);
  c->n = 0;
  return 1;
}

/*-- dcInitVA --------------------------------------------------------------*/

int dcInitVA(pinfo *pi, int n, ...)
{
  va_list va;
  int i;
  va_start(va, n);
  for( i = 0; i < n; i++ )
    if ( dcInit(pi, va_arg(va, dcube *)) == 0 )
      break;
  va_end(va);
  if ( i < n )
  {
    va_start(va, n);
    while( i-- >= 0 )
      dcDestroy(va_arg(va, dcube *));
    va_end(va);
    return 0;
  }
  return 1;
}

/*-- dcDestroy --------------------------------------------------------------*/

void dcDestroy(dcube *c)
{
  if ( c->in != NULL )
    free(c->in);
  if ( c->out != NULL )
    free(c->out);
    
  c->in = NULL;
  c->out = NULL;
}

/*-- dcDestroyVA ------------------------------------------------------------*/

void dcDestroyVA(int n, ...)
{
  va_list va;
  int i;
  va_start(va, n);
  for( i = 0; i < n; i++ )
    dcDestroy(va_arg(va, dcube *));
  va_end(va);
}

/*-- dcAdjust ---------------------------------------------------------------*/

int dcAdjust(dcube *c, int in_words, int out_words)
{
  void *ptr;

  if ( in_words == 0 )
  {
    if ( c->in != NULL )
      free(c->in);
    c->in = NULL;
  }
  else
  {
    if ( c->in == NULL )
      ptr = malloc(sizeof(c_int)*in_words);
    else
      ptr = realloc(c->in, sizeof(c_int)*in_words);
    if ( ptr == NULL )
      return 0;
    c->in = (c_int *)ptr;
  }

  if ( out_words == 0 )
  {
    if ( c->out != NULL )
      free(c->out);
    c->out = NULL;
  }
  else
  {
    if ( c->out == NULL )
      ptr = malloc(sizeof(c_int)*out_words);
    else
      ptr = realloc(c->out, sizeof(c_int)*out_words);
    if ( ptr == NULL )
      return 0;
    c->out = (c_int *)ptr;
  }
  
  return 1;
}

/*-- dcAdjustByPinfo --------------------------------------------------------*/

int dcAdjustByPinfo(pinfo *pi, dcube *c)
{
  return dcAdjust(c, pi->in_words, pi->out_words);
}

/*-- dcAdjustByPinfoVA ------------------------------------------------------*/

int dcAdjustByPinfoVA(pinfo *pi, int n, ...)
{
  va_list va;
  int i;
  va_start(va, n);
  for( i = 0; i < n; i++ )
    if ( dcAdjustByPinfo(pi, va_arg(va, dcube *)) == 0 )
    {
      va_end(va);
      return 0;
    }
  va_end(va);
  return 1;
}


/*-- dcSetIn ----------------------------------------------------------------*/

void dcSetIn(dcube *c, int pos, int code)
{
  c->in[pos/CUBE_SIGNALS_PER_IN_WORD] &= ~(3<<((pos&(CUBE_SIGNALS_PER_IN_WORD-1))*2));
  c->in[pos/CUBE_SIGNALS_PER_IN_WORD] |= code<<((pos&(CUBE_SIGNALS_PER_IN_WORD-1))*2);
}

/*-- dcGetIn ----------------------------------------------------------------*/

int dcGetIn(dcube *c, int pos)
{
  return (c->in[pos/CUBE_SIGNALS_PER_IN_WORD] >> 
          ((pos&(CUBE_SIGNALS_PER_IN_WORD-1))*2)) & 3;
}

/*-- dcSetOut ---------------------------------------------------------------*/

void dcSetOut(dcube *c, int pos, int code)
{
  c->out[pos/CUBE_SIGNALS_PER_OUT_WORD] &= ~(1<<(pos&(CUBE_SIGNALS_PER_OUT_WORD-1)));
  c->out[pos/CUBE_SIGNALS_PER_OUT_WORD] |= (code<<(pos&(CUBE_SIGNALS_PER_OUT_WORD-1)));
}

/*-- dcGetOut ----------------------------------------------------------------*/

int dcGetOut(dcube *c, int pos)
{
  return (c->out[pos/CUBE_SIGNALS_PER_OUT_WORD] >> 
    (pos&(CUBE_SIGNALS_PER_OUT_WORD-1))) & 1;
}

/*-- dcSetByStr -------------------------------------------------------------*/

int dcSetByStr(pinfo *pi, dcube *c, char *str)
{
  size_t i = 0;
  
  if ( dcAdjustByPinfo(pi, c) == 0 )
    return 0;
  dcInSetAll(pi, c, CUBE_IN_MASK_DC);
  dcOutSetAll(pi, c, 0);
  
  while( i < pi->in_cnt )
    switch(*str++)
    {
      case '0': dcSetIn(c, i++, 1); break;
      case '1': dcSetIn(c, i++, 2); break;
      case '-': dcSetIn(c, i++, 3); break;
      case '\0': return 0;
    }
  i = 0;
  while( i < pi->out_cnt )
    switch(*str++)
    {
      case '0': dcSetOut(c, i++, 0); break;
      case '1': dcSetOut(c, i++, 1); break;
      case '\0': return 0;
    }
  return 1;
}

/*-- dcSetInByStr -----------------------------------------------------------*/

int dcSetInByStr(pinfo *pi, dcube *c, char *str)
{
  size_t i = 0;
  
  if ( dcAdjustByPinfo(pi, c) == 0 )
    return 0;
  dcInSetAll(pi, c, CUBE_IN_MASK_DC);
  
  i = 0;
  while( i < pi->in_cnt )
    switch(*str++)
    {
      case '0': dcSetIn(c, i++, 1); break;
      case '1': dcSetIn(c, i++, 2); break;
      case '-': dcSetIn(c, i++, 3); break;
      case '\0': return 0;
    }
  return 1;
}

/*-- dcSetOutByStr ----------------------------------------------------------*/

int dcSetOutByStr(pinfo *pi, dcube *c, char *str)
{
  size_t i = 0;
  
  if ( dcAdjustByPinfo(pi, c) == 0 )
    return 0;
  dcInSetAll(pi, c, 0);
  
  i = 0;
  while( i < pi->out_cnt )
    switch(*str++)
    {
      case '0': dcSetOut(c, i++, 0); break;
      case '1': dcSetOut(c, i++, 1); break;
      case '\0': return 0;
    }
  return 1;
}

/*-- dcSetAllByStr ----------------------------------------------------------*/

/* dcSetAllByStr ... dcSetAllByStrPtr */

char *dcSetAllByStr(pinfo *pi, int in_cnt, int out_cnt, dcube *c_on, dcube *c_dc, char *str)
{
  size_t i = 0;
  
  if ( dcAdjustByPinfo(pi, c_on) == 0 )
    return NULL;
  if ( dcAdjustByPinfo(pi, c_dc) == 0 )
    return NULL;
  dcInSetAll(pi, c_on, CUBE_IN_MASK_DC);
  dcOutSetAll(pi, c_on, 0);
  dcInSetAll(pi, c_dc, CUBE_IN_MASK_DC);
  dcOutSetAll(pi, c_dc, 0);
  
  while( i < in_cnt )
  {
    switch(*str++)
    {
      case '0': dcSetIn(c_on, i, 1); dcSetIn(c_dc, i, 1); i++; break;
      case '1': dcSetIn(c_on, i, 2); dcSetIn(c_dc, i, 2); i++; break;
      case '-': dcSetIn(c_on, i, 3); dcSetIn(c_dc, i, 3); i++; break;
      case '\0': return 0;
    }
  }
  i = 0;
  while( i < out_cnt )
  {
    switch(*str)
    {
      case '0': dcSetOut(c_on, i, 0); dcSetOut(c_dc, i, 0); i++; break;
      case '1': dcSetOut(c_on, i, 1); dcSetOut(c_dc, i, 0); i++; break;
      case '2': dcSetOut(c_on, i, 0); dcSetOut(c_dc, i, 1); i++; break;
      case '-': dcSetOut(c_on, i, 0); dcSetOut(c_dc, i, 1); i++; break;
      case '\0': return 0;
      default: 
        if ( *str >= 'a' && *str <= 'z' ) 
          return 0;
        if ( *str >= 'A' && *str <= 'Z' ) 
          return 0;
        if ( *str == '_' ) 
          return 0;
        
    }
    str++;
  }
  return str;
}


/*-- dcToStr ----------------------------------------------------------------*/

char *dcToStr(pinfo *pi, dcube *c, char *sep, char *post)
{
  static char s[1024*8];
  int i, l;
  for( i = 0; i < pi->in_cnt; i++ )
    s[i] = "x01-"[dcGetIn(c, i)];
  strcpy(s+pi->in_cnt, sep);
  l = pi->in_cnt+strlen(sep);
  for( i = 0; i < pi->out_cnt; i++ )
    s[i+l] = "01"[dcGetOut(c, i)];
  strcpy(s+l+pi->out_cnt, post);
  return s;
}

char *dcToStr2(pinfo *pi, dcube *c, char *sep, char *post)
{
  static char s[1024*8];
  int i, l;
  for( i = 0; i < pi->in_cnt; i++ )
    s[i] = "x01-"[dcGetIn(c, i)];
  strcpy(s+pi->in_cnt, sep);
  l = pi->in_cnt+strlen(sep);
  for( i = 0; i < pi->out_cnt; i++ )
    s[i+l] = "01"[dcGetOut(c, i)];
  strcpy(s+l+pi->out_cnt, post);
  return s;
}

char *dcToStr3(pinfo *pi, dcube *c, char *sep, char *post)
{
  static char s[1024*8];
  int i, l;
  for( i = 0; i < pi->in_cnt; i++ )
    s[i] = "x01-"[dcGetIn(c, i)];
  strcpy(s+pi->in_cnt, sep);
  l = pi->in_cnt+strlen(sep);
  for( i = 0; i < pi->out_cnt; i++ )
    s[i+l] = "01"[dcGetOut(c, i)];
  strcpy(s+l+pi->out_cnt, post);
  return s;
}


/*-- dcOutToStr -------------------------------------------------------------*/

char *dcOutToStr(pinfo *pi, dcube *c, char *post)
{
  static char s[1024*16];
  int i;
  for( i = 0; i < pi->out_cnt; i++ )
    s[i] = "01"[dcGetOut(c, i)];
  strcpy(s+pi->out_cnt, post);
  return s;
}

/*-- dcInToStr --------------------------------------------------------------*/

char *dcInToStr(pinfo *pi, dcube *c, char *post)
{
  static char s[1024*16];
  int i;
  for( i = 0; i < pi->in_cnt; i++ )
    s[i] = "x01-"[dcGetIn(c, i)];
  strcpy(s+pi->in_cnt, post);
  return s;
}


/*-- dcInc ------------------------------------------------------------------*/
/* assume that there are no don't cares */
/* interpret the cube as a binary number and add 1 */
/* returns 0 if there was a overflow */
int dcInc(pinfo *pi, dcube *c)
{
  int i = 0;
  for(;;)
  {
    if ( dcGetIn(c, i) == 1 )
    {
      dcSetIn(c, i, 2);
      break;
    }
    else
    {
      dcSetIn(c, i, 1);
      i++;
    }
    if ( i >= pi->in_cnt )
      return 0;
  }
  return 1;
}

/*-- dcIncOut ---------------------------------------------------------------*/
/* assume that there are no don't cares */
/* interpret the cube as a binary number and add 1 */
/* returns 0 if there was a overflow */
int dcIncOut(pinfo *pi, dcube *c)
{
  int i = 0;
  for(;;)
  {
    if ( dcGetOut(c, i) == 0 )
    {
      dcSetOut(c, i, 1);
      break;
    }
    else
    {
      dcSetOut(c, i, 0);
      i++;
    }
    if ( i >= pi->out_cnt )
      return 0;
  }
  return 1;
}

/*-- dcIsEqualIn ------------------------------------------------------------*/
int dcIsEqualIn(pinfo *pi, dcube *a, dcube *b)
{
  register int i;
  for( i = 0; i < pi->in_words; i++ )
    if ( a->in[i] != b->in[i] )
      return 0;
  return 1;
}

/*-- dcIsEqualOut -----------------------------------------------------------*/
int dcIsEqualOut(pinfo *pi, dcube *a, dcube *b)
{
  register int i;
  for( i = 0; i < pi->out_words; i++ )
    if ( a->out[i] != b->out[i] )
      return 0;
  return 1;
}

/*-- dcIsEqualOutCnt --------------------------------------------------------*/
int dcIsEqualOutCnt(dcube *a, dcube *b, int off, int cnt)
{
  register int i;
  for( i = off; i < off+cnt; i++ )
    if ( dcGetOut(a, i) != dcGetOut(b, i) )
      return 0;
  return 1;
}

/*-- dcIsEqualOutRange ------------------------------------------------------*/
int dcIsEqualOutRange(dcube *a, int off_a, dcube *b, int off_b, int cnt)
{
  register int i;
  for( i = 0; i < cnt; i++ )
    if ( dcGetOut(a, i+off_a) != dcGetOut(b, i+off_b) )
      return 0;
  return 1;
}

/*-- dcIsEqual --------------------------------------------------------------*/
int dcIsEqual(pinfo *pi, dcube *a, dcube *b)
{
  register int i;
  for( i = 0; i < pi->in_words; i++ )
    if ( a->in[i] != b->in[i] )
      return 0;
  for( i = 0; i < pi->out_words; i++ )
    if ( a->out[i] != b->out[i] )
      return 0;
  return 1;
}

/*-- dcIntersection ---------------------------------------------------------*/

int dcIntersectionOld(pinfo *pi, dcube *r, dcube *a, dcube *b)
{
  register int i;

  if ( dcDeltaOut(pi, a, b) > 0 )
    return 0;
  if ( dcIsDeltaInNoneZero(pi, a, b) > 0 )
    return 0;
  for( i = 0; i < pi->in_words; i++ )
    r->in[i] = a->in[i] & b->in[i];
  for( i = 0; i < pi->out_words; i++ )
    r->out[i] = a->out[i] & b->out[i];
  return 1;
}

int dcIntersection(pinfo *pi, dcube *r, dcube *a, dcube *b)
{
  register int i;
  register c_int c;

  for( i = 0; i < pi->in_words; i++ )
  {
    c = a->in[i] & b->in[i];  /* Problem:      Wie oft kommt 00 vor? */
    r->in[i] = c;
    c |= c>>1;                /* Reduktion:    Wie oft kommt x0 vor? */
    c = ~c;                   /* Invertierung: Wie oft kommt x1 vor? */
    c &= CUBE_IN_MASK_ZERO;   /* Maskierung:   Wie oft kommt 01 vor? */
    if ( c > 0 )
      return 0;
  }

  if ( pi->out_cnt == 0 )
    return 1;

  c = 0;
  for( i = 0; i < pi->out_words; i++ )
  {
    r->out[i] = a->out[i] & b->out[i];
    c |= r->out[i];
  }
  if ( c == 0 )
    return 0;
    
  return 1;
}

/*-- dcIsInSubSet -----------------------------------------------------------*/

/* Ist der Eingangs-Teil von b Teilmenge vom Eingangs-Teil von a? */
/* Ja: Rueckgabe ist 1, nein: Rueckgabe ist 0 */
int dcIsInSubSet(pinfo *pi, dcube *a, dcube *b)
{
  register int i;
  for( i = 0; i < pi->in_words; i++ )
    if ( (a->in[i] & b->in[i]) != b->in[i] )
      return 0;
  return 1;
}

/*-- dcIsOutSubSet ----------------------------------------------------------*/

/* Ist der Eingangs-Teil von b Teilmenge vom Eingangs-Teil von a? */
/* Ja: Rueckgabe ist 1, nein: Rueckgabe ist 0 */
int dcIsOutSubSet(pinfo *pi, dcube *a, dcube *b)
{
  register int i;
  for( i = 0; i < pi->out_words; i++ )
    if ( (a->out[i] & b->out[i]) != b->out[i] )
      return 0;
  return 1;
}


/*-- dcIsSubSet -------------------------------------------------------------*/

/* Ist b Teilmenge von a?                     */
/* Ja: Rueckgabe ist 1, nein: Rueckgabe ist 0 */
int dcIsSubSetReadable(pinfo *pi, dcube *a, dcube *b)
{
  register int i;

  for( i = 0; i < pi->in_words; i++ )
    if ( (a->in[i] & b->in[i]) != b->in[i] )
      return 0;
  for( i = 0; i < pi->out_words; i++ )
    if ( (a->out[i] & b->out[i]) != b->out[i] )
      return 0;
  return 1;
}

int dcIsSubSet(pinfo *pi, dcube *a, dcube *b)
{
  register int i;

  for( i = 0; i < pi->in_out_words_min; i++ )
  {
    if ( (~a->in[i] & b->in[i]) != 0 )
      return 0;
    if ( (~a->out[i] & b->out[i]) != 0 )
      return 0;
  }
  for( i = pi->in_out_words_min; i < pi->in_words; i++ )
    if ( (~a->in[i] & b->in[i]) != 0 )
      return 0;
  for( i = pi->in_out_words_min; i < pi->out_words; i++ )
    if ( (~a->out[i] & b->out[i]) != 0 )
      return 0;
  return 1;
}

int dcIsSubSet4(pinfo *pi, dcube *a1, dcube *a2, dcube *a3, dcube *a4, dcube *b)
{
  register int i;
  register c_int r1, r2, r3, r4;
  r1 = 0;
  r2 = 0;
  r3 = 0;
  r4 = 0;

  for( i = 0; i < pi->in_words; i++ )
  {
    r1 |= (~a1->in[i] & b->in[i]);
    r2 |= (~a2->in[i] & b->in[i]);
    r3 |= (~a3->in[i] & b->in[i]);
    r4 |= (~a4->in[i] & b->in[i]);
    if ( r1 != 0 && r2 != 0 && r3 != 0 && r4 != 0 )
      return 0;
  }
  for( i = 0; i < pi->out_words; i++ )
  {
    r1 |= (~a1->out[i] & b->out[i]);
    r2 |= (~a2->out[i] & b->out[i]);
    r3 |= (~a3->out[i] & b->out[i]);
    r4 |= (~a4->out[i] & b->out[i]);
    if ( r1 != 0 && r2 != 0 && r3 != 0 && r4 != 0 )
      return 0;
  }
  return 1;
}

int dcIsSubSet6n(pinfo *pi, dcube *a, dcube *b1, dcube *b2, dcube *b3, dcube *b4, dcube *b5, dcube *b6)
{
  register int i;
  register c_int r1, r2, r3, r4, r5, r6;
  register c_int aw;
  r1 = 0;
  r2 = 0;
  r3 = 0;
  r4 = 0;
  r5 = 0;
  r6 = 0;

  for( i = 0; i < pi->in_words; i++ )
  {
    aw = ~a->in[i];
    r1 |= (aw & b1->in[i]);
    r2 |= (aw & b2->in[i]);
    r3 |= (aw & b3->in[i]);
    r4 |= (aw & b4->in[i]);
    r5 |= (aw & b5->in[i]);
    r6 |= (aw & b6->in[i]);
    if ( r1 != 0 && r2 != 0 && r3 != 0 && r4 != 0 && r5 != 0 && r6 != 0 )
      return 0;
  }
  for( i = 0; i < pi->out_words; i++ )
  {
    aw = ~a->out[i];
    r1 |= (aw & b1->out[i]);
    r2 |= (aw & b2->out[i]);
    r3 |= (aw & b3->out[i]);
    r4 |= (aw & b4->out[i]);
    r5 |= (aw & b5->out[i]);
    r6 |= (aw & b6->out[i]);
    if ( r1 != 0 && r2 != 0 && r3 != 0 && r4 != 0 && r5 != 0 && r6 != 0 )
      return 0;
  }
  return 1;
}

/*-- dcInDCCnt --------------------------------------------------------------*/

int dcInDCCnt(pinfo *pi, dcube *cube)
{
  register c_int c;
  register int bc = 0;
  register int i;
  for( i = 0; i < pi->in_words; i++ )
  {
    c = cube->in[i];          /* Problem:      Wie oft kommt 11 vor? */
    c = ~c;                   /* Problem:      Wie oft kommt 00 vor? */
    c |= c>>1;                /* Reduktion:    Wie oft kommt x0 vor? */
    c = ~c;                   /* Invertierung: Wie oft kommt x1 vor? */
    c &= CUBE_IN_MASK_ZERO;   /* Maskierung:   Wie oft kommt 01 vor? */
    bc += bitcount(c);
  }
  if ( ((pi->in_cnt)&(CUBE_SIGNALS_PER_IN_WORD-1)) != 0 )
    bc -= CUBE_SIGNALS_PER_IN_WORD-((pi->in_cnt)&(CUBE_SIGNALS_PER_IN_WORD-1));  
  return bc;
}

/*-- dcInDCMask--------------------------------------------------------------*/

void dcInDCMask(pinfo *pi, dcube *cube)
{
  register c_int c;
  register int i;
  for( i = 0; i < pi->in_words; i++ )
  {
    c = cube->in[i];          /* Problem:      Wie oft kommt 11 vor? */
    c &= c>>1;                /* Reduktion:    Wie oft kommt x1 vor? */
    c &= CUBE_IN_MASK_ZERO;   /* Maskierung:   Wie oft kommt 01 vor? */
    c |= c<<1;
    cube->in[i] = c;
  }
}


/*-- dcInZeroCnt ------------------------------------------------------------*/

int dcInZeroCnt(pinfo *pi, dcube *cube)
{
  register c_int c;
  register int bc = 0;
  register int i;
  for( i = 0; i < pi->in_words; i++ )
  {
    c = cube->in[i];          /* Problem:      Wie oft kommt 11 vor? */
    c = ~c;                   /* Problem:      Wie oft kommt 00 vor? */
    c |= c>>1;                /* Reduktion:    Wie oft kommt x0 vor? */
    c = ~c;                   /* Invertierung: Wie oft kommt x1 vor? */
    c &= CUBE_IN_MASK_ZERO;   /* Maskierung:   Wie oft kommt 01 vor? */
    c |= c<<1;
    c = cube->in[i] & ~c;
    c &= CUBE_IN_MASK_ZERO;   /* Maskierung:   Wie oft kommt 01 vor? */
    bc += bitcount(c);
  }
  return bc;
}

/*-- dcInOneCnt -------------------------------------------------------------*/

int dcInOneCnt(pinfo *pi, dcube *cube)
{
  register c_int c;
  register int bc = 0;
  register int i;
  for( i = 0; i < pi->in_words; i++ )
  {
    c = cube->in[i];          /* Problem:      Wie oft kommt 11 vor? */
    c = ~c;                   /* Problem:      Wie oft kommt 00 vor? */
    c |= c>>1;                /* Reduktion:    Wie oft kommt x0 vor? */
    c = ~c;                   /* Invertierung: Wie oft kommt x1 vor? */
    c &= CUBE_IN_MASK_ZERO;   /* Maskierung:   Wie oft kommt 01 vor? */
    c |= c<<1;
    c = cube->in[i] & ~c;
    c = c>>1;
    c &= CUBE_IN_MASK_ZERO;   /* Maskierung:   Wie oft kommt 01 vor? */
    bc += bitcount(c);
  }
  return bc;
}

/*-- dcDeltaIn --------------------------------------------------------------*/

int dcDeltaIn(pinfo *pi, dcube *a, dcube *b)
{
  register c_int c;
  register int bc = 0;
  register int i;
  for( i = 0; i < pi->in_words; i++ )
  {
    c = a->in[i] & b->in[i];  /* Problem:      Wie oft kommt 00 vor? */
    c |= c>>1;                /* Reduktion:    Wie oft kommt x0 vor? */
    c = ~c;                   /* Invertierung: Wie oft kommt x1 vor? */
    c &= CUBE_IN_MASK_ZERO;   /* Maskierung:   Wie oft kommt 01 vor? */
    bc += bitcount(c);
  }
  return bc;
}

/*-- dcOutCnt ---------------------------------------------------------------*/

int dcOutCnt(pinfo *pi, dcube *c)
{
  register int i;
  register int bc = 0;
  for( i = 0; i < pi->out_words; i++ )
    bc += bitcount(c->out[i]);
  return bc;
}

/*-- dcInvIn ----------------------------------------------------------------*/

int dcInvIn(pinfo *pi, dcube *cube)
{
  register c_int c;
  register int i;
  for( i = 0; i < pi->in_words; i++ )
  {
    c = cube->in[i];          /* Problem:      Wie oft kommt 11 vor? */
    c &= c>>1;                /* Reduktion:    Wie oft kommt x1 vor? */
    c &= CUBE_IN_MASK_ZERO;   /* Maskierung:   Wie oft kommt 01 vor? */
    c |= c<<1;                /* neue maske: 11 */
    cube->in[i] = ((~cube->in[i]) | c);   /* 01 -> 10, 10 -> 01, 11 -> 11 */
  }
  return 0;
}

/*-- dcInvOut ---------------------------------------------------------------*/

void dcInvOut(pinfo *pi, dcube *c)
{
  register int i;
  for( i = 0; i < pi->out_words; i++ )
    c->out[i] = ~c->out[i];
  if ( pi->out_words > 0 )
    c->out[pi->out_words-1] &= pi->out_last_mask;
}

/*-- dcDeltaOut -------------------------------------------------------------*/

int dcDeltaOut(pinfo *pi, dcube *a, dcube *b)
{
  register int i;
  if ( pi->out_cnt == 0 )
    return 0;
  for( i = 0; i < pi->out_words; i++ )
    if ( (a->out[i] & b->out[i]) != 0 )
      return 0;
  return 1;
}

/*-- dcDelta ----------------------------------------------------------------*/

int dcDelta(pinfo *pi, dcube *a, dcube *b)
{
  return dcDeltaIn(pi, a, b) + dcDeltaOut(pi, a, b);
}

/*-- dcIsOutIllegal ---------------------------------------------------------*/

int dcIsOutIllegal(pinfo *pi, dcube *c)
{
  register int i;
  if ( pi->out_words == 0 )
    return 0;
  for( i = 0; i < pi->out_words; i++ )
  {
    if ( c->out[i] != 0 )
      break;
  }
  if ( i >= pi->out_words )
    return 1;
  return 0;
}

/*-- dcIsDeltaInNoneZero ----------------------------------------------------*/

int dcIsDeltaInNoneZero(pinfo *pi, dcube *a, dcube *b)
{
  register c_int c;
  register int i;
  for( i = 0; i < pi->in_words; i++ )
  {
    c = a->in[i] & b->in[i];  /* Problem:      Wie oft kommt 00 vor? */
    c |= c>>1;                /* Reduktion:    Wie oft kommt x0 vor? */
    c = ~c;                   /* Invertierung: Wie oft kommt x1 vor? */
    c &= CUBE_IN_MASK_ZERO;   /* Maskierung:   Wie oft kommt 01 vor? */
    if ( c > 0 )
      return 1;
  }
  return 0;
}

/*-- dcIsInIllegal ----------------------------------------------------------*/

int dcIsInIllegal(pinfo *pi, dcube *cube)
{
  register c_int c;
  register int i;
  for( i = 0; i < pi->in_words; i++ )
  {
    c = cube->in[i];          /* Problem:      Wie oft kommt 00 vor? */
    c |= c>>1;                /* Reduktion:    Wie oft kommt x0 vor? */
    c = ~c;                   /* Invertierung: Wie oft kommt x1 vor? */
    c &= CUBE_IN_MASK_ZERO;   /* Maskierung:   Wie oft kommt 01 vor? */
    if ( c > 0 )
      return 1;
  }
  return 0;
}

/*-- dcIsDeltaNoneZero ------------------------------------------------------*/

int dcIsDeltaNoneZero(pinfo *pi, dcube *a, dcube *b)
{
  if ( dcDeltaOut(pi, a, b) > 0 )
    return 1;
  return dcIsDeltaInNoneZero(pi, a, b);
}

/*-- dcIsIllegal ------------------------------------------------------------*/

int dcIsIllegal(pinfo *pi, dcube *c)
{
  if ( dcIsInIllegal(pi, c) != 0 )
    return 1;
  if ( dcIsOutIllegal(pi, c) != 0 )
    return 1;
  return 0;
}

/*-- dcCofactor -------------------------------------------------------------*/

/* Berechnet den Kofaktor von a bezueglich b.  */
/* Das Ergebnis wird in r abgelegt.            */
/* Der Reuckgabewert ist 0, wenn kein Ergebnis */
/* berechnet wurde.                            */

static void dc_cofactor(pinfo *pi, dcube *r, dcube *a, dcube *b)
{
  register int i;
  for( i = 0; i < pi->in_words; i++ )
    r->in[i] = a->in[i] | ~b->in[i];
  if ( pi->out_words > 0 )
  {
    for( i = 0; i < pi->out_words-1; i++ )
      r->out[i] = a->out[i] | (~b->out[i]);
    r->out[i] = a->out[i] | ((~b->out[i]) & pi->out_last_mask);
  }
  r->n = a->n; /* notwendig fuer den irredundant algorithmus */
}

int dcCofactor(pinfo *pi, dcube *r, dcube *a, dcube *b)
{
  if ( dcIsDeltaNoneZero(pi, a, b) != 0 )
    return 0;
  dc_cofactor(pi, r, a, b);
  return 1;
}

/* cofactoren, die teilmenge des cofactors sind */
int dcSubCofactor(pinfo *pi, dcube *r, dcube *a, dcube *b)
{
  if ( dcIsSubSet(pi, b, a) == 0 )
    return 0;
  dc_cofactor(pi, r, a, b);
  return 1;
}

/* cofactoren, die reduziert werden muessen */
int dcRedCofactor(pinfo *pi, dcube *r, dcube *a, dcube *b)
{
  if ( dcIsSubSet(pi, b, a) != 0 )
    return 0;
  return dcCofactor(pi, r, a, b);
}


/*-- dcConsensus ------------------------------------------------------------*/

int dcConsensus(pinfo *pi, dcube *r, dcube *a, dcube *b)
{
  int i;
  int d_in = dcDeltaIn(pi, a, b);
  int d_out = dcDeltaOut(pi, a, b);
  if ( d_in == 0  )
  {
    for( i = 0; i < pi->in_words; i++ )
      r->in[i] = a->in[i] & b->in[i];
    for( i = 0; i < pi->out_words; i++ )
      r->out[i] = a->out[i] | b->out[i];
  }
  else if ( d_in == 1 && d_out == 0 )
  {
    c_int c;
    c_int m;
    
    for( i = 0; i < pi->in_words; i++ )
    {
      c = a->in[i] & b->in[i];
      m = (~((c)|(c>>1))) & CUBE_IN_MASK_ZERO;
      r->in[i] = c | m | (m<<1);
    }
    for( i = 0; i < pi->out_words; i++ )
      r->out[i] = a->out[i] & b->out[i];
  }
  else
  {
    return 0;
  }
  return 1;
}

/*-- dcSharpIn --------------------------------------------------------------*/

int dcSharpIn(pinfo *pi, dcube *r, dcube *a, dcube *b, int k)
{
  register int i;
  dcAllClear(pi, &(pi->tmp[5]));
  dcSetIn(&(pi->tmp[5]), k, 3);
  
  for( i = 0; i < pi->in_words; i++ )
  {
    if ( pi->tmp[5].in[i] != 0 )
    {
      if ( ((a->in[i] & ~b->in[i]) & pi->tmp[5].in[i]) == 0 )
        return 0;
      r->in[i] = (a->in[i] & ~b->in[i]) & pi->tmp[5].in[i];
      r->in[i] |= a->in[i] & ~pi->tmp[5].in[i];
    }
    else
    {
      r->in[i] = a->in[i];
    }
  }
  
  for( i = 0; i < pi->out_words; i++ )
    r->out[i] = a->out[i];
    
  return 1;
}

/*-- dcSharpOut -------------------------------------------------------------*/

int dcSharpOut(pinfo *pi, dcube *r, dcube *a, dcube *b)
{
  int i;
  for( i = 0; i < pi->out_words; i++ )
    if ( (a->out[i] & ~b->out[i]) != 0 )
      break;
  if ( i >= pi->out_words )
    return 0;
  for( i = 0; i < pi->in_words; i++ )
    r->in[i] = a->in[i];
  for( i = 0; i < pi->out_words; i++ )
    r->out[i] = a->out[i] & ~b->out[i];
  return 1;
}

/*-- dcD1SharpIn ------------------------------------------------------------*/

int dcD1SharpIn(pinfo *pi, dcube *r, dcube *a, dcube *b, int k)
{
  register int i;
  dcAllClear(pi, &(pi->tmp[5]));
  dcSetIn(&(pi->tmp[5]), k, 3);
  
  for( i = 0; i < pi->in_words; i++ )
  {
    if ( pi->tmp[5].in[i] != 0 )
    {
      if ( ((a->in[i] & ~b->in[i]) & pi->tmp[5].in[i]) == 0 )
        return 0;
      r->in[i] = (a->in[i] & ~b->in[i]) & pi->tmp[5].in[i];
      r->in[i] |= b->in[i] & ~pi->tmp[5].in[i];
    }
    else
    {
      r->in[i] = b->in[i];
    }
  }
  
  for( i = 0; i < pi->out_words; i++ )
    r->out[i] = b->out[i];
    
  return 1;
}

/*-- dcD1SharpOut -----------------------------------------------------------*/

int dcD1SharpOut(pinfo *pi, dcube *r, dcube *a, dcube *b)
{
  int i;
  for( i = 0; i < pi->out_words; i++ )
    if ( (a->out[i] & ~b->out[i]) != 0 )
      break;
  if ( i >= pi->out_words )
    return 0;
  for( i = 0; i < pi->in_words; i++ )
    r->in[i] = b->in[i];
  for( i = 0; i < pi->out_words; i++ )
    r->out[i] = a->out[i] & ~b->out[i];
  return 1;
}

/*-- dcGetCofactorForSplit --------------------------------------------------*/

int dcGetBinateInVarCofactor(pinfo *pi, dcube *r, dcube *rinv, dclist cl, dcube *cof)
{
  int i;
  for( i = 0; i < pi->in_cnt; i++ )
  {
    if ( dclIsBinateInVar(cl, i) != 0 )
    {
      dcCopy(pi, r, cof);
      dcCopy(pi, rinv, cof);
      dcInSetAll(pi, r, CUBE_IN_MASK_DC);
      dcInSetAll(pi, rinv, CUBE_IN_MASK_DC);
      dcSetIn(r, i, 2);
      dcSetIn(rinv, i, 1);
      return 1;
    }
  }
  return 0;
}

int dcGetNoneDCInVarCofactor(pinfo *pi, dcube *r, dcube *rinv, dclist cl, dcube *cof)
{
  int i;
  for( i = 0; i < pi->in_cnt; i++ )
  {
    if ( dclIsDCInVar(pi, cl, i) == 0 )
    {
      dcCopy(pi, r, cof);
      dcCopy(pi, rinv, cof);
      dcInSetAll(pi, r, CUBE_IN_MASK_DC);
      dcInSetAll(pi, rinv, CUBE_IN_MASK_DC);
      dcSetIn(r, i, 2);
      dcSetIn(rinv, i, 1);
      return 1;
    }
  }
  return 0;
}

int dcGetOutVarCofactor(pinfo *pi, dcube *r, dcube *rinv, dclist cl, dcube *cof)
{
  int i, j;
  int aktive_bit_cnt;

  aktive_bit_cnt = 0;
  for( i = 0; i < pi->out_cnt; i++ )
    if ( dcGetOut(cof, i) != 0 )
      aktive_bit_cnt++;
  
  if ( aktive_bit_cnt <= 1 )
    return 0;

  dcInSetAll(pi, r, CUBE_IN_MASK_DC);
  dcOutSetAll(pi, r, 0);
  dcInSetAll(pi, rinv, CUBE_IN_MASK_DC);
  dcOutSetAll(pi, rinv, 0);
  
  j = 0;
  for( i = 0; i < pi->out_cnt; i++ )
    if ( dcGetOut(cof, i) != 0   )
    {
      if ( j < aktive_bit_cnt/2 )
        dcSetOut(r, i, 1);
      else
        dcSetOut(rinv, i, 1);
      j++;
    }
  return 1;
}

int dcGetCofactorForSplit(pinfo *pi, dcube *l, dcube *r, dclist cl, dcube *cof)
{
  /*
  if ( dcGetBinateInVarCofactor(pi, l, r, cl, cof) != 0 )
    return 1;
  if ( dcGetOutVarCofactor(pi, l, r, cl, cof) != 0 )
    return 1;
  return 0;
  */

  /*
  if ( pinfoGetInVarDCubeCofactor(pi, l, r, cl, cof) != 0 )
    return 1;
  if ( pinfoGetOutVarDCubeCofactor(pi, l, r, cl, cof) != 0 )
    return 1;
  */
    
  return pinfoGetDCubeCofactorForSplitting(pi, l, r, cl, cof);
}

int dcGetNoneDCCofactorForSplit(pinfo *pi, dcube *l, dcube *r, dclist cl, dcube *cof)
{
  /*
  if ( pinfoGetOutVarDCubeCofactor(pi, l, r, cl, cof) != 0 )
    return 1;
  */
  /*
  if ( pinfoGetInVarDCubeCofactor(pi, l, r, cl, cof) != 0 )
    return 1;
  if ( dcGetNoneDCInVarCofactor(pi, l, r, cl, cof) != 0 )
    return 1;
  if ( pinfoGetOutVarDCubeCofactor(pi, l, r, cl, cof) != 0 )
    return 1;
  */
  /*
  if ( pinfoGetDCubeCofactorForSplitting(pi, l, r, cl, cof) != 0 )
    return 1;
  */
  /* reihenfolge wichtig??? zuerst eingangs variablen */
  if ( pinfoGetInVarDCubeCofactor(pi, l, r, cl, cof) != 0 )
    return 1;
  if ( dcGetOutVarCofactor(pi, l, r, cl, cof) != 0 )
    return 1;
  if ( dcGetNoneDCInVarCofactor(pi, l, r, cl, cof) != 0 )
    return 1;
  /* jetzt die ausgangsvariablen */
  /* die pinfo out var function zerstoert das ergebnis hmmmm??? */
  return 0;
}


/*-- dcIsInTautology --------------------------------------------------------*/
int dcIsInTautology(pinfo *pi, dcube *c)
{
  register int i;
  for( i = 0; i < pi->in_words; i++ )
    if ( c->in[i] != CUBE_IN_MASK_DC )
      return 0;
  return 1;
}

/*-- dcIsTautology ----------------------------------------------------------*/
int dcIsTautology(pinfo *pi, dcube *c)
{
  register int i;
  for( i = 0; i < pi->in_words; i++ )
    if ( c->in[i] != CUBE_IN_MASK_DC )
      return 0;
  if ( pi->out_words > 0 )
  {
    for( i = 0; i < pi->out_words-1; i++ )
    {
      if ( c->out[i] != CUBE_OUT_MASK )
        return 0;
    }
    if ( (c->out[i]&pi->out_last_mask) != pi->out_last_mask )
      return 0;
  }
  return 1;
}

/*-- dcNot ------------------------------------------------------------------*/

void dcNot(pinfo *pi, dcube *c)
{
  register int i;
  for( i = 0; i < pi->in_words; i++ )
    c->in[i] = ~c->in[i];
  for( i = 0; i < pi->out_words; i++ )
    c->out[i] = ~c->out[i];
  if ( pi->out_words > 0 )
    c->out[pi->out_words-1] &= pi->out_last_mask;
}


/*-- dcOrIn -----------------------------------------------------------------*/

void dcOrIn(pinfo *pi, dcube *r, dcube *a, dcube *b)
{
  register int i;
  for( i = 0; i < pi->in_words; i++ )
    r->in[i] = a->in[i] | b->in[i];
}

/*-- dcOrOut ----------------------------------------------------------------*/

void dcOrOut(pinfo *pi, dcube *r, dcube *a, dcube *b)
{
  register int i;
  for( i = 0; i < pi->out_words; i++ )
    r->out[i] = a->out[i] | b->out[i];
}

/*-- dcOr -------------------------------------------------------------------*/

void dcOr(pinfo *pi, dcube *r, dcube *a, dcube *b)
{
  register int i;
  for( i = 0; i < pi->in_words; i++ )
    r->in[i] = a->in[i] | b->in[i];
  for( i = 0; i < pi->out_words; i++ )
    r->out[i] = a->out[i] | b->out[i];
}

/*-- dcAnd ------------------------------------------------------------------*/

void dcAnd(pinfo *pi, dcube *r, dcube *a, dcube *b)
{
  register int i;
  for( i = 0; i < pi->in_words; i++ )
    r->in[i] = a->in[i] & b->in[i];
  for( i = 0; i < pi->out_words; i++ )
    r->out[i] = a->out[i] & b->out[i];
}

/*-- dcAndIn ----------------------------------------------------------------*/

void dcAndIn(pinfo *pi, dcube *r, dcube *a, dcube *b)
{
  register int i;
  for( i = 0; i < pi->in_words; i++ )
    r->in[i] = a->in[i] & b->in[i];
}

/*-- dcNotAndOut -----------------------------------------------------------*/

void dcNotAndOut(pinfo *pi, dcube *r, dcube *a, dcube *b)
{
  register int i;
  for( i = 0; i < pi->out_words; i++ )
    r->out[i] = (~a->out[i]) & b->out[i];
}

/*-- dcXorOut -----------------------------------------------------------*/

void dcXorOut(pinfo *pi, dcube *r, dcube *a, dcube *b)
{
  register int i;
  for( i = 0; i < pi->out_words; i++ )
    r->out[i] = a->out[i] ^ b->out[i];
}

/*-- dcGetLiteralCnt --------------------------------------------------------*/

int dcGetLiteralCnt(pinfo *pi, dcube *c)
{
  return pi->in_cnt - dcInDCCnt(pi, c) + dcOutCnt(pi, c);
}

/*-- dcGetDichotomyWeight ---------------------------------------------------*/

int dcGetDichotomyWeight(pinfo *pi, dcube *c)
{
  int z = dcInZeroCnt(pi, c);
  int o = dcInOneCnt(pi, c);
  int w;
  
  w = z-o;
  if ( w < 0 )
    w = -w;
  
  w = pi->in_cnt - w + 1;

  return w;
}

/*-- dcExpand1 --------------------------------------------------------------*/

/* Spezial Expand algorithmus fuer das URP Complement */
static void dcExpand1(pinfo *pi, dcube *c, dclist cl_off)
{
  dcube *b = &(pi->tmp[7]);
  int delta_in;
  int delta_out;
  int i, cnt = dclCnt(cl_off);
  dcSetTautology(pi, b);
  dcInSetAll(pi, b, 0);
  dcOutSetAll(pi, b, 0);
  for( i = 0; i < cnt; i++ )
  {
    delta_in = dcDeltaIn(pi, c, dclGet(cl_off, i));
    delta_out = dcDeltaOut(pi, c, dclGet(cl_off, i));
    if ( delta_in + delta_out == 1 )
      dcOr(pi, b, b, dclGet(cl_off, i));
  }
  dcNot(pi, b);
  dcOr(pi, c, c, b);
}

/*-- dcWriteBin -------------------------------------------------------------*/

int dcWriteBin(pinfo *pi, dcube *c, FILE *fp)
{
  int i, j;
  dcube *a = &(pi->tmp[7]);
  c_int w;
  if ( pi->in_words != 0 )
  {
    for( i = 0; i < pi->in_words; i++ )
    {
      w = c->in[i];
      for( j = 0; j < sizeof(c_int); j++ )
      {
        ((unsigned char *)a->in)[i*sizeof(c_int)+j] = (unsigned char)(w&255);
        w>>=8;
      }
    }
    if ( b_io_Write(fp, pi->in_words*sizeof(c_int), (unsigned char *)a->in) == 0 )  
      return 0;
  }
  if ( pi->out_words != 0 )
  {
    for( i = 0; i < pi->out_words; i++ )
    {
      w = c->out[i];
      for( j = 0; j < sizeof(c_int); j++ )
      {
        ((unsigned char *)a->out)[i*sizeof(c_int)+j] = (unsigned char)(w&255);
        w>>=8;
      }
    }
    if ( b_io_Write(fp, pi->out_words*sizeof(c_int), (unsigned char *)a->out) == 0 )  
      return 0;
  }
  if ( b_io_WriteInt(fp, c->n) == 0 )
    return 0;
  return 1;
}

/*-- dcReadBin --------------------------------------------------------------*/

int dcReadBin(pinfo *pi, dcube *c, FILE *fp)
{
  int i, j;
  dcube *a = &(pi->tmp[7]);
  c_int w;
  if ( pi->in_words != 0 )
  {
    if ( b_io_Read(fp, pi->in_words*sizeof(c_int), (unsigned char *)a->in) == 0 )  
      return 0;
    for( i = 0; i < pi->in_words; i++ )
    {
      w = 0;
      for( j = 0; j < sizeof(c_int); j++ )
      {
        w<<=8;
        w |= (c_int)((unsigned char *)a->in)[i*sizeof(c_int)+sizeof(c_int)-1-j];
      }
      c->in[i] = w;
    }
  }
  if ( pi->out_words != 0 )
  {
    if ( b_io_Read(fp, pi->out_words*sizeof(c_int), (unsigned char *)a->out) == 0 )  
      return 0;
    for( i = 0; i < pi->out_words; i++ )
    {
      w = 0;
      for( j = 0; j < sizeof(c_int); j++ )
      {
        w<<=8;
        w |= (c_int)((unsigned char *)a->out)[i*sizeof(c_int)+sizeof(c_int)-1-j];
      }
      c->out[i] = w;
    }
  }
  if ( b_io_ReadInt(fp, &(c->n)) == 0 )
    return 0;
  return 1;
}

/*===========================================================================*/

/*-- dclInit ----------------------------------------------------------------*/

int dclInit(dclist *cl)
{
  *cl = (dclist)malloc(sizeof(struct _dclist_struct));
  if ( *cl == NULL )
    return 0;
  (*cl)->list = NULL;
  (*cl)->max = 0;
  (*cl)->cnt = 0;
  (*cl)->flag_list = NULL;
  return 1;
}

/*-- dclInitCached ----------------------------------------------------------*/

int dclInitCached(pinfo *pi, dclist *cl)
{
  if ( pi->cache_cnt == 0 )
    return dclInit(cl);
  pi->cache_cnt--;
  *cl = pi->cache_cl[pi->cache_cnt];
  return 1;
}

/*-- dclInitVA --------------------------------------------------------------*/

int dclInitVA(int n, ...)
{
  va_list va;
  int i;
  va_start(va, n);
  for( i = 0; i < n; i++ )
    if ( dclInit(va_arg(va, dclist *)) == 0 )
      break;
  va_end(va);
  if ( i < n )
  {
    va_start(va, n);
    while( i-- >= 0 )
      dclDestroy(*va_arg(va, dclist *));
    va_end(va);
    return 0;
  }
  return 1;
}

/*-- dclInitCachedVA --------------------------------------------------------*/

int dclInitCachedVA(pinfo *pi, int n, ...)
{
  va_list va;
  int i;
  va_start(va, n);
  for( i = 0; i < n; i++ )
    if ( dclInitCached(pi, va_arg(va, dclist *)) == 0 )
      break;
  va_end(va);
  if ( i < n )
  {
    va_start(va, n);
    while( i-- >= 0 )
      dclDestroyCached(pi, *va_arg(va, dclist *));
    va_end(va);
    return 0;
  }
  return 1;
}

/*-- dclDestroy -------------------------------------------------------------*/

static void dcl_destroy(dclist cl)
{
  int i;
  for( i = 0; i < cl->max; i++ )
    dcDestroy(cl->list+i);
  if ( cl->list != NULL )
    free(cl->list);
  if ( cl->flag_list != NULL )
    free(cl->flag_list);
  cl->flag_list = NULL;
  cl->list = NULL;
  cl->max = 0;
  cl->cnt = 0;
}

void dclDestroy(dclist cl)
{
  dcl_destroy(cl);
  free(cl);
}

/*-- dclDestroyCached -------------------------------------------------------*/

void dclDestroyCached(pinfo *pi, dclist cl)
{
  if ( pi->cache_cnt >= PINFO_CACHE_CL )
    dclDestroy(cl);
  else
  {
    dclClear(cl);
    pi->cache_cl[pi->cache_cnt++] = cl;
  }
}

/*-- dclDestroyVA -----------------------------------------------------------*/

void dclDestroyVA(int n, ...)
{
  va_list va;
  int i;
  va_start(va, n);
  for( i = 0; i < n; i++ )
    dclDestroy(va_arg(va, dclist));
  va_end(va);
}

/*-- dclDestroyCachedVA -----------------------------------------------------*/

void dclDestroyCachedVA(pinfo *pi, int n, ...)
{
  va_list va;
  int i;
  va_start(va, n);
  for( i = 0; i < n; i++ )
    dclDestroyCached(pi, va_arg(va, dclist));
  va_end(va);
}

/*-- dclExpandFlagListTo ----------------------------------------------------*/

int dclExpandFlagListTo(dclist cl, int max)
{
  void *ptr;
  /* allocate at least one byte */
  if ( max <= 0 )
    max = 1;
  if ( cl->flag_list != NULL )
  {
    if ( max == 0 )
    {
      /* 14 jan 2002: assigning a NULL pointer seems to be an error... */
      /*
      free(cl->flag_list);
      cl->flag_list = NULL;
      return 1;
      */
      /* instead, allocate at least one byte */
      max = 1;
    }
    ptr = realloc(cl->flag_list, max);
  }
  else
  {
    ptr = malloc(max);
  }
  if ( ptr == NULL )
    return 0;
  cl->flag_list = (char *)ptr;
  return 1;
}

/*-- dclExpandTo ------------------------------------------------------------*/

int dclExpandTo(pinfo *pi, dclist cl, int max)
{
  void *ptr;
  max = (max+31)&~31;
  if ( max <= cl->max )      
    return 1;
  if ( cl->list == NULL )   
    ptr = malloc(max*sizeof(dcube));
  else                      
    ptr = realloc(cl->list, max*sizeof(dcube));
  if ( ptr == NULL )
    return 0;
  cl->list = (dcube *)ptr;
  
  if ( cl->flag_list != NULL )
    if ( dclExpandFlagListTo(cl, max) == 0 )
      return 0;
  while(cl->max < max)
  {
    if ( dcInitMem(pi, cl->list+cl->max) == 0 )
      return 0;
    if ( cl->flag_list != NULL )
      cl->flag_list[cl->max] = 0;
    cl->max++;
  }
  return 1;
}

/*-- dclAddEmpty ------------------------------------------------------------*/
/* returns position or -1 */
int dclAddEmpty(pinfo *pi, dclist cl)
{
  if ( dclCnt(cl) >= cl->max )
    if ( dclExpandTo(pi, cl, dclCnt(cl)+1) == 0 )
      return -1;
  dcInSetAll(pi, cl->list+cl->cnt, CUBE_IN_MASK_DC);
  dcOutSetAll(pi, cl->list+cl->cnt, 0);
  if ( cl->flag_list != NULL )
    cl->flag_list[cl->cnt] = 0;
  cl->cnt++;
  return cl->cnt-1;
}

dcube *dclAddEmptyCube(pinfo *pi, dclist cl)
{
  int pos;
  pos = dclAddEmpty(pi, cl);
  if ( pos < 0 )
    return NULL;
  return dclGet(cl, pos);
}


/*-- dclAdd -----------------------------------------------------------------*/

/* returns position or -1 */
int dclAdd(pinfo *pi, dclist cl, dcube *c)
{
  if ( dclCnt(cl) >= cl->max )
  {
    /* there is an interessting possible bug here */
    /* if c is an element of cl, c might become invalid */
    /* if cl->list is expanded and moved to another */
    /* location in the memory, c becomes invalid */
    if ( c >= cl->list && c < cl->list+cl->max )
    {
      dcCopy(pi, pi->tmp+12, c);
      c = pi->tmp+12;
    }
    if ( dclExpandTo(pi, cl, dclCnt(cl)+1) == 0 )
      return -1;
    assert(cl->max > cl->cnt);
  }

  {  
    register dcube *d;
    int i;
    d = cl->list+cl->cnt;
    for( i = 0; i < pi->in_out_words_min; i++ )
    {
      d->in[i] = c->in[i];
      d->out[i] = c->out[i];
    }
    for( i = pi->in_out_words_min; i < pi->in_words; i++ )
      d->in[i] = c->in[i];
    for( i = pi->in_out_words_min; i < pi->out_words; i++ )
      d->out[i] = c->out[i];
    d->n = c->n;
  }
  /* dcCopy(pi, cl->list+cl->cnt, c); */
  
  
  if ( cl->flag_list != NULL )
    cl->flag_list[cl->cnt] = 0;
  cl->cnt++;
  return cl->cnt-1;
}

/*-- dclAddUnique -----------------------------------------------------------*/

/* returns position or -1 */
int dclAddUnique(pinfo *pi, dclist cl, dcube *c)
{
  int i, cnt = dclCnt(cl);
  for( i = 0; i < cnt; i++ )
    if ( dcIsEqual(pi, dclGet(cl, i), c) != 0 )
      return i;
  return dclAdd(pi, cl, c);
}

/*-- dclJoin ----------------------------------------------------------------*/

int dclJoin(pinfo *pi, dclist dest, dclist src)
{
  int i, cnt = dclCnt(src);
  if ( dclExpandTo(pi, dest, dclCnt(dest)+dclCnt(src)) == 0 )
    return 0;
  for( i = 0; i < cnt; i++ )
    if ( dclAdd(pi, dest, dclGet(src, i)) < 0 )
      return 0;
  return 1;
}

/* only join on set of the provided output, SCC might be required after this operation */
int dclJoinByOut(pinfo *pi, dclist dest, dclist src, int out_pos)
{
  int i, cnt = dclCnt(src);
  int idx;
  for( i = 0; i < cnt; i++ )
  {
    if ( dcGetOut(dclGet(src, i), out_pos) )
    {
      idx = dclAdd(pi, dest, dclGet(src, i));
      if ( idx < 0 )
	return 0;
      dcOutSetAll(pi, dclGet(dest, idx), 0);	/* clear all outputs */
      dcSetOut(dclGet(dest, idx), out_pos, 1);	/* but keep the current one */
    }
  }
  return 1;
}


/*-- dclCopy ----------------------------------------------------------------*/

int dclCopy(pinfo *pi, dclist dest, dclist src)
{
  dclClear(dest);
  return dclJoin(pi, dest, src);
}

int dclCopyByOut(pinfo *pi, dclist dest, dclist src, int out_pos)
{
  dclClear(dest);
  return dclJoinByOut(pi, dest, src, out_pos);
}

/*-- dclClearFlags ----------------------------------------------------------*/

int dclClearFlags(dclist cl)
{
  if ( dclExpandFlagListTo(cl, cl->max) == 0 )
    return 0;
  if ( cl->max == 0 )
    return 1;
  memset(cl->flag_list, 0, cl->max);
  return 1;
}

/*-- dclSetFlag -------------------------------------------------------------*/

/*
void dclSetFlag(dclist cl, int pos)
{
  cl->flag_list[pos] = 1;
}
*/

/*-- dclIsFlag --------------------------------------------------------------*/

/*
int dclIsFlag(dclist cl, int pos)
{
  return (int)cl->flag_list[pos];
}
*/

/*-- dclDeleteCubesWithFlag -------------------------------------------------*/

void dclDeleteCubesWithFlag(pinfo *pi, dclist cl)
{
  int src, dest;
  
  src = 0;
  dest = 0;
  while( src < cl->cnt )
  {
    if ( cl->flag_list[src] != 0 )
    {
      src++;
    }
    else
    {
      if ( src != dest )
      {
        dcCopy(pi, cl->list+dest, cl->list+src);
        cl->flag_list[dest] = cl->flag_list[src];
      }

      dest++;
      src++;
    }
  }
  memset(cl->flag_list, 0, cl->cnt);
  cl->cnt = dest;
}

/*-- dclCopyCubesWithFlag ----------------------------------------------------*/

int dclCopyCubesWithFlag(pinfo *pi, dclist dest, dclist src)
{
  int i, cnt = dclCnt(src);
  
  dclClear(dest);
  for( i = 0; i < cnt; i++ )
    if ( dclIsFlag(src, i) != 0 )
      if ( dclAdd(pi, dest, dclGet(src, i)) < 0 )
        return 0;

  return 1;
}

/*-- dclDeleteCube ----------------------------------------------------------*/

void dclDeleteCube(pinfo *pi, dclist cl, int pos)
{
  if ( cl->cnt == 0 )
    return;
  if ( pos >= cl->cnt )
    return;

  pos++;
  while( pos < cl->cnt )
  {
    dcCopy(pi, cl->list+pos-1, cl->list+pos);
    pos++;
  }
  /* dcDestroy(cl->list+cl->cnt-1); */
  cl->cnt--;
}


/*-- dclDeleteByCube --------------------------------------------------------*/

int dclDeleteByCube(pinfo *pi, dclist cl, dcube *c)
{
  int i, cnt = dclCnt(cl);
  if ( dclClearFlags(cl) == 0 )
    return 0;
  for( i = 0; i < cnt; i++ )
  {
    if ( dcIsEqual(pi, c, dclGet(cl, i)) != 0 ) 
      dclSetFlag(cl, i);
  }
  dclDeleteCubesWithFlag(pi, cl);
  return 1;
}

/*-- dclDeleteByCubeList ----------------------------------------------------*/

int dclDeleteByCubeList(pinfo *pi, dclist cl, dclist del)
{
  int i, cnt = dclCnt(del);
  for( i = 0; i < cnt; i++ )
    if ( dclDeleteByCube(pi, cl, dclGet(del, i)) == 0 )
      return 0;
  return 1;
}

/*-- dclClear ---------------------------------------------------------------*/

void dclClear(dclist cl)
{
  cl->cnt = 0;
}

/*-- dclRealClear -----------------------------------------------------------*/

void dclRealClear(dclist cl)
{
  dcl_destroy(cl);
}

/*-- dclCnt -----------------------------------------------------------------*/
/*
int dclCnt(dclist cl)
{
  return cl->cnt;
}
*/

/*-- dclGet -----------------------------------------------------------------*/

/*
dcube *dclGet(dclist cl, int pos)
{
  return cl->list + pos;
}
*/

/*-- dclGetPinfoOutLength ---------------------------------------------------*/

int dclSetPinfoByLength(pinfo *pi, dclist cl)
{
  if ( pinfoSetInCnt(pi, 1) == 0 )
    return 0;
  if ( pinfoSetOutCnt(pi, dclCnt(cl)) == 0 )
    return 0;
  return 1;
}

/*-- dclInvertOutMatrix -----------------------------------------------------*/

int dclInvertOutMatrix(pinfo *dest_pi, dclist dest_cl, pinfo *src_pi, dclist src_cl)
{ 
  int i, cnt = dclCnt(src_cl);
  int j;
  dcube *inv = &(dest_pi->tmp[1]);
  
  dclRealClear(dest_cl);
  if ( dclSetPinfoByLength(dest_pi, src_cl) == 0 )
    return 0;

  for( j = 0; j < src_pi->out_cnt; j++ )
  {    
    dcSetTautology(dest_pi, inv);
    for( i = 0; i < cnt; i++ )
      if ( dcGetOut(dclGet(src_cl, i), j) == 0 )
        dcSetOut(inv, i, 0);
    if ( dclAdd(dest_pi, dest_cl, inv) < 0 )
      return 0;
  }
  return 1;
}

/*-- dclReadFP --------------------------------------------------------------*/

#define DCL_LINE_LEN (1024*32)

int dclReadFP(pinfo *pi, dclist cl, FILE *fp)
{
  static char s[DCL_LINE_LEN];
  dcube c;
  int is_cube_init = 0;
  
  dclRealClear(cl);
  for(;;)
  {
    if ( fgets(s, DCL_LINE_LEN, fp) == NULL )
      break;
    if ( s[0] == '#' )
    {
      /* nothing */
    }
    else if ( s[0] == '.' && is_cube_init == 0 )
    {
      if ( strncmp(s, ".ilb ", 5) == 0 )
      {
        /* input label names */
        if ( pinfoImportInLabels(pi, s+5, " \t\n\r") == 0 )
          return 0;
      }  
      else if ( strncmp(s, ".olb ", 5) == 0 || strncmp(s, ".ob ", 4) == 0 )
      {
        /* this is not a valid espresso command... */
        /* output label names */
        if ( pinfoImportOutLabels(pi, s+5, " \t\n\r") == 0 )
          return 0;
      }
      else if ( strncmp(s, ".ob ", 4) == 0 )
      {
        /* output label names */
        if ( pinfoImportOutLabels(pi, s+4, " \t\n\r") == 0 )
          return 0;
      }
      else if ( strncmp(s, ".i ", 3) == 0 )
        pinfoSetInCnt(pi, atoi(s+3));
      else if ( strncmp(s, ".o ", 3) == 0 )
        pinfoSetOutCnt(pi, atoi(s+3));
    }
    else if ( s[0] == '.' )
    {
      /* ignore */
    }
    else
    {
      if ( is_cube_init == 0 )
      {
        if ( dcInit(pi, &c) == 0 )
          return 0;
        is_cube_init = 1;
      }

      if ( dcSetByStr(pi, &c, s) == 0 )
        return dcDestroy(&c), 0;
      if ( dcIsOutIllegal(pi, &c) == 0 )
        if ( dclAdd(pi, cl, &c) < 0 )
          return dcDestroy(&c), 0;
    }
  }
  if ( is_cube_init != 0 )
    dcDestroy(&c);
  return 1;
}

/*-- dclReadCNFFP -----------------------------------------------------------*/
int dclReadCNFFP(pinfo *pi, dclist cl, FILE *fp)
{
  static char s[DCL_LINE_LEN];
  dcube *c = &(pi->tmp[3]);
  char *t;
  int is_init = 0;
  int cnt = -1; 
  int pos;
  
  pinfoSetOutCnt(pi, 1);

  dclRealClear(cl);

  for(;;)
  {
    if ( fgets(s, DCL_LINE_LEN, fp) == NULL )
      break;
    if ( s[0] == '\0' || s[0] == 'c' )
    {
      /* comment line */
    }
    else if ( s[0] == 'p' && is_init == 0 )
    {
      t = s+1;
      while(*t <= ' ' && *t > 0)
        t++;
      if ( strncmp(t, "cnf", 3) != 0 )
        return 0;
      t+=3;
      while(*t <= ' ' && *t > 0)
        t++;
      pinfoSetInCnt(pi, atoi(t));
      cnt = atoi(t);
      is_init = 1;
      dcInSetAll(pi, c, CUBE_IN_MASK_DC);
      dcOutSetAll(pi, c, 0);
      dcSetOut(c, 0, 1);
    }
    else if ( is_init != 0 )
    {
      t = s;
      for(;;)
      {
        while(*t <= ' ' && *t > 0)
          t++;
        if ( *t == '\0' )
          break;
        pos = strtol(t, &t, 10);
        if ( pos == 0 )
        {
          if ( cl != NULL )
            if ( pi->out_cnt == 0 || dcIsOutIllegal(pi, c) == 0 )
              if ( dclAdd(pi, cl, c) < 0 )
                return 0;
          dcInSetAll(pi, c, CUBE_IN_MASK_DC);
          dcOutSetAll(pi, c, 0);
          dcSetOut(c, 0, 1);
        }
        else if ( pos < 0 )
        {
          dcSetIn(c, -pos-1, 1);
        }
        else
        {
          dcSetIn(c, pos-1, 2);
        }
      }
    }
  }
  return 1;
}

/*-- dclReadCNF -------------------------------------------------------------*/

int dclReadCNF(pinfo *pi, dclist cl, const char *filename)
{
  FILE *fp;
  int ret;
  fp = b_fopen(filename, NULL, ".cnf", "r");
  if ( fp == NULL )
    return 0;
  ret = dclReadCNFFP(pi, cl, fp);
  fclose(fp);
  return ret;
}


/*-- dclReadPLAFP -----------------------------------------------------------*/

int dclReadPLAFP(pinfo *pi, dclist cl_on, dclist cl_dc, FILE *fp)
{
  static char buf[DCL_LINE_LEN];
  static char *s = buf;
  dcube *c_on = &(pi->tmp[3]);
  dcube *c_dc = &(pi->tmp[4]);
  int is_init = 0;
  
  if ( cl_on != NULL )
    dclRealClear(cl_on);
  if ( cl_dc != NULL )
    dclRealClear(cl_dc);
  for(;;)
  {
    s = buf;
    if ( fgets(s, DCL_LINE_LEN, fp) == NULL )
      break;
    if ( s[0] == '#' )
    {
      /* nothing */
    }
    else if ( s[0] < ' ' )
    {
      /* nothing */
    }
    else if ( s[0] == '.' && is_init == 0 )
    {
      if ( strncmp(s, ".ilb ", 5) == 0 )
      {
        /* input label names */
        if ( pinfoImportInLabels(pi, s+5, " \t\n\r") == 0 )
          return 0;
      }  
      else if ( strncmp(s, ".olb ", 5) == 0 )
      {
        /* this is not a valid espresso command... */
        /* output label names */
        if ( pinfoImportOutLabels(pi, s+5, " \t\n\r") == 0 )
          return 0;
      }
      else if ( strncmp(s, ".ob ", 4) == 0 )
      {
        /* output label names */
        if ( pinfoImportOutLabels(pi, s+4, " \t\n\r") == 0 )
          return 0;
      }
      else if ( strncmp(s, ".i ", 3) == 0 )
        pinfoSetInCnt(pi, atoi(s+3));
      else if ( strncmp(s, ".o ", 3) == 0 )
        pinfoSetOutCnt(pi, atoi(s+3));
    }
    else if ( s[0] == '.' )
    {
      /* ignore */
    }
    else if ( s[0] != '\0' )
    {
      is_init = 1;
      if ( pi->in_cnt == 0 || pi->out_cnt == 0 )
        return 0;
      while( (*s) > '\0' && (*s) < ' ' )
        s++;
      if ( *s == '\0' )
        continue;
      s = dcSetAllByStr(pi, pi->in_cnt, pi->out_cnt, c_on, c_dc, s);
      if ( s == NULL )
        return 0;
      while( (*s) > '\0' && (*s) < ' ' )
        s++;
      if ( *s != '#' && *s != '\0' )
        return 0;
      if ( cl_on != NULL )
        if ( pi->out_cnt == 0 || dcIsOutIllegal(pi, c_on) == 0 )
          if ( dclAdd(pi, cl_on, c_on) < 0 )
            return 0;
      if ( cl_dc != NULL )
        if ( dcIsOutIllegal(pi, c_dc) == 0 )
          if ( dclAdd(pi, cl_dc, c_dc) < 0 )
            return 0;
    }
  }

  if ( cl_dc != NULL && cl_on != NULL )
    if ( dclSubtract(pi, cl_on, cl_dc) == 0 )
      return 0;
      
  return 1;
}

/*-- dclReadPLAStr ----------------------------------------------------------*/

int dclReadPLAStr(pinfo *pi, dclist cl_on, dclist cl_dc, const char **t)
{
  static char s[DCL_LINE_LEN];
  dcube *c_on = &(pi->tmp[3]);
  dcube *c_dc = &(pi->tmp[4]);
  int is_init = 0;
  
  if ( cl_on != NULL )
    dclRealClear(cl_on);
  if ( cl_dc != NULL )
    dclRealClear(cl_dc);
  for(;;)
  {
    if ( *t == NULL )
      break;
    strcpy(s, *t);
    t++;
    if ( s[0] == '#' )
    {
      /* nothing */
    }
    else if ( s[0] == '.' && is_init == 0 )
    {
      if ( strncmp(s, ".i ", 3) == 0 )
        pinfoSetInCnt(pi, atoi(s+3));
      else if ( strncmp(s, ".o ", 3) == 0 )
        pinfoSetOutCnt(pi, atoi(s+3));
    }
    else if ( s[0] == '.' )
    {
      /* ignore */
    }
    else
    {
      is_init = 1;
      if ( dcSetAllByStr(pi, pi->in_cnt, pi->out_cnt, c_on, c_dc, s) == 0 )
        return 0;
      if ( cl_on != NULL )
        if ( dcIsOutIllegal(pi, c_on) == 0 )
          if ( dclAdd(pi, cl_on, c_on) < 0 )
            return 0;
      if ( cl_dc != NULL )
        if ( dcIsOutIllegal(pi, c_dc) == 0 )
          if ( dclAdd(pi, cl_dc, c_dc) < 0 )
            return 0;
    }
  }

  if ( cl_dc != NULL && cl_on != NULL )
    if ( dclSubtract(pi, cl_on, cl_dc) == 0 )
      return 0;
      
  return 1;
}

/*-- dclReadFile ------------------------------------------------------------*/

int dclReadFile(pinfo *pi, dclist cl, const char *filename)
{
  FILE *fp;
  int ret;
  fp = b_fopen(filename, NULL, ".pla", "r");
  if ( fp == NULL )
    return 0;
  ret = dclReadFP(pi, cl, fp);
  fclose(fp);
  return ret;
}


/*-- dclReadDichotomyFP -----------------------------------------------------*/

int dclReadDichotomyFP(pinfo *pi, dclist cl_on, FILE *fp)
{
  static char buf[DCL_LINE_LEN];
  static char *s = buf;
  dcube *c_on = &(pi->tmp[3]);
  int is_init = 0;
  
  if ( cl_on != NULL )
    dclRealClear(cl_on);
  for(;;)
  {
    s = buf;
    if ( fgets(s, DCL_LINE_LEN, fp) == NULL )
      break;
    while( *s == ' ' || *s == '\t' )
      s++;
    if ( s[0] == '#' )
    {
      /* nothing */
    }
    else if ( s[0] == '\0' )
    {
      /* nothing */
    }
    else
    {
      while( *s != '\0' && *s != ':' )
        s++;
      if ( *s == '\0' )
        return 0;
      s++;
      while( (*s) > '\0' && (*s) <= ' ' )
        s++;
      if ( is_init == 0 )
      {
        int i = 0;
        while( s[i] == '-' || s[i] == '1' || s[i] == '0' )
          i++;
        pinfoSetInCnt(pi, i);
        pinfoSetOutCnt(pi, 0);
        is_init = 1;
      }
      if ( dcSetByStr(pi, c_on, s) == 0 )
        return 0;
      if ( cl_on != NULL )
        if ( dclAdd(pi, cl_on, c_on) < 0 )
          return 0;
    }
  }

  return 1;
}

/*-- dclReadDichotomy -------------------------------------------------------*/

int dclReadDichotomy(pinfo *pi, dclist cl_on, const char *filename)
{
  FILE *fp;
  int ret;
  fp = b_fopen(filename, NULL, ".dichot", "r");
  if ( fp == NULL )
    return 0;
  ret = dclReadDichotomyFP(pi, cl_on, fp);
  fclose(fp);
  return ret;
}

int IsValidDichotomyFile(const char *filename)
{
  pinfo pi;
  int ret;
  if ( pinfoInit(&pi) == 0 )
    return 0;
  ret = dclReadDichotomy(&pi, NULL, filename);
  pinfoDestroy(&pi);
  return ret;
}


/*-- dclReadPLA -------------------------------------------------------------*/

int dclReadPLA(pinfo *pi, dclist cl_on, dclist cl_dc, const char *filename)
{
  FILE *fp;
  int ret;
  fp = b_fopen(filename, NULL, ".pla", "r");
  if ( fp == NULL )
    return 0;
  ret = dclReadPLAFP(pi, cl_on, cl_dc, fp);
  fclose(fp);
  return ret;
}

int IsValidPLAFile(const char *filename)
{
  pinfo pi;
  int ret;
  if ( pinfoInit(&pi) == 0 )
    return 0;
  ret = dclReadPLA(&pi, NULL, NULL, filename);
  pinfoDestroy(&pi);
  return ret;
}

/*!
  \ingroup dclist
  
  The representation of a boolean function contains three parts:
  -# The problem info structure.
  -# The ON-set of the boolean function (\a cl).
  -# An optional DC (don't care) Set of the boolean function (\a cl_dc).

  This function reads the contents for these parts from a file 
  with the name \a filename. The argument \a cl_dc can be set to \c NULL,
  if one does not care about the DC-set.

  \param pi The problem info structure for the boolean functions.
    This argument will be modified.
  \param cl The ON-set of the boolean function. 
    This argument will be modified.
  \param cl_dc The DC-set of the boolean function. 
    This argument will be modified. The value \c NULL is valid for this 
   argument.
  \param filename A file with a description of boolean functions.

  \return 0, if an error occured.
  
  \warning This function may take a very long time for certain descriptions.
  
  \see gnc_SynthDCL()
  
*/
int dclImport(pinfo *pi, dclist cl_on, dclist cl_dc, const char *filename)
{
  if ( IsValidPLAFile(filename) != 0 )
    return dclReadPLA(pi, cl_on, cl_dc, filename);

  if ( IsValidNEXFile(filename) != 0 )                /* nex.c */
    return dclReadNEX(pi, cl_on, cl_dc, filename);    /* nex.c */
    
  if ( IsValidBEXFile(filename) != 0 )                /* dcex.c */
    return dclReadBEX(pi, cl_on, cl_dc, filename);    /* dcex.c */

  if ( IsValidDichotomyFile(filename) != 0 )
    return dclReadDichotomy(pi, cl_on, filename);

  if ( IsValidBEXStr(filename) != 0 )
    return dclReadBEXStr(pi, cl_on, cl_dc, filename);
  
  return 0;
}

int IsValidDCLFile(const char *filename)
{
  if ( IsValidNEXFile(filename) != 0 )
    return 1;
  if ( IsValidPLAFile(filename) != 0 )
    return 1;
  if ( IsValidBEXFile(filename) != 0 )                /* dcex.c */
    return 1;
  if ( IsValidDichotomyFile(filename) != 0 )
    return 1;
  return 0;
}


/*-- dclWritePLA ------------------------------------------------------------*/

int dclWritePLA(pinfo *pi, dclist cl, const char *filename)
{
  int i, cnt = dclCnt(cl);
  FILE *fp;
  fp = fopen(filename, "w");
  if ( fp == NULL )
    return 0;
  if ( fprintf(fp, ".i %d\n", pi->in_cnt) < 0 ) return fclose(fp), 0;
  if ( fprintf(fp, ".o %d\n", pi->out_cnt) < 0 ) return fclose(fp), 0;
  if ( pi->in_sl != NULL )
  {
    if ( fprintf(fp, ".ilb ") < 0 ) return fclose(fp), 0;
    if ( b_sl_ExportToFP(pi->in_sl, fp, " ", "\n") == 0 ) return fclose(fp), 0;
  }
  if ( pi->out_sl != NULL )
  {
    if ( fprintf(fp, ".ob ") < 0 ) return fclose(fp), 0;
    if ( b_sl_ExportToFP(pi->out_sl, fp, " ", "\n") == 0 ) return fclose(fp), 0;
  }
  if ( fprintf(fp, ".p %d\n", cnt) < 0 ) return fclose(fp), 0;
  
  for( i = 0; i < cnt; i++ )
    if ( fprintf(fp, "%s\n", dcToStr(pi, dclGet(cl, i), " ", "")) < 0 )
      return fclose(fp), 0;
  
  return fclose(fp), 1;
}

/*-- dclShow ----------------------------------------------------------------*/

void dclShow(pinfo *pi, dclist cl)
{
  int i, cnt = dclCnt(cl);
  if ( pi->in_sl != NULL )
  {
    if ( fprintf(stdout, ".ilb ") < 0 ) return;
    if ( b_sl_ExportToFP(pi->in_sl, stdout, " ", "\n") == 0 ) return;
  }
  if ( pi->out_sl != NULL )
  {
    if ( fprintf(stdout, ".ob ") < 0 ) return;
    if ( b_sl_ExportToFP(pi->out_sl, stdout, " ", "\n") == 0 ) return;
  }
  for( i = 0; i < cnt; i++ )
    puts(dcToStr(pi, dclGet(cl, i), " ", ""));
}

/*-- dclSetOutAll -----------------------------------------------------------*/

void dclSetOutAll(pinfo *pi, dclist cl, c_int v)
{
  int i, cnt = dclCnt(cl);
  for( i = 0; i < cnt; i++ )
    dcOutSetAll(pi, dclGet(cl, i), v);
}

/*-- dclDeleteIn -----------------------------------------------------------*/

void dclDeleteIn(pinfo *pi, dclist cl, int pos)
{
  int i, cnt = dclCnt(cl);
  for( i = 0; i < cnt; i++ )
    dcDeleteIn(pi, dclGet(cl, i), pos);
}

/*-- dclDontCareExpand ------------------------------------------------------*/
int dclDontCareExpand(pinfo *pi, dclist cl)
{
  int i, j, k, cnt;
  for( i = 0; i < pi->in_cnt; i++ )
  {
    cnt = dclCnt(cl);
    for( j = 0; j < cnt; j++ )
    {
      if ( dcGetIn(dclGet(cl, j), i) == 3 )
      {
        k = dclAdd(pi, cl, dclGet(cl, j));
        if ( k < 0 )
          return 0;
        dcSetIn(dclGet(cl, j), i, 1);
        dcSetIn(dclGet(cl, k), i, 2);
      }
    }
  }
  return 1;
}

/*-- dclOutExpand -----------------------------------------------------------*/
int dclOutExpand(pinfo *pi, dclist cl)
{
  int i, j, k, cnt;
  int state;
  cnt = dclCnt(cl);
  for( j = 0; j < cnt; j++ )
  {
    state = 0;
    for( i = 0; i < pi->out_cnt; i++ )
    {
      if ( dcGetOut(dclGet(cl, j), i) == 1 )
      {
        if ( state == 0 )
        {
          state = 1;
        }
        else
        {
          k = dclAdd(pi, cl, dclGet(cl, j));
          if ( k < 0 )
            return 0;
          dcOutSetAll(pi, dclGet(cl, k), 0);
          dcSetOut(dclGet(cl, k), i, 1);
          dcSetOut(dclGet(cl, j), i, 0);
        }
      }
    }
  }
  return 1;
}

/*-- dclExpand1 -------------------------------------------------------------*/

static void dclExpand1(pinfo *pi, dclist cl, dclist cl_off)
{
  int i, cnt = dclCnt(cl);
  for( i = 0; i < cnt; i++ )
    dcExpand1(pi, dclGet(cl, i), cl_off);
}

/*-- dclIsSingleSubSet ------------------------------------------------------*/

int dclIsSingleSubSet(pinfo *pi, dclist cl, dcube *c)
{
  int i, cnt = dclCnt(cl);
  for( i = 0; i < cnt; i++ )
    if ( dcIsSubSet(pi, dclGet(cl, i), c) != 0 )
      return 1;
  return 0;
}

/*-- dclGetOutput ------------------------------------------------------*/

/* input:  input part of c */
/* result: output part of c */

void dclGetOutput(pinfo *pi, dclist cl, dcube *c)
{
  int i, cnt = dclCnt(cl);
  dcOutSetAll(pi, c, 0);
  for( i = 0; i < cnt; i++ )
    if ( dcIsInSubSet(pi, dclGet(cl, i), c) != 0 )
      dcOrOut(pi, c, c, dclGet(cl, i));
}

/*-- dclSharp ---------------------------------------------------------------*/

/* Results are added to cl */
int dclSharp(pinfo *pi, dclist cl, dcube *a, dcube *b)
{
  int k;
  for( k = 0; k < pi->in_cnt; k++ )
    if ( dcSharpIn(pi, &(pi->tmp[1]), a, b, k) != 0 )
      if ( dclAdd(pi, cl, &(pi->tmp[1])) < 0 )
        return 0;
  if ( dcSharpOut(pi, &(pi->tmp[1]), a, b) != 0 )
    if ( dclAdd(pi, cl, &(pi->tmp[1])) < 0 )
      return 0;
  return 1;
}

/*-- dclD1Sharp -------------------------------------------------------------*/

/* Die Ergebnisse werden an die Liste cl angehaengt */
int dclD1Sharp(pinfo *pi, dclist cl, dcube *a, dcube *b)
{
  int k;
  for( k = 0; k < pi->in_cnt; k++ )
    if ( dcD1SharpIn(pi, &(pi->tmp[1]), a, b, k) != 0 )
      if ( dclAdd(pi, cl, &(pi->tmp[1])) < 0 )
        return 0;
  if ( dcD1SharpOut(pi, &(pi->tmp[1]), a, b) != 0 )
    if ( dclAdd(pi, cl, &(pi->tmp[1])) < 0 )
      return 0;
  return 1;
}

/*-- dclAddDistance1 --------------------------------------------------------*/

int dclAddDistance1(pinfo *pi, dclist dest, dclist src)
{
  int i, cnt = dclCnt(src);
  for( i = 0; i < cnt; i++ )
    if ( dclD1Sharp(pi, dest, &(pi->tmp[0]), dclGet(src, i)) == 0 )
      return 0;
  dclSubtract(pi, dest, src);
  return 1;
}

/*-- dclDistance1 -----------------------------------------------------------*/

int dclDistance1(pinfo *pi, dclist dest, dclist src)
{
  dclClear(dest);
  return dclAddDistance1(pi, dest, src);
}

/*-- dclDistance1Cube -------------------------------------------------------*/

int dclDistance1Cube(pinfo *pi, dclist dest, dcube *c)
{
  dclClear(dest);
  if ( dclD1Sharp(pi, dest, &(pi->tmp[0]), c) == 0 )
    return 0;
  if ( dclSubtractCube(pi, dest, c) == 0 )
    return 0;
  return 1;
}

/*-- dclRestrictByDistance1 -------------------------------------------------*/
/* a = intersection(a, distance1(b)) */
int dclRestrictByDistance1(pinfo *pi, dclist a, dclist b)
{
  dclist d1, aa;
  if ( dclInitCachedVA(pi,2, &d1, &aa) == 0 )
    return 0;
  if ( dclDistance1(pi, d1, b) == 0 )
    return dclDestroyCachedVA(pi, 2, d1, aa), 0;
  if ( dclCopy(pi, aa, a) == 0 )
    return dclDestroyCachedVA(pi, 2, d1, aa), 0;
  dclClear(a);
  if ( dclIntersectionList(pi, a, aa, d1) == 0 )
    return dclDestroyCachedVA(pi, 2, d1, aa), 0;
  return dclDestroyCachedVA(pi, 2, d1, aa), 1;
}


/*-- dclSCCSharpAndSetFlag --------------------------------------------------*/

/* Die Ergebnisse werden an die Liste cl angehaengt */
int dclSCCSharpAndSetFlag(pinfo *pi, dclist cl, dcube *a, dcube *b)
{
  int k;
  for( k = 0; k < pi->in_cnt; k++ )
    if ( dcSharpIn(pi, &(pi->tmp[1]), a, b, k) != 0 )
      if ( dclSCCAddAndSetFlag(pi, cl, &(pi->tmp[1])) == 0 )
        return 0;
  if ( dcSharpOut(pi, &(pi->tmp[1]), a, b) != 0 )
    if ( dclSCCAddAndSetFlag(pi, cl, &(pi->tmp[1])) == 0 )
      return 0;
  return 1;
}

/*-- dclComplementCube ------------------------------------------------------*/

/* Berechnet das complement eines cubes. */
/* Ergebnisse werden angehaengt. */
int dclComplementCube(pinfo *pi, dclist cl, dcube *c)
{
  return dclSharp(pi, cl, &(pi->tmp[0]), c);
}


/*-- dclIntersection --------------------------------------------------------*/

/* nach dieser Operation ist die SCC eigenschaft NICHT mehr erfuellt */
int dclIntersection(pinfo *pi, dclist cl, dcube *c)
{
  int i, cnt = dclCnt(cl);
  if ( dclClearFlags(cl) == 0 )
    return 0;
  for( i = 0; i < cnt; i++ )
  {
    if ( dcIntersection(pi, dclGet(cl, i), dclGet(cl, i), c) == 0 )
      dclSetFlag(cl, i);
  }
  dclDeleteCubesWithFlag(pi, cl);
  return 1;
}

/*-- dclIntersectionCube ----------------------------------------------------*/

/* nach dieser Operation ist die SCC eigenschaft NICHT mehr erfuellt */
int dclIntersectionCube(pinfo *pi, dclist dest, dclist src, dcube *c)
{
  int i, cnt = dclCnt(src);
  dclClear(dest);
  for( i = 0; i < cnt; i++ )
  {
    if ( dcIntersection(pi, &(pi->tmp[1]), dclGet(src, i), c) != 0 )
      if ( dclAdd(pi, dest, &(pi->tmp[1])) < 0 )
        return 0;
  }
  return 1;
}


/*-- dclSCCIntersectionCube -------------------------------------------------*/

/* nach dieser Operation ist die SCC eigenschaft erfuellt */
int dclSCCIntersectionCube(pinfo *pi, dclist dest, dclist src, dcube *c)
{
  int i, cnt = dclCnt(src);
  dclClear(dest);
  if ( dclClearFlags(dest) == 0 )
    return 0;
  for( i = 0; i < cnt; i++ )
  {
    if ( dcIntersection(pi, &(pi->tmp[1]), dclGet(src, i), c) != 0 )
      if ( dclSCCAddAndSetFlag(pi, dest, &(pi->tmp[1])) == 0 )
        return 0;
  }
  dclDeleteCubesWithFlag(pi, dest);
  return 1;
}

/*-- dclIntersectionList ----------------------------------------------------*/
int dclIntersectionList(pinfo *pi, dclist dest, dclist a, dclist b)
{
  int a_pos, a_cnt = dclCnt(a);
  int b_pos, b_cnt = dclCnt(b);
  dclClear(dest);
  if ( dclClearFlags(dest) == 0 )
    return 0;
  for ( a_pos = 0; a_pos < a_cnt; a_pos++ )
  {
    for ( b_pos = 0; b_pos < b_cnt; b_pos++ )
      if ( dcIntersection(pi, &(pi->tmp[1]), dclGet(a, a_pos), dclGet(b, b_pos)) != 0 )
        if ( dclSCCAddAndSetFlag(pi, dest, &(pi->tmp[1])) == 0 )
          return 0;
  }
  dclDeleteCubesWithFlag(pi, dest);
  return 1;
}

/*-- dclConvertFromPOS ------------------------------------------------------*/
/*
  Interprets the cubes in 'src' as OR-Terms, so 'src' is assumed to
  be a product of sums.
  This function converts the product of sums to a sum of product form.
*/
int dclConvertFromPOS(pinfo *pi, dclist cl)
{
  int i, cnt = dclCnt(cl);
  int j;
  int v;
  dcube *c;
  dclist a, b, o;

  if ( cnt == 0 )
    return 1;

  if ( dclInitVA(3, &a, &b, &o) == 0 )
    return 0;

  for( i = 0; i < cnt; i++ )
  {
  
    /* convert OR-cube into a cube list (stored in 'o') */
    
    dclClear(o);
    for( j = 0; j < pi->in_cnt; j++ )
    {
      v = dcGetIn(dclGet(cl, i), j);
      if ( v != 3 )
      {
        c = dclAddEmptyCube(pi, o);
        if ( c == NULL )
          return dclDestroyVA(3, a, b, o), 0;
        dcCopyOut(pi, c, dclGet(cl, i));
        dcInSetAll(pi, c, CUBE_IN_MASK_DC);
        dcSetIn(c, j, v);
      }
    }

    /* calculate the intersection between 'o' and the result so far.. */

    if ( i == 0 )
    {
      if ( dclCopy(pi, a, o) == 0 )
        return dclDestroyVA(3, a, b, o), 0;
    }
    else
    {
      if ( dclCopy(pi, b, a) == 0 )
        return dclDestroyVA(3, a, b, o), 0;      
      dclClear(a);
      if ( dclIntersectionList(pi, a, b, o) == 0 )
        return dclDestroyVA(3, a, b, o), 0;      
    }
    
  }
  
  if ( dclCopy(pi, cl, a) == 0 )
    return dclDestroyVA(3, a, b, o), 0;      

  return dclDestroyVA(3, a, b, o), 1;
}

/*-- dclIsIntersection ------------------------------------------------------*/
int dclIsIntersection(pinfo *pi, dclist a, dclist b)
{
  int a_pos, a_cnt = dclCnt(a);
  int b_pos, b_cnt = dclCnt(b);
  for ( a_pos = 0; a_pos < a_cnt; a_pos++ )
    for ( b_pos = 0; b_pos < b_cnt; b_pos++ )
      if ( dcIntersection(pi, &(pi->tmp[1]), dclGet(a, a_pos), dclGet(b, b_pos)) != 0 )
        return 1;
  return 0;
}

/*-- dclIsIntersectionCube --------------------------------------------------*/
int dclIsIntersectionCube(pinfo *pi, dclist cl, dcube *c)
{
  int i, cnt = dclCnt(cl);
  for ( i = 0; i < cnt; i++ )
    if ( dcIntersection(pi, &(pi->tmp[1]), dclGet(cl, i), c) != 0 )
      return 1;
  return 0;
}

/*-- dclIntersectionListInv -------------------------------------------------*/
int dclIntersectionListInv(pinfo *pi, dclist dest, dclist a, dclist b)
{
  int a_pos, a_cnt = dclCnt(a);
  int b_pos, b_cnt = dclCnt(b);
  dcube *tc = &(pi->tmp[1]);
  dcube *ac;
  dclClear(dest);
  if ( dclClearFlags(dest) == 0 )
    return 0;
  for ( a_pos = 0; a_pos < a_cnt; a_pos++ )
  {
    ac = dclGet(a, a_pos);
    for ( b_pos = 0; b_pos < b_cnt; b_pos++ )
      if ( dcIntersection(pi, tc, ac, dclGet(b, b_pos)) != 0 )
        if ( dclSCCInvAddAndSetFlag(pi, dest, tc) == 0 )
          return 0;
  }
  dclDeleteCubesWithFlag(pi, dest);
  return 1;
}

/*-- dclAndElements ---------------------------------------------------------*/

void dclAndElements(pinfo *pi, dcube *r, dclist cl)
{
  int i, cnt = dclCnt(cl);
  dcSetTautology(pi, r);
  for( i = 0; i < cnt; i++ )
    dcAnd(pi, r, r, dclGet(cl, i));
}

/*-- dclOrElements ----------------------------------------------------------*/

void dclOrElements(pinfo *pi, dcube *r, dclist cl)
{
  int i, cnt = dclCnt(cl);
  dcInSetAll(pi, r, 0);
  dcOutSetAll(pi, r, 0);
  for( i = 0; i < cnt; i++ )
    dcOr(pi, r, r, dclGet(cl, i));
}

/*-- dclResult --------------------------------------------------------------*/

/* calculate the output of 'r' by checking the input of 'r' */
void dclResult(pinfo *pi, dcube *r, dclist cl)
{
  int i, cnt = dclCnt(cl);
  dcOutSetAll(pi, r, 0);
  for( i = 0; i < cnt; i++ )
    if ( dcIsInSubSet(pi, dclGet(cl, i), r) != 0 )
      dcOrOut(pi, r, r, dclGet(cl, i));
}

/*-- dclResultList ----------------------------------------------------------*/

/* calculate the output of 'r' by checking the input of 'r' */
int dclResultList(pinfo *pi, dclist r, dclist cl)
{
  int i, cnt;
  
  cnt = dclCnt(r);
  for( i = 0; i < cnt; i++ )
    dcSetOutTautology(pi, dclGet(r, i));
    
  if ( dclDontCareExpand(pi, r) == 0 )
    return 0; /* memory error */
  dclSCC(pi, r);

  cnt = dclCnt(r);
  for( i = 0; i < cnt; i++ )
    dclResult(pi, dclGet(r, i), cl);
    
  return 1; /* no memory error */
}

/*-- dclSuper2 --------------------------------------------------------------*/

void dclSuper2(pinfo *pi, dcube *r, dclist cl1, dclist cl2)
{
  int i, cnt;
  dcInSetAll(pi, r, 0);
  dcOutSetAll(pi, r, 0);
  cnt = dclCnt(cl1);
  for( i = 0; i < cnt; i++ )
    dcOr(pi, r, r, dclGet(cl1, i));
  cnt = dclCnt(cl2);
  for( i = 0; i < cnt; i++ )
    dcOr(pi, r, r, dclGet(cl2, i));
}

/*-- dclAndLocalElements ----------------------------------------------------*/

/* obsolete? --> dcubehf */
int dclAndLocalElements(pinfo *pi, dcube *r, dcube *m, dclist cl)
{
  int i, cnt = dclCnt(cl);
  dcSetTautology(pi, r);
  for( i = 0; i < cnt; i++ )
    if ( dcIsDeltaNoneZero(pi, m, dclGet(cl, i)) == 0 )
      dcAnd(pi, r, r, dclGet(cl, i));
  return 1;
}

/*-- dclSCCCofactor ---------------------------------------------------------*/

int dclAddSubCofactor(pinfo *pi, dclist dest, dclist src, dcube *cofactor)
{
  int i, cnt = dclCnt(src);
  dcube *r = &(pi->tmp[1]);
  for ( i = 0; i < cnt; i++ )
    if ( dcSubCofactor(pi, r, dclGet(src, i), cofactor) != 0 )
      if ( dclAdd(pi, dest, r) < 0 )
        return 0;
  return 1;
}

int dclAddRedCofactor(pinfo *pi, dclist dest, dclist src, dcube *cofactor)
{
  int i, cnt = dclCnt(src);
  dcube *r = &(pi->tmp[1]);
  for ( i = 0; i < cnt; i++ )
    if ( dcRedCofactor(pi, r, dclGet(src, i), cofactor) != 0 )
      if ( dclAdd(pi, dest, r) < 0 )
        return 0;
  return 1;
}

int xdcl_Cofactor(pinfo *pi, dclist dest, dclist src, dcube *cofactor)
{
  dclClear(dest);
  if ( dclAddRedCofactor(pi, dest, src, cofactor) == 0 )
    return 0;
  if ( dclSCC(pi, dest) == 0 )
    return 0;
  if ( dclAddSubCofactor(pi, dest, src, cofactor) == 0 )
    return 0;
  return 1;  
}

int dclSCCCofactor(pinfo *pi, dclist dest, dclist src, dcube *cofactor)
{
  int i, cnt = dclCnt(src);
  dcube *r = &(pi->tmp[1]);
  dclClear(dest);
  if ( dclClearFlags(dest) == 0 )
    return 0;
  for ( i = 0; i < cnt; i++ )
    if ( dcCofactor(pi, r, dclGet(src, i), cofactor) != 0 )
      if ( dclSCCAddAndSetFlag(pi, dest, r) == 0 )
        return 0;
  dclDeleteCubesWithFlag(pi, dest);
  return 1;
}

int dclSCCCofactorExcept(pinfo *pi, dclist dest, dclist src, dcube *cofactor, int src_except_pos)
{
  int i, cnt = dclCnt(src);
  dcube *r = &(pi->tmp[1]);
  dclClear(dest);
  if ( dclClearFlags(dest) == 0 )
    return 0;
  for ( i = 0; i < src_except_pos; i++ )
    if ( dcCofactor(pi, r, dclGet(src, i), cofactor) != 0 )
      if ( dclSCCAddAndSetFlag(pi, dest, r) == 0 )
        return 0;
  for ( i = src_except_pos+1; i < cnt; i++ )
    if ( dcCofactor(pi, r, dclGet(src, i), cofactor) != 0 )
      if ( dclSCCAddAndSetFlag(pi, dest, r) == 0 )
        return 0;

  dclDeleteCubesWithFlag(pi, dest);
  return 1;
}

/*-- dclSCCInvCofactor ------------------------------------------------------*/

int dclSCCInvCofactor(pinfo *pi, dclist dest, dclist src, dcube *cofactor)
{
  int i, cnt = dclCnt(src);
  dcube *r = &(pi->tmp[1]);
  dclClear(dest);
  if ( dclClearFlags(dest) == 0 )
    return 0;
  for ( i = 0; i < cnt; i++ )
    if ( dcCofactor(pi, r, dclGet(src, i), cofactor) != 0 )
      if ( dclSCCInvAddAndSetFlag(pi, dest, r) == 0 )
        return 0;
  dclDeleteCubesWithFlag(pi, dest);
  return 1;
}

/*-- dclCofactor ------------------------------------------------------------*/

int dclCofactor(pinfo *pi, dclist dest, dclist src, dcube *cofactor)
{
  int i, cnt = dclCnt(src);
  dcube *r = &(pi->tmp[1]);
  dclClear(dest);
  for ( i = 0; i < cnt; i++ )
    if ( dcCofactor(pi, r, dclGet(src, i), cofactor) != 0 )
      if ( dclAdd(pi, dest, r) < 0 )
        return 0;
  return 1;
}

/*-- dclGetCofactorCnt ------------------------------------------------------*/

/* bestimmt die anzahl der elemente, die bei einer kofaktorierung */
/* entstehen wuerden */

int dclGetCofactorCnt(pinfo *pi, dclist src, dcube *cofactor)
{
  int i, cnt = dclCnt(src);
  int elements = 0;
  for ( i = 0; i < cnt; i++ )
    if ( dcIsDeltaNoneZero(pi, dclGet(src, i), cofactor) == 0 )
      elements++;
  return elements;
}

/*-- dclSCCAddAndSetFlag ----------------------------------------------------*/

/* Wenn c teilmenge eines elementes aus cl ist, wird c nicht angehaengt. */
/* Alle elemente die teilmenge von c sind, werden markiert.              */
/* Die idee ist: Wenn vor dem aufruf gilt SCC(cl), dann gilt ist dies    */
/* auch nach dem aufruf, wenn dclDeleteCubesWithFlag(pi, cl) aufgerufen  */
/* wurde.                                                                */
/* Rueckgabe: 0 im Falle eines Fehlers.                                  */
int dclSCCAddAndSetFlagReadable(pinfo *pi, dclist cl, dcube *c)
{
  register int i, cnt = dclCnt(cl);
  register int flags = 0;

  for( i = cnt-1; i >= 0; i-- )
    if ( dclIsFlag(cl, i) == 0 )
    {
      if ( dcIsSubSet(pi, dclGet(cl, i), c) != 0 )
        return 1;
    }
    else
    {
      flags++;
    }

  for( i = 0; i < cnt; i++ )
    if ( dclIsFlag(cl, i) == 0 )
      if ( dcIsSubSet(pi, c, dclGet(cl, i)) != 0 )
        dclSetFlag(cl, i);
  
  if ( dclAdd(pi, cl, c) >= 0 )
    return 1;
    
  if ( flags > 30 )
    dclDeleteCubesWithFlag(pi, cl);
    
  return 0;
}

int dclSCCAddAndSetFlag(pinfo *pi, dclist cl, dcube *c)
{
  register int i, cnt = dclCnt(cl);
  register int flags = 0;

  for( i = cnt-1; i >= 4; i-=4 )
    if ( dcIsSubSet4(pi, dclGet(cl, i), dclGet(cl, i-1), dclGet(cl, i-2), dclGet(cl, i-3), c) != 0 )
      return 1;

  for( ; i >= 0; i-- )
    if ( dcIsSubSet(pi, dclGet(cl, i), c) != 0 )
      return 1;

  for( i = 0; i < cnt-6; i+=6 )
    if ( dcIsSubSet6n(pi, c, dclGet(cl, i), dclGet(cl, i+1), dclGet(cl, i+2), dclGet(cl, i+3), dclGet(cl, i+4), dclGet(cl, i+5)) != 0 )
    {
      if ( dcIsSubSet(pi, c, dclGet(cl, i)) != 0 )
      {
        dclSetFlag(cl, i);
        flags++;
      }
      if ( dcIsSubSet(pi, c, dclGet(cl, i+1)) != 0 )
      {
        dclSetFlag(cl, i+1);
        flags++;
      }
      if ( dcIsSubSet(pi, c, dclGet(cl, i+2)) != 0 )
      {
        dclSetFlag(cl, i+2);
        flags++;
      }
      if ( dcIsSubSet(pi, c, dclGet(cl, i+3)) != 0 )
      {
        dclSetFlag(cl, i+3);
        flags++;
      }
      if ( dcIsSubSet(pi, c, dclGet(cl, i+4)) != 0 )
      {
        dclSetFlag(cl, i+4);
        flags++;
      }
      if ( dcIsSubSet(pi, c, dclGet(cl, i+5)) != 0 )
      {
        dclSetFlag(cl, i+5);
        flags++;
      }
    }

  for( ; i < cnt; i++ )
    if ( dcIsSubSet(pi, c, dclGet(cl, i)) != 0 )
    {
      dclSetFlag(cl, i);
      flags++;
    }
  
  if ( dclAdd(pi, cl, c) < 0 )
    return 0;
    
  if ( flags > 0 )
    dclDeleteCubesWithFlag(pi, cl);
    
  return 1;
}

/*-- dclSCCInvAddAndSetFlag -------------------------------------------------*/

/* Wenn c teilmenge eines elementes aus cl ist, wird c nicht angehaengt. */
/* Alle elemente die teilmenge von c sind, werden markiert.              */
/* Die idee ist: Wenn vor dem aufruf gilt SCC(cl), dann gilt ist dies    */
/* auch nach dem aufruf, wenn dclDeleteCubesWithFlag(pi, cl) aufgerufen  */
/* wurde.                                                                */
/* Rueckgabe: 0 im Falle eines Fehlers.                                  */
int dclSCCInvAddAndSetFlag(pinfo *pi, dclist cl, dcube *c)
{
  register int i, cnt = dclCnt(cl);
  register int flags = 0;

  for( i = cnt-1; i >= 6; i-=6 )
    if ( dcIsSubSet6n(pi, c, dclGet(cl, i), dclGet(cl, i-1), dclGet(cl, i-2), dclGet(cl, i-3), dclGet(cl, i-4), dclGet(cl, i-5)) != 0 )
      return 1;

  for( ; i >= 0; i-- )
    if ( dcIsSubSet(pi, c, dclGet(cl, i)) != 0 )
      return 1;

  for( i = 0; i < cnt; i++ )
    if ( dcIsSubSet(pi, dclGet(cl, i), c) != 0 )
    {
      dclSetFlag(cl, i);
      flags++;
    }
  
  if ( dclAdd(pi, cl, c) < 0 )
    return 0;
    
  if ( flags > 0 )
    dclDeleteCubesWithFlag(pi, cl);
    
  return 1;
}

/*-- dclSCCConsensusCube ----------------------------------------------------*/
/* a = consensus(a,b) */
/* die flagliste von a wird benutzt */
/* wenn vorher SCC(a) galt, gilt das danach auch noch */
int dclSCCConsensusCube(pinfo *pi, dclist a, dcube *b)
{
  int a_pos, a_cnt = dclCnt(a);
  if ( dclClearFlags(a) == 0 )
    return 0;
    
  for ( a_pos = 0; a_pos < a_cnt; a_pos++ )
  {
    if ( dcConsensus(pi, &(pi->tmp[1]), dclGet(a, a_pos), b) != 0 )
    {
      if ( dclSCCAddAndSetFlag(pi, a, &(pi->tmp[1])) == 0 )
        return 0;
    }
    else
    {
      dclSetFlag(a, a_pos);
    }
  }
  dclDeleteCubesWithFlag(pi, a);
  return 1;
}



/*-- dclConsensusCube -------------------------------------------------------*/
/* a = consensus(a,b) */
/* die flagliste von a wird benutzt */
int dclConsensusCube(pinfo *pi, dclist a, dcube *b)
{
  int a_pos, a_cnt = dclCnt(a);
  if ( dclClearFlags(a) == 0 )
    return 0;
    
  for ( a_pos = 0; a_pos < a_cnt; a_pos++ )
  {
    if ( dcConsensus(pi, &(pi->tmp[1]), dclGet(a, a_pos), b) != 0 )
    {
      if ( dclAdd(pi, a, &(pi->tmp[1])) <0 )
        return 0;
    }
    else
    {
      dclSetFlag(a, a_pos);
    }
  }
  dclDeleteCubesWithFlag(pi, a);
  return 1;
}


/*-- dclConsensus -----------------------------------------------------------*/

/* consensus zwischen allen elementen   */
/* aus a und allen elementen aus b.     */
/* 'dest' enthaelt die neuen Cubes.     */
/* 'dest' erfuellt die SCC Eigenschaft  */
/* Rueckgabe: 0 im Falle eines Fehlers. */
int dclConsensus2(pinfo *pi, dclist dest, dclist a, dclist b)
{
  int a_pos, a_cnt = dclCnt(a);
  int b_pos, b_cnt = dclCnt(b);
  dclClear(dest);
  if ( dclClearFlags(dest) == 0 )
    return 0;
  for ( a_pos = 0; a_pos < a_cnt; a_pos++ )
  {
    /*
    if ( pinfoProcedure(pi, "Consensus", a_pos, a_cnt) == 0 )
      return 0;
    */
    for ( b_pos = 0; b_pos < b_cnt; b_pos++ )
      if ( dcConsensus(pi, &(pi->tmp[1]), dclGet(a, a_pos), dclGet(b, b_pos)) != 0 )
        if ( dclSCCAddAndSetFlag(pi, dest, &(pi->tmp[1])) == 0 )
          return 0;
  }
  dclDeleteCubesWithFlag(pi, dest);
  return 1;
}


#define DCL_CONS_STEP 8
int dclConsensus(pinfo *pi, dclist dest, dclist a, dclist b)
{
  int a_pos, a_cnt = dclCnt(a);
  int b_pos, b_cnt = dclCnt(b);
  int start = 0, min_cnt;
  int cnt = 0;
  dclClear(dest);
  if ( dclClearFlags(dest) == 0 )
    return 0;

  min_cnt = a_cnt > b_cnt ? b_cnt : a_cnt;

  while( start+DCL_CONS_STEP < min_cnt  )
  {
    /*
    if ( pinfoProcedure(pi, "Consensus", start, min_cnt) == 0 )
      return 0;
    */

    for ( a_pos = start; a_pos < start+DCL_CONS_STEP; a_pos++ )
      for ( b_pos = 0; b_pos < start+DCL_CONS_STEP; b_pos++ )
        if ( dcConsensus(pi, &(pi->tmp[1]), dclGet(a, a_pos), dclGet(b, b_pos)) != 0 )
          if ( dclSCCAddAndSetFlag(pi, dest, &(pi->tmp[1])) == 0 )
            return 0;

    for ( b_pos = start; b_pos < start+DCL_CONS_STEP; b_pos++ )
      for ( a_pos = 0; a_pos < start; a_pos++ )
        if ( dcConsensus(pi, &(pi->tmp[1]), dclGet(a, a_pos), dclGet(b, b_pos)) != 0 )
          if ( dclSCCAddAndSetFlag(pi, dest, &(pi->tmp[1])) == 0 )
            return 0;

    cnt += DCL_CONS_STEP*(start+DCL_CONS_STEP)+DCL_CONS_STEP*start;
    start += DCL_CONS_STEP;
  }

  /*
  if ( pinfoProcedure(pi, "Consensus", min_cnt, min_cnt) == 0 )
    return 0;
  */

  for ( a_pos = start; a_pos < a_cnt; a_pos++ )
    for ( b_pos = 0; b_pos < b_cnt; b_pos++ )
      if ( dcConsensus(pi, &(pi->tmp[1]), dclGet(a, a_pos), dclGet(b, b_pos)) != 0 )
        if ( dclSCCAddAndSetFlag(pi, dest, &(pi->tmp[1])) == 0 )
          return 0;

  for ( b_pos = start; b_pos < b_cnt; b_pos++ )
    for ( a_pos = 0; a_pos < start; a_pos++ )
      if ( dcConsensus(pi, &(pi->tmp[1]), dclGet(a, a_pos), dclGet(b, b_pos)) != 0 )
        if ( dclSCCAddAndSetFlag(pi, dest, &(pi->tmp[1])) == 0 )
          return 0;
          
  cnt += (a_cnt-start)*b_cnt + (b_cnt-start)*start;

  assert(a_cnt*b_cnt == cnt);

  dclDeleteCubesWithFlag(pi, dest);
  return 1;
}

/*-- dclIsBinateInVar -------------------------------------------------------*/

int dclIsBinateInVar(dclist cl, int var)
{
  int is_one = 0, is_zero = 0;
  int i, cnt = dclCnt(cl);
  int code;
  for( i = 0; i < cnt; i++ )
  {
    code = dcGetIn(dclGet(cl,i), var);
    if ( code == 1 )
      is_zero = 1;
    else if ( code == 2 )
      is_one = 1;
  }
  if ( is_zero != 0 && is_one != 0 )
    return 1;
  return 0;
}

/*-- dclIsDCInVar -------------------------------------------------------*/

int dclIsDCInVar(pinfo *pi, dclist cl, int var)
{
  dcube *r = &(pi->tmp[5]);
  dclAndElements(pi, r, cl);
  if ( dcGetIn(r, var) == 3 )
    return 1;
  return 0;
}

/*-- dclCheckTautology ------------------------------------------------------*/

/* Rueckgabe:  1 Ja, 'cl' ist eine Tautologie */
/*             0 Nein, keine Tautologie       */
/*            -1 Unbekannt                    */
int dclCheckTautology(pinfo *pi, dclist cl)
{
  int i, cnt = dclCnt(cl);
  dcAllClear(pi, &(pi->tmp[8]));
  for( i = 0; i < cnt; i++ )
  {
    if ( dcIsTautology(pi, dclGet(cl, i)) != 0 )
      return 1;
    dcOr(pi, &(pi->tmp[8]), &(pi->tmp[8]), dclGet(cl, i));
  }
  if ( dcIsTautology(pi, &(pi->tmp[8])) == 0 )
    return 0;
#ifdef xxxx
  for( i = 0; i < pi->in_cnt; i++ )
    if ( dcGetIn(&pi->tmp[8], i) != 3 )       /* Input: No don't care -> no tautology */
      return 0;
  for( i = 0; i < pi->out_cnt; i++ )
    if ( dcGetOut(&pi->tmp[8], i) == 0 )      /* Output: Somewhere Zero? */
      return 0;
#endif
  return -1;
}

/*-- dclTautology -----------------------------------------------------------*/

int dclTautologyCof(pinfo *pi, dclist cl, dcube *cof, int depth)
{
  dclist cl_left, cl_right;
  dcube *cofactor_left = &(pi->stack1[depth]);
  dcube *cofactor_right = &(pi->stack2[depth]);
  int check;
  
  check = dclCheckTautology(pi, cl);
  if ( check >= 0 )
  {
    return check;
  }
  
  if ( dcGetCofactorForSplit(pi, cofactor_left, cofactor_right, cl, cof) == 0 )
  {
    /* function ist unate, enthaelt aber keinen tautologie cube */
    /* --> keine tautolgy */
    return 0;
  }

  if ( dclInitCachedVA(pi, 2, &cl_left, &cl_right) == 0 )
    return 0;
    
  if (depth >= PINFO_STACK_CUBES)
    return 0;

  if ( dclSCCCofactor(pi, cl_left, cl, cofactor_left) == 0 )
    return dclDestroyCachedVA(pi, 2, cl_left, cl_right), 0;
    
  if ( dclSCCCofactor(pi, cl_right, cl, cofactor_right) == 0 )
    return dclDestroyCachedVA(pi, 2, cl_left, cl_right), 0;

/*
 *   puts("cofactored lists (left)");
 *   dclShow(pi, cl_left);
 *   puts("cofactored lists (right)");
 *   dclShow(pi, cl_right);
 */

  /* pinfoBTreeStart(pi); */
  
  if ( dclTautologyCof(pi, cl_left, cofactor_left, depth+1) == 0 )
    return /*pinfoBTreeEnd(pi),*/ dclDestroyCachedVA(pi, 2, cl_left, cl_right), 0;
  
  if ( dclTautologyCof(pi, cl_right, cofactor_right, depth+1) == 0 )
    return /*pinfoBTreeEnd(pi),*/ dclDestroyCachedVA(pi, 2, cl_left, cl_right), 0;
  
  return /*pinfoBTreeEnd(pi),*/ dclDestroyCachedVA(pi, 2, cl_left, cl_right), 1;
}

int dclTautology(pinfo *pi, dclist cl)
{
  int result;
  dcube *cof = &(pi->tmp[2]);
  dcSetTautology(pi, cof);
  /* pinfoBTreeInit(pi, "Tautology"); */
  result = dclTautologyCof(pi, cl, cof, 0);
  /* pinfoBTreeFinish(pi); */
  return result;
}

/*-- dclIsSubSet ------------------------------------------------------------*/

/* gibt 0 zurueck, wenn 'cube' nicht von 'cl' ueberdeckt wird */
/* returns 0, if 'cube' is not covered by 'cl' */
int dclIsSubSet(pinfo *pi, dclist cl, dcube *cube)
{
  dclist tmp;
  int result;
  if ( dclInitCached(pi, &tmp) == 0 )
    return 0;
  if ( dclSCCCofactor(pi, tmp, cl, cube) == 0 )
    return dclDestroyCached(pi, tmp), 0;
  result = dclTautology(pi, tmp);
  dclDestroyCached(pi, tmp);
  return result;
}

/*-- dclRemoveSubSet --------------------------------------------------------*/
/* entfernt aus cl alle elemente, die von cover ueberdeckt werden */
/* eventuell entfernte elemente werden an removed angehaengt!!! */
/* removed kann NULL sein */
int dclRemoveSubSet(pinfo *pi, dclist cl, dclist cover, dclist removed)
{
  int i, cnt = dclCnt(cl);
  if ( dclClearFlags(cl) == 0 )
    return 0;
  pinfoProcedureInit(pi, "RemoveSubSet", cnt);
  
  for( i = 0; i < cnt; i++ )
  {
    if ( pinfoProcedureDo(pi, i) == 0 )
      return pinfoProcedureFinish(pi), 0;
    if  ( dclIsSubSet(pi, cover, dclGet(cl, i)) != 0 )
    {
      if ( removed != NULL )
        if ( dclAdd(pi, removed, dclGet(cl, i)) < 0 )
          return pinfoProcedureFinish(pi), 0;
      dclSetFlag(cl, i);
    }
  }
  pinfoProcedureFinish(pi);
  dclDeleteCubesWithFlag(pi, cl);
  return 1;
}

/*-- dclSortOutput ----------------------------------------------------------*/

static pinfo *qsort_pinfo;

int dcl_compare_output(const void *ap, const void *bp)
{
  dcube *a = (dcube *)ap;
  dcube *b = (dcube *)bp;
  int i;
  for( i = 0; i < qsort_pinfo->out_words; i++ )
  {
    if ( a->out[i] < b->out[i] )
      return -1;
    if ( a->out[i] > b->out[i] )
      return 1;
  }
  return 0;
}

void dclSortOutput(pinfo *pi, dclist cl)
{
  qsort_pinfo = pi;
  
  qsort(cl->list, cl->cnt, sizeof(dcube), dcl_compare_output);
}

/*-- dclSortOutSize ---------------------------------------------------------*/

int dcl_compare_n(const void *ap, const void *bp)
{
  dcube *a = (dcube *)ap;
  dcube *b = (dcube *)bp;
  
  if ( a->n > b->n )
    return -1;
  if ( a->n < b->n )
    return 1;
  return 0;
}

void dclSortOutSize(pinfo *pi, dclist cl)
{
  int i, cnt = dclCnt(cl);
  
  for( i = 0; i < cnt; i++ )
    dclGet(cl, i)->n = dcOutCnt(pi, dclGet(cl, i));
  
  qsort(cl->list, cl->cnt, sizeof(dcube), dcl_compare_n);
}

/*-- dclSortInSize ----------------------------------------------------------*/

void dclSortInSize(pinfo *pi, dclist cl)
{
  int i, cnt = dclCnt(cl);
  
  for( i = 0; i < cnt; i++ )
    dclGet(cl, i)->n = dcInDCCnt(pi, dclGet(cl, i));
  
  qsort(cl->list, cl->cnt, sizeof(dcube), dcl_compare_n);
}



/*-- dclSCC -----------------------------------------------------------------*/

int dclSCC(pinfo *pi, dclist cl)
{
  int i, j, cnt = dclCnt(cl);
  int scc_cnt = 0;
  
  /* dclSortSize(pi, cl); */
  
  if ( dclClearFlags(cl) == 0 )
    return 0;
  pinfoProcedureInit(pi, "SCC", cnt);
  for( i = 0; i < cnt; i++ )
  {
    if ( pinfoProcedureDo(pi, i) == 0 )
      return 0;
    if ( dclIsFlag(cl, i) == 0 )
      for( j = i+1; j < cnt; j++ )
      {
        if ( dclIsFlag(cl, j) == 0 )
        {
          if ( dcIsSubSet(pi, dclGet(cl, j), dclGet(cl, i)) != 0 )
          {
            dclSetFlag(cl, i);
            scc_cnt++;
          }
          else if ( dcIsEqual(pi, dclGet(cl, i), dclGet(cl, j)) == 0 )
            if ( dcIsSubSet(pi, dclGet(cl, i), dclGet(cl, j)) != 0 )
            {
              dclSetFlag(cl, j);
              scc_cnt++;
            }
        }
      }
  }
  /* printf("SCC: %d   \n", scc_cnt); */
  dclDeleteCubesWithFlag(pi, cl);
  pinfoProcedureFinish(pi);
  return 1;
}

/*-- dclSCCInv --------------------------------------------------------------*/

int dclSCCInv(pinfo *pi, dclist cl)
{
  int i, j, cnt = dclCnt(cl);
  int scc_cnt = 0;
  
  /* dclSortSize(pi, cl); */
  
  if ( dclClearFlags(cl) == 0 )
    return 0;
  pinfoProcedureInit(pi, "SCCInv", cnt);
  for( i = 0; i < cnt; i++ )
  {
    if ( pinfoProcedureDo(pi, i) == 0 )
      return 0;
    if ( dclIsFlag(cl, i) == 0 )
      for( j = i+1; j < cnt; j++ )
      {
        if ( dclIsFlag(cl, j) == 0 )
        {
          if ( dcIsSubSet(pi, dclGet(cl, i), dclGet(cl, j)) != 0 )
          {
            dclSetFlag(cl, i);
            scc_cnt++;
          }
          else if ( dcIsEqual(pi, dclGet(cl, i), dclGet(cl, j)) == 0 )
            if ( dcIsSubSet(pi, dclGet(cl, j), dclGet(cl, i)) != 0 )
            {
              dclSetFlag(cl, j);
              scc_cnt++;
            }
        }
      }
  }
  /* printf("SCC: %d   \n", scc_cnt); */
  dclDeleteCubesWithFlag(pi, cl);
  pinfoProcedureFinish(pi);
  return 1;
}

/*-- dclRemoveEqual ---------------------------------------------------------*/

int dclRemoveEqual(pinfo *pi, dclist cl)
{
  int i, j, cnt = dclCnt(cl);
  
  /* dclSortSize(pi, cl); */
  
  if ( dclClearFlags(cl) == 0 )
    return 0;
  pinfoProcedureInit(pi, "RemoveEqual", cnt);
  for( i = 0; i < cnt; i++ )
  {
    if ( pinfoProcedureDo(pi, i) == 0 )
      return 0;
    if ( dclIsFlag(cl, i) == 0 )
      for( j = i+1; j < cnt; j++ )
      {
        if ( dclIsFlag(cl, j) == 0 )
        {
          if ( dcIsEqual(pi, dclGet(cl, i), dclGet(cl, j)) != 0 )
          {
            dclSetFlag(cl, j);
          }
        }
      }
  }
  dclDeleteCubesWithFlag(pi, cl);
  pinfoProcedureFinish(pi);
  return 1;
}

/*-- dclSCCUnionSubset ------------------------------------------------------*/
/* annahme: dest und src haben SCC eigenschaft */
/* dest = SCC(dest + src) */
/* src bleibt unveraendert */
/* Elemente von src nicht groesser als die elemente von dest */

int dclSCCUnionSubset(pinfo *pi, dclist dest, dclist src)
{
  int dest_i, dest_cnt = dclCnt(dest);
  int src_i, src_cnt = dclCnt(src);
  for( src_i = 0; src_i < src_cnt; src_i++ )
  {
    for( dest_i = 0; dest_i < dest_cnt; dest_i++ )
      if ( dcIsSubSet(pi, dclGet(dest, dest_i), dclGet(src, src_i)) != 0 )
        break;
    if ( dest_i >= dest_cnt )
      if ( dclAdd(pi, dest, dclGet(src, src_i)) < 0 )
        return 0;
  }
    
  return 1;
}

/*-- dclSCCUnion ------------------------------------------------------------*/

/* annahme: dest und src haben SCC eigenschaft */
/* dest = SCC(dest + src) */
/* src bleibt unveraendert */
int dclSCCUnion(pinfo *pi, dclist dest, dclist src)
{
  int dest_i, dest_cnt = dclCnt(dest);
  int src_i, src_cnt = dclCnt(src);
  
  if ( dclClearFlags(dest) == 0 )
    return 0;
    
  for( dest_i = 0; dest_i < dest_cnt; dest_i++ )
  {
    for( src_i = 0; src_i < src_cnt; src_i++ )
    {
      if ( dcIsSubSet(pi, dclGet(src, src_i), dclGet(dest, dest_i)) != 0 )
      {
        dclSetFlag(dest, dest_i);
        break;
      }
    }
  }
  
  dclDeleteCubesWithFlag(pi, dest);
  dest_cnt = dclCnt(dest);
  
  for( src_i = 0; src_i < src_cnt; src_i++ )
  {
    for( dest_i = 0; dest_i < dest_cnt; dest_i++ )
      if ( dcIsSubSet(pi, dclGet(dest, dest_i), dclGet(src, src_i)) != 0 )
        break;
    if ( dest_i >= dest_cnt )
      if ( dclAdd(pi, dest, dclGet(src, src_i)) < 0 )
        return 0;
  }
    
  return 1;
}

/*-- dclSCCInvUnionSubset ----------------------------------------------------*/
/* annahme: dest und src haben SCC eigenschaft */
/* dest = SCC(dest + src) */
/* src bleibt unveraendert */
/* Elemente von src nicht groesser als die elemente von dest */

int dclSCCInvUnionSubset(pinfo *pi, dclist dest, dclist src)
{
  int dest_i, dest_cnt = dclCnt(dest);
  int src_i, src_cnt = dclCnt(src);

  for( src_i = 0; src_i < src_cnt; src_i++ )
  {
    for( dest_i = 0; dest_i < dest_cnt; dest_i++ )
      if ( dcIsSubSet(pi, dclGet(src, src_i), dclGet(dest, dest_i)) != 0 )
        break;
    if ( dest_i >= dest_cnt )
      if ( dclAdd(pi, dest, dclGet(src, src_i)) < 0 )
        return 0;
  }
    
  return 1;
}

/*-- dclSCCInvUnion ---------------------------------------------------------*/

int dclSCCInvUnion(pinfo *pi, dclist dest, dclist src)
{
  int dest_i, dest_cnt = dclCnt(dest);
  int src_i, src_cnt = dclCnt(src);
  
  if ( dclClearFlags(dest) == 0 )
    return 0;
    
  for( dest_i = 0; dest_i < dest_cnt; dest_i++ )
  {
    for( src_i = 0; src_i < src_cnt; src_i++ )
    {
      if ( dcIsSubSet(pi, dclGet(dest, dest_i), dclGet(src, src_i)) != 0 )
      {
        dclSetFlag(dest, dest_i);
        break;
      }
    }
  }
  
  dclDeleteCubesWithFlag(pi, dest);
  dest_cnt = dclCnt(dest);
  
  for( src_i = 0; src_i < src_cnt; src_i++ )
  {
    for( dest_i = 0; dest_i < dest_cnt; dest_i++ )
      if ( dcIsSubSet(pi, dclGet(src, src_i), dclGet(dest, dest_i)) != 0 )
        break;
    if ( dest_i >= dest_cnt )
      if ( dclAdd(pi, dest, dclGet(src, src_i)) < 0 )
        return 0;
  }
    
  return 1;
}

/*-- dclSCCSubtractCube -----------------------------------------------------*/

/* a = a - b */
/* nach dieser operation erfuellt a die SCC eigenschaft */
int dclSCCSubtractCube(pinfo *pi, dclist a, dcube *b)
{
  int a_i;
  int a_cnt;
  dclist result;
  if ( dclClearFlags(a) == 0 )
    return 0;
  if ( dclInitCached(pi, &result) == 0 )
    return 0;
  if ( dclClearFlags(result) == 0 )
    return dclDestroyCached(pi, result), 0;
    
  a_cnt = dclCnt(a);
  dclClear(result);
  for( a_i = 0; a_i < a_cnt; a_i++ )
  {
    if ( dclSCCSharpAndSetFlag(pi, result, dclGet(a, a_i), b) == 0 )
      return dclDestroyCached(pi, result), 0;
    if ( (a_i % 32) == 0 )
      dclDeleteCubesWithFlag(pi, result);
  }
  dclDeleteCubesWithFlag(pi, result);
  if ( dclCopy(pi, a, result) == 0 )
    return dclDestroyCached(pi, result), 0;
    
  dclDestroyCached(pi, result);
  return 1;
}

/*-- dclSubtractCube -------------------------------------------------------*/

/* a = a - b */
/* nach dieser operation erfuellt a die SCC eigenschaft nicht mehr */
int dclSubtractCube(pinfo *pi, dclist a, dcube *b)
{
  int a_i;
  int a_cnt;
  dclist result;
  
  if ( dclInit(&result) == 0 )
    return 0;
    
  a_cnt = dclCnt(a);
  for( a_i = 0; a_i < a_cnt; a_i++ )
  {
    if ( dclSharp(pi, result, dclGet(a, a_i), b) < 0 )
      return dclDestroy(result), 0;
  }
  if ( dclCopy(pi, a, result) == 0 )
    return dclDestroy(result), 0;
    
  dclDestroy(result);
  return 1;
}

/*-- dclSubtract ------------------------------------------------------------*/

/* a = a - b */
int dclSubtract(pinfo *pi, dclist a, dclist b)
{
  int a_i, b_i;
  int b_cnt = dclCnt(b);
  int a_cnt;
  dclist result;
  if ( dclInit(&result) == 0 )
    return 0;
  if ( dclClearFlags(a) == 0 )
    return dclDestroy(result), 0;
  if ( dclClearFlags(result) == 0 )
    return dclDestroy(result), 0;
  pinfoProcedureInit(pi, "Subtract", b_cnt);
  for( b_i = 0; b_i < b_cnt; b_i++ )
  {
    a_cnt = dclCnt(a);
    dclClear(result);
    if ( pinfoProcedureDo(pi, b_i) == 0 )
      return pinfoProcedureFinish(pi), 0;
    for( a_i = 0; a_i < a_cnt; a_i++ )
    {
      if ( dclSCCSharpAndSetFlag(pi, result, dclGet(a, a_i), dclGet(b, b_i)) == 0 )
        return pinfoProcedureFinish(pi), dclDestroy(result), 0;
      if ( (a_i % 32) == 0 )
        dclDeleteCubesWithFlag(pi, result);
    }
    dclDeleteCubesWithFlag(pi, result);
    if ( dclCopy(pi, a, result) == 0 )
      return pinfoProcedureFinish(pi), dclDestroy(result), 0;
  }
  pinfoProcedureFinish(pi);
  dclDestroy(result);
  return 1;
}

/*-- dclComplementOut -------------------------------------------------------*/

/* subtract from the cube -----...---- 0000..00100...000 */
/* result is stored in cl */
int dclComplementOut(pinfo *pi, int o, dclist cl)
{
  dcube *c;
  dclist a;
  if ( dclInit(&a) == 0 )
    return 0;
  c = dclAddEmptyCube(pi, a);
  dcInSetAll(pi, c, CUBE_IN_MASK_DC);
  dcOutSetAll(pi, c, 0);
  dcSetOut(c, o, 1);
  if ( dclSubtract(pi, a, cl) == 0 )
    return dclDestroy(cl), 0;
  if ( dclCopy(pi, cl, a) == 0 )
    return dclDestroy(cl), 0;
  return dclDestroy(cl), 1;
}


/*-- dcl_calc_substract -----------------------------------------------------*/
/*
  Calculates the difference a - b and b - a.
  Information is returend, wether these differences is empty.
  
  there are four cases:
    a-b = empty and b-a = empty
      --> 'a' is equal to 'b'
    a-b != empty and b-a = empty
      --> 'b' is a subset of 'a'
    a-b = empty and b-a != empty
      --> 'a' is a subset of 'b'
    a-b != empty and b-a != empty
      --> 'a' and 'b' are partially disjunct
      
*/
/*
 * static int dcl_calc_substract(pinfo *pi, dclist a, dclist b, 
 *   int *is_ab_empty, int *is_ba_empty)
 * {
 *   dclist aa, bb;
 *   if ( dclInitVA(2, &aa, &bb) == 0 )
 *     return 0;
 *   if ( dclCopy(pi, aa, a) == 0 )
 *     return dclDestroyVA(2, aa, bb), 0;
 *   if ( dclCopy(pi, bb, b) == 0 )
 *     return dclDestroyVA(2, aa, bb), 0;
 *   if( dclSubtract(pi, aa, bb) == 0 )
 *     return dclDestroyVA(2, aa, bb), 0;
 *    
 *   *is_ab_empty = 0;
 *   if ( dclCnt(aa) != 0 ) 
 *     *is_ab_empty = 1;
 *     
 *   if ( dclCopy(pi, aa, a) == 0 )
 *     return dclDestroyVA(2, aa, bb), 0;
 *   if ( dclCopy(pi, bb, b) == 0 )
 *     return dclDestroyVA(2, aa, bb), 0;
 *   if( dclSubtract(pi, bb, aa) == 0 )
 *     return dclDestroyVA(2, aa, bb), 0;
 * 
 *   *is_ba_empty = 0;
 *   if ( dclCnt(bb) != 0 ) 
 *     *is_ba_empty = 1;
 * 
 *   return dclDestroyVA(2, aa, bb), 1;
 * }
 */


/*-- dclIsEquivalent --------------------------------------------------------*/
/* Ist b Teilmenge von a?                     */
int dclIsSubsetList(pinfo *pi, dclist a, dclist b)
{
  dclist aa, bb;
  if ( dclInitCachedVA(pi, 2, &aa, &bb) == 0 )
    return 0;
  if ( dclCopy(pi, aa, a) == 0 )
    return dclDestroyCachedVA(pi, 2, aa, bb), 0;
  if ( dclCopy(pi, bb, b) == 0 )
    return dclDestroyCachedVA(pi, 2, aa, bb), 0;
  if( dclSubtract(pi, bb, aa) == 0 )
    return dclDestroyCachedVA(pi, 2, aa, bb), 0;

  if ( dclCnt(bb) != 0 )
    return dclDestroyCachedVA(pi, 2, aa, bb), 0;
  return dclDestroyCachedVA(pi, 2, aa, bb), 1;
}

/*-- dclIsEquivalent --------------------------------------------------------*/

/*
  zwei moeglichkeiten:
  1. a-b = leer und b-a = leer
  2. Kein element von b (a) ist im complement von a (b) enthalten.
*/

int dclIsEquivalent(pinfo *pi, dclist a, dclist b)
{
  dclist aa, bb;
  if ( dclInitVA(2, &aa, &bb) == 0 )
    return 0;
  if ( dclCopy(pi, aa, a) == 0 )
    return dclDestroyVA(2, aa, bb), 0;
  if ( dclCopy(pi, bb, b) == 0 )
    return dclDestroyVA(2, aa, bb), 0;
  if( dclSubtract(pi, aa, bb) == 0 )
    return dclDestroyVA(2, aa, bb), 0;
  if ( dclCnt(aa) != 0 ) /* nicht equivalent */
    return dclDestroyVA(2, aa, bb), 0;
  if ( dclCopy(pi, aa, a) == 0 )
    return dclDestroyVA(2, aa, bb), 0;
  if ( dclCopy(pi, bb, b) == 0 )
    return dclDestroyVA(2, aa, bb), 0;
  if( dclSubtract(pi, bb, aa) == 0 )
    return dclDestroyVA(2, aa, bb), 0;
  if ( dclCnt(bb) != 0 ) /* nicht equivalent */
    return dclDestroyVA(2, aa, bb), 0;
  return dclDestroyVA(2, aa, bb), 1;
}

/*-- dclIsEquivalentDC ------------------------------------------------------*/

/*
  bedingungen:
  1. cl-cl_on-cl_dc = leere menge
  2. cl_on-cl = leere menge
*/

int dclIsEquivalentDC(pinfo *pi, dclist cl, dclist cl_on, dclist cl_dc)
{
  dclist aa, bb;
  if ( dclInitVA(2, &aa, &bb) == 0 )
    return 0;
  if ( dclCopy(pi, aa, cl) == 0 )
    return dclDestroyVA(2, aa, bb), 0;
  if ( dclCopy(pi, bb, cl_on) == 0 )
    return dclDestroyVA(2, aa, bb), 0;
  if( dclSubtract(pi, aa, bb) == 0 )
    return dclDestroyVA(2, aa, bb), 0;
  if ( cl_dc != NULL )
  {
    if ( dclCopy(pi, bb, cl_dc) == 0 )
      return dclDestroyVA(2, aa, bb), 0;
    if( dclSubtract(pi, aa, bb) == 0 )
      return dclDestroyVA(2, aa, bb), 0;
  }
  if ( dclCnt(aa) != 0 ) /* nicht equivalent */
    return dclDestroyVA(2, aa, bb), 0;
  if ( dclCopy(pi, aa, cl_on) == 0 )
    return dclDestroyVA(2, aa, bb), 0;
  if ( dclCopy(pi, bb, cl) == 0 )
    return dclDestroyVA(2, aa, bb), 0;
  if( dclSubtract(pi, aa, bb) == 0 )
    return dclDestroyVA(2, aa, bb), 0;
  if ( dclCnt(aa) != 0 ) /* nicht equivalent */
    return dclDestroyVA(2, aa, bb), 0;
  return dclDestroyVA(2, aa, bb), 1;
}

/*-- dclArea ----------------------------------------------------------------*/
/*
  berechnet die groesste zusammenhaengende menge die teilmenge
  von cl_area ist und cl_s enthaelt.

  Das ergebnis wird in cl_s abgelegt

  Bedingung:
    cl_s ist teilmenge von cl_area
    Das kann mit hilfe von dclIntersectionList erzwungen werden.
    nunja: die bedingung spielt keine groessere rolle, aber
    man kann dann natuerlich nicht sagen, dass das ergebnis
    teilmenge von cl_area ist. 
    
  wenn cl_s und cl_area die SCC eigenschaft erfuellen, dann
  erfuellt das ergebnis in cl_s ebenfalls die SCC eigenschaft
    
      
*/
int dclAreaIsMaxLarge(pinfo *pi, dclist cl_s, dclist cl_area, int *is_max_large)
{
  dclist cl_a, cl_t, cl_r;
  int i, j;
  int is_found;
  if ( dclInitVA(3, &cl_a, &cl_t, &cl_r) == 0 )
    return 0;
  if ( dclCopy(pi, cl_a, cl_area) == 0 )
    return dclDestroyVA(3, cl_a, cl_t, cl_r), 0;
  do
  {
    is_found = 0;
    for( i = 0; i < dclCnt(cl_s); i++ )
    {
      dclClear(cl_t);
      if ( dclClearFlags(cl_t) == 0 )
        return dclDestroyVA(3, cl_a, cl_t, cl_r), 0;
      if ( dclClearFlags(cl_a) == 0 )
        return dclDestroyVA(3, cl_a, cl_t, cl_r), 0;
      for( j = 0; j < dclCnt(cl_a); j++ )
      {
        if ( dcDelta(pi, dclGet(cl_s, i), dclGet(cl_a, j)) <= 1 )
        { 
          if ( dclSCCAddAndSetFlag(pi, cl_t, dclGet(cl_a, j)) == 0 )
            return dclDestroyVA(3, cl_a, cl_t, cl_r), 0;
          dclSetFlag(cl_a, j);
          is_found = 1;
        }
      }
      dclDeleteCubesWithFlag(pi, cl_a);
      dclDeleteCubesWithFlag(pi, cl_t);
    }
    if ( dclSCCUnion(pi, cl_r, cl_s) == 0 )
      return dclDestroyVA(3, cl_a, cl_t, cl_r), 0;
    if ( dclCopy(pi, cl_s, cl_t) == 0 )
      return dclDestroyVA(3, cl_a, cl_t, cl_r), 0;
  } while( is_found != 0 && dclCnt(cl_a) > 0 );
  
  /* cubes in cl_s are still part of the result */
  if ( dclSCCUnion(pi, cl_s, cl_r) == 0 )
    return dclDestroyVA(3, cl_a, cl_t, cl_r), 0;

  if ( is_max_large != NULL )
  {
    if ( dclCnt(cl_a) == 0 )
      *is_max_large = 1;
    else
      *is_max_large = 0;
  }
  
  return dclDestroyVA(3, cl_a, cl_t, cl_r), 1;
}

int dclArea(pinfo *pi, dclist cl_s, dclist cl_area)
{
  return dclAreaIsMaxLarge(pi, cl_s, cl_area, NULL);
}

/*-- dclIsRelated -----------------------------------------------------------*/

int dclIsRelated(pinfo *pi, dclist cl)
{
  dclist cl_s;
  int is_max_large = 0;
  if ( dclInit(&cl_s) == 0 )
    return 0;
  if ( dclAdd(pi, cl_s, dclGet(cl, 0)) < 0 )
    return dclDestroy(cl_s), 0;
  if ( dclAreaIsMaxLarge(pi, cl_s, cl, &is_max_large) == 0 )
    return dclDestroy(cl_s), 0;
  return is_max_large;
}

/*-- dclComplementWithSharp -------------------------------------------------*/

/* Das ergebnis erfuellt die SCC eigenschaft und enthaelt alle (!) */
/* primeimplikanten. */
int dclComplementWithSharp(pinfo *pi, dclist cl)
{
  dclist ncl;
  dcube *tautologie_cube = &(pi->tmp[0]);

  if ( dclCnt(cl) == 0 )
  {
    if ( dclAdd(pi, cl, tautologie_cube) < 0 )
      return 0;
    return 1;
  }
  else if ( dclCnt(cl) == 1 )
  {
    dcube *c = &(pi->tmp[13]);
    dcCopy(pi, c, dclGet(cl, 0));
    dclClear(cl);
    if ( dclComplementCube(pi, cl, c) == 0 )
      return 0;
    return 1;
  }
  
  if ( dclInit(&ncl) == 0 )
    return 0;
  if ( dclAdd(pi, ncl, tautologie_cube) < 0 )
    return dclDestroy(ncl), 0;
  if ( dclSubtract(pi, ncl, cl) == 0 )
    return dclDestroy(ncl), 0;
  if ( dclCopy(pi, cl, ncl) == 0 )
    return dclDestroy(ncl), 0;
  
  dclDestroy(ncl);
  return 1;  
}

/*-- dclComplementWithURP ---------------------------------------------------*/

/* Komplementiert eine function */
/* Das ergebnis erfuellt die SCC eigenschaft und enthaelt nur */
/* primeimplikanten (aber nicht alle!) */
int dclComplementCof(pinfo *pi, dclist cl, dcube *cof, int depth)
{
  int i, j;
  dclist cl_left, cl_right, cl_c;
  dcube *cof_left = &(pi->stack1[depth]);
  dcube *cof_right = &(pi->stack2[depth]);
  int left_cnt, right_cnt;

  if (depth >= PINFO_STACK_CUBES)
    return 0;
    
  /* wenn die liste leer ist, ist das ergebnis der universelle cube */
  if ( dclCnt(cl) == 0 )
    return dclAdd(pi, cl, &(pi->tmp[0])) < 0 ? 0 : 1;
    
  /* wenn die liste nur ein element hat, complementiere dieses */
  if ( dclCnt(cl) == 1 )
  {
    dcCopy(pi, &(pi->tmp[6]), dclGet(cl, 0));
    dclClear(cl);
    return dclComplementCube(pi, cl, &(pi->tmp[6]));
  }
  
  /* wenn die liste ein einer spalte 0 hat, gibt es einen specialfall: */
  /* dann ist naemlich: F = a AND F_a */
  /* dessen complement ist: F = a' OR F'_a */
  /*
  {
    dclist cl_cof;
    dclOrElements(pi, cof_left, cl);
    {
      for( i = 0; i < pi->out_cnt; i++ )
        if ( dcGetOut(cof_left, i) == 0 && dcGetOut(cof, i) != 0 )
          break;
      if ( i < pi->out_cnt || dcIsInTautology(pi, cof_left) == 0 )
      {
        for( i = 0; i < pi->out_words; i++ )
          cof_left->out[i] &= cof->out[i];
        if ( dclInit(&cl_cof) == 0 )
          return 0;
        if ( dclSCCCofactor(pi, cl_cof, cl, cof_left) == 0 )
          return dclDestroy(cl_cof), 0;
        if ( dclComplementCof(pi, cl_cof, cof_left, depth+1) == 0 )
          return dclDestroy(cl_cof), 0;
        dclClear(cl);
        if ( dclComplementCube(pi, cl, cof_left) == 0 )
          return dclDestroy(cl_cof), 0;
        if ( dclSCCUnion(pi, cl, cl_cof) == 0 )
          return dclDestroy(cl_cof), 0;
        return dclDestroy(cl_cof), 1;
      }
    }
  }
  */

  if ( dcGetNoneDCCofactorForSplit(pi, cof_left, cof_right, cl, cof) == 0 )
    return 0;

  if ( dclInitVA(3, &cl_left, &cl_right, &cl_c) == 0 )
    return 0;
    
  if ( dclSCCCofactor(pi, cl_left, cl, cof_left) == 0 )
    return dclDestroyVA(3, cl_left, cl_right, cl_c), 0;
  if ( dclSCCCofactor(pi, cl_right, cl, cof_right) == 0 )
    return dclDestroyVA(3, cl_left, cl_right, cl_c), 0;
  left_cnt = dclCnt(cl_left);
  right_cnt = dclCnt(cl_right);
  
  pinfoBTreeStart(pi);
  
  if ( dclComplementCof(pi, cl_left, cof_left, depth+1) == 0 )
    return pinfoBTreeEnd(pi), dclDestroyVA(3, cl_left, cl_right, cl_c), 0;
  if ( dclComplementCof(pi, cl_right, cof_right, depth+1) == 0 )
    return pinfoBTreeEnd(pi), dclDestroyVA(3, cl_left, cl_right, cl_c), 0;

  pinfoBTreeEnd(pi);

  /* Multiple-Valued Minimization for PLA Optimization: p736*/
  /* Die beiden teillisten werden vergroessert. Da fuer beide */
  /* Haelften die SCC eigenschaft erfuellt ist, ist sie nach  */
  /* dem erweitern ebenfalls noch erfuellt. */
    
  dclExpand1(pi, cl_left, cl);
  dclExpand1(pi, cl_right, cl);
    
  /* Multiple-Valued Minimization for PLA Optimization: p735*/
  /* Durch die Bildung der Schnittmenge mit dem Kofaktor */
  /* werden Terme in zwei Haelften zerschnitten, die man besser */
  /* auch haette zusammenfassen koennen. Um diesen Split zu ver- */
  /* meiden, werden die Terme in eine dritte Liste transferiert. */
  /* Gleiches gilt auf fuer die Subsets der jeweiligen anderen */
  /* Haelfte. Interessanterweise kann dadurch spaeter die SCC */
  /* operation gespart werden, wenn die identischen Elemente */
  /* korrekt behandelt werden. */

  dclClearFlags(cl_left);
  dclClearFlags(cl_right);
  dclClearFlags(cl_c);
  
  for( i = 0; i < dclCnt(cl_left); i++ )
    for( j = 0; j < dclCnt(cl_right); j++ )
    {
      if ( dclIsFlag(cl_right, j) == 0 )
        if ( dcIsSubSet(pi, dclGet(cl_left, i), dclGet(cl_right, j)) != 0 )
        {
          dclSetFlag(cl_right, j);
          if ( dclSCCAddAndSetFlag(pi, cl_c, dclGet(cl_right, j)) == 0 )
            return dclDestroyVA(3, cl_left, cl_right, cl_c), 0;
          if ( dcIsEqual(pi, dclGet(cl_left, i), dclGet(cl_right, j)) != 0 )
            dclSetFlag(cl_left, i);
        }
      if ( dclIsFlag(cl_left, i) == 0 )
        if ( dcIsSubSet(pi, dclGet(cl_right, j), dclGet(cl_left, i)) != 0 )
        {
          dclSetFlag(cl_left, i);
          if ( dclSCCAddAndSetFlag(pi, cl_c, dclGet(cl_left, i)) == 0 )
            return dclDestroyVA(3, cl_left, cl_right, cl_c), 0;
          if ( dcIsEqual(pi, dclGet(cl_left, i), dclGet(cl_right, j)) != 0 )
            dclSetFlag(cl_right, j);
        }
    }

  dclDeleteCubesWithFlag(pi, cl_left);
  dclDeleteCubesWithFlag(pi, cl_right);
  dclDeleteCubesWithFlag(pi, cl_c);

  if ( dclIntersection(pi, cl_left, cof_left) == 0 )
    return dclDestroyVA(3, cl_left, cl_right, cl_c), 0;
  if ( dclIntersection(pi, cl_right, cof_right) == 0 )
    return dclDestroyVA(3, cl_left, cl_right, cl_c), 0;

  dclClear(cl);
  if ( dclJoin(pi, cl, cl_left) == 0 )
    return dclDestroyVA(3, cl_left, cl_right, cl_c), 0;
  if ( dclJoin(pi, cl, cl_right) == 0 )
    return dclDestroyVA(3, cl_left, cl_right, cl_c), 0;
  if ( dclJoin(pi, cl, cl_c) == 0 )
    return dclDestroyVA(3, cl_left, cl_right, cl_c), 0;
  
  return dclDestroyVA(3, cl_left, cl_right, cl_c), 1;
}

int dclComplementWithURP(pinfo *pi, dclist cl)
{
  int result;
  dcube *cof = &(pi->tmp[2]);
  if ( dclSCC(pi, cl) == 0 )
    return 0;
  dcSetTautology(pi, cof);
  pinfoBTreeInit(pi, "Complement URP");
  result = dclComplementCof(pi, cl, cof, 0);
  pinfoBTreeFinish(pi);
  return result;
}

int dclComplement(pinfo *pi, dclist cl)
{
  return dclComplementWithURP(pi, cl);
}


/*-- dclPrimes --------------------------------------------------------------*/

int dclPrimesCof(pinfo *pi, dclist cl, dcube *cof, int depth)
{
  dclist cl_left, cl_right;
  dcube *cof_left = &(pi->stack1[depth]);
  dcube *cof_right = &(pi->stack2[depth]);
  int left_cnt, right_cnt; 

  if (depth >= PINFO_STACK_CUBES)
    return 0;
    
  if ( dclCnt(cl) <= 1 )
    return 1;
  
  if ( dcGetCofactorForSplit(pi, cof_left, cof_right, cl, cof) == 0 )
    return 1;
    
  if ( dclInitCachedVA(pi, 2, &cl_left, &cl_right) == 0 )
    return 0;
    
  if ( dclSCCCofactor(pi, cl_left, cl, cof_left) == 0 )
    return dclDestroyCachedVA(pi, 2, cl_left, cl_right), 0;
  if ( dclSCCCofactor(pi, cl_right, cl, cof_right) == 0 )
    return dclDestroyCachedVA(pi, 2, cl_left, cl_right), 0;
  left_cnt = dclCnt(cl_left);
  right_cnt = dclCnt(cl_right);
  pinfoBTreeStart(pi);
  if ( dclPrimesCof(pi, cl_left, cof_left, depth+1) == 0 )
    return pinfoBTreeEnd(pi), dclDestroyCachedVA(pi, 2, cl_left, cl_right), 0;
  if ( dclPrimesCof(pi, cl_right, cof_right, depth+1) == 0 )
    return pinfoBTreeEnd(pi), dclDestroyCachedVA(pi, 2, cl_left, cl_right), 0;
  pinfoBTreeEnd(pi);
  if ( dclIntersection(pi, cl_left, cof_left) == 0 )
    return dclDestroyCachedVA(pi, 2, cl_left, cl_right), 0;
  if ( dclIntersection(pi, cl_right, cof_right) == 0 )
    return dclDestroyCachedVA(pi, 2, cl_left, cl_right), 0;
  
  /*
  printf("Split: %d -> %d + %d (%d + %d)   \n", dclCnt(cl), left_cnt, right_cnt, dclCnt(cl_left), dclCnt(cl_right));
  */

  if ( dcIsInTautology(pi, cof_left) != 0 )
  {
    dclSortOutSize(pi, cl_left);
    dclSortOutSize(pi, cl_right);
  }
  else
  {
    dclSortInSize(pi, cl_left);
    dclSortInSize(pi, cl_right);
  }

  /* dclClear(cl); */
  if ( dclConsensus(pi, cl, cl_left, cl_right) == 0 )
    return dclDestroyCachedVA(pi, 2, cl_left, cl_right), 0;
  if ( dclSCCUnionSubset(pi, cl, cl_left) == 0 )
    return dclDestroyCachedVA(pi, 2, cl_left, cl_right), 0;
  if ( dclSCCUnionSubset(pi, cl, cl_right) == 0 )
    return dclDestroyCachedVA(pi, 2, cl_left, cl_right), 0;
  
  return dclDestroyCachedVA(pi, 2, cl_left, cl_right), 1;
}

int dclAddNoneSubsetList(pinfo *pi, dclist dest, dclist src, dcube *l, dcube *r)
{
  int i, cnt = dclCnt(src);
  for( i = 0; i < cnt; i++ )
  {
    if ( dcIsSubSet(pi, l, dclGet(src, i)) != 0 )
      continue;
    if ( dcIsSubSet(pi, r, dclGet(src, i)) != 0 )
      continue;
    if ( dclAdd(pi, dest, dclGet(src, i)) < 0 )
      return 0;
  }
  return 1;
}

int xdclPrimesCof(pinfo *pi, dclist cl, dcube *cof, int depth)
{
  dclist cl_l_sub, cl_r_sub, cl_l_red, cl_r_red, cl_c;
  dcube *cof_left = &(pi->stack1[depth]);
  dcube *cof_right = &(pi->stack2[depth]);
  int left_cnt, right_cnt;

  if (depth >= PINFO_STACK_CUBES)
    return 0;
    
  if ( dclCnt(cl) <= 1 )
    return 1;
  
  if ( dcGetCofactorForSplit(pi, cof_left, cof_right, cl, cof) == 0 )
    return 1;
  dclInitVA(5, &cl_l_sub, &cl_r_sub, &cl_l_red, &cl_r_red, &cl_c);
    
  pinfoDepth(pi, "Primes", depth);
  
  if ( dcIsInTautology(pi, cof_left) != 0 )
  {
    /* output split */

    dclAddSubCofactor(pi, cl_l_sub, cl, cof_left);
    dclAddSubCofactor(pi, cl_r_sub, cl, cof_right);
    dclAddRedCofactor(pi, cl_l_red, cl, cof_left);
    dclAddRedCofactor(pi, cl_r_red, cl, cof_right);
    left_cnt = dclCnt(cl_l_sub) + dclCnt(cl_l_red);
    right_cnt = dclCnt(cl_r_sub) + dclCnt(cl_r_red);

    dclSCC(pi, cl_l_red);
    dclSCC(pi, cl_r_red);
    dclJoin(pi, cl_l_red, cl_l_sub);
    dclJoin(pi, cl_r_red, cl_r_sub);
    dclPrimesCof(pi, cl_l_red, cof_left, depth+1);
    dclPrimesCof(pi, cl_r_red, cof_right, depth+1);
    dclIntersection(pi, cl_l_red, cof_left);
    dclIntersection(pi, cl_r_red, cof_right);

    printf("Split: %d -> %d + %d (%d + %d)   \n", dclCnt(cl), left_cnt,
    right_cnt, dclCnt(cl_l_red), dclCnt(cl_r_red));
    dclClear(cl);
    dclConsensus(pi, cl, cl_l_red, cl_r_red);
    dclSCCUnionSubset(pi, cl, cl_l_red);
    dclSCCUnionSubset(pi, cl, cl_r_red);
  }
  else
  {
    dclAddSubCofactor(pi, cl_l_sub, cl, cof_left);
    dclAddSubCofactor(pi, cl_r_sub, cl, cof_right);
    dclAddNoneSubsetList(pi, cl_c, cl, cof_left, cof_right);
    assert(dclCnt(cl_l_sub)+dclCnt(cl_r_sub)+dclCnt(cl_c) == dclCnt(cl));

    left_cnt = dclCnt(cl_l_sub) + dclCnt(cl_l_red);
    right_cnt = dclCnt(cl_r_sub) + dclCnt(cl_r_red);

    /* input split */
    dclJoin(pi, cl_l_sub, cl_c);
    dclJoin(pi, cl_r_sub, cl_c);
    dclPrimesCof(pi, cl_l_sub, cof_left, depth+1);
    dclPrimesCof(pi, cl_r_sub, cof_right, depth+1);
    dclIntersection(pi, cl_l_sub, cof_left);
    dclIntersection(pi, cl_r_sub, cof_right);

    puts(dcToStr(pi, cof_left, " ", ""));
    printf("Split: %d -> %d + %d (%d + %d)   \n", dclCnt(cl), left_cnt,
    right_cnt, dclCnt(cl_l_sub), dclCnt(cl_r_sub));
    dclClear(cl);
    dclConsensus(pi, cl, cl_l_sub, cl_r_sub);
    dclSCCUnion(pi, cl, cl_c);
    dclSCCUnionSubset(pi, cl, cl_l_sub);
    dclSCCUnionSubset(pi, cl, cl_r_sub);
  }
  
  return dclDestroyVA(5, cl_l_sub, cl_r_sub, cl_l_red, cl_r_red, cl_c), 1;
}

int dclPrimes(pinfo *pi, dclist cl)
{
  int result;
  dcube *cof = &(pi->tmp[2]);
  if ( dclSCC(pi, cl) == 0 )
    return 0;
  dcSetTautology(pi, cof);
  pinfoBTreeInit(pi, "Primes");
  result = dclPrimesCof(pi, cl, cof, 0);
  pinfoBTreeFinish(pi);
  return result;
}

/*-- dclPrimesDC ------------------------------------------------------------*/

int dclPrimesDC(pinfo *pi, dclist cl, dclist cl_dc)
{
  if ( cl_dc != NULL )
    if ( dclJoin(pi, cl, cl_dc) == 0 )
      return 0;
  if ( dclPrimes(pi, cl) == 0 )
    return 0;
  if ( cl_dc != NULL )
    if ( dclRemoveSubSet(pi, cl, cl_dc, NULL) == 0 )
      return 0;
  return 1;
}


/*-- dclPrimesInv -----------------------------------------------------------*/
/*
  
  A(L)
  
  A(L) = SCCINV(x A(L_x) + !x (L_!x) + INTERSECTION(x A(L_x), !x (L_!x)))
  
*/

static int dcPrimesInvGetCofactorForSplit(pinfo *pi, dcube *l, dcube *r, dclist cl, dcube *cof)
{
  /* geht nicht --- may be, but why and... is this true???
  if ( pinfoGetInVarDCubeCofactor(pi, l, r, cl, cof) != 0 )
    return 1;
  */
  /*
  if ( dcGetBinateInVarCofactor(pi, l, r, cl, cof) != 0 )
    return 1;
  if ( dcGetOutVarCofactor(pi, l, r, cl, cof) != 0 )
    return 1;
  if ( pinfoGetInVarDCubeCofactor(pi, l, r, cl, cof) != 0 )
    return 1;
  if ( dcGetBinateInVarCofactor(pi, l, r, cl, cof) != 0 )
    return 1;
  */

  if ( pinfoGetInVarDCubeCofactor(pi, l, r, cl, cof) != 0 )
    return 1;
/*
  if ( dcGetBinateInVarCofactor(pi, l, r, cl, cof) != 0 )
    return 1;
*/
  if ( pinfoGetOutVarDCubeCofactor(pi, l, r, cl, cof) != 0 )
    return 1;

  return 0;
}

int dclPrimesInvCof(pinfo *pi, dclist cl, dcube *cof, int depth)
{
  dclist cl_left, cl_right;
  dcube *cof_left = &(pi->stack1[depth]);
  dcube *cof_right = &(pi->stack2[depth]);
  int left_cnt, right_cnt;

  if (depth >= PINFO_STACK_CUBES)
    return 0;
    
  if ( dclCnt(cl) <= 1 )
    return 1;
  
  if ( dcPrimesInvGetCofactorForSplit(pi, cof_left, cof_right, cl, cof) == 0 )
  {
    if ( dclCnt(cl) >= 2 )
    {
      int i;
      for( i = 1; i < dclCnt(cl); i++ )
      {
        dcIntersection(pi, dclGet(cl, 0), dclGet(cl, 0), dclGet(cl, i));
        dclSetFlag(cl, i);
      }
      dclDeleteCubesWithFlag(pi, cl);
      if ( dcIsIllegal(pi, dclGet(cl, 0)) != 0 )
        return 0; /* should never occur, but happend on 9 oct 2002 */
    }
    return 1;
  }
  
  if ( dclInitCachedVA(pi, 2, &cl_left, &cl_right) == 0 )
    return 0;
    
  if ( dclSCCInvCofactor(pi, cl_left, cl, cof_left) == 0 )
    return dclDestroyCachedVA(pi, 2, cl_left, cl_right), 0;

  if ( dclSCCInvCofactor(pi, cl_right, cl, cof_right) == 0 )
    return dclDestroyCachedVA(pi, 2, cl_left, cl_right), 0;

  left_cnt = dclCnt(cl_left);
  right_cnt = dclCnt(cl_right);
  pinfoBTreeStart(pi);
  if ( dclPrimesInvCof(pi, cl_left, cof_left, depth+1) == 0 )
    return pinfoBTreeEnd(pi), dclDestroyCachedVA(pi, 2, cl_left, cl_right), 0;
  if ( dclPrimesInvCof(pi, cl_right, cof_right, depth+1) == 0 )
    return pinfoBTreeEnd(pi), dclDestroyCachedVA(pi, 2, cl_left, cl_right), 0;
  pinfoBTreeEnd(pi);
  if ( dclIntersection(pi, cl_left, cof_left) == 0 )
    return dclDestroyCachedVA(pi, 2, cl_left, cl_right), 0;
  if ( dclIntersection(pi, cl_right, cof_right) == 0 )
    return dclDestroyCachedVA(pi, 2, cl_left, cl_right), 0;
  

  dclClear(cl);
  /*
  if ( dclIntersectionListInv(pi, cl, cl_left, cl_right) == 0 )
    return dclDestroyCachedVA(pi, 2, cl_left, cl_right), 0;
  */
  if ( dclSCCInvUnionSubset(pi, cl, cl_left) == 0 )
    return dclDestroyCachedVA(pi, 2, cl_left, cl_right), 0;
  if ( dclSCCInvUnionSubset(pi, cl, cl_right) == 0 )
    return dclDestroyCachedVA(pi, 2, cl_left, cl_right), 0;
  
  /*
  printf("Split: %d -> %d + %d (%d + %d)   \n", dclCnt(cl), left_cnt, right_cnt, dclCnt(cl_left), dclCnt(cl_right));
  */
  
  return dclDestroyCachedVA(pi, 2, cl_left, cl_right), 1;
}


int dclPrimesInv(pinfo *pi, dclist cl)
{
  int result;
  dcube *cof = &(pi->tmp[2]);
  dclSCCInv(pi, cl);
  dcSetTautology(pi, cof);
  pinfoBTreeInit(pi, "PrimesInv");
  result = dclPrimesInvCof(pi, cl, cof, 0);
  pinfoBTreeFinish(pi);
  return result;
}


/*-- dclIsEssential ---------------------------------------------------------*/

int dclIsEssential(pinfo *pi, dclist cl_on, dclist cl_dc, dcube *a)
{
  dclist cl;
  if ( dclInitCached(pi, &cl) == 0 )
    return 0;
  if ( dclCopy(pi, cl, cl_on) == 0 )
    return dclDestroyCached(pi, cl), 0;
  if ( cl_dc != NULL )
    if ( dclJoin(pi, cl, cl_dc) == 0 )
      return dclDestroyCached(pi, cl), 0;
  if ( dclSubtractCube(pi, cl, a) == 0 )
    return dclDestroyCached(pi, cl), 0;
  if ( dclConsensusCube(pi, cl, a) == 0 )
    return dclDestroyCached(pi, cl), 0;
  if ( cl_dc != NULL )
    if ( dclJoin(pi, cl, cl_dc) == 0 )
      return dclDestroyCached(pi, cl), 0;
  if ( dclIsSubSet(pi, cl, a) != 0 )
    return dclDestroyCached(pi, cl), 0;
  return dclDestroyCached(pi, cl), 1; /* essential prime */
}

/*-- dclIsRelativeEssential -------------------------------------------------*/

int dclIsRelativeEssential(pinfo *pi, dclist cl, int pos)
{
  dclist cl_tmp;
  if ( dclInitCached(pi, &cl_tmp) == 0 )
    return 0;
  if ( dclSCCCofactorExcept(pi, cl_tmp, cl, dclGet(cl, pos), pos) == 0 )
    return dclDestroyCached(pi, cl_tmp), 0;
  if ( dclTautology(pi, cl_tmp) != 0 )
    return dclDestroyCached(pi, cl_tmp), 0;
  return dclDestroyCached(pi, cl_tmp), 1;
}

int dclIsRelativeEssentialDC(pinfo *pi, dclist cl, int pos, dclist cl_dc)
{
  dclist cl_tmp;
  int i;
  if ( dclInitCached(pi, &cl_tmp) == 0 )
    return 0;
  for( i = 0; i < dclCnt(cl); i++ )
    if ( i != pos )
      if ( dclAdd(pi, cl_tmp, dclGet(cl, i)) < 0 )
        return dclDestroyCached(pi, cl_tmp), 0;  
  if ( cl_dc != NULL )
    if ( dclJoin(pi, cl_tmp, cl_dc) == 0 )
      return dclDestroyCached(pi, cl_tmp), 0;
  if ( dclIsSubSet(pi, cl_tmp, dclGet(cl, pos)) != 0)
    return dclDestroyCached(pi, cl_tmp), 0;
  return dclDestroyCached(pi, cl_tmp), 1; /* relative essential */
}

/*-- dclEssential -----------------------------------------------------------*/

int dclEssential(pinfo *pi, dclist cl_es, dclist cl_nes, dclist cl, dclist cl_on, dclist cl_dc)
{
  int i, cnt;
  if ( cl_es != NULL )
    dclClear(cl_es);
  if ( cl_nes != NULL )
    dclClear(cl_nes);
  if ( cl_on == NULL && cl == NULL )
    return 0;
  if ( cl_on == NULL )
    cl_on = cl;
  if ( cl == NULL )
    cl = cl_on;
  cnt = dclCnt(cl);
  pinfoProcedureInit(pi, "Essential", cnt);
  for( i = 0; i < cnt; i++ )
  {
    if ( pinfoProcedureDo(pi, i) == 0 )
      return pinfoProcedureFinish(pi), 0;
    if ( dclIsEssential(pi, cl, cl_dc, dclGet(cl, i)) != 0 )
    {
      if ( cl_es != NULL )
        if ( dclAdd(pi, cl_es, dclGet(cl, i)) < 0 )
          return pinfoProcedureFinish(pi), 0;
    }
    else
    {
      if ( cl_nes != NULL )
        if ( dclAdd(pi, cl_nes, dclGet(cl, i)) < 0 )
          return pinfoProcedureFinish(pi), 0;
    }
  }
  pinfoProcedureFinish(pi);
  return 1;
}

/*-- dclRelativeEssential --------------------------------------------------*/

int dclRelativeEssential(pinfo *pi, dclist cl_es, dclist cl_nes, dclist cl, dclist cl_dc)
{
  int i, cnt;
  dclist cl_tmp;
  if ( dclInitCached(pi, &cl_tmp) == 0 )
    return 0;
  if ( cl_es != NULL )
    dclClear(cl_es); 
  if ( cl_nes != NULL )
    dclClear(cl_nes);
  if ( dclCopy(pi, cl_tmp, cl) == 0 )
    return dclDestroyCached(pi, cl_tmp), 0;
  if ( cl_dc != NULL )
    if ( dclJoin(pi, cl_tmp, cl_dc) == 0 )
      return dclDestroyCached(pi, cl_tmp), 0;
  cnt = dclCnt(cl); /* !!! sic !!! */
  pinfoProcedureInit(pi, "Essential", cnt);
  for( i = 0; i < cnt; i++ )
  {
    if ( pinfoProcedureDo(pi, i) == 0 )
      return dclDestroyCached(pi, cl_tmp), pinfoProcedureFinish(pi), 0;
    if ( dclIsRelativeEssential(pi, cl_tmp, i) != 0 )
    {
      if ( cl_es != NULL )
        if ( dclAdd(pi, cl_es, dclGet(cl, i)) < 0 )
          return dclDestroyCached(pi, cl_tmp), pinfoProcedureFinish(pi), 0;
    }
    else
    {
      if ( cl_nes != NULL )
        if ( dclAdd(pi, cl_nes, dclGet(cl, i)) < 0 )
          return dclDestroyCached(pi, cl_tmp), pinfoProcedureFinish(pi), 0;
    }
  }
  dclDestroyCached(pi, cl_tmp);
  pinfoProcedureFinish(pi);
  return 1;
}

/*-- dclGetEssential --------------------------------------------------------*/
int dclGetEssential(pinfo *pi, dclist cl, dclist cl_on, dclist cl_dc)
{
  dclist cl_es;
  if ( cl_on == NULL )
    cl_on = cl;
  if ( dclInit(&cl_es) == 0 )
    return 0;
  if ( dclEssential(pi, cl_es, NULL, cl, cl_on, cl_dc) == 0 )
    return dclDestroy(cl_es), 0;
  if ( dclCopy(pi, cl, cl_es) == 0 )
    return dclDestroy(cl_es), 0;
  return dclDestroy(cl_es), 1;
}

/*-- dclSplitEssential ------------------------------------------------------*/
/*
  Spaltet eine function auf in:
    - essentielle cubes [cl_es]
    - vollkommen redundante cubes [cl_fr] (cl_vr wird von cl_es ueberdeckt)
    - teilweise redundaten cubes [cl_pr] (die uebrigen cubes)
*/

int dclSplitEssential(pinfo *pi, dclist cl_es, dclist cl_fr, dclist cl_pr, dclist cl, dclist cl_on, dclist cl_dc)
{
  int i, cnt;
  dclist cl_es_dc;

  dclClear(cl_es);
  dclClear(cl_pr);
  if ( cl_fr != NULL )
    dclClear(cl_fr);
  
  /*
  if ( dclEssential(pi, cl_es, cl_pr, cl, cl_on, cl_dc) == 0 )
    return dclClear(cl_es), dclClear(cl_pr), 0;
  */
  if ( dclRelativeEssential(pi, cl_es, cl_pr, cl, cl_dc) == 0 )
    return dclClear(cl_es), dclClear(cl_pr), 0;
  
  if ( dclClearFlags(cl_pr) == 0 )
    return dclClear(cl_es), dclClear(cl_pr), 0;

  if ( dclInit(&cl_es_dc) == 0 )
    return dclClear(cl_es), dclClear(cl_pr), 0;

  if ( dclCopy(pi, cl_es_dc, cl_es) == 0 )
    return dclClear(cl_es), dclClear(cl_pr), dclDestroy(cl_es_dc), 0;
    
  if ( cl_dc != NULL )
    if ( dclJoin(pi, cl_es_dc, cl_dc) == 0 )
      return dclClear(cl_es), dclClear(cl_pr), dclDestroy(cl_es_dc), 0;

  cnt = dclCnt(cl_pr);
  pinfoProcedureInit(pi, "SplitEssential", cnt);
  for( i = 0; i < cnt; i++ )
  {
    if ( pinfoProcedureDo(pi, i) == 0 )
      return pinfoProcedureFinish(pi), 0;
    if  ( dclIsSubSet(pi, cl_es_dc, dclGet(cl_pr, i)) != 0 )
    {
      if ( cl_fr != NULL )
      {
        if ( dclAdd(pi, cl_fr, dclGet(cl_pr, i)) < 0 )
        {
          dclClear(cl_es);
          dclClear(cl_pr);
          dclDestroy(cl_es_dc);
          pinfoProcedureFinish(pi);
          return 0;
        }
      }
      dclSetFlag(cl_pr, i);
    }
  }
  pinfoProcedureFinish(pi);

  dclDeleteCubesWithFlag(pi, cl_pr);
  dclDestroy(cl_es_dc);
  return 1;
}

/*-- dclSplitRelativEssential ----------------------------------------------*/
/*
  Spaltet eine function auf in:
    - essentielle cubes [cl_es]
    - vollkommen redundante cubes [cl_fr] (cl_vr wird von cl_es ueberdeckt,
      darf aber kein element aus cl_rc ueberdecken)
    - teilweise redundaten cubes [cl_pr] (die uebrigen cubes)
*/

int dclSplitRelativeEssential(pinfo *pi, dclist cl_es, dclist cl_fr, dclist cl_pr, dclist cl, dclist cl_dc, dclist cl_rc)
{
  int i, cnt;
  int j;
  dclist cl_es_dc;
  
  dclClear(cl_es);
  dclClear(cl_pr);
  if ( cl_fr != NULL )
    dclClear(cl_fr);
  
  if ( dclRelativeEssential(pi, cl_es, cl_pr, cl, cl_dc) == 0 )
    return dclClear(cl_es), dclClear(cl_pr), 0;
  
  if ( dclClearFlags(cl_pr) == 0 )
    return dclClear(cl_es), dclClear(cl_pr), 0;

  if ( dclInit(&cl_es_dc) == 0 )
    return dclClear(cl_es), dclClear(cl_pr), 0;

  if ( dclCopy(pi, cl_es_dc, cl_es) == 0 )
    return dclClear(cl_es), dclClear(cl_pr), dclDestroy(cl_es_dc), 0;
    
  if ( cl_dc != NULL )
    if ( dclJoin(pi, cl_es_dc, cl_dc) == 0 )
      return dclClear(cl_es), dclClear(cl_pr), dclDestroy(cl_es_dc), 0;

  cnt = dclCnt(cl_pr);
  pinfoProcedureInit(pi, "SplitRelativeEssential", cnt);
  for( i = 0; i < cnt; i++ )
  {
    if ( pinfoProcedureDo(pi, i) == 0 )
      return dclDestroy(cl_es_dc), pinfoProcedureFinish(pi), 0;
    if ( cl_rc != NULL )
    {
      for( j = 0; j < dclCnt(cl_rc); j++ )
        if ( dcIsSubSet(pi, dclGet(cl_pr, i), dclGet(cl_rc, j)) != 0 )
          break;
      if ( j < dclCnt(cl_rc) )
        continue; /* do not remove the pr prime, go to next one */
    }
    if ( dclIsSubSet(pi, cl_es_dc, dclGet(cl_pr, i)) != 0 )
    {
      if ( cl_fr != NULL )
      {
        if ( dclAdd(pi, cl_fr, dclGet(cl_pr, i)) < 0 )
        {
          dclClear(cl_es);
          dclClear(cl_pr);
          dclDestroy(cl_es_dc);
          pinfoProcedureFinish(pi);
          return 0;
        }
      }
      dclSetFlag(cl_pr, i);
    }
  }
  pinfoProcedureFinish(pi);
  dclDestroy(cl_es_dc);
  
  dclDeleteCubesWithFlag(pi, cl_pr);
  return 1;
}

/*-- dclIrredundantMark -----------------------------------------------------*/

/*
 * static int dclIrredundantStoreList(pinfo *pi_m, dclist cl_m, pinfo *pi, int n, dclist cl)
 * {
 *   dcube *c = pi_m->tmp+9;
 *   int i, cnt = dclCnt(cl);
 * 
 *   dcInSetAll(pi_m, c, CUBE_IN_MASK_DC);
 *   dcOutSetAll(pi_m, c, 0);
 *   dcSetOut(c, n, 1);
 *   for( i = 0; i < cnt; i++ )
 *     if ( dcIsTautology(pi, dclGet(cl, i)) != 0 )
 *       if ( dclGet(cl, i)->n >= 0 )
 *         dcSetOut(c, dclGet(cl, i)->n, 1);
 *   if ( dclSCCInvAddAndSetFlag(pi_m, cl_m, c) == 0 )
 *     return 0;
 *   return 1;
 * }
 */

static int dcIrredundantGetCofactorForSplit(pinfo *pi, dcube *l, dcube *r, dclist cl, dcube *cof)
{
  if ( pinfoGetInVarDCubeCofactor(pi, l, r, cl, cof) != 0 )
    return 1;
  /*
  if ( dcGetBinateInVarCofactor(pi, l, r, cl, cof) != 0 )
    return 1;
  */
  if ( dcGetOutVarCofactor(pi, l, r, cl, cof) != 0 )
    return 1;
  return 0;
}

static int dclIrredundantCofactor(pinfo *pi, dclist dest, dclist src, dcube *cofactor, dcube *mark)
{
  int i, cnt = dclCnt(src);
  dcube *r = &(pi->tmp[1]);
  dclClear(dest);
  for ( i = 0; i < cnt; i++ )
    if ( dcCofactor(pi, r, dclGet(src, i), cofactor) != 0 )
    {
      if ( dcIsTautology(pi, r) != 0 && r->n >= 0 )
      {
        dcSetOut(mark, r->n, 1);
      }
      else
      {
        if ( dclAdd(pi, dest, r) < 0 )
          return 0;
      }
    }
  return 1;
}


int dclIrredundantMarkTautCof(pinfo *pi_m, dclist cl_m, pinfo *pi, dclist cl, dcube *cof, dcube *mark, int depth)
{
  dclist cl_left, cl_right;
  dcube *cofactor_left = &(pi->stack1[depth]);
  dcube *cofactor_right = &(pi->stack2[depth]);
  dcube *mark_left = &(pi_m->stack1[depth]);
  dcube *mark_right = &(pi_m->stack2[depth]);
  int i, cnt;
  
  
  cnt = dclCnt(cl);

  if ( cnt == 0 )
  {
    if ( dclSCCInvAddAndSetFlag(pi_m, cl_m, mark) == 0 )
      return 0;
    return 1;
  }

  {
    dcube *c;
    for( i = 0; i < cnt; i++ )
    {
      c = dclGet(cl, i);
      /* wenn tautologie und element der DC oder essentiell set, dann abbruch */
      if ( dcIsTautology(pi, c) != 0 && c->n < 0 )
      {
        return 1;
      }
    }

    /* ueberpruefen, ob ueberhaupt noch partiell redundante terme vorhanden sind */
    /*
    for( i = 0; i < cnt; i++ )
    {
      if ( dclGet(cl, i)->n >= 0 )
        break;
    }
    if ( i == cnt )
      return 1;
    */
  }


  if ( dcIrredundantGetCofactorForSplit(pi, cofactor_left, cofactor_right, cl, cof) == 0 )
  {

    /* alle markierten elemente sind teil der loesung */
    if ( dclSCCInvAddAndSetFlag(pi_m, cl_m, mark) == 0 )
      return 0;
    return 1;
  }

  dcCopy(pi_m, mark_left, mark);
  dcCopy(pi_m, mark_right, mark);


  if ( dclInitCachedVA(pi, 2, &cl_left, &cl_right) == 0 )
    return 0;
    
  if (depth >= PINFO_STACK_CUBES)
    return 0;

  if ( dclIrredundantCofactor(pi, cl_left, cl, cofactor_left, mark_left) == 0 )
    return dclDestroyCachedVA(pi, 2, cl_left, cl_right), 0;

  if ( dclIrredundantCofactor(pi, cl_right, cl, cofactor_right, mark_right) == 0 )
    return dclDestroyCachedVA(pi, 2, cl_left, cl_right), 0;

  /*pinfoBTreeStart(pi);*/
  
  if ( dclIrredundantMarkTautCof(pi_m, cl_m, pi, cl_left, cofactor_left, mark_left, depth+1) == 0 )
    return /*pinfoBTreeEnd(pi),*/ dclDestroyCachedVA(pi, 2, cl_left, cl_right), 0;
  
  if ( dclIrredundantMarkTautCof(pi_m, cl_m, pi, cl_right, cofactor_right, mark_right, depth+1) == 0 )
    return /*pinfoBTreeEnd(pi),*/ dclDestroyCachedVA(pi, 2, cl_left, cl_right), 0;
  
  return /*pinfoBTreeEnd(pi),*/ dclDestroyCachedVA(pi, 2, cl_left, cl_right), 1;
}

int dclIrredundantMarkTaut(pinfo *pi_m, dclist cl_m, pinfo *pi, int n, dclist cl)
{
  int result;
  dcube *cof = &(pi->tmp[2]);
  dcube *mark = &(pi_m->tmp[2]);
  dcSetTautology(pi, cof);
  dcInSetAll(pi_m, mark, CUBE_IN_MASK_DC);
  dcOutSetAll(pi_m, mark, 0);
  dcSetOut(mark, n, 1);
  /* pinfoBTreeInit(pi, "TautologyMark"); */
  result = dclIrredundantMarkTautCof(pi_m, cl_m, pi, cl, cof, mark, 0);
  /* pinfoBTreeFinish(pi); */
  return result;
}

/*
  Es wird der cube dclGet(cl_pr, pos) betrachtet.
  Diese Routine markiert diejenige Elemente in cl_pr,
  die in ihrer Vereinung dclGet(cl_pr, pos) ueberdecken.
  cl_es und cl_dc koennen auch NULL sein.
*/

int dclIrredundantMark(pinfo *pi_m, dclist cl_m, pinfo *pi, dclist cl_pr, int pos, dclist cl_es, dclist cl_dc)
{
  dclist cl_u;
  int i, added_index;
  if ( dclInit(&cl_u) == 0 )
    return 0;
  /* Alle Elemente aus cl_pr, bis auf das Element 'pos' */
  for( i = 0; i < dclCnt(cl_pr); i++ )
  {
    if ( i != pos )
    {
      added_index = dclAdd(pi, cl_u, dclGet(cl_pr, i));
      if ( added_index < 0 )
        return dclDestroy(cl_u), 0;
      dclGet(cl_u, added_index)->n = i;
    }
  }
  /* Alle Elemente aus cl_es */
  if ( cl_es != NULL )
    for( i = 0; i < dclCnt(cl_es); i++ )
    {
      added_index = dclAdd(pi, cl_u, dclGet(cl_es, i));
      if ( added_index < 0 )
        return dclDestroy(cl_u), 0;
      dclGet(cl_u, added_index)->n = -1;
    }
  /* Alle Elemente aus cl_dc */
  if ( cl_dc != NULL )
    for( i = 0; i < dclCnt(cl_dc); i++ )
    {
      added_index = dclAdd(pi, cl_u, dclGet(cl_dc, i));
      if ( added_index < 0 )
        return dclDestroy(cl_u), 0;
      dclGet(cl_u, added_index)->n = -1;
    }

  /* Berechne den Kofaktor (hmm... koennte man auch gleich machen) */
  dclClearFlags(cl_u);
  for ( i = 0; i < dclCnt(cl_u); i++ )
    if ( dcCofactor(pi, dclGet(cl_u, i), dclGet(cl_u, i), dclGet(cl_pr, pos)) == 0 )
      dclSetFlag(cl_u, i);
  dclDeleteCubesWithFlag(pi, cl_u);


  /* es wird auf jedenfall ein element drangehaengt */
  /* Das problem ist, dass die liste nicht notwendigerweise */
  /* frei von essentiellen oder relativ essentiellen cubes ist */
  
  /*
  if ( dclCnt(cl_u) == 0 || dclTautology(pi, cl_u) == 0 )
  { 
    int p, t = 0;
    dcube *c;
    p = dclAddEmpty(pi_m, cl_m);
    if ( p < 0 )
      return 0;
    c = dclGet(cl_m, p);
    dcInSetAll(pi_m, c, CUBE_IN_MASK_DC);
    dcOutSetAll(pi_m, c, 0);
    dcSetOut(c, pos, 1);
  }
  else
  */
  {
    dclClearFlags(cl_pr);   /* obsolete? */
    dclSetFlag(cl_pr, pos); /* obsolete? */
    if ( dclIrredundantMarkTaut(pi_m, cl_m, pi, pos, cl_u) == 0 )
      return dclDestroy(cl_u), 0;
  }
  
  
  return dclDestroy(cl_u), 1;
}

/*-- dclIrredundantMatrix ---------------------------------------------------*/


/*-- dclIrredundantDCubeMatrix ----------------------------------------------*/

/* diese routine erfordert vorher ZWINGEND den aufruf von dclSplitRelativeEssential */
/* cl_pr darf keine relativ essentiellen cubes enthalten */
int dclIrredundantDCubeMatrix(pinfo *pi_m, dclist cl_m, pinfo *pi, dclist cl_es, dclist cl_pr, dclist cl_dc)
{
  int i, cnt = dclCnt(cl_pr);
  if ( dclSetPinfoByLength(pi_m, cl_pr) == 0 )
    return 0;
  dclClear(cl_m);
  dclClearFlags(cl_m);
  pinfoProcedureInit(pi, "IrredundantDCubeMatrix", cnt);
  for(i = 0; i < cnt; i++ )
  {
    if ( pinfoProcedureDo(pi, i) == 0 )
      return pinfoProcedureFinish(pi), 0;
    if ( dclIrredundantMark(pi_m, cl_m, pi, cl_pr, i, cl_es, cl_dc) == 0 )
      return pinfoProcedureFinish(pi), 0;
  }

  /*
  dclSCCInv(pi_m, cl_m);
  */
  dclDeleteCubesWithFlag(pi_m, cl_m);
  
  return pinfoProcedureFinish(pi), 1;
}

/*-- dclMarkRequiredCube ----------------------------------------------------*/
int dclMarkRequiredCube(pinfo *pi_m, dclist cl_m, pinfo *pi, dclist cl_es, dclist cl_pr, dclist cl_rc)
{
  int i, j;
  int o;
  dcube *mark = &(pi_m->tmp[2]);
  dclClearFlags(cl_m);
  for( i = 0; i < dclCnt(cl_rc); i++ )
  {
    /*
    printf("--> required cube '%s'.\n", dcToStr(pi, dclGet(cl_rc, i), " ", ""));
    */
    for( o = 0; o < pi->out_cnt; o++ ) 
    {
      for( j = 0; j < dclCnt(cl_es); j++ )
        if ( dcGetOut(dclGet(cl_es, j), o) != 0 )
          if ( dcIsInSubSet(pi, dclGet(cl_es, j), dclGet(cl_rc, i)) != 0 )
            break;
      /* cube is part of an essential cube, consider next output */
      if ( j < dclCnt(cl_es) )
      {
        /*
        printf("----> %o: required cube '%s' is part of essential set.\n", o, dcToStr(pi, dclGet(cl_rc, i), " ", ""));
        */
        continue;
      }
    
      dcInSetAll(pi_m, mark, CUBE_IN_MASK_DC);
      dcOutSetAll(pi_m, mark, 0);
      for( j = 0; j < dclCnt(cl_pr); j++ )
      {
        if ( dcGetOut(dclGet(cl_pr, j), o) != 0 )
        {
          if ( dcIsInSubSet(pi, dclGet(cl_pr, j), dclGet(cl_rc, i)) != 0 )
          {
            dcSetOut(mark, j, 1);
            /*
            printf("----> %o: required cube '%s' is subset of '%s'.\n", o, dcToStr(pi, dclGet(cl_rc, i), " ", ""), dcToStr2(pi, dclGet(cl_pr, j), " ", ""));
            */
          }
        }
      }
      if ( dcIsIllegal(pi_m, mark) == 0 )
      {
        /*
        if ( o == 1 )
          printf("--> %o: required cube '%s' added.\n", o, dcToStr(pi, dclGet(cl_rc, i), " ", ""));
        */
        if ( dclSCCInvAddAndSetFlag(pi_m, cl_m, mark) == 0 )
          return 0;
      }
      else
      {
        /*
        printf("----> %o: required cube '%s' NOT added.\n", o, dcToStr(pi, dclGet(cl_rc, i), " ", ""));
        */
      }
      /*
      if ( dcIsIllegal(pi_m, mark) == 0 )
        if ( dclAdd(pi_m, cl_m, mark) < 0 )
          return 0;
      */
    }
  }
  /* dclSCCInv(pi_m, cl_m); */
  dclDeleteCubesWithFlag(pi_m, cl_m);
  return 1;
}

int xxxdclMarkRequiredCube(pinfo *pi_m, dclist cl_m, pinfo *pi, dclist cl_es, dclist cl_pr, dclist cl_rc)
{
  int i, j;
  dcube *mark = &(pi_m->tmp[2]);
  dclClearFlags(cl_m);
  for( i = 0; i < dclCnt(cl_rc); i++ )
  {
      for( j = 0; j < dclCnt(cl_es); j++ )
        if ( dcIsSubSet(pi, dclGet(cl_es, j), dclGet(cl_rc, i)) != 0 )
          break;
      /* cube is part of an essential cube, consider next output */
      if ( j < dclCnt(cl_es) )
      {
        printf("--> required cube '%s' is part of essential set.\n", dcToStr(pi, dclGet(cl_rc, i), " ", ""));
        continue;
      }
    
      dcInSetAll(pi_m, mark, CUBE_IN_MASK_DC);
      dcOutSetAll(pi_m, mark, 0);
      for( j = 0; j < dclCnt(cl_pr); j++ )
      {
        if ( dcIsSubSet(pi, dclGet(cl_pr, j), dclGet(cl_rc, i)) != 0 )
        {
            dcSetOut(mark, j, 1);
            printf("--> required cube '%s' is subset of '%s'.\n", dcToStr(pi, dclGet(cl_rc, i), " ", ""), dcToStr2(pi, dclGet(cl_pr, j), " ", ""));
        }
      }
      if ( dcIsIllegal(pi_m, mark) == 0 )
      {
        printf("--> required cube '%s' added.\n", dcToStr(pi, dclGet(cl_rc, i), " ", ""));
        if ( dclSCCInvAddAndSetFlag(pi_m, cl_m, mark) == 0 )
          return 0;
      }
      else
      {
        printf("--> required cube '%s' NOT added.\n", dcToStr(pi, dclGet(cl_rc, i), " ", ""));
      }
  }
  dclDeleteCubesWithFlag(pi_m, cl_m);
  return 1;
}

/*-- dclIrredundantDCubeTMatrix ---------------------------------------------*/

/* diese routine erfordert vorher ZWINGEND den aufruf von dclSplitRelativeEssential */
/* cl_pr darf keine relativ essentiellen cubes enthalten */
int dclIrredundantDCubeTMatrix(pinfo *pi_m, dclist cl_m, pinfo *pi, dclist cl_es, dclist cl_pr, dclist cl_dc)
{
  pinfo local_pi;
  dclist local_cl;
  if ( pinfoInit(&local_pi) == 0 )
    return 0;
  if (pi->progress != NULL)
    pinfoInitProgress(&local_pi);  
  if ( dclInit(&local_cl) == 0 )
    return pinfoDestroy(&local_pi), 0;
  if ( dclIrredundantDCubeMatrix(&local_pi, local_cl, pi, cl_es, cl_pr, cl_dc) == 0 )
    return dclDestroy(local_cl), pinfoDestroy(&local_pi), 0;
  if ( dclInvertOutMatrix(pi_m, cl_m, &local_pi, local_cl) == 0 )
    return dclDestroy(local_cl), pinfoDestroy(&local_pi), 0;
/*
 *   puts("-- 111 ---");
 *   dclShow(&local_pi, local_cl);
 *   puts("-- 222 ---");
 *   dclShow(pi_m, cl_m);
 */
  return dclDestroy(local_cl), pinfoDestroy(&local_pi), 1;
}

int dclIrredundantDCubeTMatrixRC(pinfo *pi_m, dclist cl_m, pinfo *pi, dclist cl_es, dclist cl_pr, dclist cl_dc, dclist cl_rc)
{
  pinfo local_pi;
  dclist local_cl;
  if ( pinfoInit(&local_pi) == 0 )
    return 0;
  if (pi->progress != NULL)
    pinfoInitProgress(&local_pi);  
  if ( dclInit(&local_cl) == 0 )
    return pinfoDestroy(&local_pi), 0;
  if ( dclIrredundantDCubeMatrix(&local_pi, local_cl, pi, cl_es, cl_pr, cl_dc) == 0 )
    return dclDestroy(local_cl), pinfoDestroy(&local_pi), 0;

  if ( cl_rc != NULL )
    if ( dclMarkRequiredCube(&local_pi, local_cl, pi, cl_es, cl_pr, cl_rc) == 0 )
      return dclDestroy(local_cl), pinfoDestroy(&local_pi), 0;
  

  if ( dclInvertOutMatrix(pi_m, cl_m, &local_pi, local_cl) == 0 )
    return dclDestroy(local_cl), pinfoDestroy(&local_pi), 0;
/*
 *   puts("-- 111 ---");
 *   dclShow(&local_pi, local_cl);
 *   puts("-- 222 ---");
 *   dclShow(pi_m, cl_m);
 */
  return dclDestroy(local_cl), pinfoDestroy(&local_pi), 1;
}

    


/*-- dclReduceDCubeTMatrix --------------------------------------------------*/
/* diese routine entfernt nochmals einige spalten */
/* zu diesen zweck wird auch cl_pr angepasst und  */
/* die entsprechenden cubes aus cl_pr entfernt.   */
/* streng genommen muessten diese cubes in die    */
/* fully redundant liste aufgenommen werden.      */
int dclReduceDCubeTMatrix(pinfo *pi_m, dclist cl_m, pinfo *pi_pr, dclist cl_pr)
{
  int i, cnt = dclCnt(cl_m);
  int j;
  
  if ( dclCnt(cl_m) != dclCnt(cl_pr) )
    return 0;
  
  /* clear empty lines */
  
  if ( dclClearFlags(cl_m) == 0 )
    return 0;
  if ( dclClearFlags(cl_pr) == 0 )
    return 0;
  for( i = 0; i < cnt; i++ )
  {
    if ( dcOutCnt(pi_m, dclGet(cl_m, i)) == 0 )
    {
      dclSetFlag(cl_m, i);
      dclSetFlag(cl_pr, i);
    }
  }
  dclDeleteCubesWithFlag(pi_m, cl_m);
  dclDeleteCubesWithFlag(pi_pr, cl_pr);

  /* clear elements, which are subset and are more expensive */

  if ( dclClearFlags(cl_m) == 0 )
    return 0;
  if ( dclClearFlags(cl_pr) == 0 )
    return 0;
  
  cnt = dclCnt(cl_pr);
  for( i = 0; i < cnt; i++ )
    dclGet(cl_pr, i)->n = pi_pr->in_cnt-dcInDCCnt(pi_pr, dclGet(cl_pr, i));
  
  pinfoProcedureInit(pi_pr, "ReduceDCubeTMatrix", cnt);
  for( i = 0; i < cnt; i++ )
  {
    pinfoProcedureDo(pi_pr, i);
    for( j = i+1; j < cnt; j++ )
      if ( dcIsSubSet(pi_m, dclGet(cl_m, i), dclGet(cl_m, j)) != 0 )
        if ( dclGet(cl_pr, i)->n <= dclGet(cl_pr, j)->n )
        {
          dclSetFlag(cl_m, j);
          dclSetFlag(cl_pr, j);
        }
  }
  pinfoProcedureFinish(pi_pr);
  dclDeleteCubesWithFlag(pi_m, cl_m);
  dclDeleteCubesWithFlag(pi_pr, cl_pr);
  
  return 1;
}

/*-- dclIrredundant ---------------------------------------------------------*/

int dclIrredundant(pinfo *pi, dclist cl, dclist cl_dc)
{
  dclist cl_es, cl_pr;
  if ( dclInitVA(2, &cl_es, &cl_pr) == 0 )
    return 0;
  if ( dclSplitRelativeEssential(pi, cl_es, NULL, cl_pr, cl, cl_dc, NULL) == 0 )
    return dclDestroyVA(2, cl_es, cl_pr), 0;
  if ( dclIrredundantGreedy(pi, cl_es, cl_pr, cl_dc, NULL) == 0 )
    return dclDestroyVA(2, cl_es, cl_pr), 0;
  if ( dclCopy(pi, cl, cl_es) == 0 )
    return dclDestroyVA(2, cl_es, cl_pr), 0;
  if ( dclJoin(pi, cl, cl_pr) == 0 )
    return dclDestroyVA(2, cl_es, cl_pr), 0;
  return dclDestroyVA(2, cl_es, cl_pr), 1;
}

/*-- dclRestrictOutput ------------------------------------------------------*/

/* Setzt Ausgaenge, die ueberfluessigerweise auf 1 gesetzt sind auf 0. */
/* Ueblicherweise wird diese Funktion aufgerufen nachdem eine minimale */
/* loesung gefunden wurde. */

void dclRestrictOutput(pinfo *pi, dclist cl)
{
  int i, j, cnt = dclCnt(cl);
  int out;
  for( out = 0; out < pi->out_cnt; out++ )
  {
    for( i = 0; i < cnt; i++ )
    {
      if ( dcGetOut(dclGet(cl, i), out) != 0 )
      {
        for( j = i+1; j < cnt; j++ )
        {
          if ( dcGetOut(dclGet(cl, j), out) != 0 )
          {
            if ( dcIsInSubSet(pi, dclGet(cl, i), dclGet(cl, j)) != 0 )
            {
              dcSetOut(dclGet(cl, j), out, 0);
            }
            else if ( dcIsInSubSet(pi, dclGet(cl, j), dclGet(cl, i)) != 0 )
            {
              dcSetOut(dclGet(cl, i), out, 0);
              break;
            }
          }
        }
      }
    }
  }
}

/*-- dclMinimize ------------------------------------------------------------*/

int dclMinimize(pinfo *pi, dclist cl)
{
  dclist cl_es, cl_fr, cl_pr;
  dclInitVA(3, &cl_es, &cl_fr, &cl_pr);
  
  if ( dclPrimes(pi, cl) == 0 )
    return dclDestroyVA(3, cl_es, cl_fr, cl_pr), 0;
  
  if ( dclSplitRelativeEssential(pi, cl_es, cl_fr, cl_pr, cl, NULL, NULL) == 0 )
    return dclDestroyVA(3, cl_es, cl_fr, cl_pr), 0;
  
  if ( dclIrredundantGreedy(pi, cl_es, cl_pr, NULL, NULL) == 0 )
    return dclDestroyVA(3, cl_es, cl_fr, cl_pr), 0;
  
  if ( dclJoin(pi, cl_pr, cl_es) == 0 )
    return dclDestroyVA(3, cl_es, cl_fr, cl_pr), 0;
    
  dclRestrictOutput(pi, cl_pr);
  
  if( dclIsEquivalent(pi, cl, cl_pr) == 0 )
    return dclDestroyVA(3, cl_es, cl_fr, cl_pr), 0;
    
  if ( dclCopy(pi, cl, cl_pr) == 0 )
    return dclDestroyVA(3, cl_es, cl_fr, cl_pr), 0;   
    
  dclDestroyVA(3, cl_es, cl_fr, cl_pr);
  return 1;

}

/*-- dclMinimizeDC ----------------------------------------------------------*/

/* result is stored in cl */
/* inputs are cl and cl_dc */
/* cl and cl_dc should be disjunct! */
/* dclSubtract(pi, cl, cl_dc) or dclSubtract(pi, cl_dc, cl) would force this */

/*!
  \ingroup dclist
  
  The representation of a boolean function contains three parts:
  -# The problem info structure.
  -# The ON-set of the boolean function (\a cl).
  -# An optional DC (don't care) Set of the boolean function (\a cl_dc).

  This function tries to reduce the number of cubes that are stored in \a cl.
  One often assumes that the area of a hardware implementation of boolean 
  functions decreases with the number of cubes. So it is often
  a good idea to call this function before hardware synthesis (gnc_SynthDCL()).
  
  
  \pre \a cl and \a cl_dc must be disjunct, they must have 
  an empty intersection. This condition can be forced with
  dclSubtract(pi, cl, cl_dc) or dclSubtract(pi, cl_dc, cl).
  
  \post \a cl and \a cl_dc are not disjunct any more. After a successful
  minimization, \a cl and \a cl_dc may have a nonempty intersection.

  \param pi The problem info structure for the boolean functions.
  \param cl The ON-set of the boolean function. This argument will be modified
    by dclMinimizeDC().
  \param cl_dc The DC-set of the boolean function. Set this argument to \c NULL if there
    is not DC-set.
  \param greedy Use the recommended value 0 for an exact minimzation.

  \return 0, if an error occured.
  
  \warning This function may take a very long time.
  
  \see gnc_SynthDCL()
  \see dclImport()
  
*/
int dclMinimizeDC(pinfo *pi, dclist cl, dclist cl_dc, int greedy, int is_literal)
{
  dclist cl_es, cl_fr, cl_pr, cl_on;
  dclInitVA(4, &cl_es, &cl_fr, &cl_pr, &cl_on);
  
  if ( dclCopy(pi, cl_on, cl) == 0 )
    return dclDestroyVA(4, cl_es, cl_fr, cl_pr, cl_on), 0;   
  
  if ( dclPrimesDC(pi, cl, cl_dc) == 0 )
    return dclDestroyVA(4, cl_es, cl_fr, cl_pr, cl_on), 0;
  
  if ( dclSplitRelativeEssential(pi, cl_es, cl_fr, cl_pr, cl, cl_dc, NULL) == 0 )
    return dclDestroyVA(4, cl_es, cl_fr, cl_pr, cl_on), 0;
  
/* ---- dclIrredundantGreedy has been disabled for the benefit of (the following function) maMatrixIrredundant:

  if ( dclIrredundantGreedy(pi, cl_es, cl_pr, cl_dc, NULL) == 0 )
    return dclDestroyVA(4, cl_es, cl_fr, cl_pr, cl_on), 0;                   
*/

  if ( is_literal != 0 )
  {
    if ( maMatrixIrredundant(pi, cl_es, cl_pr, cl_dc, NULL, greedy, MA_LIT_SOP) == 0 )
      return dclDestroyVA(4, cl_es, cl_fr, cl_pr, cl_on), 0;
  }
  else
  {
    if ( maMatrixIrredundant(pi, cl_es, cl_pr, cl_dc, NULL, greedy, MA_LIT_NONE) == 0 )
      return dclDestroyVA(4, cl_es, cl_fr, cl_pr, cl_on), 0;
  }

  if ( dclJoin(pi, cl_pr, cl_es) == 0 )
    return dclDestroyVA(4, cl_es, cl_fr, cl_pr, cl_on), 0;
    
  dclRestrictOutput(pi, cl_pr);
  
  if( dclIsEquivalentDC(pi, cl_pr, cl_on, cl_dc) == 0 )
    return dclDestroyVA(4, cl_es, cl_fr, cl_pr, cl_on), 0;
    
  if ( dclCopy(pi, cl, cl_pr) == 0 )
    return dclDestroyVA(4, cl_es, cl_fr, cl_pr, cl_on), 0;   
    
  dclDestroyVA(4, cl_es, cl_fr, cl_pr, cl_on);
  return 1;

}

/*-- dclWriteBin ------------------------------------------------------------*/

int dclWriteBin(pinfo *pi, dclist cl, FILE *fp)
{
  int i;

  if ( cl == NULL )
  {
    if ( b_io_WriteInt(fp, 0) == 0 )
      return 0;
    return 1;
  }
    
  if ( b_io_WriteInt(fp, 1) == 0 )
    return 0;

  if ( b_io_WriteInt(fp, cl->cnt) == 0 )
    return 0;

  for( i = 0; i < cl->cnt; i++ )
    if ( dcWriteBin(pi, cl->list+i, fp) == 0 )
      return 0;
  
  return 1;
}


/*-- dclReadBin -------------------------------------------------------------*/

int dclReadBin(pinfo *pi, dclist *cl, FILE *fp)
{
  int i;

  if ( *cl != NULL )
    dclDestroy(*cl);
  *cl = NULL;

  if ( b_io_ReadInt(fp, &(i)) == 0 )
    return 0;
  if ( i == 0 )
    return 1;

  if ( dclInit(cl) == 0 )
  {
    *cl = NULL;
    return 0;
  }
  
  if ( b_io_ReadInt(fp, &((*cl)->cnt)) == 0 )
    return 0;
  if ( dclExpandTo(pi, *cl, (*cl)->cnt) == 0 )
    return 0;
  
  for( i = 0; i < (*cl)->cnt; i++ )
    if ( dcReadBin(pi, (*cl)->list+i, fp) == 0 )
      return 0;
  
  return 1;
}

/*-- dclGetLiteralCnt -------------------------------------------------------*/

int dclGetLiteralCnt(pinfo *pi, dclist cl)
{
  int i, cnt = dclCnt(cl);
  int lit_cnt = 0;
  
  for( i = 0; i < cnt; i++ )
  {
    /*
    lit_cnt += pi->in_cnt - dcInDCCnt(pi, dclGet(cl, i));
    lit_cnt += dcOutCnt(pi, dclGet(cl, i));
    */
    lit_cnt += dcGetLiteralCnt(pi, dclGet(cl, i));
  }
  return lit_cnt;
}

