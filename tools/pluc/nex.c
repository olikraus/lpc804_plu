/*

  nex.c
  
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
  
  a nex file as a header section and one ore more eval sections
  
  1. port declaration
  input a(a b c d) b(e f g h)
  output y(x y z)
  2. eval declaration
  onset a(start end) b(start end) ....
  
  dcset a(start end) b(start end)
  

*/

#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include "dcube.h"
#include "b_sl.h"
#include "b_set.h"
#include "b_eval.h"
#include "b_ff.h"

struct _nexv_struct
{
  char *name;
  b_sl_type sl;
  int is_input;
  int start;
  int end;
  int curr;
};
typedef struct _nexv_struct *nexv_type;


#define NEX_LINE_LEN 1024
struct _nex_struct
{
  b_eval_type eval;
  b_set_type vars;
  b_sl_type in_sl;
  b_sl_type out_sl;
  char *buf_ptr;
  size_t buf_len;
  size_t buf_max;
  int state;
  pinfo *pi;      /* only a pointer, not allocated */
  dclist cl_on;   /* only a pointer, not allocated */
  dclist cl_dc;   /* only a pointer, not allocated */
  dclist cl;      /* only a pointer, not allocated */
  char line[NEX_LINE_LEN];
};
typedef struct _nex_struct *nex_type;

nexv_type nexv_Open()
{
  nexv_type v;
  v = (nexv_type)malloc(sizeof(struct _nexv_struct));
  if ( v != NULL )
  {
    v->is_input = 0;
    v->name = NULL;
    v->start = 0;
    v->end = 0;
    v->curr = 0;
    v->sl = b_sl_Open();
    if ( v->sl != NULL )
    {
      return v;
    }
    free(v);
  }
  return NULL;
}

void nexv_Close(nexv_type v)
{
  b_sl_Close(v->sl);
  free(v->name);
  free(v);
}

static void nex_skip_space(const char **s, int *i)
{
  while( **s == ' ' || **s == '\t' )
  {
    if ( i != NULL )
      (*i)++;
    (*s)++;
  }
}

static const char *nex_identifier(const char **s, int *i)
{
  int j;
  static char id[256];
  nex_skip_space(s, i);
  j = 0;
  for(;;)
  {
    if ( (**s >= '0' && **s <= '9') || (**s >= 'a' && **s <= 'z')
      || (**s >= 'A' && **s <= 'Z') || (**s == '_') )
    {
      id[j] = **s;
      j++;
      if ( i != NULL )
        (*i)++;
      (*s)++;
    }
    else
      break;
  }
  id[j] = '\0';
  nex_skip_space(s, i);
  return id;
}

static int nex_val(const char **s)
{
  int x = 0;
  for(;;)
  {
    if ( **s < '0' || **s > '9' )
      break;
    x *= 10;
    x += **s - '0';
    (*s)++;
  }
  nex_skip_space(s, NULL);
  return x;
}

/* name(str1 str2 str3 ) */
int nexv_Set(nexv_type v, const char **s, int is_input)
{
  int j;
  const char *id;
  id = nex_identifier(s, NULL);
  if ( **s != '(' )
    return 0;
  (*s)++;
  if ( v->name != NULL )
    free(v->name);
  v->name = strdup(id);
  if ( v->name == NULL )
    return 0;
  b_sl_Clear(v->sl);
  j = b_sl_ImportByStr(v->sl, *s, " \t", ")");
  if ( j < 0 )
    return 0;
  (*s) += j;
  nex_skip_space(s, NULL);
  if ( **s != ')' )
    return 0;
  (*s)++;
  nex_skip_space(s, NULL);
  v->is_input = is_input;
  return 1;
}

nexv_type nex_GetV(nex_type nex, int pos)
{
  return (nexv_type)b_set_Get(nex->vars, pos);
}

nex_type nex_Open()
{
  nex_type nex;
  nex = (nex_type)malloc(sizeof(struct _nex_struct));
  if ( nex != NULL )
  {
    nex->pi = NULL;
    nex->cl = NULL;
    nex->cl_on = NULL;
    nex->cl_dc = NULL;
    nex->buf_ptr = NULL;
    nex->buf_len = 0;
    nex->buf_max = 0;
    nex->vars = b_set_Open();
    if ( nex->vars != NULL )
    {
      nex->eval = b_eval_Open();
      if ( nex->eval != NULL )
      {
        nex->in_sl = b_sl_Open();
        if ( nex->in_sl != NULL )
        {
          nex->out_sl = b_sl_Open();
          if ( nex->out_sl != NULL )
          {
            return nex;
          }
          b_sl_Close(nex->in_sl);
        }
        b_eval_Close(nex->eval);
      }
      b_set_Close(nex->vars);
    }
    free(nex);
  }
  return NULL;
}

void nex_Close(nex_type nex)
{
  int i = -1;
  if ( nex->buf_ptr != NULL )
    free(nex->buf_ptr);
  while( b_set_WhileLoop(nex->vars, &i) != 0 )
    nexv_Close(nex_GetV(nex, i));
  b_sl_Close(nex->in_sl);
  b_sl_Close(nex->out_sl);
  b_set_Close(nex->vars);
  b_eval_Close(nex->eval);
  free(nex);
}

nexv_type nex_FindV(nex_type nex, const char *s)
{
  int i = -1;
  while( b_set_WhileLoop(nex->vars, &i) != 0 )
    if ( strcmp(nex_GetV(nex, i)->name, s) == 0 )
      return nex_GetV(nex, i);
  return NULL;
}

int nex_AddV(nex_type nex, const char **s, int is_input)
{
  int i;
  nexv_type v;
  v = nexv_Open();
  if ( v != NULL )
  {
    if ( nexv_Set(v, s, is_input) != 0 )
    {
      i = b_set_Add(nex->vars, v);
      if ( i >= 0 )
      {
        return i;
      }
    }
    nexv_Close(v);
  }
  return -1;
}

void nex_Error(nex_type nex, const char *fmt, ...)
{
  va_list va;
  va_start(va, fmt);
  vprintf(fmt, va);
  puts("");
  va_end(va);
}

int nex_SetRange(nex_type nex, const char **s)
{
  const char *id = nex_identifier(s, NULL);
  nexv_type nexv = nex_FindV(nex, id);
  if ( nexv == NULL )
  {
    nex_Error(nex, "NEX Error: Variable '%s' not found.", id);
    return 0;
  }
  if ( nexv->is_input == 0 )
  {
    nex_Error(nex, "NEX Error: Variable '%s' is not a input variable.", id);
    return 0;
  }
  if ( **s != '(' )
  {
    nex_Error(nex, "NEX Error: Expected '(' after variable '%s'.", id);
    return 0;
  }
  (*s)++;
  nex_skip_space(s, NULL);
  if ( **s < '0' || **s > '9' )
  {
    nex_Error(nex, "NEX Error: Expected start value.");
    return 0;
  }
  nexv->start = nex_val(s);
  if ( **s < '0' || **s > '9' )
  {
    nex_Error(nex, "NEX Error: Expected end value.");
    return 0;
  }
  nexv->end = nex_val(s);
  if ( **s != ')' )
  {
    nex_Error(nex, "NEX Error: Expected ')'.", id);
    return 0;
  }
  (*s)++;
  nex_skip_space(s, NULL);
  return 1;
}

int nex_BuildLabelLists(nex_type nex)
{
  int i = -1;
  int j;
  nexv_type v;
  b_sl_type sl;
  b_sl_Clear(nex->in_sl);
  b_sl_Clear(nex->out_sl);
  while( b_set_WhileLoop(nex->vars, &i) != 0 )
  {
    v = nex_GetV(nex, i);
    
    if ( v->is_input )
      sl = nex->in_sl;
    else
      sl = nex->out_sl;

    for( j = 0; j < b_sl_GetCnt(v->sl); j++ )
    {
      if ( b_sl_Find(sl, b_sl_GetVal(v->sl, j)) >= 0 )
      {
        nex_Error(nex, "NEX Error: Pin '%s' of variable '%s' already used.", 
          b_sl_GetVal(v->sl, j), v->name);
        return 0;
      }
      if ( b_sl_Add(sl, b_sl_GetVal(v->sl, j)) < 0 )
        return 0;
    }
  }
  return 1;
}

int nex_SetProblemInfo(nex_type nex)
{
  if ( nex->pi == NULL )
    return 1;
  
  if ( nex_BuildLabelLists(nex) == 0 )
    return 0;

  if ( pinfoSetInCnt(nex->pi, b_sl_GetCnt(nex->in_sl)) == 0 )
  {
    nex_Error(nex, "NEX Error: Out of memory (pinfoSetInCnt).");
    return 0;
  }

  if ( pinfoSetOutCnt(nex->pi, b_sl_GetCnt(nex->out_sl)) == 0 )
  {
    nex_Error(nex, "NEX Error: Out of memory (pinfoSetOutCnt)."); 
    return 0;
  }

  if ( pinfoCopyInLabels(nex->pi, nex->in_sl) == 0 )
  {
    nex_Error(nex, "NEX Error: Out of memory (pinfoCopyInLabels).");
    return 0;
  }

  if ( pinfoCopyOutLabels(nex->pi, nex->out_sl) == 0 )
  {
    nex_Error(nex, "NEX Error: Out of memory (pinfoCopyOutLabels).");
    return 0;
  }
  
  return 1;
}

int nex_SetAllRange(nex_type nex, const char **s)
{
  int i = -1;
  nexv_type v;
  while( b_set_WhileLoop(nex->vars, &i) != 0 )
  {
    v = nex_GetV(nex, i);
    v->start = 0;
    v->end = (1<<b_sl_GetCnt(v->sl))-1;
  }
  
  for(;;)
  {
    if ( **s == '\0' || **s == '\n' )
      break;
    if ( nex_SetRange(nex, s) == 0 ) 
      return 0;
  }
  return 1;
}


void nex_ClrBuf(nex_type nex)
{
  if ( nex->buf_ptr == NULL )
  {
    nex->buf_len = 0;
    nex->buf_max = 0;
    return;
  }
  nex->buf_ptr[0] = '\0';
  nex->buf_len = 0;
}


int nex_AddBuf(nex_type nex, const char *s)
{
  if ( nex->buf_ptr == NULL )
  {
    nex->buf_ptr = strdup(s);
    if ( nex->buf_ptr == NULL )
      return 0;
    nex->buf_len = strlen(s);
    nex->buf_max = strlen(s);
    return 1;
  }
  if ( nex->buf_len+strlen(s) > nex->buf_max )
  {
    void *p;
    p = realloc(nex->buf_ptr, nex->buf_len + strlen(s) + 1);
    if ( p == NULL )
      return 0;
    nex->buf_ptr = p;
    nex->buf_max = nex->buf_len + strlen(s);
    strcat(nex->buf_ptr, s);
    nex->buf_len += strlen(s);
    return 1;
  }
  strcat(nex->buf_ptr, s);
  nex->buf_len += strlen(s);
  return 1;
}

#define NEX_STATE_HEADER 0
#define NEX_STATE_ONSET 1
#define NEX_STATE_DCSET 2

int nex_ClrCurrVal(nex_type nex)
{
  int i = -1;
  nexv_type v;
  while( b_set_WhileLoop(nex->vars, &i) != 0 )
  {
    v = nex_GetV(nex, i);
    v->curr = v->start;
    if ( b_eval_SetValByName(nex->eval, v->name, v->curr) == 0 )
    {
      nex_Error(nex, "NEX Error: Variable '%s' not found inside math expression.", v->name);
      return 0;
    }
  }
  return 1;
}

int nex_IncCurrVal(nex_type nex)
{
  int i = -1;
  nexv_type v;
  while( b_set_WhileLoop(nex->vars, &i) != 0 )
  {
    v = nex_GetV(nex, i);
    if ( v->is_input != 0 )
    {
      if ( v->curr < v->end )
      {
        v->curr++;
        if ( b_eval_SetValByName(nex->eval, v->name, v->curr) == 0 )
        {
          nex_Error(nex, "NEX Error: Variable '%s' not found inside math expression.", v->name);
          return 0;
        }
        return 1;
      }
      v->curr = v->start;
      if ( b_eval_SetValByName(nex->eval, v->name, v->curr) == 0 )
      {
        nex_Error(nex, "NEX Error: Variable '%s' not found inside math expression.", v->name);
        return 0;
      }
    }
  }
  return 0;
}

void nex_ShowCurrVal(nex_type nex)
{
  int i = -1;
  nexv_type v;
  while( b_set_WhileLoop(nex->vars, &i) != 0 )
  {
    v = nex_GetV(nex, i);
    printf("%s: %d ", v->name, b_eval_GetValByName(nex->eval, v->name));
  }
  printf("\n");
}

int nex_AddCube(nex_type nex)
{
  int i = -1;
  int j;
  nexv_type v;
  dcube *c = &(nex->pi->tmp[11]);
  int v_val;
  int v_bit;

  dcInSetAll(nex->pi, c, CUBE_IN_MASK_DC);
  dcOutSetAll(nex->pi, c, 0);
  
  while( b_set_WhileLoop(nex->vars, &i) != 0 )
  {
    v = nex_GetV(nex, i);
    v_val = b_eval_GetValByName(nex->eval, v->name);
    for( j = 0; j < b_sl_GetCnt(v->sl); j++ )
    {
      v_bit = (v_val>>(b_sl_GetCnt(v->sl)-1-j))&1;
      if ( v->is_input != 0 )
      {
        int pos = pinfoFindInLabelPos(nex->pi, b_sl_GetVal(v->sl, j));
        if ( pos < 0 )
          nex_Error(nex, "NEX Error: Port '%s' not found.", 
            b_sl_GetVal(v->sl, j));
        dcSetIn(c, pos, v_bit+1);
      }
      else
      {
        int pos = pinfoFindOutLabelPos(nex->pi, b_sl_GetVal(v->sl, j));
        if ( pos < 0 )
          nex_Error(nex, "NEX Error: Port '%s' not found.", 
            b_sl_GetVal(v->sl, j));
        dcSetOut(c, pos, v_bit);
      }
    }
  }
  
  if ( dcIsIllegal(nex->pi, c) == 0 )
  {
    if ( dclAdd(nex->pi, nex->cl, c) < 0 )
      return 0;
  }
  
  return 1;
}

int nex_ApplySet(nex_type nex)
{
  b_en_type en;
  
  if ( nex->buf_ptr == NULL )
    return 1;

  if ( nex->pi == NULL )
    return 1;
    
  en = b_eval_ParseStr(nex->eval, nex->buf_ptr);
  if ( en == NULL )
  {
    nex_Error(nex, "NEX Error: Syntex error (math expression).");
    return 1;
  }

  if ( nex->state != NEX_STATE_DCSET && nex->state != NEX_STATE_ONSET )
    return 1;

  if ( nex_ClrCurrVal(nex) == 0 )
    return 0;

  dclClear(nex->cl);

  do
  {  
    b_eval_Eval(nex->eval, en); /* returns the value of the expression */
    if ( nex_AddCube(nex) == 0 )
      return b_en_Close(en), 0;
  } while( nex_IncCurrVal(nex) != 0 );
  
  b_en_Close(en);
  
  if ( nex->state == NEX_STATE_DCSET )
  {
    if ( dclSubtract(nex->pi, nex->cl_dc, nex->cl) == 0 )
      return nex_Error(nex, "NEX Error: Out of memory (dclSubtract)."), 0;
    if ( dclSubtract(nex->pi, nex->cl_on, nex->cl) == 0 )
      return nex_Error(nex, "NEX Error: Out of memory (dclSubtract)."), 0;
    if ( dclSCCUnion(nex->pi, nex->cl_dc, nex->cl) == 0 )
      return nex_Error(nex, "NEX Error: Out of memory (dclSCCUnion)."), 0;
  }
  else if ( nex->state == NEX_STATE_ONSET )
  {
    if ( dclSubtract(nex->pi, nex->cl_dc, nex->cl) == 0 )
      return nex_Error(nex, "NEX Error: Out of memory (dclSubtract)."), 0;
    if ( dclSubtract(nex->pi, nex->cl_on, nex->cl) == 0 )
      return nex_Error(nex, "NEX Error: Out of memory (dclSubtract)."), 0;
    if ( dclSCCUnion(nex->pi, nex->cl_on, nex->cl) == 0 )
      return nex_Error(nex, "NEX Error: Out of memory (dclSCCUnion)."), 0;
  }
  
  nex_ClrBuf(nex);
  return 1;
}

int nex_Line(nex_type nex, const char *s)
{
  if ( s == NULL )
    return 0;
  if ( *s != '\0' )
    while( *s == ' ' || *s == '\t' )
      s++;

  if ( *s == '\0' )
  {
    return 1;
  }
  else if ( *s == '#' )
  {
    return 1;
  }
  else if ( *s == '\n' )
  {
    return 1;
  }
  else if ( nex->state == NEX_STATE_HEADER && strncmp(s, "input", 5) == 0 )
  {
    s+=5;
    for(;;)
    {
      while( *s == ' ' || *s == '\t' )
        s++;
      if ( *s == '\0' || *s == '\n' )
        break;
      if ( nex_AddV(nex, &s, 1) < 0 )
        return 0;
    }
    return 1;
  }
  else if ( nex->state == NEX_STATE_HEADER && strncmp(s, "output", 6) == 0 )
  {
    s+=6;
    for(;;)
    {
      while( *s == ' ' || *s == '\t' )
        s++;
      if ( *s == '\0' || *s == '\n' )
        break;
      if ( nex_AddV(nex, &s, 0) < 0 )
        return 0;
    }
    return 1;
  }
  else if ( strncmp(s, "onset", 5) == 0 )
  {
    s+=5;
    while( *s == ' ' || *s == '\t' )
      s++;
    if ( nex->state == NEX_STATE_HEADER )
      if ( nex_SetProblemInfo(nex) == 0 )
        return 0;
    nex_ApplySet(nex);
    nex_SetAllRange(nex, &s);
    nex->state = NEX_STATE_ONSET;
    return 1;
  }
  else if ( strncmp(s, "dcset", 5) == 0 )
  {
    s+=5;
    while( *s == ' ' || *s == '\t' )
      s++;
    if ( nex->state == NEX_STATE_HEADER )
      if ( nex_SetProblemInfo(nex) == 0 )
        return 0;
    nex_ApplySet(nex);
    nex_SetAllRange(nex, &s);
    nex->state = NEX_STATE_DCSET;
    return 1;
  }
  else if ( nex->state == NEX_STATE_ONSET || nex->state == NEX_STATE_DCSET )
  {
    return nex_AddBuf(nex, s);
  }
  if ( nex->pi != NULL )
    nex_Error(nex, "NEX Error: Illegal command '%s'.", s);
  
  return 0;
}

int nex_ReadFP(nex_type nex, FILE *fp)
{
  const char *s;
  nex->state = NEX_STATE_HEADER;
  nex_ClrBuf(nex);
  for(;;)
  {
    s = fgets(nex->line, NEX_LINE_LEN, fp);
    if ( s == NULL )
    {
      if ( nex->state == NEX_STATE_ONSET || nex->state == NEX_STATE_DCSET )
        return nex_ApplySet(nex);
    }
    if ( nex_Line(nex, s) == 0 )
      return 0;
  }
  return 1;
}

int nex_ReadFile(nex_type nex, const char *name)
{
  FILE *fp;
  fp = b_fopen(name, NULL, ".bex", "r");
  if ( fp == NULL )
    return 0;
  if ( nex_ReadFP(nex, fp) == 0 )
    return fclose(fp), 0;
  return fclose(fp), 1;
}

int dclReadNEX(pinfo *pi, dclist cl_on, dclist cl_dc, const char *filename)
{
  int ret;
  nex_type nex = nex_Open();
  if ( nex == NULL )
    return 0;
  if ( dclInit(&(nex->cl)) == 0 )
    return nex_Close(nex), 0;
  nex->pi = pi;
  nex->cl_on = cl_on;
  nex->cl_dc = cl_dc;
  ret = nex_ReadFile(nex, filename);
  dclDestroy(nex->cl);
  nex_Close(nex);
  return ret;
}

int IsValidNEXFile(const char *filename)
{
  int ret;
  nex_type nex = nex_Open();
  if ( nex == NULL )
    return 0;
  ret = nex_ReadFile(nex, filename);
  nex_Close(nex);
  return ret;
}

