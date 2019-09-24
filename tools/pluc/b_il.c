/*

  b_il.c

  basic integer list 

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


#include "b_il.h"
#include "b_io.h"
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <assert.h>
#include "mwc.h"

#define B_IL_EXPAND 32

int b_il_init(b_il_type b_il)
{
  b_il->cnt = 0;
  b_il->max = 0;
  b_il->list = (int *)malloc(B_IL_EXPAND*sizeof(int));
  if ( b_il->list == NULL )
    return 0;
  b_il->max = B_IL_EXPAND;
  return 1;
}

int b_il_expand(b_il_type b_il)
{
  void *ptr;
  ptr = realloc(b_il->list, (b_il->max+B_IL_EXPAND)*sizeof(int));
  if ( ptr == NULL )
    return 0;
  b_il->list = (int *)ptr;
  b_il->max += B_IL_EXPAND;
  return 1;
}

int b_il_set_max(b_il_type b_il, int max)
{
  void *ptr;

  if ( b_il->max >= max )
    return 1;
  
  max = (max+(B_IL_EXPAND-1))/B_IL_EXPAND;
  max *= B_IL_EXPAND;
  
  if ( b_il->max >= max )
    return 1;
  
  ptr = realloc(b_il->list, max*sizeof(int));
  if ( ptr == NULL )
    return 0;
  b_il->list = (int *)ptr;
  b_il->max = max;
  return 1;
}

void b_il_destroy(b_il_type b_il)
{
  b_il_Clear(b_il);
  free(b_il->list);
  b_il->max = 0;
}

/*----------------------------------------------------------------------------*/

b_il_type b_il_Open()
{
  b_il_type b_il;
  b_il = (b_il_type)malloc(sizeof(b_il_struct));
  if ( b_il != NULL )
  {
    if ( b_il_init(b_il) != 0 )
    {
      return b_il;
    }
    free(b_il);
  }
  return NULL;
}

b_il_type b_il_OpenVA(int cnt, ...)
{
  b_il_type il;
  il = b_il_Open();
  if ( il != NULL )
  {
    va_list va;
    int val;
    va_start(va, cnt);
    while( cnt > 0 )
    {
      val = va_arg(va, int);
      if ( b_il_Add(il, val) < 0 )
      {
        va_end(va);
        b_il_Close(il);
        return NULL;
      }
      cnt--;
    }
    va_end(va);
    return il;
  }
  return NULL;
}

int b_il_OpenMultiple(int cnt, ...)
{
  b_il_type *ilp;
  va_list va;
  int i;

  va_start(va, cnt);
  for( i = 0; i < cnt; i++ )
    *(va_arg(va, b_il_type *)) = NULL;
  va_end(va);

  va_start(va, cnt);
  for( i = 0; i < cnt; i++ )
  {
    ilp = va_arg(va, b_il_type *);
    *ilp = b_il_Open();
    if ( *ilp == NULL )
      break;
  }
  va_end(va);
  
  if ( i >= cnt )
    return 1;

  va_start(va, cnt);
  for( i = 0; i < cnt; i++ )
  {
    ilp = va_arg(va, b_il_type *);
    if ( *ilp != NULL )
      b_il_Close(*ilp);
    *ilp = NULL;
  }
  va_end(va);

  return 0;
}

void b_il_Close(b_il_type il)
{
  b_il_destroy(il);
  free(il);
}

void b_il_CloseMultiple(int cnt, ...)
{
  va_list va;
  int i;
  va_start(va, cnt);
  for( i = 0; i < cnt; i++ )
    b_il_Close(va_arg(va, b_il_type));
  va_end(va);
}

/* returns position or -1 */
int b_il_Find(b_il_type il, int val)
{
  int i;
  for( i = 0; i < il->cnt; i++ )
    if ( il->list[i] == val )
      return i;
  return -1;
}

/* 0 not equal, 1 equal */
int b_il_CmpValues(b_il_type il, int *values, int cnt)
{
  int i;
  if ( cnt != il->cnt )
    return 0;
  for( i = 0; i < cnt; i++ )
    if ( b_il_Find(il, values[i]) < 0 )
      return 0;
  return 1;
}

/* 0 error */
int b_il_SetValues(b_il_type il, int *values, int cnt)
{
  if ( b_il_CmpValues(il, values, cnt) != 0 )
    return 1;
  while( cnt > il->max )
    if ( b_il_expand(il) == 0 )
      return 0;
  il->cnt = cnt;
  memcpy(il->list, values, sizeof(int)*cnt);
  /* a_obj_ValueChange(il, -1); */
  return 1;
}


/* returns list with integers, must be free´d */
/* NULL: error */
int *b_il_GetValues(b_il_type il, int *cnt)
{
  int *l;
  l = (int *)malloc(sizeof(int)*il->cnt);
  if ( l == NULL )
    return NULL;
  memcpy(l, il->list, sizeof(int)*il->cnt);
  if ( cnt != NULL )
    *cnt = il->cnt;
  return l;
}

/* 0: error */
int b_il_Copy(b_il_type dest, b_il_type src)
{
  return b_il_SetValues(dest, src->list, src->cnt);
}

/* returns 0 or 1 */
int b_il_CopyRange(b_il_type dest, b_il_type src, int start, int cnt)
{
  int i;
  b_il_Clear(dest);
  if ( start > b_il_GetCnt(src) )
    return 0;
  if ( cnt > b_il_GetCnt(src) - start )
    cnt = b_il_GetCnt(src) - start;
  for( i = 0; i < cnt; i++ )
    if ( b_il_Add(dest, b_il_GetVal(src, i+start)) < 0 )
      return 0;
  return 1;
}

/* 0 not equal, 1 equal */
int b_il_Cmp(b_il_type dest, b_il_type src)
{
  return b_il_CmpValues(dest, src->list, src->cnt);
}

/* returns position or -1 */
int b_il_Add(b_il_type il, int val)
{
  while( il->cnt >= il->max )
    if ( b_il_expand(il) == 0 )
      return -1;
  
  il->list[il->cnt] = val;
  il->cnt++;
  /* a_obj_ValueChange(il, -1); */
  return il->cnt-1;
}

int b_il_AddVA(b_il_type il, int cnt, ...)
{
  va_list va;
  int val;
  va_start(va, cnt);
  while( cnt > 0 )
  {
    val = va_arg(va, int);
    if ( b_il_Add(il, val) < 0 )
    {
      va_end(va);
      return 0;
    }
    cnt--;
  }
  va_end(va);
  return 1;
}

int b_il_SetVal(b_il_type il, int pos, int val)
{
  if ( b_il_set_max(il, pos+1) == 0 )
    return 0;
  assert(pos < il->max);
  while(il->cnt < pos)
  {
    il->list[il->cnt] = 0;
    il->cnt++;
  }
  if(il->cnt <= pos)
    il->cnt = pos+1;
  il->list[pos] = val;
  assert(pos < il->cnt);
  return 1;
}

void b_il_Swap(b_il_type il, int p1, int p2)
{
  int v;
  assert(p1 >= 0 && p1 < il->cnt);
  assert(p2 >= 0 && p2 < il->cnt);
  v = il->list[p1];
  il->list[p1] = il->list[p2];
  il->list[p2] = v;
}

int b_il_AddUnique(b_il_type il, int val)
{
  int pos;
  pos = b_il_Find(il, val);
  if ( pos >= 0 )
    return pos;
  return b_il_Add(il, val);
}

void b_il_DelRange(b_il_type il, int pos, int cnt)
{
  while( pos+cnt < il->cnt )
  {
    il->list[pos] = il->list[pos+cnt];
    pos++;
  }
  if ( il->cnt > 0 )
  {
    il->cnt -= cnt;
    if ( il->cnt < 0 )
      il->cnt = 0;
  }
}

void b_il_DelByPos(b_il_type il, int pos)
{
  while( pos+1 < il->cnt )
  {
    il->list[pos] = il->list[pos+1];
    pos++;
  }
  if ( il->cnt > 0 )
  {
    il->cnt--;
    /* a_obj_ValueChange(il, -1); */
  }
}

void b_il_DelByVal(b_il_type il, int val)
{
  int pos;
  for(;;)
  {
    pos = b_il_Find(il, val);
    if ( pos < 0 )
      break;
    b_il_DelByPos(il, pos);
  }
}

/* 0: error */
int b_il_InsByPos(b_il_type il, int pos, int val)
{
  int i;
  if ( b_il_Add(il, val) < 0 )
    return 0;
  i = il->cnt-1;
  if ( pos >= i )
    return 1;
  while( i > pos )
  {
    il->list[i] = il->list[i-1];
    i--;
  }
  il->list[pos] = val;
  return 1;
}

static int b_il_int_compare(const void *i, const void *j)
{
  if ( *(const int *)i > *(const int *)j )
    return (1);
  if ( *(const int *)i < *(const int *)j )
    return (-1);
  return (0);
}

void b_il_Sort(b_il_type il)
{
  qsort(il->list, il->cnt,  sizeof(int), b_il_int_compare);
}

int b_il_IsElEq(b_il_type a, b_il_type b)
{
  int i;
  if ( a->cnt != b->cnt )
    return 0;
  for( i = 0; i < a->cnt; i++ )
    if ( a->list[i] != b->list[i] )
      return 0;
  return 1;  
}

int b_il_CmpEl(b_il_type a, b_il_type b)
{
  int i;
  int cnt;
  
  if ( a->cnt < b->cnt )
    cnt = a->cnt;
  else
    cnt = b->cnt;
    
  for( i = 0; i < cnt; i++ )
  {
    if ( a->list[i] < b->list[i] )
      return -1;
    if ( a->list[i] > b->list[i] )
      return 1;
  } 
  
  if ( a->cnt < b->cnt )
    return -1;
  if ( a->cnt > b->cnt )
    return 1;
    
  return 0;
}

/* is b subset of a */
int b_il_IsSubSetEl(b_il_type a, b_il_type b)
{
  int i, j;
  i = 0;
  j = 0;
  for(;;)
  {
  
    if ( j == b->cnt )
      return 1;
      
    if ( i == a->cnt )
    {
      if ( j == b->cnt )
        return 1;
      break;
    }
    
    if ( a->list[i] == b->list[j] )
    {
      i++;
      j++;
    }
    else
    {
      i++;
    }
  }
  return 0;
}

/*----------------------------------------------------------------------------*/

#define B_IL_CHECK 0x0b00494c

int b_il_Write(b_il_type il, FILE *fp)
{
  int i;
  if ( b_io_WriteInt(fp, B_IL_CHECK) == 0 )
    return 0;
  if ( b_io_WriteInt(fp, il->cnt) == 0 )
    return 0;
  for( i = 0; i < il->cnt; i++ )
    if ( b_io_WriteInt(fp, il->list[i]) == 0 )
      return 0;
  return 1;
}

int b_il_Read(b_il_type il, FILE *fp)
{
  int i, cnt, val;
  if ( b_io_ReadInt(fp, &val) == 0 )
    return 0;
  if ( val != B_IL_CHECK )
    return 0;
  if ( b_io_ReadInt(fp, &cnt) == 0 )
    return 0;
  b_il_Clear(il);
  for( i = 0; i < cnt; i++ )
  {
    if ( b_io_ReadInt(fp, &val) == 0 )
      return 0;
    if ( b_il_Add(il, val) < 0 )
      return 0;
  }
  return 1;
}

int b_il_WhileLoop(b_il_type il, int *pos)
{
  if ( il->cnt == 0 )
  {
    *pos = -1;
    return 0;
  }
  else if ( *pos < 0 )
  {
    *pos = 0;
    return 1;
  }
  else if ( *pos+1 >= il->cnt )
  {
    *pos = -1;
    return 0;
  }
  (*pos)++;
  return 1;
}

int b_il_Loop(b_il_type il, int *pos, int *val)
{
  if ( b_il_WhileLoop(il, pos) == 0 )
    return 0;
  *val = il->list[*pos];
  return 1;
}

int b_il_IsInvLoop(b_il_type il, int *pos, int *val, int is_inv)
{
  if ( b_il_WhileLoop(il, pos) == 0 )
    return 0;
  if ( is_inv == 0 )
    *val = il->list[*pos];
  else
    *val = il->list[il->cnt-1-(*pos)];
  return 1;
}


/*----------------------------------------------------------------------------*/

size_t b_il_GetMemUsage(b_il_type il)
{
  size_t m = sizeof(b_il_struct);
  m += sizeof(int)*il->max;
  return m;
}



