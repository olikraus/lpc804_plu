/*

  b_set.h

  A set with pointer values

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
#include <assert.h>
#include "b_set.h"
#include "b_io.h"
#include "mwc.h"

#define B_SET_EXPAND 16

b_set_type b_set_Open()
{
  b_set_type bs;
  int i;
  bs = malloc(sizeof(b_set_struct));
  if ( bs != NULL )
  {
    bs->list_ptr = (void **)malloc(B_SET_EXPAND*sizeof(void *));
    if ( bs->list_ptr != NULL )
    {
      for( i = 0; i < B_SET_EXPAND; i++ )
        bs->list_ptr[i] = NULL;
      bs->list_max = B_SET_EXPAND;
      bs->list_cnt = 0;
      bs->search_pos_start = 0;
      return bs;
    }
    free(bs);
  }
  return NULL;
}

void b_set_Clear(b_set_type bs)
{
  int i;
  for( i = 0; i < bs->list_max; i++  )
    bs->list_ptr[i] = NULL;
  bs->list_cnt = 0;
}

void b_set_Close(b_set_type bs)
{
  b_set_Clear(bs);
  assert ( bs->list_ptr != NULL );
  free(bs->list_ptr);
  free(bs);
}

int b_set_IsValid(b_set_type bs, int pos)
{
  if ( pos < 0 )
    return 0;
  if ( pos >= bs->list_max )
    return 0;
  if ( bs->list_ptr[pos] == NULL )
    return 0;
  return 1;
}

static int b_set_expand(b_set_type bs)
{
  void *ptr;
  int pos;
  assert(bs != NULL);
  assert(bs->list_ptr != NULL);
  assert((bs->list_max % B_SET_EXPAND) == 0);
  
  ptr = (void *)realloc(bs->list_ptr, sizeof(void *)*(bs->list_max + B_SET_EXPAND));
  if ( ptr == NULL )
    return 0;
  
  bs->list_ptr = ptr;
  pos = bs->list_max;
  bs->search_pos_start = bs->list_max;
  bs->list_max += B_SET_EXPAND;
  while( pos < bs->list_max )
  {
    bs->list_ptr[pos] = NULL;
    pos++;
  }
  return 1;
}


static int b_set_find_empty(b_set_type bs)
{
  int i;
  for( i = bs->search_pos_start; i < bs->list_max; i++ )
  {
    if ( bs->list_ptr[i] == NULL )
    {
      bs->search_pos_start = i + 1;
      if ( bs->search_pos_start >= bs->list_max )
        bs->search_pos_start = 0;
      return i;
    }
  }
  assert(bs->search_pos_start < bs->list_max);
  for( i = 0; i < bs->search_pos_start; i++ )
  {
    if ( bs->list_ptr[i] == NULL )
    {
      bs->search_pos_start = i + 1;
      if ( bs->search_pos_start >= bs->list_max )
        bs->search_pos_start = 0;
      return i;
    }
  }
  return -1;
}

/* returns access handle or -1 */
int b_set_Add(b_set_type bs, void *ptr)
{
  int pos;

  assert(bs != NULL);
  assert(bs->list_ptr != NULL);
  assert(ptr != NULL);
  assert(bs->list_max >= bs->list_cnt);
  
  pos = b_set_find_empty(bs);
  if ( pos < 0 )
  {
    if ( b_set_expand(bs) == 0 )
      return -1;
    pos = b_set_find_empty(bs);
    if ( pos < 0 )
      return -1;
  }
  
  bs->list_ptr[pos] = ptr;
  bs->list_cnt++;
  return pos;
}

int b_set_Set(b_set_type bs, int pos, void *ptr)
{
  while(bs->list_max <= pos)
    if ( b_set_expand(bs) == 0 )
      return 0;
  assert(pos >= 0);
  assert(pos < bs->list_max);
  if ( bs->list_ptr[pos] == NULL && ptr != NULL )
    bs->list_cnt++;
  if ( bs->list_ptr[pos] != NULL && ptr == NULL )
    bs->list_cnt--;
  bs->list_ptr[pos] = ptr;
  return 1;
}

/* delete a pointer by its access handle */
void b_set_Del(b_set_type bs, int pos)
{
  assert(pos >= 0);
  assert(pos < bs->list_max);
  assert(bs->list_ptr[pos] != NULL);
  assert(bs->list_cnt > 0 );
  bs->list_ptr[pos] = NULL;
  bs->list_cnt--;
}

int b_set_Next(b_set_type bs, int pos)
{
  for(;;)
  {
    pos++;
    if ( pos >= bs->list_max )
      return -1;
    if ( bs->list_ptr[pos] != NULL )
      return pos;
  }
}

int b_set_First(b_set_type bs)
{
  return b_set_Next(bs, -1);
}

/* *pos must be initilized with -1 */
int _b_set_WhileLoop(b_set_type bs, int *pos)
{
  *pos = b_set_Next(bs, *pos);
  if ( *pos < 0 )
    return 0;
  return 1;
}



/* *pos must be initilized with -1 */
int b_set_WhileLoop(b_set_type bs, int *pos_ptr)
{
  int pos = *pos_ptr;
  
  for(;;)
  {
    pos++;
    if ( pos >= bs->list_max )
    {
      *pos_ptr = -1;
      return 0;
    }
    if ( bs->list_ptr[pos] != NULL )
    {
      *pos_ptr = pos;
      return 1;
    }
  }
}

int b_set_Prev(b_set_type bs, int pos)
{
  for(;;)
  {
    pos--;
    if ( pos < 0 )
      return -1;
    if ( bs->list_ptr[pos] != NULL )
      return pos;
  }
}

int b_set_InvWhileLoop(b_set_type bs, int *pos)
{
  if ( *pos < 0 )
    *pos = bs->list_max;
  *pos = b_set_Prev(bs, *pos);
  if ( *pos < 0 )
    return 0;
  return 1;
}

/*----------------------------------------------------------------------------*/

#define B_SET_CHECK 0x0b005345

int b_set_Write(b_set_type bs, FILE *fp, int (*write_el)(FILE *fp, void *el, void *ud), void *ud)
{
  int i, j, cnt;
  if ( b_io_WriteInt(fp, B_SET_CHECK) == 0 )
    return 0;
  if ( b_io_WriteInt(fp, bs->list_max) == 0 )
    return 0;
    
  cnt = bs->list_max;
  while( cnt > 0 )
  {
    if ( bs->list_ptr[cnt-1] != NULL )
      break;
    cnt--;
  }
  if ( b_io_WriteInt(fp, cnt) == 0 )
    return 0;
    
  i = -1;
  j = 0;
  for( i = 0; i < cnt; i++ )
  {
    if ( bs->list_ptr[i] != NULL )
    {
      if ( b_io_WriteInt(fp, -1) == 0 )
        return 0;
      if ( write_el(fp, bs->list_ptr[i], ud) == 0 )
        return 0;
      j++;
    }
    else
    {
      if ( b_io_WriteInt(fp, 0) == 0 )
        return 0;
    }
  }
  assert(j == bs->list_cnt);
  return 1;
}

int b_set_Read(b_set_type bs, FILE *fp, void *(*read_el)(FILE *fp, void *ud), void *ud)
{
  int i, chk, cnt;
  assert(bs->list_cnt == 0);
  if ( b_io_ReadInt(fp, &chk) == 0 )
    return 0;
  if ( chk != B_SET_CHECK )
    return 0;
  if ( b_io_ReadInt(fp, &(bs->list_max)) == 0 )
    return 0;
  if ( b_io_ReadInt(fp, &(cnt)) == 0 )
    return 0;
  if ( bs->list_ptr != NULL )
    free(bs->list_ptr);
  bs->list_ptr = (void **)malloc(sizeof(void *)*bs->list_max);
  if ( bs->list_ptr == NULL )
    return bs->list_cnt = 0, bs->list_max = 0, 0;
  for( i = 0; i < bs->list_max; i++ )
    bs->list_ptr[i] = NULL;

  bs->list_cnt = 0;    
  for( i = 0; i < cnt; i++ )
  {
    if ( b_io_ReadInt(fp, &chk) == 0 )
      return 0;
    if ( chk != 0 )
    {
      bs->list_ptr[i] = read_el(fp, ud);
      if ( bs->list_ptr[i] == NULL )
        return 0;
      bs->list_cnt++;
    }
    else
    {
      bs->list_ptr[i] = NULL;
    }
  }
  
  return 1;
}

int b_set_ReadMerge(b_set_type bs, FILE *fp, void *(*read_el)(FILE *fp, void *ud), void *ud)
{
  int i, chk, cnt, pos;
  void *ptr;
  assert(bs->list_cnt == 0);
  if ( b_io_ReadInt(fp, &chk) == 0 )
    return 0;
  if ( chk != B_SET_CHECK )
    return 0;

  /* just discard the max value */
  if ( b_io_ReadInt(fp, &(cnt)) == 0 )
    return 0;

  if ( b_io_ReadInt(fp, &(cnt)) == 0 )
    return 0;

  for( i = 0; i < cnt; i++ )
  {
    if ( b_io_ReadInt(fp, &chk) == 0 )
      return 0;
    if ( chk != 0 )
    {
      pos = b_set_Add(bs, &bs);   /* add some dummy data */
      if ( pos < 0 )
        return 0;
      ptr = read_el(fp, ud);
      if ( ptr == NULL )
        return b_set_Del(bs, pos), 0;
      bs->list_ptr[pos] = ptr;    /* replace dummy data */
    }
  }
  
  return 1;
}

/*----------------------------------------------------------------------------*/

size_t b_set_GetMemUsage(b_set_type bs)
{
  size_t m = sizeof(b_set_struct);
  m += sizeof(void *)*bs->list_max;
  return m;
}

size_t b_set_GetAllMemUsage(b_set_type bs, size_t (*fn)(void *))
{
  int i = -1;
  size_t m = b_set_GetMemUsage(bs);
  while( b_set_WhileLoop(bs, &i) != 0 )
    m += fn(bs->list_ptr[i]);
  return m;
}
