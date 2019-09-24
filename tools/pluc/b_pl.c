/* 

  b_pl.c
  
  basic pointer list

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

#include "b_pl.h"
#include "b_io.h"

#include <stdlib.h>
#include <assert.h>
#include "mwc.h"

#define B_PL_EXPAND 32  
  
int b_pl_init(b_pl_type b_pl)
{
  b_pl->cnt = 0;
  b_pl->max = 0;
  b_pl->link_cnt = 0;
  b_pl->list = (void **)malloc(B_PL_EXPAND*sizeof(void *));
  if ( b_pl->list == NULL )
    return 0;
  b_pl->max = B_PL_EXPAND;
  return 1;
}

int b_pl_expand(b_pl_type b_pl)
{
  void *ptr;
  ptr = realloc(b_pl->list, (b_pl->max+B_PL_EXPAND)*sizeof(void *));
  if ( ptr == NULL )
    return 0;
  b_pl->list = (void **)ptr;
  b_pl->max += B_PL_EXPAND;
  return 1;
}

void b_pl_destroy(b_pl_type b_pl)
{
  b_pl_Clear(b_pl);
  free(b_pl->list);
  b_pl->max = 0;
}

/*----------------------------------------------------------------------------*/

b_pl_type b_pl_Open()
{
  b_pl_type b_pl;
  b_pl = (b_pl_type)malloc(sizeof(b_pl_struct));
  if ( b_pl != NULL )
  {
    if ( b_pl_init(b_pl) != 0 )
    {
      return b_pl;
    }
    free(b_pl);
  }
  return NULL;
}

b_pl_type b_pl_Link(b_pl_type b_pl)
{
  b_pl->link_cnt++;
  return b_pl;
}

void b_pl_Close(b_pl_type b_pl)
{
  if ( b_pl->link_cnt > 0 )
  {
    b_pl->link_cnt--;
    return;
  }
  b_pl_destroy(b_pl);
  free(b_pl);
}

/* returns position or -1 */
int b_pl_Add(b_pl_type b_pl, void *ptr)
{
  while( b_pl->cnt >= b_pl->max )
    if ( b_pl_expand(b_pl) == 0 )
      return -1;
  
  b_pl->list[b_pl->cnt] = ptr;
  b_pl->cnt++;
  return b_pl->cnt-1;
}

void b_pl_DelByPos(b_pl_type b_pl, int pos)
{
  while( pos+1 < b_pl->cnt )
  {
    b_pl->list[pos] = b_pl->list[pos+1];
    pos++;
  }
  if ( b_pl->cnt > 0 )
    b_pl->cnt--;
}

/* 0: error */
int b_pl_InsByPos(b_pl_type b_pl, void *ptr, int pos)
{
  int i;
  if ( b_pl_Add(b_pl, ptr) < 0 )
    return 0;
  assert(b_pl->cnt > 0 );
  i = b_pl->cnt-1;
  while( i > pos + 4 )
  {
    b_pl->list[i-0] = b_pl->list[i-1];
    b_pl->list[i-1] = b_pl->list[i-2];
    b_pl->list[i-2] = b_pl->list[i-3];
    b_pl->list[i-3] = b_pl->list[i-4];
    i-=4;
  }
  while( i > pos )
  {
    b_pl->list[i] = b_pl->list[i-1];
    i--;
  }
  b_pl->list[i] = ptr;
  return 1;
}

/*----------------------------------------------------------------------------*/

#define B_PL_CHECK 0x0b00504c

int b_pl_Write(b_pl_type b_pl, FILE *fp, int (*write_el)(FILE *fp, void *el, void *ud), void *ud)
{
  int i;
  if ( b_io_WriteInt(fp, B_PL_CHECK) == 0 )
    return 0;
  if ( b_io_WriteInt(fp, b_pl->cnt) == 0 )
    return 0;
  for( i = 0; i < b_pl->cnt; i++ )
    if ( write_el(fp, b_pl->list[i], ud) == 0 )
      return 0;
  return 1;
}

int b_pl_Read(b_pl_type b_pl, FILE *fp, void *(*read_el)(FILE *fp, void *ud), void *ud)
{
  int i, cnt, chk;
  void *ptr;
  assert( b_pl->cnt == 0 );
  if ( b_io_ReadInt(fp, &chk) == 0 )
    return 0;
  if ( chk != B_PL_CHECK )
    return 0;
  if ( b_io_ReadInt(fp, &cnt) == 0 )
    return 0;
  for( i = 0; i < cnt; i++ )
  {
    ptr = read_el(fp, ud);
    if ( ptr == NULL )
      return 0;
    if ( b_pl_Add(b_pl, ptr) < 0 )
      return 0;
  }
  return 1;
}

/*----------------------------------------------------------------------------*/

void b_pl_Sort(b_pl_type b_pl, int (*compar)(const void *, const void *))
{
  qsort(b_pl->list, b_pl->cnt, sizeof(void *), compar);
}

/*----------------------------------------------------------------------------*/

size_t b_pl_GetMemUsage(b_pl_type b_pl)
{
  size_t m;
  m = sizeof(b_pl_struct);
  m += sizeof(void *)*b_pl->max;
  return m;
}
