/*

  b_rdic.c

  reference dictionary

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
#include "b_rdic.h"
#include "mwc.h"

int b_rdic_dummy_fn(void *data, int el, const void *key)
{
  return 0;
}

b_rdic_type b_rdic_Open()
{
  b_rdic_type b_rdic;
  b_rdic = malloc(sizeof(b_rdic_struct));
  if ( b_rdic != NULL )
  {
    b_rdic->cmp_fn = b_rdic_dummy_fn;
    b_rdic->il = b_il_Open();
    if ( b_rdic->il != NULL )
    {
      return b_rdic;
    }
    free(b_rdic);
  }
  return NULL;
}

void b_rdic_Close(b_rdic_type b_rdic)
{
  b_il_Close(b_rdic->il);
  b_rdic->il = NULL;
  free(b_rdic);
}

void b_rdic_Clear(b_rdic_type b_rdic)
{
  b_il_Clear(b_rdic->il);
}

void b_rdic_SetCmpFn(b_rdic_type b_rdic, int (*cmp_fn)(void *data, int el, const void *key), void *data)
{
  b_rdic->cmp_fn = cmp_fn;
  b_rdic->data = data;
}

int b_rdic_FindInsPos(b_rdic_type b_rdic, const void *key)
{
  int l,u,m;
  int result;

  if ( b_rdic_GetCnt(b_rdic) == 0 )
    return 0;

  l = 0;
  u = b_rdic_GetCnt(b_rdic);
  while(l < u)
  {
    m = (size_t)((l+u)/2);
    result = b_rdic->cmp_fn(b_rdic->data, b_rdic_GetVal(b_rdic, m), key);
    if ( result < 0 )
       l = m+1;
    else if ( result > 0 )
       u = m;
    else
       return m;
  }
  return u;
}

int b_rdic_FindPos(b_rdic_type b_rdic, const void *key)
{
  int pos;
  
  pos = b_rdic_FindInsPos(b_rdic, key);
  if ( pos >= b_rdic_GetCnt(b_rdic) )
    return -1;
  if ( b_rdic->cmp_fn(b_rdic->data, b_rdic_GetVal(b_rdic, pos), key) != 0 )
    return -1;
  for(;;)
  {
    if ( pos == 0 )
      break;
    if ( b_rdic->cmp_fn(b_rdic->data, b_rdic_GetVal(b_rdic, pos-1), key) != 0 )
      break;
    pos--;
  }
  return pos;
}

/* 0: error */
int b_rdic_Ins(b_rdic_type b_rdic, int val, const void *key)
{
  int ins_pos;
  ins_pos = b_rdic_FindInsPos(b_rdic, key);
  return b_il_InsByPos(b_rdic->il, ins_pos, val);
}

/* returns val or -1 */
int b_rdic_Find(b_rdic_type b_rdic, const void *key)
{
  int pos;
  pos = b_rdic_FindPos(b_rdic, key);
  if ( pos < 0 )
    return -1;
  return b_rdic_GetVal(b_rdic, pos);
}

void b_rdic_DelByPos(b_rdic_type b_rdic, int pos)
{
  if ( pos < 0 )
    return;
  b_il_DelByPos(b_rdic->il, pos);
}

void b_rdic_Del(b_rdic_type b_rdic, const void *key)
{
  b_rdic_DelByPos(b_rdic, b_rdic_FindPos(b_rdic, key));
}

size_t b_rdic_GetMemUsage(b_rdic_type b_rdic)
{
  size_t m = sizeof(b_rdic_struct);
  m += b_il_GetMemUsage(b_rdic->il);
  return m;
}

int b_rdic_GetLast(b_rdic_type b_rdic)
{
  if ( b_il_GetCnt(b_rdic->il) == 0 )
    return -1;
  return b_il_GetLastVal(b_rdic->il);
}

void b_rdic_DelLast(b_rdic_type b_rdic)
{
  if ( b_il_GetCnt(b_rdic->il) == 0 )
    return;
  b_rdic_DelByPos(b_rdic, b_il_GetCnt(b_rdic->il)-1);
}

int b_rdic_ExtractLast(b_rdic_type b_rdic)
{
  int val;
  if ( b_il_GetCnt(b_rdic->il) == 0 )
    return -1;
  val = b_rdic_GetLast(b_rdic);
  b_rdic_DelLast(b_rdic);
  return val;
}

