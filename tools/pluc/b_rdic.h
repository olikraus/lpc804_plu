/*

  b_rdic.h

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

#ifndef _B_RDIC_H
#define _B_RDIC_H

#include "b_il.h"

struct _b_rdic_struct
{
  b_il_type il;
  int (*cmp_fn)(void *data, int el, const void *key);
  void *data;
};
typedef struct _b_rdic_struct b_rdic_struct;
typedef struct _b_rdic_struct *b_rdic_type;

#define b_rdic_GetCnt(dic) b_il_GetCnt((dic)->il)
#define b_rdic_GetVal(dic, pos) b_il_GetVal((dic)->il, (pos))

b_rdic_type b_rdic_Open();
void b_rdic_Close(b_rdic_type b_rdic);
void b_rdic_Clear(b_rdic_type b_rdic);
void b_rdic_SetCmpFn(b_rdic_type b_rdic, int (*cmp_fn)(void *data, int el, const void *key), void *data);
int b_rdic_FindInsPos(b_rdic_type b_rdic, const void *key);
int b_rdic_FindPos(b_rdic_type b_rdic, const void *key);
/* 0: error */
int b_rdic_Ins(b_rdic_type b_rdic, int val, const void *key);
int b_rdic_Find(b_rdic_type b_rdic, const void *key);
void b_rdic_DelByPos(b_rdic_type b_rdic, int pos);
void b_rdic_Del(b_rdic_type b_rdic, const void *key);
size_t b_rdic_GetMemUsage(b_rdic_type b_rdic);

int b_rdic_GetLast(b_rdic_type b_rdic);
void b_rdic_DelLast(b_rdic_type b_rdic);
int b_rdic_ExtractLast(b_rdic_type b_rdic);


#endif /* _B_RDIC_H */
