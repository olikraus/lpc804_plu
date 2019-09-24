/*

  b_pl.h

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

#ifndef _B_PL_H
#define _B_PL_H

#include <stddef.h>
#include <stdio.h>

struct _b_pl_struct
{
  int cnt;
  int max;
  void **list;
  
  /* this allows to close b_pl structure more than once */
  /* this is used by the pg_obj modul */
  int link_cnt;  
};
typedef struct _b_pl_struct b_pl_struct;
typedef struct _b_pl_struct *b_pl_type;

int b_pl_init(b_pl_type b_pl);
int b_pl_expand(b_pl_type b_pl);
void b_pl_destroy(b_pl_type b_pl);

#define b_pl_GetCnt(b_pl)         ((b_pl)->cnt)
#define b_pl_GetVal(b_pl,pos)     ((b_pl)->list[pos])
#define b_pl_SetVal(b_pl,pos,val) ((b_pl)->list[pos] = (val))
#define b_pl_Clear(b_pl)          ((b_pl)->cnt = 0)


b_pl_type b_pl_Open();
b_pl_type b_pl_Link(b_pl_type b_pl);
void b_pl_Close(b_pl_type b_pl);
/* returns position or -1 */
int b_pl_Add(b_pl_type b_pl, void *ptr);
void b_pl_DelByPos(b_pl_type b_pl, int pos);
int b_pl_InsByPos(b_pl_type b_pl, void *ptr, int pos);

int b_pl_Write(b_pl_type b_pl, FILE *fp, int (*write_el)(FILE *fp, void *el, void *ud), void *ud);
int b_pl_Read(b_pl_type b_pl, FILE *fp, void *(*read_el)(FILE *fp, void *ud), void *ud);

void b_pl_Sort(b_pl_type b_pl, int (*compar)(const void *, const void *));

size_t b_pl_GetMemUsage(b_pl_type b_pl);

#endif

