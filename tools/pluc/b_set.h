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

#ifndef _B_SET_H
#define _B_SET_H

#include <stddef.h>
#include <stdio.h>

struct _b_set_struct
{
  void **list_ptr;
  int list_max;
  int list_cnt;

  int search_pos_start;
};
typedef struct _b_set_struct b_set_struct;
typedef struct _b_set_struct *b_set_type;

#define b_set_Get(bs, pos) ((bs)->list_ptr[(pos)])
#define b_set_Cnt(bs) ((bs)->list_cnt)
#define b_set_Max(bs) ((bs)->list_max)

b_set_type b_set_Open();
void b_set_Clear(b_set_type bs);
void b_set_Close(b_set_type bs);
int b_set_IsValid(b_set_type bs, int pos);
/* returns access handle or -1 */
int b_set_Add(b_set_type bs, void *ptr);
int b_set_Set(b_set_type bs, int pos, void *ptr);
void b_set_Del(b_set_type bs, int pos);
int b_set_Next(b_set_type bs, int pos);
int b_set_First(b_set_type bs);
int b_set_WhileLoop(b_set_type bs, int *pos);
int b_set_InvWhileLoop(b_set_type bs, int *pos);

int b_set_Write(b_set_type bs, FILE *fp, int (*write_el)(FILE *fp, void *el, void *ud), void *ud);
int b_set_Read(b_set_type bs, FILE *fp, void *(*read_el)(FILE *fp, void *ud), void *ud);
int b_set_ReadMerge(b_set_type bs, FILE *fp, void *(*read_el)(FILE *fp, void *ud), void *ud);

size_t b_set_GetMemUsage(b_set_type bs);
size_t b_set_GetAllMemUsage(b_set_type bs, size_t (*fn)(void *));

#endif
