/*

  b_sl.h
  
  string list

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

#ifndef _B_SL_H
#define _B_SL_H

#include "b_pl.h"
#include <stdio.h>

typedef b_pl_type b_sl_type;


#define b_sl_GetCnt(b_sl)       b_pl_GetCnt(b_sl)
#define b_sl_GetVal(b_sl,pos)   ((char *)b_pl_GetVal(b_sl,pos))

b_sl_type b_sl_Open();
void b_sl_Close(b_sl_type b_sl);
void b_sl_Clear(b_sl_type b_sl);
/* returns position or -1 (= error) */
int b_sl_Add(b_sl_type b_sl, const char *s);
/* add a substring of 's' starting at 'start' position with 'cnt' chars */
void b_sl_Del(b_sl_type b_sl, int pos);
int b_sl_AddRange(b_sl_type b_sl, const char *s, size_t start, size_t cnt);
int b_sl_Set(b_sl_type b_sl, int pos, const char *s); /* returns 0 (err) or 1 (ok) */
/* returns 0 (error) or 1 (ok) */
int b_sl_Union(b_sl_type dest, b_sl_type src);
/* returns 0 (error) or 1 (ok) */
int b_sl_Copy(b_sl_type dest, b_sl_type src);
/* find a string, returns position or -1 (= not found) */
int b_sl_Find(b_sl_type b_sl, const char *s);

int b_sl_Write(b_sl_type b_sl, FILE *fp);
int b_sl_Read(b_sl_type b_sl, FILE *fp);

/* strings are added to 'b_sl', returns the number of char's read or -1 */
int b_sl_ImportByStr(b_sl_type b_sl, const char *s, const char *delim, const char *term);

int b_sl_ExportToFP(b_sl_type b_sl, FILE *fp, const char *delim, const char *last);

const char *b_sl_ToStr(b_sl_type b_sl, const char *delim);


#endif /* _B_SL_H */
