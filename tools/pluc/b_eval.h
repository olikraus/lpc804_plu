/*

  b_evel.h
  
  compact mathematical (integer) evaluation

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

#ifndef _B_EVAL_H
#define _B_EVAL_H

#include "b_set.h"

#define B_EVAL_VAL  ((int)(unsigned char)'V')
#define B_EVAL_ID   ((int)(unsigned char)'I')
#define B_EVAL_IF   2000
#define B_EVAL_LSHIFT ((int)(unsigned char)'l')
#define B_EVAL_RSHIFT ((int)(unsigned char)'r')
#define B_EVAL_LO   1000   
#define B_EVAL_HI   1001
#define B_EVAL_LOEQ 1002
#define B_EVAL_HIEQ 1003
#define B_EVAL_MUL  ((int)(unsigned char)'*')
#define B_EVAL_DIV  ((int)(unsigned char)'/')
#define B_EVAL_ADD  ((int)(unsigned char)'+')
#define B_EVAL_SUB  ((int)(unsigned char)'-')
#define B_EVAL_NEG  ((int)(unsigned char)'~')
#define B_EVAL_ASSIGN ((int)(unsigned char)'=')
#define B_EVAL_STATEMENT ((int)(unsigned char)';')

typedef struct _b_en_struct *b_en_type;
struct _b_en_struct
{
  b_en_type left;
  b_en_type right;
  int type;
  int data;
};

void b_en_Close(b_en_type en);

struct _b_ev_struct
{
  char *str;
  int val;
  int bit_inputs;
};
typedef struct _b_ev_struct *b_ev_type;

struct _b_eval_struct
{
  b_set_type variables;
  const char *s;
};
typedef struct _b_eval_struct *b_eval_type;

b_eval_type b_eval_Open();
void b_eval_Close(b_eval_type e);
b_en_type b_eval_ParseStr(b_eval_type e, const char *s);
int b_eval_Eval(b_eval_type e, b_en_type en);
int b_eval_SetValByName(b_eval_type e, const char *name, int val);
int b_eval_GetValByName(b_eval_type e, const char *name);

#endif /* _B_EVAL_H */

