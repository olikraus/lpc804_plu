/*

  dcex.h
  
  dcube expressions

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

#ifndef _DCEX_H
#define _DCEX_H

#include "b_set.h"
#include "b_rdic.h"
#include "b_sl.h"
#include "b_il.h"
#include "pinfo.h"
#include "dcube.h"
#include <stdarg.h>

typedef struct _dcexstr_struct *dcexstr;
typedef struct _dcexnode_struct *dcexn;

dcexn dcexnOpen();
void dcexnClose(dcexn n);


#define DCEX_IDENTIFIER_LEN 128
struct _dcex_struct
{
  char identifer[DCEX_IDENTIFIER_LEN];
  int value;
  const char *s;    /* current symbol */

  /* support variable commands */
  int is_vars_command;

  /* string table */  
  b_set_type id_set;
  b_rdic_type id_index;
  
  /* user defined strings */
  b_sl_type in_variables;
  b_sl_type out_variables;
  b_sl_type inout_variables;
  
  /* cube increment */
  b_il_type pn_vars;
  
  /* global target (cube) of the assignement evaluation */
  dcube *assignment_cube;
  /* global source (cube) of the evaluation */
  dcube *input_cube;

  /* error callback */
  void (*error_fn)(void *error_data, char *fmt, va_list va);
  void *error_data;
  char *error_prefix;
  
  /* a local variable for _dcexToDCL */
  pinfo *pi_to_dcl;
};
typedef struct _dcex_struct dcex_struct;
typedef struct _dcex_struct *dcex_type;

#define DCEXN_STR_OFFSET 1024
#define DCEXN_INVALID 0
#define DCEXN_OR ((unsigned)'|')
#define DCEXN_AND ((unsigned)'&')
#define DCEXN_NOT ((unsigned)'!')
#define DCEXN_ONE ((unsigned)'1')
#define DCEXN_ZERO ((unsigned)'0')
#define DCEXN_EQ ((unsigned)300)
#define DCEXN_NEQ ((unsigned)301)
#define DCEXN_ASSIGN ((unsigned)302)
#define DCEXN_CMDLIST ((unsigned)303)
#define DCEXN_NOP ((unsigned)304)


struct _dcexstr_struct
{
  char *str;
  unsigned is_left_side:1;
  unsigned is_right_side:1;
  unsigned is_visited:1;
  unsigned is_positiv:1;
  unsigned is_negativ:1;
  int cube_in_var_index;		/* 2019: renamed from cube_var_index to cube_in_var_index */
  int cube_out_var_index;		/* 2019: new var */
};

struct _dcexnode_struct
{ 
  dcexn down;
  dcexn next;
  unsigned data;
};

dcex_type dcexOpen();
void dcexClose(dcex_type dcex);

int dcexAssignCubeVarReference(dcex_type dcex, dcexn n);
void dcexMarkSide(dcex_type dcex, dcexn n, int is_left_side);

int dcexSetStrRef(dcex_type dcex, int ref, int var_index, const char *str);

int dcexSetErrorPrefix(dcex_type dcex, char *p);

#define DCEX_TYPE_BOOLE 0
#define DCEX_TYPE_ASSIGNMENT 1

void dcexClearVariables(dcex_type dcex);

/* NOTE: parser functions usually require a previous  */
/*       call to dcexClearVariables */
dcexn dcexParser(dcex_type dcex, const char *expr, int type);
int dcexIsValid(dcex_type dcex, const char *expr, int type);
dcexn dcexGenlibParser(dcex_type dcex, const char *expr);

int dcexEval(dcex_type dcex, pinfo *pi, dclist cl, dcexn n, int type);

void dcexShow(dcex_type dcex, dcexn n);
void dcexClearVariables(dcex_type dcex);

void dcexMarkVisited(dcex_type dcex, dcexn n);
void dcexMarkPosNeg(dcex_type dcex, dcexn n);

int dcexEvalBoole(dcex_type dcex, dcexn n);

int dcexEvalInCubes(dcex_type dcex, pinfo *pi, dclist cl, dcexn n);
int dcexEvalInOutCubes(dcex_type dcex, pinfo *pi, dclist cl, dcexn n);

int dcexResetCube(dcex_type dcex, pinfo *pi, dcube *c, dcexn n, int is_posneg);
int dcexIncCube(dcex_type dcex, pinfo *pi, dcube *c);

/* dcexop.c */
dcexn dcexReduceNot(dcex_type dcex, dcexn n);
int dcexToDCL(dcex_type dcex, pinfo *pi, dclist cl, dcexn n);


#endif  /* _DCEX_H */
