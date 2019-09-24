/*

  dcex.c
  
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


  
  dcube expressions (file extension ".bex")


  a == 1     (a & b) | c 
  
  y <= 1;
  
  bool_fn   ::= <or_fn> {('=='|'!=') <or_fn>}
  or_fn     ::= <and_fn> {('|') <and_fn>}
  and_fn    ::= <not_atom> {('&') <not_atom>}
  not_atom  ::= ['!'] <atom>
  atom      ::= <para_atom> | <identifier>
  para_atom ::= '(' <bool_fn> ')'
  
  
  const_def ::= 'const' <identifier> '<=' <number> ';'
  condition ::= 'cond' <bool_fn> ';'
  assignment::= <identifier> '<=' <bool_fn>;
  if        ::= 'if' <para_atom> <statements>
  for       ::= 'for' <identifier> 'in' <range> <statements>
  range     ::= '(' <range_part> {',' <range_part>} ')'
  range_part::= <number> | <number> 'to' <number> | <number> 'downto' <number>
  statements::= {<assignment>|<if>|<for>}
  
  
  TODO: Do more consistency checks, especially between variables and pinfo
  
*/

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <ctype.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <assert.h>
#include "dcex.h"
#include "mwc.h"
#include "b_ff.h"

#define DCEX_PARSE_BOOLE DCEX_TYPE_BOOLE
#define DCEX_PARSE_ASSIGNMENT DCEX_TYPE_ASSIGNMENT




static dcexn dcexBooleParser(dcex_type dcex);


/*---------------------------------------------------------------------------*/

dcexstr dcexstrOpen(const char *str)
{
  dcexstr s;
  s = (dcexstr)malloc(sizeof(struct _dcexstr_struct));
  if ( s != NULL )
  {
    if ( str != NULL )
    {
      s->str = strdup(str);
      if ( s->str == NULL )
        return free(s), (dcexstr)NULL;
    }
    s->is_left_side = 0;
    s->is_right_side = 0;
    s->is_visited = 0;
    s->is_positiv = 0;
    s->is_negativ = 0;
    s->cube_in_var_index = -1;		/* 2019: hmmm... default value should be -1 instead of 0 */
    s->cube_out_var_index = -1;
    return s;
  }
  return NULL;
}

void dcexstrClose(dcexstr s)
{
  if ( s->str != NULL )
    free(s->str);
  free(s);
}

/*---------------------------------------------------------------------------*/

dcexn dcexnOpen()
{
  dcexn n;
  n = (dcexn)malloc(sizeof(struct _dcexnode_struct));
  if ( n != NULL )
  {
    n->down = NULL;
    n->next = NULL;
    n->data = DCEXN_INVALID;
    return n;
  }
  return NULL;
}

void dcexnCloseNode(dcexn n)
{
  n->down = NULL;
  n->next = NULL;
  n->data = DCEXN_INVALID;
  free(n);
}

void dcexnClose(dcexn n)
{
  if ( n == NULL ) return;
  dcexnClose(n->down); 
  dcexnClose(n->next); 
  dcexnCloseNode(n);
}

/*---------------------------------------------------------------------------*/

int dcex_str_cmp_fn(void *data, int el, const void *key)
{
  dcex_type dcex = (dcex_type)data;
  dcexstr s = (dcexstr)b_set_Get(dcex->id_set, el);
  return strcasecmp(s->str, (const char *)key);
}

int dcexInit(dcex_type dcex)
{
  dcex->in_variables = NULL;
  dcex->out_variables = NULL;
  dcex->inout_variables = NULL;
  dcex->pn_vars = NULL;

  dcex->in_variables = b_sl_Open();
  dcex->out_variables = b_sl_Open();
  dcex->inout_variables = b_sl_Open();
  dcex->pn_vars = b_il_Open();
  
  if ( dcex->in_variables != NULL && 
       dcex->out_variables != NULL &&
       dcex->inout_variables != NULL &&
       dcex->pn_vars != NULL )
    return 1;

  if ( dcex->in_variables != NULL ) b_sl_Close(dcex->in_variables);
  if ( dcex->out_variables != NULL ) b_sl_Close(dcex->out_variables);
  if ( dcex->inout_variables != NULL ) b_sl_Close(dcex->inout_variables);
  if ( dcex->pn_vars != NULL ) b_il_Close(dcex->pn_vars);
  
  return 0;
}

void dcex_default_error_fn(void *data, char *fmt, va_list va)
{
  vprintf(fmt, va);
  puts("");
}

dcex_type dcexOpen()
{
  dcex_type dcex;
  dcex = (dcex_type)malloc(sizeof(dcex_struct));
  if ( dcex != NULL )
  {
    dcex->is_vars_command = 0;
    dcex->assignment_cube = NULL;
    dcex->input_cube = NULL;
    dcex->id_set = b_set_Open();
    dcex->error_data = NULL;
    dcex->error_fn = dcex_default_error_fn;
    dcex->error_prefix = strdup("BEX Import: ");
    if ( dcex->error_prefix != NULL )
    {
      if ( dcexInit(dcex) != 0 )
      {    
        if ( dcex->id_set != NULL )
        {
          dcex->id_index = b_rdic_Open();
          if ( dcex->id_index != NULL )
          {
            b_rdic_SetCmpFn(dcex->id_index, dcex_str_cmp_fn, dcex);
            return dcex;
          }
          b_set_Close(dcex->id_set);
        }
      }
      free(dcex->error_prefix);
    }
    free(dcex);
  }
  return NULL;
}

void dcexClose(dcex_type dcex)
{
  int pos = -1;
  if ( dcex->in_variables != NULL ) b_sl_Close(dcex->in_variables);
  if ( dcex->out_variables != NULL ) b_sl_Close(dcex->out_variables);
  if ( dcex->inout_variables != NULL ) b_sl_Close(dcex->inout_variables);
  if ( dcex->pn_vars != NULL ) b_il_Close(dcex->pn_vars);
  while( b_set_WhileLoop(dcex->id_set, &pos) != 0 )
    dcexstrClose((dcexstr)b_set_Get(dcex->id_set, pos));
  b_rdic_Close(dcex->id_index);
  b_set_Close(dcex->id_set);
  free(dcex->error_prefix);
  free(dcex);
}

void dcexError(dcex_type dcex, char *fmt, ...)
{
  va_list va;
  va_start(va, fmt);
  dcex->error_fn(dcex->error_data, fmt, va);
  va_end(va);
}

int dcexSetErrorPrefix(dcex_type dcex, char *p)
{
  p = strdup(p);
  if ( p == NULL )
    return 0;
  free(dcex->error_prefix);
  dcex->error_prefix = p;
  return 1;
}

/*---------------------------------------------------------------------------*/

static int dcexGetStrRef(dcex_type dcex, char *str)
{
  int ref;
  dcexstr s;
  if ( str == NULL )
    return -1;
  ref = b_rdic_Find(dcex->id_index, str);
  if ( ref >= 0 )
    return ref;
  s = dcexstrOpen(str);
  if ( s == NULL )
    return -1;
  ref = b_set_Add(dcex->id_set, s);
  if ( ref < 0 )
    return dcexstrClose(s), -1;
  if ( b_rdic_Ins(dcex->id_index, ref, str) == 0 )
    return -1;
  return ref;
}

/* 2019: called from gnetdcex.c, probably we should pass another var telling whether the variable is input or output */
int dcexSetStrRef(dcex_type dcex, int ref, int var_index, const char *str)
{
  dcexstr s;
  if ( str == NULL )
    return 0;
  if ( ref < 0 )
    return 0;
    
  s = dcexstrOpen(str);
  if ( s == NULL )
    return 0;
  
  /* 2019: until we know, what variable this is, the index is assigned to both in and out */
  s->cube_in_var_index = var_index;
  s->cube_out_var_index = var_index;
  if ( b_set_Set(dcex->id_set, ref, s) == 0 )
    return dcexstrClose(s), 0;
  if ( b_rdic_Ins(dcex->id_index, ref, str) == 0 )
    return 0;
  return 1;
}

static dcexn dcexNewStrNode(dcex_type dcex, char *str)
{
  int pos = dcexGetStrRef(dcex, str);
  dcexn n;
  if ( pos < 0 )
    return NULL;
  n = dcexnOpen();
  if ( n == NULL )
    return dcexError(dcex, "%sOut of Memory.", dcex->error_prefix), (dcexn)NULL;
  n->data = pos+DCEXN_STR_OFFSET;
  return n;
}

dcexstr dcexGetDCEXSTR(dcex_type dcex, int pos)
{
  return (dcexstr)b_set_Get(dcex->id_set, pos);
}

/*---------------------------------------------------------------------------*/

static int dcexIsSymF(dcex_type dcex)
{
  if ( *(dcex->s) >= 'a' && *(dcex->s) <= 'z' )
    return 1;
  if ( *(dcex->s) >= 'A' && *(dcex->s) <= 'Z' )
    return 1;
  if ( *(dcex->s) == '_' )
    return 1;
  return 0;
}

static int dcexIsSym(dcex_type dcex)
{
  if ( *(dcex->s) >= '0' && *(dcex->s) <= '9' )
    return 1;
  return dcexIsSymF(dcex);
}

static int dcexIsSpace(dcex_type dcex)
{
  if ( *(dcex->s) > '\0' && *(dcex->s) <= ' ' )
    return 1;
  return 0;
}

static void dcexSkipSpace(dcex_type dcex)
{
  for(;;)
  {
    while(dcexIsSpace(dcex) != 0)
      dcex->s++;
    if ( *dcex->s == '#' )
    {
      dcex->s++;
      for(;;)
      {
        if ( *dcex->s == '\0' )
          break;
        if ( *dcex->s == '\n' )
        {
          dcex->s++;
          break;
        }
        dcex->s++;
      }
    }
    else
    {
      break;
    }
  }
}

static void dcexReadIdentifier(dcex_type dcex)
{
  int i = 0;
  if ( dcexIsSymF(dcex) != 0 )
  {
    do
    { 
      if ( i < DCEX_IDENTIFIER_LEN-1 )
      {
        dcex->identifer[i] = *(dcex->s);
        i++;
      }
      dcex->s++;
    } while(dcexIsSym(dcex) != 0);
  }
  dcex->identifer[i] = '\0';
  dcexSkipSpace(dcex);
}

/* unused at the moment...
static void dcexDecimalValue(dcex_type dcex)
{
  dcex->value = 0;
  if ( isdigit(*(dcex->s)) != 0 )
  {
    do
    {
      dcex->value *= 10;
      dcex->value += (int)(*(dcex->s)-'0');
    } while( isdigit(*(dcex->s)) );
  }
}
*/

/* 1: equal */
/* 0: not equal */
static int dcexCompareStream(dcex_type dcex, const char *s)
{
  const char *t = dcex->s;
  for(;;)
  {
    if ( *s == '\0' )
      break;
    if ( *t == '\0' )
      return 0;
    if ( *s != *t )
      return 0;
    s++;
    t++;
  }
  dcex->s = t;
  dcexSkipSpace(dcex);
  return 1;
}

static dcexn dcexNewCharSymNode(dcex_type dcex)
{
  dcexn n;
  n = dcexnOpen();
  if ( n == NULL )
    return dcexError(dcex, "%sOut of Memory.", dcex->error_prefix), (dcexn)NULL;
  n->data = *(dcex->s);
  dcex->s++;
  dcexSkipSpace(dcex);
  return n;
}

static dcexn dcexNewSymNode(dcex_type dcex, unsigned data)
{
  dcexn n;
  n = dcexnOpen();
  if ( n == NULL )
    return dcexError(dcex, "%sOut of Memory.", dcex->error_prefix), (dcexn)NULL;
  n->data = data;
  return n;
}

/*
static dcexn dcexIdentifier(dcex_type dcex)
{
  dcexReadIdentifier(dcex);
  return dcexNewStrNode(dcex, dcex->identifer);
}
*/

static dcexn dcexAssignVariables(dcex_type dcex, b_sl_type sl)
{
  if ( dcexCompareStream(dcex, "(") == 0 )
    return NULL;
  while( dcexCompareStream(dcex, ")") == 0 )
  {
    if ( dcexIsSymF(dcex) == 0 )
      return NULL;
    dcexReadIdentifier(dcex);
    if ( b_sl_Add(sl, dcex->identifer) < 0 )
      return NULL;
    if ( dcexGetStrRef(dcex, dcex->identifer) < 0 )
      return NULL;
  }

  return dcexNewSymNode(dcex, DCEXN_NOP);
}

static dcexn dcexAtom(dcex_type dcex)
{
  if ( dcexIsSymF(dcex) != 0 )
  {
    dcexReadIdentifier(dcex);
    if ( dcex->is_vars_command != 0 )
    {
      if ( strcmp(dcex->identifer, "invars") == 0 )
        return dcexAssignVariables(dcex, dcex->in_variables);
      else if ( strcmp(dcex->identifer, "outvars") == 0 )
        return dcexAssignVariables(dcex, dcex->out_variables);
      else if ( strcmp(dcex->identifer, "inoutvars") == 0 )
        return dcexAssignVariables(dcex, dcex->inout_variables);
    }
    return dcexNewStrNode(dcex, dcex->identifer);
  }
  if ( *(dcex->s) == '0' || *(dcex->s) == '1' )
    return dcexNewCharSymNode(dcex);
  dcexError(dcex, "%sUnknown symbol '%c'.", dcex->error_prefix, *(dcex->s));
  return NULL;
}

static dcexn dcexParenthesis(dcex_type dcex, dcexn (*top)(dcex_type dcex), dcexn (*atom)(dcex_type dcex))
{
  if ( dcexCompareStream(dcex, "(") == 0 )
    return atom(dcex);
  {
    dcexn n = top(dcex);
    if ( n == NULL )
      return NULL;
    if ( dcexCompareStream(dcex, ")") == 0 )
      return dcexError(dcex, "%sMissing ')'.", dcex->error_prefix), dcexnClose(n), (dcexn)NULL;
    return n;
  }
}

static dcexn dcexNOT(dcex_type dcex)
{
  if ( dcexCompareStream(dcex, "!") == 0 )
    return dcexParenthesis(dcex, dcexBooleParser, dcexAtom);
    
  {
    dcexn n = dcexNewSymNode(dcex, DCEXN_NOT);
    if ( n == NULL ) 
      return NULL;
    n->down = dcexParenthesis(dcex, dcexBooleParser, dcexAtom);
    if ( n->down == NULL )
      return dcexnClose(n), (dcexn)NULL;
    return n;
  }
}

static dcexn dcexParseBinaryOp(dcex_type dcex, dcexn (*fn)(dcex_type dcex), const char *ops, unsigned data)
{
  dcexn arg;
  
  arg = fn(dcex);
  if ( arg == NULL ) 
    return NULL;
  if ( dcexCompareStream(dcex, ops) == 0 ) 
    return arg;
    
  {
    dcexn op = dcexNewSymNode(dcex, data);
    if ( op == NULL )
      return NULL;
    op->down = arg;
    do
    {
      arg->next = fn(dcex);
      if ( arg->next == NULL )
        return dcexnClose(op), (dcexn)NULL;
      arg = arg->next;
    } while( dcexCompareStream(dcex, ops) != 0 );
    return op;
  }
}

static dcexn dcexAND(dcex_type dcex)
{
  return dcexParseBinaryOp(dcex, dcexNOT, "&", DCEXN_AND);
}

static dcexn dcexOR(dcex_type dcex)
{
  return dcexParseBinaryOp(dcex, dcexAND, "|", DCEXN_OR);
}

static dcexn dcexNEQ(dcex_type dcex)
{
  return dcexParseBinaryOp(dcex, dcexOR, "!=", DCEXN_NEQ);
}

static dcexn dcexEQ(dcex_type dcex)
{
  return dcexParseBinaryOp(dcex, dcexNEQ, "==", DCEXN_EQ);
}

static dcexn dcexBooleParser(dcex_type dcex)
{
  return dcexEQ(dcex);
}

static dcexn dcexAssignmentParser(dcex_type dcex)
{
  return dcexParseBinaryOp(dcex, dcexBooleParser, "<=", DCEXN_ASSIGN);
}

static dcexn dcexAssignmentListParser(dcex_type dcex)
{
  dcexn op = dcexNewSymNode(dcex, DCEXN_CMDLIST);
  dcexn n;
  if ( op == NULL )
    return NULL;
    
  op->down = dcexAssignmentParser(dcex);
  if ( op->down == NULL )
    return dcexnClose(op), (dcexn)NULL;
  n = op->down;
  dcexSkipSpace(dcex);
  for(;;)
  {
    if ( *dcex->s != ';' )
      break;
    dcex->s++;
    dcexSkipSpace(dcex);
    if ( *dcex->s == '\0' )
      break;
    n->next = dcexAssignmentParser(dcex);
    if ( n->next == NULL )
      return dcexnClose(op), (dcexn)NULL;
    n = n->next;
  }
  if ( *dcex->s != '\0' )
  {
    dcexReadIdentifier(dcex);
    if ( dcex->identifer[0] != '\0' )
    {
      dcexError(dcex, "%sUnexpected '%s' (missing operand or missing ';'?).", dcex->error_prefix, dcex->identifer);
      dcexnClose(op);
      return (dcexn)NULL;
    }
    dcexError(dcex, "%sUnexpected '%c' (unknown operand?).", dcex->error_prefix, *(dcex->s));
    dcexnClose(op);
    return (dcexn)NULL;
  }
  return op;
}

/*---------------------------------------------------------------------------*/

int dcexEvalEQ(dcex_type dcex, dcexn first, int (*eval_fn)(dcex_type dcex, dcexn n))
{
  int v;
  v = eval_fn(dcex, first);
  while( first->next != NULL )
  {
    v = (v == eval_fn(dcex, first->next));
    first = first->next;
  }
  return v;
}

int dcexEvalNEQ(dcex_type dcex, dcexn first, int (*eval_fn)(dcex_type dcex, dcexn n))
{
  int v;
  v = eval_fn(dcex, first);
  while( first->next != NULL )
  {
    v = (v != eval_fn(dcex, first->next));
    first = first->next;
  }
  return v;
}

int dcexEvalAND(dcex_type dcex, dcexn first, int (*eval_fn)(dcex_type dcex, dcexn n))
{
  int v;
  v = eval_fn(dcex, first);
  while( first->next != NULL )
  {
    v = (v && eval_fn(dcex, first->next));
    first = first->next;
  }
  return v;
}

int dcexEvalOR(dcex_type dcex, dcexn first, int (*eval_fn)(dcex_type dcex, dcexn n))
{
  int v;
  v = eval_fn(dcex, first);
  while( first->next != NULL )
  {
    v = (v || eval_fn(dcex, first->next));
    first = first->next;
  }
  return v;
}

int dcexEvalInput(dcex_type dcex, dcexn n)
{
  int v;
  dcexstr xs;
  assert( n->data >= DCEXN_STR_OFFSET );
  
  xs = dcexGetDCEXSTR(dcex, n->data-DCEXN_STR_OFFSET);
  if ( xs->cube_in_var_index < 0 )
  {
    dcexError(dcex, "%sInput variable '%s' no legal source.", dcex->error_prefix, xs->str);
    return 0;
  }
  
  v = 0;
  if ( dcGetIn(dcex->input_cube, xs->cube_in_var_index) == 2 )
    v = 1;
      
  return v;
}

int dcexEvalBoole(dcex_type dcex, dcexn n)
{
  if ( n == NULL )
    return 0;
  switch(n->data)
  {
    case DCEXN_ONE:
      return 1;
    case DCEXN_ZERO:
      return 0;
    case DCEXN_NOT:
      return !dcexEvalBoole(dcex, n->down);
    case DCEXN_EQ:
      return dcexEvalEQ(dcex, n->down, dcexEvalBoole);
    case DCEXN_NEQ:
      return dcexEvalNEQ(dcex, n->down, dcexEvalBoole);
    case DCEXN_OR:
      return dcexEvalOR(dcex, n->down, dcexEvalBoole);
    case DCEXN_AND:
      return dcexEvalAND(dcex, n->down, dcexEvalBoole);
    case DCEXN_NOP:
      return 0;
    default:
      if ( n->data >= DCEXN_STR_OFFSET )
        return dcexEvalInput(dcex, n);
      dcexError(dcex, "%sIllegal node (dcexEvalBoole)", dcex->error_prefix);
      break;
  }
  
  return 0;
}

int dcexEvalCmdList(dcex_type dcex, dcexn first, int (*eval_fn)(dcex_type dcex, dcexn n))
{
  if ( first == NULL )
    return 0;
  eval_fn(dcex, first);
  while( first->next != NULL )
  {
    eval_fn(dcex, first->next);
    first = first->next;
  }
  return 0;
}

int dcexEvalAssign(dcex_type dcex, dcexn first, int (*eval_fn)(dcex_type dcex, dcexn n))
{
  int v;
  if ( first->data < DCEXN_STR_OFFSET )
  {
    dcexError(dcex, "%sThe left side of an assignment must be a legal identifier.", dcex->error_prefix);
    return 0;
  }
  if ( first->next == NULL )
  {
    dcexError(dcex, "%sIllegal right side of an assignment.", dcex->error_prefix);
    return 0;
  }
  v = eval_fn(dcex, first->next);
  {
    dcexstr xs;
    xs = dcexGetDCEXSTR(dcex, first->data-DCEXN_STR_OFFSET);
    if ( xs->cube_out_var_index < 0 )
    {
      dcexError(dcex, "%sThe left side (%s) of the assignment has no legal target.", dcex->error_prefix, xs->str);
      return 0;
    }
    if ( v == 0 )
      dcSetOut( dcex->assignment_cube, xs->cube_out_var_index, 0);
    else
      dcSetOut( dcex->assignment_cube, xs->cube_out_var_index, 1);
  }
  return v;
}


int dcexEvalAssignment(dcex_type dcex, dcexn n)
{
  if ( n == NULL )
    return 0;
  switch(n->data)
  {
    case DCEXN_ONE:
      return 1;
    case DCEXN_ZERO:
      return 0;
    case DCEXN_NOT:
      return !dcexEvalAssignment(dcex, n->down);
    case DCEXN_EQ:
      return dcexEvalEQ(dcex, n->down, dcexEvalAssignment);
    case DCEXN_NEQ:
      return dcexEvalNEQ(dcex, n->down, dcexEvalAssignment);
    case DCEXN_OR:
      return dcexEvalOR(dcex, n->down, dcexEvalAssignment);
    case DCEXN_AND:
      return dcexEvalAND(dcex, n->down, dcexEvalAssignment);
    case DCEXN_CMDLIST:
      return dcexEvalCmdList(dcex, n->down, dcexEvalAssignment);
    case DCEXN_ASSIGN:
      return dcexEvalAssign(dcex, n->down, dcexEvalAssignment);
    default:
      if ( n->data >= DCEXN_STR_OFFSET )
        return dcexEvalInput(dcex, n);
      dcexError(dcex, "%sIllegal node (dcexEvalAssignment)", dcex->error_prefix);
      break;
  }
  
  return 0;
}


/*---------------------------------------------------------------------------*/

void dcexMarkSide(dcex_type dcex, dcexn n, int is_left_side)
{
  if ( n == NULL )
    return;
  if ( n->data == DCEXN_ASSIGN )
  {
    if ( n->down != NULL )
    {
      if ( n->down->data >= DCEXN_STR_OFFSET )
      {
        dcexGetDCEXSTR(dcex, n->down->data-DCEXN_STR_OFFSET)->is_left_side = 1;
      }
      dcexMarkSide(dcex, n->down->down, 1);
      dcexMarkSide(dcex, n->down->next, 0);
    }
  }
  else
  {
    dcexMarkSide(dcex, n->down, is_left_side);
  }
  
  dcexMarkSide(dcex, n->next, is_left_side);
  
  if ( n->data >= DCEXN_STR_OFFSET )
  {
    if ( is_left_side != 0 )
      dcexGetDCEXSTR(dcex, n->data-DCEXN_STR_OFFSET)->is_left_side = 1;
    else
      dcexGetDCEXSTR(dcex, n->data-DCEXN_STR_OFFSET)->is_right_side = 1;
  }
}


/*---------------------------------------------------------------------------*/

int dcex_sl_find_str(b_sl_type sl, const char *name)
{
  int i, cnt = b_sl_GetCnt(sl);
  for( i = 0; i < cnt; i++ )
    if ( strcasecmp(b_sl_GetVal(sl,i), name) == 0 )
      return i;
  return -1;
}

b_sl_type dcexGetStrListByDCEXSTR(dcex_type dcex, dcexstr xs)
{
  if ( xs->is_left_side == 0 && xs->is_right_side == 0 )
    return 0;
  if ( xs->is_left_side == 0 && xs->is_right_side != 0 )
    return dcex->in_variables;
  if ( xs->is_left_side != 0 && xs->is_right_side == 0 )
    return dcex->out_variables;
  return dcex->inout_variables;
}


/* obsolete
static void dcexClearCubeVarReference(dcex_type dcex)
{
  int pos = -1;
  while( b_set_WhileLoop(dcex->id_set, &pos) != 0 )
    dcexGetDCEXSTR(dcex, pos)->cube_var_index = -1;
}
*/

static int dcexSubAssignCubeVarReference(dcex_type dcex, dcexn n)
{
  if ( n == NULL )
    return 1;

  if ( n->data >= DCEXN_STR_OFFSET )
  {
    dcexstr xs;
    b_sl_type sl;
    xs = dcexGetDCEXSTR(dcex, n->data-DCEXN_STR_OFFSET);
    sl = dcexGetStrListByDCEXSTR(dcex, xs);
    if ( sl == NULL )
    {
      dcexError(dcex, "%sSyntax tree not marked (internal error).", dcex->error_prefix);
      return 0;
    }
    
    /* START 2019: try to support inout vars better */
    if ( sl == dcex->inout_variables )
    {
      
      xs->cube_out_var_index = dcex_sl_find_str(dcex->out_variables, xs->str);
      if ( xs->cube_out_var_index < 0 )
      {
	xs->cube_out_var_index = b_sl_Add(dcex->out_variables, xs->str);
	if ( xs->cube_out_var_index < 0 )
	{
	  dcexError(dcex, "%sOut of memory (b_sl_Add).", dcex->error_prefix);
	  return 0;
	}
      }

      xs->cube_in_var_index = dcex_sl_find_str(dcex->inout_variables, xs->str);
      if ( xs->cube_in_var_index < 0 )
      {
	xs->cube_in_var_index = b_sl_Add(dcex->inout_variables, xs->str);
	if ( xs->cube_in_var_index < 0 )
	{
	  dcexError(dcex, "%sOut of memory (b_sl_Add).", dcex->error_prefix);
	  return 0;
	}
      }

      /* finally, add it to the in variables, keep the index for the in variables */
      xs->cube_in_var_index = dcex_sl_find_str(dcex->in_variables, xs->str);
      if ( xs->cube_in_var_index < 0 )
      {
	xs->cube_in_var_index = b_sl_Add(dcex->in_variables, xs->str);
	if ( xs->cube_in_var_index < 0 )
	{
	  dcexError(dcex, "%sOut of memory (b_sl_Add).", dcex->error_prefix);
	  return 0;
	}
      }
      
    }
    else    
    {
      /* 2019: we could assign the index to the individual in or out var index */
      /* as of now, the index is assigned to both */
      xs->cube_in_var_index = dcex_sl_find_str(sl, xs->str);
      if ( xs->cube_in_var_index < 0 )
      {
	xs->cube_in_var_index = b_sl_Add(sl, xs->str);
	if ( xs->cube_in_var_index < 0 )
	{
	  dcexError(dcex, "%sOut of memory (b_sl_Add).", dcex->error_prefix);
	  return 0;
	}
      }
      xs->cube_out_var_index = xs->cube_in_var_index;	/* might be wrong, but let's see.. */
    }    
    
  }

  if ( dcexSubAssignCubeVarReference(dcex, n->down) == 0 )
    return 0;
  if ( dcexSubAssignCubeVarReference(dcex, n->next) == 0 ) 
    return 0;
    
  return 1;
}


int dcexAssignCubeVarReference(dcex_type dcex, dcexn n)
{
  return dcexSubAssignCubeVarReference(dcex, n);
}

/*---------------------------------------------------------------------------*/

void dcexClearVariables(dcex_type dcex)
{
  b_sl_Clear(dcex->in_variables);
  b_sl_Clear(dcex->out_variables);
  b_sl_Clear(dcex->inout_variables);
}

/*---------------------------------------------------------------------------*/

dcexn dcexParser(dcex_type dcex, const char *expr, int type)
{ 
  dcexn n = NULL;
  dcex->s = expr;
  
  dcexSkipSpace(dcex);
  
  switch(type)
  {
    case DCEX_PARSE_BOOLE:
      n = dcexBooleParser(dcex);
      break;
    case DCEX_PARSE_ASSIGNMENT:
      dcex->is_vars_command = 1;
      n = dcexAssignmentListParser(dcex);
      break;
      
  }
  if ( n == NULL )
    return NULL;
  dcexMarkSide(dcex, n, 0);
  dcexAssignCubeVarReference(dcex, n);
  return n;
}

static int dcex_is_valid = 1;

void dcex_null_error_fn(void *data, char *fmt, va_list va)
{
  dcex_is_valid = 0;
}


int dcexIsValid(dcex_type dcex, const char *expr, int type)
{
  void (*error_fn)(void *error_data, char *fmt, va_list va);
  dcexn n = NULL;
  dcex->s = expr;

  error_fn = dcex->error_fn;              /* backup old error function */
  dcex->error_fn = dcex_null_error_fn;
  dcex_is_valid = 1;
  
  dcexSkipSpace(dcex);
  
  switch(type)
  {
    case DCEX_PARSE_BOOLE:
      n = dcexBooleParser(dcex);
      break;
    case DCEX_PARSE_ASSIGNMENT:
      dcex->is_vars_command = 1;
      n = dcexAssignmentListParser(dcex);
      break;
  }

  dcex->error_fn = error_fn;              /* restore the old function */

  if ( n == NULL )
    return 0;
    
  if ( dcex_is_valid == 0 )
    return dcexnClose(n), 0;

  dcexMarkSide(dcex, n, 0);
  dcexAssignCubeVarReference(dcex, n);
    
  if ( b_sl_GetCnt(dcex->out_variables) == 0 )
    return dcexnClose(n), 0;
 
  if ( n->down == NULL && n->next == NULL )
    return dcexnClose(n), 0;
    
  dcexnClose(n);
  return 1;
}

/*---------------------------------------------------------------------------*/

static dcexn dcexGenlibBooleParser(dcex_type dcex);

static dcexn dcexGenlibAtom(dcex_type dcex)
{
  if ( dcexCompareStream(dcex, "CONST") != 0 )
    return dcexNewCharSymNode(dcex);
  return dcexAtom(dcex);
}

static dcexn dcexGenlibNOT(dcex_type dcex)
{
  if ( dcexCompareStream(dcex, "!") == 0 )
    return dcexParenthesis(dcex, dcexGenlibBooleParser, dcexGenlibAtom);
    
  {
    dcexn n = dcexNewSymNode(dcex, DCEXN_NOT);
    if ( n == NULL ) 
      return NULL;
    n->down = dcexParenthesis(dcex, dcexGenlibBooleParser, dcexGenlibAtom);
    if ( n->down == NULL )
      return dcexnClose(n), (dcexn)NULL;
    return n;
  }
}

static dcexn dcexGenlibAND(dcex_type dcex)
{
  return dcexParseBinaryOp(dcex, dcexGenlibNOT, "*", DCEXN_AND);
}

static dcexn dcexGenlibOR(dcex_type dcex)
{
  return dcexParseBinaryOp(dcex, dcexGenlibAND, "+", DCEXN_OR);
}

static dcexn dcexGenlibBooleParser(dcex_type dcex)
{
  return dcexParseBinaryOp(dcex, dcexGenlibOR, "=", DCEXN_ASSIGN);
}

dcexn dcexGenlibParser(dcex_type dcex, const char *expr)
{
  dcexn n = NULL;
  dcex->s = expr;

  dcexSkipSpace(dcex);
  
  n = dcexGenlibBooleParser(dcex);
  if ( n == NULL )
    return NULL;
  dcexMarkSide(dcex, n, 0);
  dcexAssignCubeVarReference(dcex, n);
  return n;
}

/*---------------------------------------------------------------------------*/

void dcexClearVisited(dcex_type dcex)
{
  int pos = -1;
  while( b_set_WhileLoop(dcex->id_set, &pos) != 0 )
    dcexGetDCEXSTR(dcex, pos)->is_visited = 0;
}

static void dcexSubMarkVisited(dcex_type dcex, dcexn n)
{
  if ( n == NULL )
    return;
  if ( n->data >= DCEXN_STR_OFFSET )
  {
    dcexstr xs = dcexGetDCEXSTR(dcex, n->data-DCEXN_STR_OFFSET);
    xs->is_visited = 1;
  }
  dcexSubMarkVisited(dcex, n->down);
  dcexSubMarkVisited(dcex, n->next);
}

void dcexMarkVisited(dcex_type dcex, dcexn n)
{
  dcexClearVisited(dcex);
  dcexSubMarkVisited(dcex, n);
}

/*---------------------------------------------------------------------------*/

void dcexClearPosNeg(dcex_type dcex)
{
  int pos = -1;
  while( b_set_WhileLoop(dcex->id_set, &pos) != 0 )
  {
    dcexGetDCEXSTR(dcex, pos)->is_negativ = 0;
    dcexGetDCEXSTR(dcex, pos)->is_positiv = 0;
  }
}


static void dcexMarkUnknownPosNeg(dcex_type dcex, dcexn n)
{
  if ( n == NULL )
    return;
  if ( n->data >= DCEXN_STR_OFFSET )
  {
    dcexstr xs = dcexGetDCEXSTR(dcex, n->data-DCEXN_STR_OFFSET);
    xs->is_positiv = 1;
    xs->is_negativ = 1;
  }
  dcexMarkUnknownPosNeg(dcex, n->down);
  dcexMarkUnknownPosNeg(dcex, n->next);
}

static void dcexSubMarkPosNeg(dcex_type dcex, dcexn n, int is_positiv)
{
  if ( n == NULL )
    return;
  if ( n->data >= DCEXN_STR_OFFSET )
  {
    if ( is_positiv != 0 )
      dcexGetDCEXSTR(dcex, n->data-DCEXN_STR_OFFSET)->is_positiv = 1;
    else
      dcexGetDCEXSTR(dcex, n->data-DCEXN_STR_OFFSET)->is_negativ = 1;
  }
  
  dcexSubMarkPosNeg(dcex, n->next, is_positiv);
  
  if ( n->data == DCEXN_EQ || n->data == DCEXN_NEQ )
  {
    dcexMarkUnknownPosNeg(dcex, n->down);
  }
  else if ( n->data == DCEXN_NOT )
  {
    dcexSubMarkPosNeg(dcex, n->down, !is_positiv);
  }
  else
  {
    dcexSubMarkPosNeg(dcex, n->down, is_positiv);
  }
}


void dcexMarkPosNeg(dcex_type dcex, dcexn n)
{
  dcexClearPosNeg(dcex);
  dcexSubMarkPosNeg(dcex, n, 1);
}

/*---------------------------------------------------------------------------*/

int dcexResetCube(dcex_type dcex, pinfo *pi, dcube *c, dcexn n, int is_posneg)
{
  dcexstr xs;
  int pos;
  
  dcInSetAll(pi, c, CUBE_IN_MASK_DC);
  b_il_Clear(dcex->pn_vars);
  
  dcexMarkVisited(dcex, n);
  if ( is_posneg != 0 )
    dcexMarkPosNeg(dcex, n);
  
  pos = -1;
  while( b_set_WhileLoop(dcex->id_set, &pos) != 0 )
  {
    xs = dcexGetDCEXSTR(dcex, pos);
    if ( xs->is_visited != 0 && xs->is_right_side != 0 )
    {
      if ( is_posneg != 0 )
      {
        if ( xs->is_positiv != 0 && xs->is_negativ != 0 )
        {
          dcSetIn(c, xs->cube_in_var_index, 1);
          if ( b_il_Add(dcex->pn_vars, xs->cube_in_var_index) < 0 )
          {
            dcexError(dcex, "%sOut of memory (b_il_Add).", dcex->error_prefix);
            return 0;
          }
        }
        else
        {
          if ( xs->is_positiv == 0 )
            dcSetIn(c, xs->cube_in_var_index, 1);
          else
            dcSetIn(c, xs->cube_in_var_index, 2);
        }
      }
      else
      {
        dcSetIn(c, xs->cube_in_var_index, 1);
        if ( b_il_Add(dcex->pn_vars, xs->cube_in_var_index) < 0 )
        {
          dcexError(dcex, "%sOut of memory (b_il_Add).", dcex->error_prefix);
          return 0;
        }
      }
    }
  }
  
  /* remove this if pn problems are corrected */
  /* dcInSetAll(pi, c, CUBE_IN_MASK_ZERO);  */

  return 1;
}

int dcexIncCube(dcex_type dcex, pinfo *pi, dcube *c)
{
  int i = 0;
  int j = 0;
  if ( b_il_GetCnt(dcex->pn_vars) == 0 )
    return 0;
  for(;;)
  {
    j = b_il_GetVal(dcex->pn_vars, i);
    /*
    j = i;
    */
    if ( dcGetIn(c, j) == 1 )
    {
      dcSetIn(c, j, 2);
      break;
    }
    else
    {
      dcSetIn(c, j, 1);
      i++;
    }
    if ( i >= b_il_GetCnt(dcex->pn_vars) )
      return 0;
    /*
    if ( i >= pi->in_cnt )
      return 0;
    */
  }
  return 1;
}

int dcexEvalInCubes(dcex_type dcex, pinfo *pi, dclist cl, dcexn n)
{
  dcube c;
  if ( dcInit(pi, &c) == 0 )
    return 0;
  dclClear(cl);

  dcex->input_cube = &c;

  if ( dcexResetCube(dcex, pi, &c, n, 1) == 0 )
    return dcDestroy(&c), 0;
    
  do
  {
    if ( dcexEvalBoole(dcex, n) != 0 )
    {
      if ( dclAdd(pi, cl, &c) < 0 )
      {
        dcexError(dcex, "%sOut of memory (dclAdd).", dcex->error_prefix);
        return dcDestroy(&c), 0;
      }
    }
  } while(dcexIncCube(dcex, pi, &c) != 0);

  dcDestroy(&c);
  return 1;
}

int dcexEvalInOutCubes(dcex_type dcex, pinfo *pi, dclist cl, dcexn n)
{
  dcube c;
  if ( dcInit(pi, &c) == 0 )
    return 0;
  dclClear(cl);

  dcex->assignment_cube = &c;
  dcex->input_cube = &c;

  if ( dcexResetCube(dcex, pi, &c, n, 0) == 0 )
    return dcex->assignment_cube = NULL, dcDestroy(&c), 0;
  
  do
  {
    dcOutSetAll(pi, &c, 0);
    dcexEvalAssignment(dcex, n);
    if ( dcIsOutIllegal(pi, &c) == 0 )
    {
      if ( dclAdd(pi, cl, &c) < 0 )
      {
        dcexError(dcex, "%sOut of memory (dclAdd).", dcex->error_prefix);
        return dcex->assignment_cube = NULL, dcDestroy(&c), 0;
      }
    }
  } while(dcexIncCube(dcex, pi, &c) != 0);

  dcex->assignment_cube = NULL;
  dcDestroy(&c);
  return 1;
  
}

int dcexEval(dcex_type dcex, pinfo *pi, dclist cl, dcexn n, int type)
{ 
  switch(type)
  {
    case DCEX_TYPE_BOOLE:
      dcexEvalInCubes(dcex, pi, cl, n);
      break;
    case DCEX_TYPE_ASSIGNMENT:
       dcexEvalInOutCubes(dcex, pi, cl, n);
      break;
  }
  return 1;
}

/*---------------------------------------------------------------------------*/

static void dcexSubShow(dcex_type dcex, dcexn n)
{
  if ( n == NULL )
    return;
    
  printf(" ( ");
  do
  {
    printf("%d ", n->data);
    if ( n->data >= DCEXN_STR_OFFSET )
      printf("[%s:%c%c%c%c] ", dcexGetDCEXSTR(dcex, n->data-DCEXN_STR_OFFSET)->str,
        dcexGetDCEXSTR(dcex, n->data-DCEXN_STR_OFFSET)->is_left_side?'L':' ',
        dcexGetDCEXSTR(dcex, n->data-DCEXN_STR_OFFSET)->is_right_side?'R':' ',
        dcexGetDCEXSTR(dcex, n->data-DCEXN_STR_OFFSET)->is_positiv?'P':' ',
        dcexGetDCEXSTR(dcex, n->data-DCEXN_STR_OFFSET)->is_negativ?'N':' '
        );
    dcexSubShow(dcex, n->down);
    n = n->next;
  } while(n != NULL);
  printf(") ");
}

void dcexShow(dcex_type dcex, dcexn n)
{
  dcexMarkPosNeg(dcex, n);
  dcexSubShow(dcex, n);
}


/*---------------------------------------------------------------------------*/

int dclReadBEXStr(pinfo *pi, dclist cl_on, dclist cl_dc, const char *content)
{
  dcex_type dcex;
  dcexn n;
  
  dclClear(cl_on);
  dclClear(cl_dc);
  
  dcex = dcexOpen();
  if ( dcex == NULL )
    return 0;
    
  n = dcexParser(dcex, content, DCEX_TYPE_ASSIGNMENT);
  
  if ( n == NULL )
    return dcexClose(dcex), 0;

  /* 2019: This block is now removed and bex format partly supports in and out variables. */
  /* For backward compatibility the following block could be uncommented. */
/*  
  if ( b_sl_GetCnt(dcex->inout_variables) != 0 )
  {
    dcexError(dcex, "%sVariable used as input and output.", dcex->error_prefix);
    return 0;
  }
*/

  n = dcexReduceNot(dcex, n);
  if ( n == NULL )
    return dcexClose(dcex), 0;

  if ( dcexToDCL(dcex, pi, cl_on, n) == 0 )
  {
    dcexError(dcex, "%sTree conversion failed.", dcex->error_prefix);
    return dcexnClose(n), dcexClose(dcex), 0;
  }
  
  return dcexnClose(n), dcexClose(dcex), 1;
}

int IsValidBEXStr(const char *content)
{
  dcex_type dcex;
  
  dcex = dcexOpen();
  if ( dcex == NULL )
    return 0;
    
  if ( dcexIsValid(dcex, content, DCEX_TYPE_ASSIGNMENT) == 0 )
    return dcexClose(dcex), 0;
    
  return dcexClose(dcex), 1;
}

/*---------------------------------------------------------------------------*/

static char *dcex_filename_to_string(const char *filename)
{
  struct stat buf;
  char *s;
  FILE *fp;
  char *content;
  
  s = b_ff(filename, NULL, ".bex");
  if ( s == NULL )
    return NULL;
  if ( stat(s, &buf) != 0 )
  {
    free(s);
    return NULL;
  }
  if ( buf.st_size > ( ((size_t)1)<<(sizeof(size_t)*8-2)) )
  {
    free(s);
    return NULL;
  }
  fp = fopen(s, "r");
  if ( fp == NULL )
  {
    free(s);
    return NULL;
  }
  free(s);
  content = (char *)malloc(buf.st_size+1);
  if ( content == NULL )
  {
    fclose(fp);
    return NULL;
  }
    
  if ( fread(content, buf.st_size, 1, fp) != 1 )
  {
    free(content);
    fclose(fp);
    return NULL;
  }
  content[buf.st_size] = '\0';
  
  fclose(fp);
  
  return content;
}

int dclReadBEX(pinfo *pi, dclist cl_on, dclist cl_dc, const char *filename)
{
  char *content = dcex_filename_to_string(filename);
  if ( content == NULL )
    return 0;
  
  if ( dclReadBEXStr(pi, cl_on, cl_dc, content) == 0 )
    return free(content), 0;
  
  return free(content), 1;
}

int IsValidBEXFile(const char *filename)
{
  char *content = dcex_filename_to_string(filename);
  if ( content == NULL )
    return 0;
  if ( IsValidBEXStr(content) == 0 )
    return free(content), 0;
  return free(content), 1;
}


/*---------------------------------------------------------------------------*/

static int dclWriteBEXInLabel(pinfo *pi, int i, FILE *fp)
{
  const char *s = pinfoGetInLabel(pi, i);
  if ( s != NULL )
    fprintf(fp, "%s", s);
  else
    fprintf(fp, "x%03d", i);
  return 1;
}

static int dclWriteBEXOutLabel(pinfo *pi, int o, FILE *fp)
{
  const char *s = pinfoGetOutLabel(pi, o);
  if ( s != NULL )
    fprintf(fp, "%s", s);
  else
    fprintf(fp, "y%03d", o);
  return 1;
}

static int dclWriteBEXCubeFP(pinfo *pi, dcube *c, int o, FILE *fp)
{
  int i;
  int s;
  int is_first = 1;
  if ( dcGetOut(c, o) == 0 )
    return 1;
  for( i = 0; i < pi->in_cnt; i++ )
  {
    s = dcGetIn(c, i);
    if ( s == 1 || s == 2 )
    {
      if ( is_first == 0 )
        fprintf(fp, " & ");
      else
        is_first = 0;
      if ( s == 1 )
      {
        fprintf(fp, "!");
      }
      dclWriteBEXInLabel(pi, i, fp);
    }
  }
  return 1;
}

static int dclWriteBEXFP(pinfo *pi, dclist cl, FILE *fp)
{
  int i, cnt = dclCnt(cl);
  int o;
  int is_first;
  
  fprintf(fp, "# generated file\n");
  
  fprintf(fp, "invars(");
  for( i = 0; i < pi->in_cnt; i++ )
  {
    dclWriteBEXInLabel(pi, i, fp);
    if ( i < pi->in_cnt-1 )
      fprintf(fp, " ");
  }
  fprintf(fp, ");\n");

  fprintf(fp, "outvars(");
  for( i = 0; i < pi->out_cnt; i++ )
  {
    dclWriteBEXOutLabel(pi, i, fp);
    if ( i < pi->out_cnt-1 )
      fprintf(fp, " ");
  }
  fprintf(fp, ");\n");


  for( o = 0; o < pi->out_cnt; o++ )
  {
    fprintf(fp, "\n");
    dclWriteBEXOutLabel(pi, o, fp);
    fprintf(fp, "\n <= ");
    is_first = 1;
    for( i = 0; i < cnt; i++ )
    {
      if ( dcGetOut(dclGet(cl, i), o) != 0 )
      {
        if ( is_first == 0 )
          fprintf(fp, "\n  | ");
        else
          is_first = 0;
        dclWriteBEXCubeFP(pi, dclGet(cl, i), o, fp);
      }
    }
    if ( is_first != 0 )
      fprintf(fp, "0");
    fprintf(fp, ";\n");
  }
  return 1;
}

int dclWriteBEX(pinfo *pi, dclist cl, const char *filename)
{
  FILE *fp;
  fp = fopen(filename, "w");
  if ( fp == NULL )
    return 0;
  dclWriteBEXFP(pi, cl, fp);
  fclose(fp);
  return 1;
}

/*---------------------------------------------------------------------------*/

static int dclWrite3DEQNInLabel(pinfo *pi, int i, FILE *fp)
{
  const char *s = pinfoGetInLabel(pi, i);
  if ( s != NULL )
    fprintf(fp, "%s", s);
  else
    fprintf(fp, "x%03d", i);
  return 1;
}

static int dclWrite3DEQNOutLabel(pinfo *pi, int o, FILE *fp)
{
  const char *s = pinfoGetOutLabel(pi, o);
  if ( s != NULL )
    fprintf(fp, "%s", s);
  else
    fprintf(fp, "y%03d", o);
  return 1;
}

static int dclWrite3DEQNCubeFP(pinfo *pi, dcube *c, int o, FILE *fp)
{
  int i;
  int s;
  if ( dcGetOut(c, o) == 0 )
    return 1;
  fprintf(fp, "  ");
  for( i = 0; i < pi->in_cnt; i++ )
  {
    s = dcGetIn(c, i);
    if ( s == 1 || s == 2 )
    {
      dclWrite3DEQNInLabel(pi, i, fp);
      if ( s == 1 )
      {
        fprintf(fp, "'");
      }
      fprintf(fp, " ");
    }
  }
  return 1;
}

static int dclWrite3DEQNFP(pinfo *pi, dclist cl, FILE *fp)
{
  int i, j, cnt = dclCnt(cl);
  int o;
  
  for( o = 0; o < pi->out_cnt; o++ )
  {
    dclWrite3DEQNOutLabel(pi, o, fp);
    fprintf(fp, " =\n");
    for( i = 0; i < cnt; i++ )
    {
      if ( dcGetOut(dclGet(cl, i), o) != 0 )
      {
        dclWrite3DEQNCubeFP(pi, dclGet(cl, i), o, fp);
        for( j = i+1; j < cnt; j++ )
          if ( dcGetOut(dclGet(cl, j), o) != 0 )
            break;
        if ( j != cnt )
          fprintf(fp, "+\n");
        else
          fprintf(fp, "\n");
      }
    }
    fprintf(fp, "\n");
  }
  return 1;
}

int dclWrite3DEQN(pinfo *pi, dclist cl, const char *filename)
{
  FILE *fp;
  fp = fopen(filename, "w");
  if ( fp == NULL )
    return 0;
  dclWrite3DEQNFP(pi, cl, fp);
  fclose(fp);
  return 1;
}

