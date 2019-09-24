/*

  b_evel.c
  
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



/*---------------------------------------------------------------------------*/

#include <stdlib.h>
#include <string.h>
#include "b_eval.h"

b_en_type b_en_Open()
{
  b_en_type en;
  en = (b_en_type)malloc(sizeof(struct _b_en_struct));
  if ( en != NULL )
  {
    memset(en, 0, sizeof(struct _b_en_struct));
    return en;
  }
  return NULL;
}

void b_en_Close(b_en_type en)
{
  if ( en == NULL ) 
    return;
  b_en_Close(en->left);
  b_en_Close(en->right);
  free(en);
}

/*---------------------------------------------------------------------------*/

b_ev_type b_ev_Open(const char *str)
{
  b_ev_type ev;
  ev = (b_ev_type)malloc(sizeof(struct _b_ev_struct));
  if ( ev != NULL )
  {
    ev->str = strdup(str);
    if ( ev->str != NULL )
    {
      ev->val = 0;
      return ev;
    }
    free(ev);
  }
  return NULL;
}

void b_ev_Close(b_ev_type ev)
{
  free(ev->str);
  free(ev);
}

/*---------------------------------------------------------------------------*/

b_en_type b_eval_ParseTop(b_eval_type e);

int b_eval_FindEV(b_eval_type e, const char *str)
{
  int loop = -1;
  b_ev_type ev;
  while( b_set_WhileLoop(e->variables, &loop) != 0 )
  {
    ev = (b_ev_type)b_set_Get(e->variables, loop);
    if ( strcmp(ev->str, str) == 0 )
      return loop;
  }
  return -1;
}

int b_eval_AddEV(b_eval_type e, const char *str)
{
  int pos;
  b_ev_type ev;
  
  pos = b_eval_FindEV(e, str);
  if ( pos >= 0 )
    return pos;
  ev = b_ev_Open(str);
  if ( ev == NULL )
    return -1;
  pos = b_set_Add(e->variables, ev);
  if ( pos < 0 )
    return b_ev_Close(ev), 0;
  return pos;
}


b_ev_type b_eval_GetEV(b_eval_type e, int pos)
{
  return (b_ev_type)b_set_Get(e->variables, pos);
}

int b_eval_SetValByName(b_eval_type e, const char *name, int val)
{
  int pos;
  b_ev_type ev;
  
  pos = b_eval_FindEV(e, name);
  if ( pos < 0 )
    return 0;
  
  ev = b_eval_GetEV(e, pos);
  ev->val = val;
  return 1;
}

int b_eval_GetValByName(b_eval_type e, const char *name)
{
  int pos;
  b_ev_type ev;
  
  pos = b_eval_FindEV(e, name);
  if ( pos < 0 )
    return 0;
  
  ev = b_eval_GetEV(e, pos);
  return ev->val;
}

b_eval_type b_eval_Open()
{
  b_eval_type e;
  e = (b_eval_type)malloc(sizeof(struct _b_eval_struct));
  if ( e != NULL )
  {
    e->variables = b_set_Open();
    if ( e->variables != NULL )
    {
      return e;
    }
    free(e);
  }
  return NULL;
}

void b_eval_Close(b_eval_type e)
{
  int loop = -1;
  while( b_set_WhileLoop(e->variables, &loop) != 0 )
    b_ev_Close( b_eval_GetEV(e, loop) );
  b_set_Close(e->variables);
  free(e);
}



void b_eval_SkipSpace(b_eval_type e)
{
  while ( *(e->s) > '\0' && *(e->s) <= ' ' )
    e->s++;
  return;
}

int b_eval_Compare(b_eval_type e, const char *s)
{
  const char *t = e->s;
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
  e->s = t;
  b_eval_SkipSpace(e);
  return 1;
}

b_en_type b_eval_NewIdNode(b_eval_type e, const char *id)
{
  b_en_type arg;
  arg = b_en_Open();
  if ( arg == NULL )
    return NULL;
  arg->type = B_EVAL_ID;
  arg->data = b_eval_AddEV(e, id);
  if ( arg->data >= 0 )
    return arg;
  b_en_Close(arg);
  return NULL;
}

b_en_type b_eval_NewValNode(b_eval_type e, int val)
{
  b_en_type arg;
  arg = b_en_Open();
  if ( arg == NULL )
    return NULL;
  arg->type = B_EVAL_VAL;
  arg->data = val;
  return arg;
}

b_en_type b_eval_NewLNode(b_eval_type e, b_en_type (*left)(b_eval_type e), int type)
{
  b_en_type parent = b_en_Open();
  if ( parent == NULL )
    return NULL;
  parent->type = type;
  parent->left = left(e);
  parent->right = NULL;
  if ( parent->left == NULL )
  {
    b_en_Close(parent);
    return NULL;
  }
  return parent;
}

b_en_type b_eval_NewLRNode(b_eval_type e, b_en_type left, b_en_type (*right)(b_eval_type e), int type)
{
  b_en_type parent = b_en_Open();
  if ( parent == NULL )
    return NULL;
  parent->type = type;
  parent->data = 0;
  parent->left = left;
  parent->right = right(e);
  if ( parent->left == NULL || parent->right == NULL )
  {
    b_en_Close(parent);
    return NULL;
  }
  return parent;
}

b_en_type b_eval_ParseBinaryOp(b_eval_type e, b_en_type (*fn)(b_eval_type e), const char *op, int type)
{
  b_en_type arg = fn(e);
  if ( b_eval_Compare(e, op) != 0 )
    return b_eval_NewLRNode(e, arg, fn, type);
  return arg;
}

b_en_type b_eval_ParseBinaryOp2(b_eval_type e, b_en_type (*fn)(b_eval_type e), const char *op1, int type1, const char *op2, int type2)
{
  b_en_type arg = fn(e);
  if ( b_eval_Compare(e, op1) != 0 )
    return b_eval_NewLRNode(e, arg, fn, type1);
  else if ( b_eval_Compare(e, op2) != 0 )
    return b_eval_NewLRNode(e, arg, fn, type2);
  return arg;
}

b_en_type b_eval_ParseBinaryOp4(b_eval_type e, b_en_type (*fn)(b_eval_type e), const char *op1, int type1, const char *op2, int type2, const char *op3, int type3, const char *op4, int type4)
{
  b_en_type arg = fn(e);
  if ( b_eval_Compare(e, op1) != 0 )
    return b_eval_NewLRNode(e, arg, fn, type1);
  else if ( b_eval_Compare(e, op2) != 0 )
    return b_eval_NewLRNode(e, arg, fn, type2);
  else if ( b_eval_Compare(e, op3) != 0 )
    return b_eval_NewLRNode(e, arg, fn, type3);
  else if ( b_eval_Compare(e, op4) != 0 )
    return b_eval_NewLRNode(e, arg, fn, type4);
  return arg;
}

b_en_type b_eval_ParseUnaryOp(b_eval_type e, b_en_type (*fn)(b_eval_type e), const char *op, int type)
{
  if ( b_eval_Compare(e, op) != 0 )
    return b_eval_NewLNode(e, fn, type);
  return fn(e);
}

static int b_eval_IsSymF(b_eval_type e)
{
  if ( *(e->s) >= 'a' && *(e->s) <= 'z' )
    return 1;
  if ( *(e->s) >= 'A' && *(e->s) <= 'Z' )
    return 1;
  if ( *(e->s) == '_' )
    return 1;
  return 0;
}

static int b_eval_IsDigit(b_eval_type e)
{
  if ( *(e->s) >= '0' && *(e->s) <= '9' )
    return 1;
  return 0;
}

static int b_eval_IsSym(b_eval_type e)
{
  if ( b_eval_IsDigit(e) != 0 )
    return 1;
  return b_eval_IsSymF(e);
}

static int b_eval_IsEnd(b_eval_type e)
{
  if ( *(e->s) == '\0' )
    return 1;
  return 0;
}


const char *b_eval_ParseIdentifier(b_eval_type e)
{
  static char identifier[256];
  int i = 0;
  if ( b_eval_IsSymF(e) != 0 )
  {
    do
    { 
      if ( i < 256-1 )
      {
        identifier[i] = *(e->s);
        i++;
      }
      e->s++;
    } while(b_eval_IsSym(e) != 0);
  }
  identifier[i] = '\0';
  b_eval_SkipSpace(e);
  return identifier;
}

int b_eval_ParseNumber(b_eval_type e)
{
  int val = 0;
  while( b_eval_IsDigit(e) != 0 )
  {
    val = val * 10;
    val += *(e->s) - '0';
    e->s++;
  }
  b_eval_SkipSpace(e);
  return val;
}

void b_eval_Error(b_eval_type e, const char *s)
{
  puts(s);
}

b_en_type b_eval_ParseList(b_eval_type e, int type)
{
  b_en_type arg, parent;
  if ( b_eval_Compare(e, "(") == 0 )
    return NULL;
  parent = b_eval_NewLNode(e, b_eval_ParseTop, type);
  if ( parent == NULL )
    return NULL;
  arg = parent;
  while(b_eval_Compare(e, ",") != 0)
  {
    arg->right = b_eval_NewLNode(e, b_eval_ParseTop, type);
    if ( arg->right == NULL )
    {
      b_en_Close(parent);
      return NULL;
    }
    arg = arg->right;
  }
  if ( b_eval_Compare(e, ")") == 0 )
  {
    b_en_Close(parent);
    return NULL;
  }
  return parent;
}


b_en_type b_eval_ParseIf(b_eval_type e)
{ 
  b_en_type arg = b_eval_ParseList(e, B_EVAL_IF);
  if ( arg == NULL || arg->right == NULL || arg->right->right == NULL )
  {
    b_eval_Error(e, "Syntax error (if).");
    b_en_Close(arg);
    return NULL;
  }
  return arg;
}


b_en_type b_eval_ParseAtom(b_eval_type e)
{
  b_en_type arg;
  if ( b_eval_Compare(e, "(") != 0 )
  {
    arg = b_eval_ParseTop(e);
    if ( b_eval_Compare(e, ")") == 0 )
    {
      b_en_Close(arg);
      return NULL;
    }
    return arg;
  }
  if ( b_eval_IsSymF(e) != 0 )
  {
    const char *id = b_eval_ParseIdentifier(e);
    if ( strcmp(id, "if") == 0 )
      return b_eval_ParseIf(e);
    return b_eval_NewIdNode(e, id);
  }
  if ( b_eval_IsDigit(e) != 0 )
  {
    return b_eval_NewValNode(e, b_eval_ParseNumber(e));
  }
  b_eval_Error(e, "Syntax error.");
  /* static int b_eval_IsDigit(b_eval_type e) */
  return NULL;
}

b_en_type b_eval_ParseNeg(b_eval_type e)
{
  return b_eval_ParseUnaryOp(e, b_eval_ParseAtom, "-", B_EVAL_NEG);
}

b_en_type b_eval_ParseShift(b_eval_type e)
{
  return b_eval_ParseBinaryOp2(e, b_eval_ParseNeg, "<<", B_EVAL_LSHIFT, ">>", B_EVAL_RSHIFT);
}

b_en_type b_eval_ParseMulDiv(b_eval_type e)
{
  return b_eval_ParseBinaryOp2(e, b_eval_ParseShift, "*", B_EVAL_MUL, "/", B_EVAL_DIV);
}

b_en_type b_eval_ParseAddSub(b_eval_type e)
{
  return b_eval_ParseBinaryOp2(e, b_eval_ParseMulDiv, "+", B_EVAL_ADD, "-", B_EVAL_SUB);
}

b_en_type b_eval_ParseCompare(b_eval_type e)
{
  return b_eval_ParseBinaryOp4(e, b_eval_ParseAddSub, 
    "<=", B_EVAL_LOEQ,
    ">=", B_EVAL_HIEQ,
    "<", B_EVAL_LO, 
    ">", B_EVAL_HI
    );
}

b_en_type b_eval_ParseAssign(b_eval_type e)
{
  return b_eval_ParseBinaryOp(e, b_eval_ParseCompare, "=", B_EVAL_ASSIGN);
}

b_en_type b_eval_ParseTop(b_eval_type e)
{
  return b_eval_ParseAssign(e);
}

b_en_type b_eval_ParseStatement(b_eval_type e)
{
  b_en_type arg = b_eval_ParseTop(e);
  if ( b_eval_Compare(e, ";") != 0 )
  {
    while ( b_eval_Compare(e, ";") != 0 )
      ;
    if ( b_eval_IsEnd(e) != 0 )
      return arg;
    return b_eval_NewLRNode(e, arg, b_eval_ParseStatement, B_EVAL_STATEMENT);
  }
  return arg;
}


b_en_type b_eval_ParseStr(b_eval_type e, const char *s)
{
  if ( s == NULL )
    return NULL;
  e->s = s;
  b_eval_SkipSpace(e);
  return b_eval_ParseStatement(e);
}

void b_eval_Show(b_eval_type e, b_en_type en)
{
  if ( en == NULL )
    return;
  printf(" ");
  if ( en->type == B_EVAL_ID )
  {
    printf("%s", b_eval_GetEV(e, en->data)->str);
  }
  else
  {
    if ( en->type < 128 )
      printf("(%c", en->type);
    else
      printf("(%d", en->type);
    b_eval_Show(e, en->left);
    b_eval_Show(e, en->right);
    printf(")");
  }
}

int _b_eval_Eval(b_eval_type e, b_en_type en)
{
  if ( en == NULL )
    return 0;
  switch(en->type)
  {
    case B_EVAL_LSHIFT:
      return b_eval_Eval(e, en->left)<<b_eval_Eval(e, en->right);
    case B_EVAL_RSHIFT:
      return b_eval_Eval(e, en->left)>>b_eval_Eval(e, en->right);
    case B_EVAL_LO:
      return b_eval_Eval(e, en->left)<b_eval_Eval(e, en->right);
    case B_EVAL_HI:
      return b_eval_Eval(e, en->left)>b_eval_Eval(e, en->right);
    case B_EVAL_LOEQ:
      return b_eval_Eval(e, en->left)<=b_eval_Eval(e, en->right);
    case B_EVAL_HIEQ:
      return b_eval_Eval(e, en->left)>=b_eval_Eval(e, en->right);
    case B_EVAL_MUL:
      return b_eval_Eval(e, en->left)*b_eval_Eval(e, en->right);
    case B_EVAL_DIV:
      return b_eval_Eval(e, en->left)/b_eval_Eval(e, en->right);
    case B_EVAL_ADD:
      return b_eval_Eval(e, en->left)+b_eval_Eval(e, en->right);
    case B_EVAL_SUB:
      return b_eval_Eval(e, en->left)+b_eval_Eval(e, en->right);
    case B_EVAL_NEG:
      return -b_eval_Eval(e, en->left);
    case B_EVAL_IF:
      if ( b_eval_Eval(e, en->left) != 0 )
        return b_eval_Eval(e, en->right->left);
      return b_eval_Eval(e, en->right->right->left);
    case B_EVAL_ASSIGN:
      if ( en->left == NULL )
        return b_eval_Error(e, "Empty left side of assignment."), 0;
      if ( en->left->type != B_EVAL_ID )
        return b_eval_Error(e, "Left side of assignment is not a variable."), 0;
      return b_eval_GetEV(e, en->left->data)->val = b_eval_Eval(e, en->right);
    case B_EVAL_ID:
      return b_eval_GetEV(e, en->data)->val;
    case B_EVAL_VAL:
      return en->data;
    case B_EVAL_STATEMENT:
      return b_eval_Eval(e, en->left), b_eval_Eval(e, en->right);
    default:
      b_eval_Error(e, "Unknown node type.");
      /* b_eval_Show(e, en); */
      break;
  }
  return 0;
}

int b_eval_Eval(b_eval_type e, b_en_type en)
{
  /*
  b_eval_Show(e, en);
  puts("");
  */
  return _b_eval_Eval(e, en);
}


#ifdef B_EVAL_MAIN
int main()
{
  b_eval_type e = b_eval_Open();
  if ( e != NULL )
  {
    b_en_type en = b_eval_ParseStr(e, "y = a+b*c; y2 = a*b;; ; b ; c;");
    b_eval_Show(e, en);
    puts("");
    b_en_Close(en);
  }
  return 0;
}

#endif
