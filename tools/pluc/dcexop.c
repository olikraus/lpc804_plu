/*

  dcexop.c
  
  additional operations for the dcex modul
  
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

  
  At the moment there are two important functions here:

    dcexn dcexReduceNot(dcex_type dcex, dcexn n)
      will try to reduce the number of boolean complements (not's) for
      the tree. This is done by trying to move complements to the leafs.
      Typical use is 
      n = dcexReduceNot(dcex, n)
      If an error occured, dcexReduceNot returns NULL and discards 'n'!
      
    int dcexToDCL(dcex_type dcex, pinfo *pi, dclist cl, dcexn n)
      Converts a tree (given by n) into a dclist. The number of
      input and output variables and their lables are applied to 'pi'.
      It is a good idea to call dcexReduceNot on 'n' before this
      function.
          
*/


#include "dcex.h"

void dcexnCloseNode(dcexn n);
void dcexError(dcex_type dcex, char *fmt, ...);

/* invert a subtree */
static dcexn dcexOpenNot(dcex_type dcex, dcexn n)
{
  dcexn nn;
  nn = dcexnOpen();
  if ( nn != NULL )
  {
    nn->data = DCEXN_NOT;
    nn->down = n;
    return nn;
  }
  dcexnClose(n);
  dcexError(dcex, "%sOut of memory for 'RecuceNot'.", dcex->error_prefix);
  return NULL;
}

/* process all childs, possible inverting them */
static dcexn dcexProcessList(dcex_type dcex, dcexn n, int is_not)
{
  dcexn next;
  dcexn *ptr;
  
  ptr = &(n->down);
  while( (*ptr) != NULL )
  {
    next = (*ptr)->next;
    (*ptr)->next = NULL;
    if ( is_not != 0 ) 
      (*ptr) = dcexReduceNot(dcex, dcexOpenNot(dcex, (*ptr)));
    else
      (*ptr) = dcexReduceNot(dcex, (*ptr));
    if ( (*ptr) == NULL )
    {
      dcexnClose(n);
      dcexnClose(next);
      dcexError(dcex, "%sOut of memory for 'RecuceNot'.", dcex->error_prefix);
      return NULL;
    }
    (*ptr)->next = next;
    ptr = &((*ptr)->next);
  }
  return n;
}

/* remove double inversion and apply DeMorgan's rule if possible */
dcexn dcexReduceNot(dcex_type dcex, dcexn n)
{
  if ( n == NULL )
    return NULL;
  if ( n->data == DCEXN_NOT )
  {
    switch(n->down->data)
    {
      case DCEXN_ONE:
        n->data = DCEXN_ZERO;
        n->down = NULL;
        dcexnCloseNode(n->down);
        return n;
      case DCEXN_ZERO:
        n->data = DCEXN_ONE;
        n->down = NULL;
        dcexnCloseNode(n->down);
        return n;
      case DCEXN_NOT:
        {
          dcexn nn;
          nn = n->down->down;
          dcexnCloseNode(n->down);
          dcexnCloseNode(n);
          return dcexReduceNot(dcex, nn);
        }
      case DCEXN_OR:
        {
          dcexn nn;
          nn = n->down;
          nn->data = DCEXN_AND;
          dcexnCloseNode(n);
          return dcexProcessList(dcex, nn, 1);
        }
      case DCEXN_AND:
        {
          dcexn nn;
          nn = n->down;
          nn->data = DCEXN_OR;
          dcexnCloseNode(n);
          return dcexProcessList(dcex, nn, 1);
        }
      case DCEXN_EQ:
      case DCEXN_NEQ:
      case DCEXN_ASSIGN:
      case DCEXN_CMDLIST:
      default:
        n->down = dcexReduceNot(dcex, n->down);
        if ( n->down == NULL )
        {
          dcexnClose(n);
          return NULL;
        }
        break;
    }
  }
  else
  {
    n = dcexProcessList(dcex, n, 0);
  }
  return n;
}


static int _dcexToDCL(dcex_type dcex, dcexn n, dclist cl);
dcexstr dcexGetDCEXSTR(dcex_type dcex, int pos);

static int _dclOR(pinfo *pi, dclist a, dclist b)
{
  /* hmmm... that's quite easy :-) */
  if ( dclSCCUnion(pi, a, b) == 0 )     /* merge b and a into a */
    return 0;
  return 1;
}

static int _dcexToDCL_OR(dcex_type dcex, dcexn n, dclist cl)
{
  dclist cl_next;
  if ( dclInitCached(dcex->pi_to_dcl, &cl_next) == 0 )
    return 0;
  if ( _dcexToDCL(dcex, n, cl) == 0 )
    return dclDestroyCached(dcex->pi_to_dcl, cl_next), 0;
  while( n->next != NULL )
  {
    if ( _dcexToDCL(dcex, n->next, cl_next) == 0 )
      return dclDestroyCached(dcex->pi_to_dcl, cl_next), 0;
    if ( _dclOR(dcex->pi_to_dcl, cl, cl_next) == 0 )
    {
      dcexError(dcex, "%sOR operation failed.", dcex->error_prefix);
      return dclDestroyCached(dcex->pi_to_dcl, cl_next), 0;
    }
    n = n->next;
  }
  return dclDestroyCached(dcex->pi_to_dcl, cl_next), 1;
}

static int _dclAND(pinfo *pi, dclist a, dclist b)
{
  dclist c;
  if ( dclInitCached(pi, &c) == 0 )
    return 0;
  if ( dclIntersectionList(pi, c, a, b) == 0 )
    return dclDestroyCached(pi, c), 0;
  if ( dclCopy(pi, a, c) == 0 )
    return dclDestroyCached(pi, c), 0;
  return dclDestroyCached(pi, c), 1;
}

static int _dcexToDCL_AND(dcex_type dcex, dcexn n, dclist cl)
{
  dclist cl_next;
  if ( dclInitCached(dcex->pi_to_dcl, &cl_next) == 0 )
    return 0;
  if ( _dcexToDCL(dcex, n, cl) == 0 )
    return dclDestroyCached(dcex->pi_to_dcl, cl_next), 0;
  while( n->next != NULL )
  {
    if ( _dcexToDCL(dcex, n->next, cl_next) == 0 )
      return dclDestroyCached(dcex->pi_to_dcl, cl_next), 0;
    if ( _dclAND(dcex->pi_to_dcl, cl, cl_next) == 0 )
    {
      dcexError(dcex, "%sAND operation failed.", dcex->error_prefix);
      return dclDestroyCached(dcex->pi_to_dcl, cl_next), 0;
    }
    n = n->next;
  }
  return dclDestroyCached(dcex->pi_to_dcl, cl_next), 1;
}

/* a == b <=> a&b | !a & !b */
static int _dclEQ(pinfo *pi, dclist a, dclist b)
{
  dclist c;
  
  if ( dclInitCached(pi, &c) == 0 )
    return 0;
  if ( dclIntersectionList(pi, c, a, b) == 0 )
    return dclDestroyCached(pi, c), 0;
  if ( dclComplementWithSharp(pi, a) == 0 )
    return dclDestroyCached(pi, c), 0;
  if ( dclComplementWithSharp(pi, b) == 0 )
    return dclDestroyCached(pi, c), 0;
  if ( _dclAND(pi, a, b) == 0 )          /* result is in 'a' */
    return dclDestroyCached(pi, c), 0;
  if ( dclSCCUnion(pi, a, c) == 0 )     /* merge a and c into a */
    return dclDestroyCached(pi, c), 0;
  
  return dclDestroyCached(pi, c), 1;
}

static int _dcexToDCL_EQ(dcex_type dcex, dcexn n, dclist cl)
{
  dclist cl_next;
  if ( dclInitCached(dcex->pi_to_dcl, &cl_next) == 0 )
    return 0;
  if ( _dcexToDCL(dcex, n, cl) == 0 )
    return dclDestroyCached(dcex->pi_to_dcl, cl_next), 0;
  while( n->next != NULL )
  {
    if ( _dcexToDCL(dcex, n->next, cl_next) == 0 )
      return dclDestroyCached(dcex->pi_to_dcl, cl_next), 0;
    if ( _dclEQ(dcex->pi_to_dcl, cl, cl_next) == 0 )
    {
      dcexError(dcex, "%sEQ operation failed.", dcex->error_prefix);
      return dclDestroyCached(dcex->pi_to_dcl, cl_next), 0;
    }
    n = n->next;
  }
  return dclDestroyCached(dcex->pi_to_dcl, cl_next), 1;
}

/* a != b <=> a&!b | !a & b */
static int _dclNEQ(pinfo *pi, dclist a, dclist b)
{
  dclist an, bn;
  
  if ( dclInitCachedVA(pi, 2, &an, &bn) == 0 )
    return 0;
  if ( dclCopy(pi, an, a) == 0 )
    return dclDestroyCachedVA(pi, 2, an, bn), 0;
  if ( dclCopy(pi, bn, b) == 0 )
    return dclDestroyCachedVA(pi, 2, an, bn), 0;
  if ( dclComplementWithSharp(pi, an) == 0 )
    return dclDestroyCachedVA(pi, 2, an, bn), 0;
  if ( dclComplementWithSharp(pi, bn) == 0 )
    return dclDestroyCachedVA(pi, 2, an, bn), 0;
  if ( _dclAND(pi, an, b) == 0 )          /* result is in 'an' */
    return dclDestroyCachedVA(pi, 2, an, bn), 0;
  if ( _dclAND(pi, a, bn) == 0 )          /* result is in 'a' */
    return dclDestroyCachedVA(pi, 2, an, bn), 0;
  if ( dclSCCUnion(pi, a, an) == 0 )     /* merge an and a into a */
    return dclDestroyCachedVA(pi, 2, an, bn), 0;
  return dclDestroyCachedVA(pi, 2, an, bn), 1;
}

static int _dcexToDCL_NEQ(dcex_type dcex, dcexn n, dclist cl)
{
  dclist cl_next;
  if ( dclInitCached(dcex->pi_to_dcl, &cl_next) == 0 )
    return 0;
  if ( _dcexToDCL(dcex, n, cl) == 0 )
    return dclDestroyCached(dcex->pi_to_dcl, cl_next), 0;
  while( n->next != NULL )
  {
    if ( _dcexToDCL(dcex, n->next, cl_next) == 0 )
      return dclDestroyCached(dcex->pi_to_dcl, cl_next), 0;
    if ( _dclNEQ(dcex->pi_to_dcl, cl, cl_next) == 0 )
    {
      dcexError(dcex, "%sNEQ operation failed.", dcex->error_prefix);
      return dclDestroyCached(dcex->pi_to_dcl, cl_next), 0;
    }
    n = n->next;
  }
  return dclDestroyCached(dcex->pi_to_dcl, cl_next), 1;
}

static int _dcexToDCL_assign(dcex_type dcex, dcexn n, dclist cl)
{
  int i, cnt;
  dcexstr xs;
  if ( n->data < DCEXN_STR_OFFSET )
  {
    dcexError(dcex, "%sThe left side of an assignment must be a legal identifier.", dcex->error_prefix);
    return 0;
  }
  if ( n->next == NULL )
  {
    dcexError(dcex, "%sIllegal right side of an assignment.", dcex->error_prefix);
    return 0;
  }
  xs = dcexGetDCEXSTR(dcex, n->data-DCEXN_STR_OFFSET);
  if ( xs->cube_out_var_index < 0 )
  {
    dcexError(dcex, "%sThe left side (%s) of the assignment has no legal target.", dcex->error_prefix, xs->str);
    return 0;
  }
  if ( _dcexToDCL(dcex, n->next, cl) == 0 )
    return 0;
    
  cnt = dclCnt(cl);
  for( i = 0; i < cnt; i++ )
  {
    dcOutSetAll(dcex->pi_to_dcl, dclGet(cl, i), 0);
    dcSetOut(dclGet(cl, i), xs->cube_out_var_index, 1);
  }
  
  return 1;
}


static int _dcexToDCL_cmdlist(dcex_type dcex, dcexn n, dclist cl)
{
  if ( n == NULL )
    return 1;
  return _dcexToDCL_OR(dcex, n, cl);
}

static int _dcexToDCL(dcex_type dcex, dcexn n, dclist cl)
{
  if ( n == NULL )
    return 1;
  switch(n->data)
  {
    case DCEXN_ONE:
      dclClear(cl);
      if ( dclAdd(dcex->pi_to_dcl, cl, &(dcex->pi_to_dcl->tmp[0])) < 0 )
      {
        dcexError(dcex, "%sONE operation failed.", dcex->error_prefix);
        return 0;
      }
      return 1;
    case DCEXN_ZERO:
      dclClear(cl);
      return 0;
    case DCEXN_NOT:
      if ( _dcexToDCL(dcex, n->down, cl) == 0 )
        return 0;
      dclComplementWithSharp(dcex->pi_to_dcl, cl);
      return 1;
    case DCEXN_EQ:
      return _dcexToDCL_EQ(dcex, n->down, cl);
    case DCEXN_NEQ:
      return _dcexToDCL_NEQ(dcex, n->down, cl);
    case DCEXN_OR:
      return _dcexToDCL_OR(dcex, n->down, cl);
    case DCEXN_AND:
      return _dcexToDCL_AND(dcex, n->down, cl);
    case DCEXN_ASSIGN:
      return _dcexToDCL_assign(dcex, n->down, cl);
    case DCEXN_CMDLIST:
      return _dcexToDCL_cmdlist(dcex, n->down, cl);
    case DCEXN_NOP:
      return 1;
    default:
      if ( n->data >= DCEXN_STR_OFFSET )
      {
        dcexstr xs;
        int pos;
        xs = dcexGetDCEXSTR(dcex, n->data-DCEXN_STR_OFFSET);
        if ( xs->cube_in_var_index < 0 )
        {
	  /* 2019: is this the correct error message???? */
          dcexError(dcex, "%sThe left [?] side (%s) of the assignment has no legal target.", dcex->error_prefix, xs->str);
          return 0;
        }
        dclClear(cl);
        pos = dclAddEmpty(dcex->pi_to_dcl, cl);
        if ( pos < 0 )
        {
          dcexError(dcex, "%sVARIABLE operation failed.", dcex->error_prefix);
          return 0;
        }
        dcInSetAll(dcex->pi_to_dcl, dclGet(cl, pos), CUBE_IN_MASK_DC);
        dcOutSetAll(dcex->pi_to_dcl, dclGet(cl, pos), CUBE_OUT_MASK);
        dcSetIn(dclGet(cl, pos), xs->cube_in_var_index, 2);
        return 1;
      }
      else
      {
        dcexError(dcex, "%sIllegal node (dcexToDCL)", dcex->error_prefix);
      }
      break;
  }
  return 0;
}

int dcexToDCL(dcex_type dcex, pinfo *pi, dclist cl, dcexn n)
{
  if ( pinfoSetInCnt(pi, b_sl_GetCnt(dcex->in_variables)) == 0 )
    return dcexnClose(n), dcexClose(dcex), 0;

  if ( pinfoSetOutCnt(pi, b_sl_GetCnt(dcex->out_variables)) == 0 )
    return dcexnClose(n), dcexClose(dcex), 0;

  dclClear(cl);
  
  dcex->pi_to_dcl = pi;
  
  _dcexToDCL(dcex, n, cl);
  
  dcex->pi_to_dcl = NULL;

  if ( pinfoCopyInLabels(pi, dcex->in_variables) == 0 )
    return 0;

  if ( pinfoCopyOutLabels(pi, dcex->out_variables) == 0 )
    return 0;

  return 1;
}

