/*

  b_bcp.c

  binate cover algorithm

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
  
  
  [1] Chris J. Myers, "Asynchronous Circuit Design", 2001
  
  [2] Olivier Coudert, "Two-level logic minimization: an overview". 1994
  
  [3] James F. Gimpel, "A reduction technique for prime implicant tables", 1965
  
  
  log:
      4   b_bcp_Do()
      2   bcm_SetVariable
      3   b_bcp_check_better_result
      3   b_bcp_do (lower bound)
      2   bcm_ReduceCols
      2   bcm_ReduceEssentialColumn
      2   bcm_ChooseBranchColumn
      1   bcm_ChooseBranchColumn
      
  
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <assert.h>
#include "b_bcp.h"

/* #define BCM_LOG */


/*---------------------------------------------------------------------------*/

bcs_type bcs_Open(int cnt)
{
  bcs_type s;
  s = (bcs_type)malloc(sizeof(struct _bcs_struct));
  if ( s != NULL )
  {
    s->sel = (int *)malloc(cnt*sizeof(int));
    if ( s->sel != NULL )
    {
      s->cnt = cnt;
      return s;
    }
    free(s);
  }
  return NULL;
}

void bcs_Close(bcs_type s)
{
  free(s->sel);
  free(s);
}

void bcs_Copy(bcs_type dest, bcs_type src)
{
  assert(dest->cnt == src->cnt);
  memcpy(dest->sel, src->sel, dest->cnt*sizeof(int));
}

bcs_type bcs_OpenCopy(bcs_type src)
{
  bcs_type dest = bcs_Open(src->cnt);
  if ( dest == NULL )
    return NULL;
  bcs_Copy(dest, src);
  return dest;
}

/* val: BCN_VAL_xxx */
void bcs_Set(bcs_type s, int pos, int val)
{
  s->sel[pos] = val;
}

int bcs_GetVal(bcs_type s, int pos)
{
  if ( pos < 0 || pos >= s->cnt )
    return BCN_VAL_ZERO;
  return s->sel[pos];
}

int bcs_GetCnt(bcs_type s, int val)
{
  int i;
  int cnt = 0;
  for( i = 0; i < s->cnt; i++ )
    if ( s->sel[i] == val )
      cnt++;
  return cnt;
}

void bcs_Fill(bcs_type s, int val)
{
  int i;
  for( i = 0; i < s->cnt; i++ )
    s->sel[i] = val;
}

/*---------------------------------------------------------------------------*/

bcn_type bcn_Open(b_bcp_type bc)
{
  bcn_type n;
  n = (bcn_type)malloc(sizeof(struct _bcn_struct));
  if ( n != NULL )
  {
    n->l[0] = NULL;   /* left: horizontal+previous */
    n->l[1] = NULL;   /* up: vertical+previous */
    n->l[2] = NULL;   /* right: horizontal+next */
    n->l[3] = NULL;   /* down: vertical+next */
    n->xy[0] = -1;    /* x */
    n->xy[1] = -1;    /* y */
    n->val = BCN_VAL_NONE;
    n->copy = NULL;
    return n;
  }
  return NULL;
}

int bcn_OpenCopy(b_bcp_type bc, bcn_type n)
{
  //bcn_type nn;
  n->copy = bcn_Open(bc);
  if ( n->copy == NULL )
    return 0;
  *(n->copy) = *n;
  n->copy->copy = NULL;
  return 1;
}

void bcn_Close(b_bcp_type bc, bcn_type n)
{
  free(n);
}

#define P(n, v) ((n)->l[(v)])
#define N(n, v) ((n)->l[(v)+2])
#define XY(n, v) ((n)->xy[v])

/* delete a node from a double linked list */
/* v = 0: horizontal, x-direction */
/* v = 1: vertical, y-direction */
void bcn_dl_delete(bcn_type n, int v)
{
  if ( P(n, v) != NULL )   N(P(n, v), v) = N(n, v);
  if ( N(n, v) != NULL )   P(N(n, v), v) = P(n, v);
}

/* insert node 'new' after node 'n', form a double linked list */
/* v = 0: horizontal, x-direction */
/* v = 1: vertical, y-direction */
void bcn_dl_insert_after(bcn_type n, bcn_type new, int v)
{
                           P(new, v)     = n;
                           N(new, v)     = N(n, v);
  if ( N(n, v) != NULL )   P(N(n, v), v) = new;
                           N(n, v)       = new;
}

/* insert node 'new' before node 'n', form a double linked list */
/* v = 0: horizontal, x-direction */
/* v = 1: vertical, y-direction */
void bcn_dl_insert_before(bcn_type n, bcn_type new, int v)
{
                           P(new, v)     = P(n, v);
                           N(new, v)     = n;
  if ( P(n, v) != NULL )   N(P(n, v), v) = new;
                           P(n, v)       = new;
}

/* find node with position p (x or y) */
/* ignores first element */
/* v = 0: horizontal search, compare x-value 'p' */
/* v = 1: vertical search, compare y-value 'p' */
/* never returns a NULL element */
/* returns the last element with XY() < p */
bcn_type bcn_dl_find(bcn_type n, int v, int p)
{
  bcn_type nn;
  assert( n != NULL );
  assert( XY(n, v) < p );
  for(;;)
  {
    nn = N(n, v);
    if ( nn == NULL || XY(nn, v) > p )
      return n;
    n = nn;
  }
}

/* return number of elements after 'n' */
int bcn_dl_get_cnt(bcn_type n, int v)
{ 
  int cnt = -1;
  while( n != NULL )
  {
    cnt++;
    n = N(n, v);
  }
  return cnt;
}

bcn_type bcn_first(bcn_type n, int v)
{
  for(;;)
  {
    if ( P(n, v) == NULL )
      return n;
    n = P(n, v);
  }
}

bcn_type bcn_last(bcn_type n, int v)
{
  for(;;)
  {
    if ( N(n, v) == NULL )
      return n;
    n = N(n, v);
  }
}

/* delete all nodes from a line, starting with 'n' */
/* v = 0: delete horizontal */
/* v = 1: delete vertical */
void bcn_CloseLine(b_bcp_type bc, bcn_type n, int v)
{
  bcn_type nn;
  int nv = 1-v;
  
  while( n != NULL )
  {
    nn = N(n, v);
    bcn_dl_delete(n, v);
    bcn_dl_delete(n, nv);
    bcn_Close(bc, n);
    n = nn;
  }
}

void bcn_dl_copy_free(b_bcp_type bc, bcn_type n, int v)
{
  while( n != NULL )
  {
    if ( n->copy != NULL )
      bcn_Close(bc, n->copy);
    n->copy = NULL;
    n = N(n, v);
  }
}

void bcn_dl_copy_null(bcn_type n, int v)
{
  while( n != NULL )
  {
    n->copy = NULL;
    n = N(n, v);
  }
}

int bcn_dl_copy_alloc(b_bcp_type bc, bcn_type n, int v)
{
  bcn_type nn = n;
  bcn_dl_copy_null(n, v);
  while( n != NULL )
  {
    if ( bcn_OpenCopy(bc, n) == 0 )
    {
      bcn_dl_copy_free(bc, nn, v);
      return 0;
    }
    n = N(n, v);
  }
  return 1;
}

static void bcn_fix_copy(bcn_type n)
{
  /*
  N(n->copy, 0) = N(n,0)==NULL ? NULL : N(n,0)->copy;
  N(n->copy, 1) = N(n,1)==NULL ? NULL : N(n,1)->copy;
  P(n->copy, 0) = P(n,0)==NULL ? NULL : P(n,0)->copy;
  P(n->copy, 1) = P(n,1)==NULL ? NULL : P(n,1)->copy;
  */
  
  int i;
  for( i = 0; i < 4; i++ )
    n->copy->l[i] = (n->l[i] == NULL ? NULL : n->l[i]->copy);
    
  /* coooool data structures */
}

void bcn_dl_fix_copy(bcn_type n, int v)
{
  while( n != NULL )
  {
    bcn_fix_copy(n);
    n = N(n, v);
  }
}

bcn_type bcn_dl_dub(b_bcp_type bc, bcn_type n, int v)
{
  bcn_type nn;
  if ( bcn_dl_copy_alloc(bc, n, v) == 0 )
    return NULL;
  nn = n->copy;
  bcn_dl_fix_copy(n, v);
  bcn_dl_copy_null(n, v);
  
  return nn;
}

/* does NOT show the first item */
void bcn_dl_show_line(bcn_type n, int v, int max)
{
  bcn_type nn;
  int p = 0;
  assert( n != NULL );
  for(;;)
  {
    nn = N(n, v);
    if ( nn == NULL )
      break;
    while( p < XY(nn, v) )
    {
      printf("-");
      p++;
    }
    printf("%c", "x01-"[nn->val]);
    p++;
    n = nn;
  }
  while( p < max )
  {
    printf("-");
    p++;
  }
}


/* returns a node with position value 'p' */
/* creates a new node, if such a node was not found */
/* v = 0: 'p' is x-value */
/* v = 1: 'p' is y-value */
/* ignores first element */
bcn_type bcn_dl_get_create(b_bcp_type bc, bcn_type n, int v, int p)
{
  bcn_type new;
  n = bcn_dl_find(n, v, p);
  if ( XY(n, v) == p )
    return n;
  new = bcn_Open(bc);
  if ( new == NULL )
    return NULL;
  XY(new, v) = p;
  bcn_dl_insert_after(n, new, v);
  return new;
}


bcm_type bcm_Open(b_bcp_type bc)
{
  bcm_type m;
  m = (bcm_type)malloc(sizeof(struct _bcm_struct));
  if ( m != NULL )
  {
    m->width = 0;
    m->height = 0;
    m->bc = bc;
    m->ul = bcn_Open(bc);
    if ( m->ul != NULL )
    {
      return m;
    }
    bcn_Close(bc, m->ul);
  }
  free(m);
  return NULL;
}

bcn_type bcm_copy_matrix(bcm_type m)
{
  bcn_type n;
  bcn_type c;
  int is_ok = 1;
  

  /* create an exact copy of all nodes */

  if ( bcn_OpenCopy(m->bc, m->ul) == 0 )
    return NULL;
  
  is_ok &= bcn_dl_copy_alloc(m->bc, N(m->ul, 1), 1);
  is_ok &= bcn_dl_copy_alloc(m->bc, N(m->ul, 0), 0);
  
  n = N(m->ul, 1);
  while( n != NULL )
  {
    is_ok &= bcn_dl_copy_alloc(m->bc, N(n, 0), 0);
    n = N(n, 1);
  }

  /* is some mem is missing, delete all, return error value */
  
  if ( is_ok == 0 )
  {
    bcn_dl_copy_free(m->bc, N(m->ul, 1), 1);
    bcn_dl_copy_free(m->bc, N(m->ul, 0), 0);
    n = N(m->ul, 1);
    while( n != NULL )
    {
      bcn_dl_copy_free(m->bc, N(n, 0), 0);
      n = N(n, 1);
    }
    return NULL;
  }

  /* build correct links */

  bcn_fix_copy(m->ul);
  bcn_dl_fix_copy(N(m->ul, 1), 1);
  bcn_dl_fix_copy(N(m->ul, 0), 0);
  n = N(m->ul, 1);
  while( n != NULL )
  {
    bcn_dl_fix_copy(N(n, 0), 0);
    n = N(n, 1);
  }
  
  /* clear the links */

  bcn_fix_copy(m->ul);
  bcn_dl_copy_null(N(m->ul, 1), 1);
  bcn_dl_copy_null(N(m->ul, 0), 0);
  n = N(m->ul, 1);
  while( n != NULL )
  {
    bcn_dl_copy_null(N(n, 0), 0);
    n = N(n, 1);
  }
  
  c = m->ul->copy;
  m->ul->copy = NULL;
  
  return c;
}

void bcm_Clear(bcm_type m)
{
  int v = 0;
  bcn_type n, nn;
  n = N(m->ul,v);
  while( n != NULL )
  {
    nn = N(n, v);
    bcn_CloseLine(m->bc, n, 1-v);
    n = nn;
  }
  bcn_CloseLine(m->bc, N(m->ul, 1-v), 1-v);
  assert( N(m->ul,v) == NULL );
  assert( N(m->ul,1-v) == NULL );
}

#ifdef BCM_LOG
void bcm_Log(bcm_type m, int level, const char *fmt, ...)
{
  va_list va;
  
  if ( level < 3 )
    return;
  
  va_start(va, fmt);
  vprintf(fmt, va);
  puts("");
  va_end(va);
}
#endif

bcn_type bcm_GetCreate(bcm_type m, int x, int y)
{
  bcn_type row_head, row_p, col_head, col_p, new;

  /* step 1: try to find the element */
  col_head = bcn_dl_get_create(m->bc, m->ul, 0, x);
  if ( col_head == NULL )
    return NULL;
  col_p = bcn_dl_find(col_head, 1, y);
  if ( XY(col_p, 1) == y )
    return col_p;

  /* step 2: create a new element */
  new = bcn_Open(m->bc);
  if ( new == NULL )
    return NULL;
  XY(new, 0) = x;
  XY(new, 1) = y;
  if ( m->width <= x )
    m->width = x+1;
  if ( m->height <= y )
    m->height = y+1;
  
  /* step 3: add element to the matrix */
  bcn_dl_insert_after(col_p, new, 1);
  row_head = bcn_dl_get_create(m->bc, m->ul, 1, y);
  row_p = bcn_dl_find(row_head, 0, x);
  assert( XY(row_p, 0) != x );        /* it must not exist.. */
  bcn_dl_insert_after(row_p, new, 0);
  
  return new;
}

int bcm_Set(bcm_type m, int x, int y, int val)
{
  bcn_type n;
  n = bcm_GetCreate(m, x, y);
  if ( n == NULL )
    return 0;
  n->val = val;
  return 1;
}

void bcm_Close(bcm_type m)
{
  bcm_Clear(m);
  bcn_Close(m->bc, m->ul);
  free(m);
}

bcm_type bcm_OpenCopy(bcm_type m)
{
  bcm_type mm;
  bcn_type n;

  mm = bcm_Open(m->bc);
  if ( mm == NULL )
    return NULL;

  n = bcm_copy_matrix(m);
  if ( n == NULL )
  {
    bcm_Close(mm);
    return NULL;
  }
    
  bcn_Close( m->bc, mm->ul );
  mm->ul = n;
  mm->height = m->height;
  mm->width = m->width;
  return mm;  
}

void bcm_Show(bcm_type m)
{
  bcn_type n = N(m->ul, 1);
  int i;
  for( i = 0; i < m->width; i++ )
    printf("%d", i/10);
  printf("\n");
  for( i = 0; i < m->width; i++ )
    printf("%d", i%10);
  printf("\n");
  
  while( n != NULL )
  {
    bcn_dl_show_line(n, 0, m->width);
    n = N(n, 1);
    printf("\n");
  }
}


/* delete lines (usually columns) */
/* this procedure is used by removal of essentials and by columns dominance */
/* v = 0: xy is column number, remove rows with the specified val */
/* v = 1: xy is row number, remove cols with the specified val */
void bcm_DeleteLineByVal(bcm_type m, int v, int xy, int val)
{
  /* assume v = 0 */
  int nv = 1-v;
  bcn_type n = N(m->ul, nv);
  bcn_type nn;
  /* go through all rows */
  while( n != NULL )
  {
    nn = bcn_dl_find(n, v, xy);
    if ( XY(nn, v) == xy && nn->val == val )
    {
      nn = N(n, nv);    /* rescue the next pointer */
      bcn_CloseLine(m->bc, n, v);  
      n = nn;
    }
    else
    {
      n = N(n, nv);
    }
  }
}

int bcm_SetVariable(bcm_type m, bcs_type x, int xy, int val)
{
  if ( x != NULL )
    bcs_Set(x, xy, val);
#ifdef BCM_LOG
  bcm_Log(m, 2, "Set variable '%d' to '%c'.", xy, "x01-"[val]);
#endif
  return 1;
}


/* must be called after bcm_DeleteLineByVal()! */
/*
  assume the following matrix:
     11
     -1
  left col is essential and can be deleted.
  bcm_DeleteLineByVal() will remove the first row.
  bcm_CleanupEmpty() will delete the empty first column
  and assign a ZERO value to 'x'.
  v = 0: delete empty columns
*/
int bcm_CleanupEmpty(bcm_type m, bcs_type x, int v)
{
  bcn_type n = N(m->ul, v);
  int nv = 1-v;
  bcn_type nn;
  while( n != NULL )
  {
    if ( N(n, nv) == NULL )
    {
      nn = N(n, v);    /* rescue the next pointer */
      if ( bcm_SetVariable(m, x, XY(n, v), BCN_VAL_ZERO) == 0 )
        return 0;
      bcn_CloseLine(m->bc, n, nv); /* ok, it's only a single element */
      n = nn;
    }
    else
    {
      n = N(n, v);
    }
  }
  return 1;
}

/* v = 0: delete column */
void bcm_DeleteLine(bcm_type m, int v, int xy)
{
  bcn_type n = bcn_dl_find(m->ul, v, xy);
  if ( n != NULL )
    if ( XY(n, v) == xy )
      bcn_CloseLine(m->bc, n, 1-v);
}

/* val = BCN_VAL_ZERO deselect variable */
/* val = BCN_VAL_ONE  select variable */
int bcm_SelectColumn(bcm_type m, bcs_type x, int col, int val)
{
  bcm_DeleteLineByVal(m, 0, col, val);
  bcm_DeleteLine(m, 0, col);
  if ( bcm_SetVariable(m, x, col, val) == 0 )
    return 0;
  if ( bcm_CleanupEmpty(m, x, 0) == 0 )
    return 0;
  return 1;
}

/*--- essentials -----------------------------------------------------------*/

/* find essential columns (v=0) or rows (v=1) */
/* XY(returnval, v) is the column or row number */
/* probably only v=0 will be used */
bcn_type bcm_FindEssential(bcm_type m, int v, int *no_solution)
{
  /* comments assume v = 0 */
  int nv = 1-v;
  bcn_type n = N(m->ul, nv);
  bcn_type current, found;
  
  if ( no_solution != NULL )
    *no_solution = 0;
  found = NULL;
  /* go through all rows */
  while( n != NULL )
  {
    current = N(n, v);
    if ( current != NULL )            /* is there a column? */
    {
      if ( N(current, v) == NULL )    /* is there more than one? */
      {
        if ( found == NULL )          /* nothing found yet... */
        {
          found = current;            /* store the essential row */
        }
        else if ( found->val != current->val )
        {                             /* oh dear, no solution possible */
          if ( no_solution != NULL )
            *no_solution = 1;
          return NULL;
        }
      }
    }
    n = N(n, nv);
  }
  return found;
}

/* find and remove essential columns */
/* 'x' is the current selection */
int bcm_ReduceEssentialColumn(bcm_type m, bcs_type x, int *no_solution)
{
  bcn_type n;
  int val;
  int xy;
  for(;;)
  {
    n = bcm_FindEssential(m, 0, no_solution);
    if ( n == NULL )
      break;
    val = n->val;
    xy = XY(n, 0);
#ifdef BCM_LOG
    bcm_Log(m, 2, "Essential variable '%d'.", xy);
#endif
    if ( bcm_SelectColumn(m, x, xy, val) == 0 )
      return 0;
    m->is_something_done = 1;
  }
  return 1;
}

/*--- row dominance --------------------------------------------------------*/

/* check if n is subset of o */
/* the name 'row' only applies to the check... well, v should indeed be 0 */
/* as usual, the first element is not checked */
int bcn_is_subset(bcn_type n, bcn_type o, int v)
{
  //int pos;
  n = N(n, v);
  o = N(o, v);
  for(;;)
  {
  
    if ( o == NULL && n == NULL )
      return 1;
    if ( o == NULL && n != NULL )
      return 1;
    if ( o != NULL && n == NULL )
      return 0;
      
    if ( XY(n,v) != XY(o,v) )
    {
      if ( XY(n,v) > XY(o,v) )
        return 0;     /* o has a value, where n has not --> no subset */
      while( n != NULL && XY(n,v) < XY(o,v) )
        n = N(n,v);
    }
    else 
    {
      if ( n->val != o->val )
        return 0;     /* different values --> no subset */
      n = N(n,v);
      o = N(o,v);
    }
  }
}

void bcm_ReduceRows(bcm_type m)
{
  bcn_type n;
  bcn_type o;
  bcn_type p;
  //int v = 1;
  
  n = N(m->ul,1);
  while( n != NULL )
  {
    o = N(m->ul,1);
    while( o != NULL )
    {
      if ( o != n )
      {
        if ( bcn_is_subset(o, n, 0) != 0 )
        {
          p = N(o,1);
          bcn_CloseLine(m->bc, o, 0);
          m->is_something_done = 1;
          o = p;
        }
        else
        {
          o = N(o,1);
        }
      }
      else
      {
        o = N(o,1);
      }
    }
    n = N(n,1);
  }
}

/*--- col dominance --------------------------------------------------------*/

/*
  true conditions:
    o->val == BCN_VAL_ONE
    o->val == BCN_VAL_DC   and   n->val != BCN_VAL_ONE
    o->val == BCN_VAL_ZERO and   n->val == BCN_VAL_ZERO
    
  ref: [1] p136
*/
static int check_col_dom_clause(int k_val, int j_val)
{
  if ( j_val == BCN_VAL_ONE )
    return 1;
  if ( j_val == BCN_VAL_DC && k_val != BCN_VAL_ONE )
    return 1;
  if ( j_val == BCN_VAL_ZERO && k_val == BCN_VAL_ZERO )
    return 1;
  return 0;
}

/* does a column dominance check */
/* check if o dominates n */
/* assumes v = 1 */
int bcn_is_col_dom(bcn_type n, bcn_type o, int v)
{
  //int pos;
  int o_val;
  int n_val;
  n = N(n, v);
  o = N(o, v);  

  for(;;)
  {
    o_val = BCN_VAL_DC;
    n_val = BCN_VAL_DC;
    
    /* check_col_dom_clause(DC, DC) --> true !!! */
    if ( o != NULL && n != NULL )
    {
      if ( XY(o,v) < XY(n,v) )
      {
        o_val = o->val;
        o = N(o,v);
      }
      else if ( XY(o,v) > XY(n,v) )
      {
        n_val = n->val;
        n = N(n,v);
      }
      else
      {
        o_val = o->val;
        o = N(o,v);
        n_val = n->val;
        n = N(n,v);
      }
    }
    else if ( o != NULL )
    {
      o_val = o->val;
      o = N(o,v);
    }
    else if ( n != NULL )
    {
      n_val = n->val;
      n = N(n,v);
    }
    else break;
    
      
    /* compare */
    if ( check_col_dom_clause(n_val, o_val) == 0 )
      return 0;
  }

  return 1;  
}

/* column dominance */
int bcm_ReduceCols(bcm_type m, bcs_type x)
{
  bcn_type n;
  bcn_type o;
  bcn_type p;
  //int v = 0;
  int xy;

#ifdef BCM_LOG
  /* bcm_Log(m, 2, "Reduce columns (%d x %d).", m->width, m->height); */
#endif
  
  n = N(m->ul,0);
  while( n != NULL )
  {
    o = N(m->ul,0);
    while( o != NULL )
    {
      if ( o != n )
      {
        if ( bcn_is_col_dom(o, n, 1) != 0 )
        {
          p = N(o,0);

          xy = XY(o, 0);
#ifdef BCM_LOG
          bcm_Log(m, 2, "Column '%d' dominated by column '%d'.",
            XY(o, 0), XY(n, 0));
#endif

          if ( bcm_SelectColumn(m, x, xy, BCN_VAL_ZERO) == 0 )
            return 0;
            
          m->is_something_done = 1;
          o = p;
        }
        else
        {
          o = N(o,0);
        }
      }
      else
      {
        o = N(o,0);
      }
    }
    n = N(n,0);
  }
  return 1;
}

/*--- classic reduce algorithm ---------------------------------------------*/

int bcm_Reduce(bcm_type m, bcs_type x, int *no_solution)
{
  assert(no_solution != NULL);
  do
  {
    m->is_something_done = 0;
    if ( bcm_ReduceCols(m, x) == 0 )
      return 0;
    if ( bcm_ReduceEssentialColumn(m, x, no_solution) == 0 )
      return 0;
    if ( *no_solution != 0 )
      return 1;
    bcm_ReduceRows(m);
  } while(m->is_something_done);  
  return 1;
}


/*--- select branch variable -----------------------------------------------*/

int bcm_ChooseBranchColumn(bcm_type m)
{
  bcn_type row;
  bcn_type col;
  double col_max_weight = 0.0;
  double col_curr_weight;
  int col_max_pos = -1;
  bcn_type n;

  /* calculate row weights */
  row = N(m->ul,1);
  while( row != NULL )
  {
    row->val = bcn_dl_get_cnt(row, 0);
#ifdef BCM_LOG
    if ( row->val > 0 )
      bcm_Log(m, 1, "Row '%d' has weight '%lf'.", XY(row, 1), 1.0/(double)row->val);
#endif
    row = N(row, 1);
  }
  
  /* calculate max col weight */
  col = N(m->ul,0);
  while( col != NULL )
  {
    col_curr_weight = 0.0;
    row = N(col,1);
    while( row != NULL )
    {
      n = bcn_first(row, 0);
      assert( n != NULL );
      if ( row->val > 0 )
        col_curr_weight += 1.0/(double)n->val;
      row = N(row, 1);
    }
  
#ifdef BCM_LOG
    bcm_Log(m, 1, "Column '%d' has weight '%lf'.", XY(col, 0), col_curr_weight);
#endif
    
    if ( col_max_weight < col_curr_weight )
    {
      col_max_weight = col_curr_weight;
      col_max_pos = XY(col, 0);
    }
    col = N(col,0);
  }

#ifdef BCM_LOG
  bcm_Log(m, 2, "Branch variable '%d' selected.", col_max_pos);
#endif

  return col_max_pos;
}

/*--- alloc, copy and free of current matrix and solution ------------------*/

static int alloc_matrix_and_selection(bcm_type *m_new, bcs_type *x_new, bcm_type m_old, bcs_type x_old)
{
  if ( m_new != NULL )
  {
    *m_new = bcm_OpenCopy(m_old);
    if ( *m_new == NULL )
      return 0;
  }

  if ( x_new != NULL )
  {
    *x_new = bcs_OpenCopy(x_old);
    if ( *x_new == NULL )
    {
      bcm_Close(*m_new);
      return 0;
    }
  }
  return 1;
}

static void free_matrix_and_selection(bcm_type m, bcs_type x)
{
  if ( m != NULL )
    bcm_Close(m);
  if ( x != NULL )
    bcs_Close(x);
}

/*--- lower bound calculation ----------------------------------------------*/

bcn_type bcm_FindRowsWithZeroVars(bcm_type m)
{
  bcn_type n;
  bcn_type o;
  
  /* go through all rows */
  n = N(m->ul, 1);
  while( n != NULL )
  {
    o = N(n, 0);
    while( o != NULL )
    {
      if ( o->val == BCN_VAL_ZERO ) 
        return n;
      o = N(o, 0);
    }
    n = N(n, 1);
  }
  return NULL;
}

void bcm_DeleteRowsWithZeroVars(bcm_type m)
{
  bcn_type n;
  for(;;)
  {
    n = bcm_FindRowsWithZeroVars(m);
    if ( n == NULL )
      break;
    bcn_CloseLine(m->bc, n, 0);      
  }
}

bcn_type bcm_FindShortestRow(bcm_type m)
{
  bcn_type n;
  int cnt;
  bcn_type short_n = NULL;
  int short_cnt = m->width+m->height;
  
  /* go through all rows */
  n = N(m->ul, 1);
  while( n != NULL )
  {
    cnt = bcn_dl_get_cnt(n, 0);
    if ( short_cnt > cnt )
    {
      short_cnt = cnt;
      short_n = n;
    }
    n = N(n, 1);
  }
  return short_n;
}

int bcm_GetLowerBound(bcm_type m, bcs_type x)
{
  bcm_type mm;
  bcs_type xx; /* will contain the maximal independet set */
  
  int *a_ptr;
  int a_cnt;
  int mis_cnt;
  //int i;
  int cost;
  
  bcn_type n, o;

  a_ptr = malloc(m->width*sizeof(int));
  if ( a_ptr == NULL )  
    return -1;
  
  if ( alloc_matrix_and_selection(&mm, &xx, m, x) == 0 )
    return free(a_ptr), -1;

  bcm_DeleteRowsWithZeroVars(mm);

  if ( bcm_CleanupEmpty(mm, xx, 0) == 0 )
    return -1;
  
  mis_cnt = 0;
  for(;;)
  {

    if ( N(mm->ul,1) == NULL )
      break;
  
    /* find shortest row */
    
    n = bcm_FindShortestRow(mm);
    assert( n != NULL );
    
    /* store columns in the array */
    a_cnt = 0;
    o = N(n, 0);
    while( o != NULL )
    {
      a_ptr[a_cnt++] = XY(o, 0);
      o = N(o, 0);
    }
    mis_cnt += 1;
    
    /* delete all columns */
    while( a_cnt > 0 )
    {
      a_cnt--;
      if ( bcm_SelectColumn(mm, xx, a_ptr[a_cnt], BCN_VAL_ONE) == 0 )
        return free(a_ptr), free_matrix_and_selection(mm, xx), -1;
    }    
  }
  
  cost = bcs_GetCnt(x, BCN_VAL_ONE) + mis_cnt;
  
#ifdef BCM_LOG
  bcm_Log(m, 2, "Lower bound is '%d'.", cost);
#endif
  
  free(a_ptr);
  free_matrix_and_selection(mm, xx);
  
  return cost;
}

/*--- gimpel reduction -----------------------------------------------------*/
/* reference [2] and [3] */

#ifdef not_jet_implemented

static int bcm_build_gimpel_row(bcm_type m, bcn_type n, int y)
{
  bcn_type k, l;
  l = N(bcn_first(n, 0), 0);
  while( l != NULL )
  {
    k = bcm_GetCreate(m, XY(l, 0), y);
    if ( k == NULL )
      return 0;
    if ( k->val == BCN_VAL_NONE )
      k->val = l->val;
    else if ( k->val != l->val )
      return 0;     /* can not apply gimpel */
  }
  return 1;
}

static int bcm_build_gimpel_matrix(bcm_type m, int row)
{
  int col1, col2;
  bcn_type n1, n2;
  bcn_type m1, m2;
  bcn_type k, l;
  int y = m->height;
  
  bcn_type gimpel_row = bcn_dl_find(m->ul, row, 1);

  /* get the column numbers */
  
  col1 = XY(N(gimpel_row, 0), 0);
  col2 = XY(N(N(gimpel_row, 0), 0), 0);
  assert(N(N(N(gimpel_row, 0), 0), 0) == NULL);

  /* remove the gimple clause */

  bcn_CloseLine(m->bc, gimpel_row, 0);

  n1 = bcn_dl_find(m->ul, 0, col1);
  n2 = bcn_dl_find(m->ul, 0, col2);
  
  /* create the new clauses */
  
  m1 = N(n1, 1);
  while( m1 != NULL )
  {
    m2 = N(n2, 1);
    while( m2 != NULL )
    {
    
      if ( bcm_build_gimpel_row(m, m1, y) == 0 )
        return 0;
      if ( bcm_build_gimpel_row(m, m2, y) == 0 )
        return 0;
      y++;
      
      m2 = N(m2, 1);
    }
    m1 = N(m1, 1);
  }
  
  return 1;  
}

bcm_type bcm_get_gimpel_matrix(bcm_type m, bcs_type x, bcn_type *n_ptr)
{
  bcn_type n;
  bcm_type mm = NULL;
  int col1, col2;
    
  n = N(m->ul, 1);
  while( n != NULL )
  {
    if ( bcn_dl_get_cnt(n, 0) == 2 )
    {
      if ( N(n,0)->val == BCN_VAL_ONE && N(N(n,0),0)->val == BCN_VAL_ONE )
      {
        if ( alloc_matrix_and_selection(&mm, NULL, m, NULL) == 0 )
          return NULL;
        if ( bcm_build_gimpel_matrix(mm, XY(n, 1)) != 0 )
          break;
        free_matrix_and_selection(mm, NULL);
        mm = NULL;
      }
    }
    n = N(n, 1);
  }
  
  if ( mm == NULL )
    return NULL;   /* no gimpel */

  col1 = XY(N(n, 0), 0);
  col2 = XY(N(N(n, 0), 0), 0);

  /* col1 and col2 are now identical */
  /* delete col1 */
  
  /* missing: delete old clauses!!! */

  if ( bcm_SelectColumn(mm, x, col1, BCN_VAL_ZERO) == 0 )
    return 0;

  *n_ptr = n;
  return mm;
}

void bcm_fix_gimpel_result(bcm_type m, bcm_type mm, bcs_type x, bcn_type gimpel_row)
{
  bcn_type n;
  bcn_type o;
  int col1, col2;
  col1 = XY(N(gimpel_row, 0), 0);
  col2 = XY(N(N(gimpel_row, 0), 0), 0);

  free_matrix_and_selection(mm, NULL);

  n = N(bcn_dl_find(m->ul, 0, col1), 1);
  while( n != NULL )
  {
    o = N(bcn_first(n, 0),0);
    while( o != NULL )
    {
      if ( XY(o, 0) != col1 )
      {
        if ( bcs_GetVal(x, XY(o, 0)) != o->val )
        {
          bcs_Set(x, col1, BCN_VAL_ONE);
          return;
        }
      }
      o = N(o, 0);
    }
    n = N(n, 1);
  }
  
  bcs_Set(x, col2, BCN_VAL_ONE);
}

#endif

/*--- BCP ------------------------------------------------------------------*/

int b_bcp_GetBestCost(b_bcp_type bc)
{
  return bcs_GetCnt(bc->b, BCN_VAL_ONE);
}

static void b_bcp_check_better_result(b_bcp_type bc, bcs_type x)
{
  int x_cost = bcs_GetCnt(x, BCN_VAL_ONE);

  if ( b_bcp_GetBestCost(bc) > x_cost )
  {
    bcs_Copy(bc->b, x);
#ifdef BCM_LOG
    {
      static char s[80];
      int i;
      for( i = 0; i < 75 && i < bc->b->cnt; i++ )
      {
        switch( bc->b->sel[i] )
        {
          case BCN_VAL_ZERO:
            s[i] = '0';
            break;
          case BCN_VAL_ONE:
            s[i] = '1';
            break;
          default:
            s[i] = '-';
            break;
        }
      }
      s[i] = '\0';
      if ( i < bc->b->cnt )
        strcat(s, "...");
      bcm_Log(bc->m, 3, "New solution with cost '%d': %s", x_cost, s);
    }
#endif
  }
}

static int b_bcp_recursion(b_bcp_type bc, bcm_type m, bcs_type x)
{
  int branch_column;
  int cost_lower;
  
  bcm_type mm = NULL;
  bcs_type xx = NULL;


  if ( bcm_Reduce(m, x, &(bc->is_no_solution)) == 0 )
    return 0;
    
  if ( bc->is_no_solution != 0 )
    return 1;

  if ( N(m->ul,0) == NULL )
  {
    b_bcp_check_better_result(bc, x);
    return 1;
  }
  
  /*
  {
    bcn_type n;
    mm = bcm_get_gimpel_matrix(m, x, &n);
    if ( mm != NULL )
    {
      if ( b_bcp_recursion(bc, m, x) == 0 )
        return 0;
      bcm_fix_gimpel_result(m, mm, x, n);
      return 1;
    }
  }
  */
  

  cost_lower = bcm_GetLowerBound(m, x);
  if ( b_bcp_GetBestCost(bc) <= cost_lower )
  {
#ifdef BCM_LOG
    bcm_Log(m, 3, "Lower bound (best result %d, lower bound %d).", b_bcp_GetBestCost(bc), cost_lower);
#endif
    return 1;
  }

  branch_column = bcm_ChooseBranchColumn(m);
  if ( branch_column < 0 )
    return 1; /* ??? */


  if ( bc->is_greedy == 0 )
    if ( alloc_matrix_and_selection(&mm, &xx, m, x) == 0 )
      return 0;


  /* ONE */
    
  if ( bcm_SelectColumn(m, x, branch_column, BCN_VAL_ONE) == 0 )
    return free_matrix_and_selection(mm, xx), 0;
    
  if ( b_bcp_recursion(bc, m, x) == 0 )
    return free_matrix_and_selection(mm, xx), 0;

  if ( bc->is_greedy == 0 )
  {

  /* ZERO */
  
    if ( bcm_SelectColumn(mm, xx, branch_column, BCN_VAL_ZERO) == 0 )
      return free_matrix_and_selection(mm, xx), 0;

    if ( b_bcp_recursion(bc, mm, xx) == 0 )
      return free_matrix_and_selection(mm, xx), 0;
  }

  return free_matrix_and_selection(mm, xx), 1;
}


/*--- BCP-API --------------------------------------------------------------*/

/* result: 0 memory error */
/* results are stored in 
  bc->is_no_solution != 0   --> no valid solution found
  bc->b                     --> selection if 'bc->is_no_solution == 0'
*/
int b_bcp_Do(b_bcp_type bc)
{
  bcs_type x;
  bcm_type m;

#ifdef BCM_LOG
  bcm_Log(bc->m, 4, "Binate cover problem (%d x %d).", bc->m->width, bc->m->height);
#endif


  /* assume a solution */
    
  bc->is_no_solution = 0;
  
  /* assume the selection of all variables, which have a column */

  if ( bc->b != NULL )
    bcs_Close(bc->b);
  bc->b = bcs_Open(bc->m->width);
  if ( bc->b == NULL )
    return 0;
    
  bcs_Fill(bc->b, BCN_VAL_ZERO);
  {
    bcn_type n = N(bc->m->ul, 0);
    while( n != NULL )
    {
      bcs_Set(bc->b, XY(n, 0), BCN_VAL_ONE);
      n = N(n, 0);
    }
  }
  
  /* the recursion requires a 'current' selection */
  
  x = bcs_OpenCopy(bc->b);
  if ( x == NULL )
    return 0;
  bcs_Fill(x, BCN_VAL_ZERO);
  
  /* work on a copy of the matrix */
  
  m = bcm_OpenCopy(bc->m);
  if ( m == NULL )
    return bcs_Close(x), 0;

  if ( b_bcp_recursion(bc, m, x) == 0 )
    return bcs_Close(x), bcm_Close(m), 0;

#ifdef BCM_LOG
  bcm_Log(bc->m, 4, "Binate cover problem finished.");
#endif
  
  return bcs_Close(x), bcm_Close(m), 1;
}


int b_bcp_Set(b_bcp_type bc, int x, int y, int val)
{
  return bcm_Set(bc->m, x, y, val);
}


int b_bcp_Get(b_bcp_type bc, int x)
{
  return bcs_GetVal(bc->b, x);
}


int b_bcp_SetStrLine(b_bcp_type bc, int y, const char *s)
{
  int i, cnt = strlen(s);
  for( i = 0; i < cnt; i++ )
  {
    switch(s[i])
    {
      case '0':
        if ( b_bcp_Set(bc, i, y, BCN_VAL_ZERO) == 0 )
          return 0;
        break;
      case '1':
        if ( b_bcp_Set(bc, i, y, BCN_VAL_ONE) == 0 )
          return 0;
        break;
      default:
        break;
    }
  }
  return 1;
}


void b_bcp_Show(b_bcp_type bc)
{
  bcm_Show(bc->m);
}

b_bcp_type b_bcp_Open()
{
  b_bcp_type bc;
  bc = (b_bcp_type)malloc(sizeof(struct _b_bcp_struct));
  if ( bc != NULL )
  {
    bc->is_greedy = 0;
    bc->is_no_solution = 0;
    bc->b = NULL;
    bc->m = bcm_Open(bc);
    if ( bc->m != NULL )
    {
      return bc;
    }
    free(bc);
  }
  return NULL;
}


void b_bcp_Close(b_bcp_type bc)
{
  if ( bc->b != NULL )
    bcs_Close(bc->b);
  bcm_Close(bc->m);
  free(bc);
}

#ifdef TESTMAIN

int main1()
{
  int x;
  /* example 5.1.4 from [1] */
  b_bcp_type bc = b_bcp_Open();
  bcm_Set(bc->m, 0, 0, BCN_VAL_ZERO);
  bcm_Set(bc->m, 1, 0, BCN_VAL_ONE);
  bcm_Set(bc->m, 1, 1, BCN_VAL_ONE);
  bcm_Set(bc->m, 2, 1, BCN_VAL_ONE);
  bcm_Set(bc->m, 3, 1, BCN_VAL_ONE);
  bcm_Set(bc->m, 2, 2, BCN_VAL_ZERO);
  bcm_Set(bc->m, 3, 2, BCN_VAL_ONE);
  bcm_Show(bc->m);
  puts("-------------");
  bcm_Reduce(bc->m, NULL, &x);
  bcm_Show(bc->m);
  b_bcp_Close(bc);
  
  return 0;
}


int main2()
{
  /* example 5.1.7 from [1] */
  int col;
  bcm_type m;
  b_bcp_type bc = b_bcp_Open();
  bcm_Set(bc->m, 0, 0, BCN_VAL_ONE);
  bcm_Set(bc->m, 1, 0, BCN_VAL_ONE);
  bcm_Set(bc->m, 0, 1, BCN_VAL_ONE);
  bcm_Set(bc->m, 2, 1, BCN_VAL_ONE);
  bcm_Set(bc->m, 3, 2, BCN_VAL_ONE);
  bcm_Set(bc->m, 4, 2, BCN_VAL_ONE);
  bcm_Set(bc->m, 3, 3, BCN_VAL_ONE);
  bcm_Set(bc->m, 5, 3, BCN_VAL_ONE);
  bcm_Set(bc->m, 2, 4, BCN_VAL_ONE);
  bcm_Set(bc->m, 4, 4, BCN_VAL_ONE);
  bcm_Set(bc->m, 5, 4, BCN_VAL_ONE);
  bcm_Set(bc->m, 2, 5, BCN_VAL_ONE);
  bcm_Set(bc->m, 6, 5, BCN_VAL_ONE);
  bcm_Set(bc->m, 1, 6, BCN_VAL_ONE);
  bcm_Set(bc->m, 6, 6, BCN_VAL_ONE);
  bcm_Set(bc->m, 3, 7, BCN_VAL_ONE);
  bcm_Set(bc->m, 7, 7, BCN_VAL_ONE);
  bcm_Set(bc->m, 3, 8, BCN_VAL_ONE);
  bcm_Set(bc->m, 8, 8, BCN_VAL_ONE);
  bcm_Set(bc->m, 1, 9, BCN_VAL_ONE);
  bcm_Set(bc->m, 7, 9, BCN_VAL_ONE);
  bcm_Set(bc->m, 8, 9, BCN_VAL_ONE);
  bcm_Show(bc->m);
  m = bcm_OpenCopy(bc->m);
  puts("-------------copy");
  bcm_Show(m);
  col = bcm_ChooseBranchColumn(bc->m);
  /* example 5.1.10 from [1] */
  puts("-------------1");
  bcm_SelectColumn(bc->m, NULL, col, BCN_VAL_ONE);
  bcm_Show(bc->m);
  puts("-------------0");
  bcm_SelectColumn(m, NULL, col, BCN_VAL_ZERO);
  bcm_Show(m);
  /*
  bcm_Reduce(bc->m, NULL);
  */
  b_bcp_Close(bc);
  
  return 0;
}

int main3()
{
  b_bcp_type bc = b_bcp_Open();
  
  /*
  b_bcp_SetStrLine(bc, 0, "11-------");
  b_bcp_SetStrLine(bc, 1, "1-1------");
  b_bcp_SetStrLine(bc, 2, "---11----");
  b_bcp_SetStrLine(bc, 3, "---1-1---");
  b_bcp_SetStrLine(bc, 4, "--1-11---");
  b_bcp_SetStrLine(bc, 5, "--1---1--");
  b_bcp_SetStrLine(bc, 6, "-1----1--");
  b_bcp_SetStrLine(bc, 7, "---1---1-");
  b_bcp_SetStrLine(bc, 8, "---1----1");
  b_bcp_SetStrLine(bc, 9, "-1-----11");
  */

  b_bcp_SetStrLine(bc, 0, "1-1");
  b_bcp_SetStrLine(bc, 1, "11-");
  b_bcp_SetStrLine(bc, 2, "-11");
  
  b_bcp_Show(bc);
  
  b_bcp_Do(bc);
  
  printf("no solution: %d\n", bc->is_no_solution);
  
  b_bcp_Close(bc);
  return 0;
}

#endif
