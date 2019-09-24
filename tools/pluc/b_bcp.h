/*

  b_bcp.h

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
  
*/

typedef struct _bcn_struct *bcn_type;
typedef struct _b_bcp_struct *b_bcp_type;

#define BCN_VAL_NONE 0
#define BCN_VAL_ZERO 1
#define BCN_VAL_ONE 2
#define BCN_VAL_DC 3

/* binate cover selection */
struct _bcs_struct
{
  int cnt;
  int *sel;
};
typedef struct _bcs_struct *bcs_type;

/* binate cover node */
struct _bcn_struct
{
  bcn_type l[4];    /* left, up, right, down */
  bcn_type copy;
  int xy[2];
  int val;
};

/* binate cover matrix */
struct _bcm_struct
{
  int width, height;
  b_bcp_type bc;
  bcn_type ul;   /* upper left corner */
  int is_something_done;
};
typedef struct _bcm_struct *bcm_type;

/* binate cover problem */
struct _b_bcp_struct
{
  int is_greedy;
  int is_no_solution;
  bcs_type b; /* result */
  bcm_type m; /* matrix */
};


/* API */

/* perform binate cover algoritm */
/* returns 0, if a memory error occured */
int b_bcp_Do(b_bcp_type bc);

/* setup the problem */
int b_bcp_Set(b_bcp_type bc, int x, int y, int val);
int b_bcp_SetStrLine(b_bcp_type bc, int y, const char *s);

/* get the result BCN_VAL_ZERO/BCN_VAL_ONE */
int b_bcp_Get(b_bcp_type bc, int x);

/* show the current problem */
void b_bcp_Show(b_bcp_type bc);

/* create and destroy data for the algorithm */
b_bcp_type b_bcp_Open();
void b_bcp_Close(b_bcp_type bc);

