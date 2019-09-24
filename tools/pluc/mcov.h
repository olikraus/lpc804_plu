/*

  mcov.h
  
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

#ifndef _MCOV_H
#define _MCOV_H

#include "dcube.h"

struct _mcov_struct
{
  pinfo *pi_pr;
  dclist cl_pr;
  
  pinfo pi_matrix;
  dclist cl_matrix;
  dcube matrix_or;
  int is_matrix_init;
  
  pinfo    pi_select;     /* matrix width */
  dcube    select_curr;
  unsigned cost_curr;
  dcube    select_opt;
  unsigned cost_opt;
  int is_select_init;
};
typedef struct _mcov_struct *mcov;

int mcovInit(mcov *mc, pinfo *pi);
void mcovDestroy(mcov mc);
int mcovExact(mcov mc, dclist cl_es, dclist cl_pr, dclist cl_dc, dclist cl_rc);

/* diese routine erfordert vorher ZWINGEND den aufruf von dclSplitRelativeEssential */
/* cl_pr darf keine relativ essentiellen cubes enthalten */
int dclIrredundantExact(pinfo *pi, dclist cl_es, dclist cl_pr, dclist cl_dc, dclist cl_rc);

/* diese routine erfordert vorher ZWINGEND den aufruf von dclSplitRelativeEssential */
/* cl_pr darf keine relativ essentiellen cubes enthalten */
int dclIrredundantGreedy(pinfo *pi, dclist cl_es, dclist cl_pr, dclist cl_dc, dclist cl_rc);


#endif /* _MCOV_H */
