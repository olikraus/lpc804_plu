/*

  cubedefs.h

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

#ifndef _CUBE_DEFS_H
#define _CUBE_DEFS_H

/* autoconf compatible */
#ifndef SIZEOF_INT
#define SIZEOF_INT 4
#endif

typedef unsigned int c_int;
#define CUBE_IN_WORDS 8
/* #define CUBE_SIGNALS_PER_WORD (SIZEOF_INT*4) */
#define CUBE_SIGNALS_PER_IN_WORD (SIZEOF_INT*4)
#define CUBE_SIGNALS_PER_OUT_WORD (SIZEOF_INT*8)
#define CUBE_IN_SIGNALS (CUBE_SIGNALS_PER_IN_WORD*CUBE_IN_WORDS)

#if SIZEOF_INT == 2
#define CUBE_IN_MASK_ZERO 0x05555
#define CUBE_IN_MASK_DC   0x0ffff
#define CUBE_OUT_MASK     0x0ffff
#elif SIZEOF_INT == 4
#define CUBE_IN_MASK_ZERO 0x055555555
#define CUBE_IN_MASK_DC   0x0ffffffff
#define CUBE_OUT_MASK     0x0ffffffff
#elif SIZEOF_INT == 8
#define CUBE_IN_MASK_ZERO 0x05555555555555555
#define CUBE_IN_MASK_DC   0x0ffffffffffffffff
#define CUBE_OUT_MASK     0x0ffffffffffffffff
#elif SIZEOF_INT == 16
#define CUBE_IN_MASK_ZERO 0x055555555555555555555555555555555
#define CUBE_IN_MASK_DC   0x0ffffffffffffffffffffffffffffffff
#define CUBE_OUT_MASK     0x0ffffffffffffffffffffffffffffffff
#endif


/* BEGIN_LATEX cube.tex */
struct _cube_struct
{
  c_int in[CUBE_IN_WORDS];
  c_int out;
};
typedef struct _cube_struct cube;
/* END_LATEX */

struct _dcube_struct
{
  c_int *in;
  c_int *out;
  int n;	// used by some algorithms, usually to count the dc's or to mark something (-1)
};
typedef struct _dcube_struct dcube;

typedef struct _dclist_struct *dclist;

#endif /* _CUBE_DEFS_H */
