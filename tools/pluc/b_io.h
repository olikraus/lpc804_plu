/*

  b_io.h

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

#ifndef _B_IO_H
#define _B_IO_H

#include <stdio.h>

int b_io_WriteChar(FILE *fp, char c);
int b_io_ReadChar(FILE *fp, char *c);

int b_io_Write(FILE *fp, int cnt, const unsigned char *ptr);
int b_io_Read(FILE *fp, int cnt, unsigned char *ptr);

int b_io_WriteInt(FILE *fp, int val);
int b_io_ReadInt(FILE *fp, int *val);
int b_io_WriteUnsignedInt(FILE *fp, unsigned int val);
int b_io_ReadUnsignedInt(FILE *fp, unsigned int *val);

int b_io_WriteString(FILE *fp, const char *s);
int b_io_ReadAllocString(FILE *fp, char **s_ptr);

int b_io_WriteIntArray(FILE *fp, int cnt, int *values);
int b_io_ReadIntArray(FILE *fp, int cnt, int *values);

int b_io_WriteDouble(FILE *fp, double x);
int b_io_ReadDouble(FILE *fp, double *x);

#endif
