/*

  MWC.H

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
   
  history
  25 may 2000  changed first arg of strdup to 'const char *'
  11 dec 2000  added fwrite/fputc/putc
  17 okt 2001  changed default for lower_space to 8

*/
#ifndef _MWC_H
#define _MWC_H

#include <stdlib.h>
#include <stdio.h>

#ifndef NDEBUG
#ifndef MWC_DISABLE
#define ___MWC_ENABLE
#endif /* MWC_DISABLE */
#endif /* NDEBUG */


#ifdef ___MWC_ENABLE

void *mwc_malloc(size_t size, const char *arg, const char *file, int line);
char *mwc_strdup(const char *str, const char *arg, const char *file, int line);
void *mwc_calloc(size_t num, size_t size, const char *arg, const char *file, int line);
void *mwc_realloc(void *old_ptr, size_t new_len, const char *arg, const char *file, int line);
void mwc_free(void *ptr, const char *arg, const char *file, int line);
void _mwc_ptr_check(void *ptr, const char *arg, const char *file, int line);

int mwc_fputc(int c, FILE *stream, const char *arg, const char *file, int line);

#define malloc(size)      \
   mwc_malloc((size), #size, __FILE__, __LINE__)
#define strdup(str)      \
   mwc_strdup((str), #str, __FILE__, __LINE__)
#define calloc(num,size)  \
   mwc_calloc((num), (size), #num "," #size, __FILE__, __LINE__)
#define realloc(ptr,size) \
   mwc_realloc((ptr), (size), #ptr "," #size, __FILE__, __LINE__);
#define free(ptr)         \
   mwc_free((ptr), #ptr, __FILE__, __LINE__)
#define ptrcheck(ptr)     \
   _mwc_ptr_check((ptr), #ptr, __FILE__, __LINE__)

#define fwrite(ptr, size, nitems, stream)\
  mwc_fwrite((ptr), (size), (nitems), (stream), #ptr "," #size "," #nitems "," #stream, __FILE__, __LINE__ )
#define fputc(c,fp)\
  mwc_fputc((c), (fp), #c "," #fp, __FILE__, __LINE__ )
#ifdef putc
#undef putc
#endif
#define putc(c,fp)\
  mwc_fputc((c), (fp), #c "," #fp, __FILE__, __LINE__ )

#else

#define ptrcheck(ptr)

#endif /* ___MWC_ENABLE */

#endif /* _MWC_H */
