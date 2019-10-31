/*

  b_sl.c
  
  string list

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

#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "b_sl.h"
#include "b_io.h"
#include "mwc.h"

b_sl_type b_sl_Open()
{
  return b_pl_Open();
}

void b_sl_Clear(b_sl_type b_sl)
{
  int i, cnt;
  cnt = b_sl_GetCnt(b_sl);
  for( i = 0; i < cnt; i++ )
    if ( b_sl_GetVal(b_sl,i) != NULL )
      free(b_sl_GetVal(b_sl,i));
  b_pl_Clear(b_sl);
}

void b_sl_Close(b_sl_type b_sl)
{
  b_sl_Clear(b_sl);
  b_pl_Close(b_sl);
}

int b_sl_Add(b_sl_type b_sl, const char *s)
{
  char *t;
  if ( s == NULL )
    return b_pl_Add(b_sl, NULL);
    
  t = strdup(s);
  if ( t == NULL )
    return -1;
  return b_pl_Add(b_sl, t);
}

void b_sl_Del(b_sl_type b_sl, int pos)
{
    if ( b_sl_GetVal(b_sl, pos) != NULL )
      free(b_sl_GetVal(b_sl, pos));
    
  b_pl_DelByPos(b_sl, pos);  
}

int b_sl_AddRange(b_sl_type b_sl, const char *s, size_t start, size_t cnt)
{
  char *t;
  size_t l;
  if ( s == NULL )
    return b_pl_Add(b_sl, NULL);
    
  l = strlen(s);
  
  if ( start > l )
    start = l;
  assert( start <= l );
  if ( cnt > l-start )
    cnt = l-start;
    
  t = (char *)malloc(cnt+1);
  if ( t == NULL )
    return -1;
  
  strncpy(t, s+start, cnt);
  t[cnt] = '\0';
  
  return b_pl_Add(b_sl, t);
}

int b_sl_Set(b_sl_type b_sl, int pos, const char *s)
{
  char *t;
  if ( pos < 0 )
    return 0;
  while( pos >= b_sl_GetCnt(b_sl) )
    if ( b_sl_Add(b_sl, NULL ) < 0 )
      return 0;
  t = strdup(s);
  if ( t == NULL )
    return 0;
  if ( b_sl_GetVal(b_sl, pos) != NULL )
    free(b_sl_GetVal(b_sl, pos));
  b_pl_SetVal(b_sl, pos, t);
  return 1;
}

static int b_sl_write_el(FILE *fp, void *el, void *ud)
{
  return b_io_WriteString(fp, (const char *)el);
}

int b_sl_Write(b_sl_type b_sl, FILE *fp)
{
  return b_pl_Write(b_sl, fp, b_sl_write_el, NULL);
}

static void *b_sl_read_el(FILE *fp, void *ud)
{
  char *s;
  if ( b_io_ReadAllocString(fp, &s) == 0 )
    return NULL;
  return (void *)s;
}

int b_sl_Read(b_sl_type b_sl, FILE *fp)
{
  return b_pl_Read(b_sl, fp, b_sl_read_el, NULL);
}

int b_sl_Union(b_sl_type dest, b_sl_type src)
{
  int i, cnt = b_sl_GetCnt(src);
  for( i = 0; i < cnt; i++ )
    if ( b_sl_Add(dest, b_sl_GetVal(src, i)) < 0 )
      return 0;
  return 1;
}

int b_sl_Copy(b_sl_type dest, b_sl_type src)
{
  b_sl_Clear(dest);
  return b_sl_Union(dest, src);
}

int b_sl_Find(b_sl_type b_sl, const char *s)
{
  char *t;
  int i, cnt;
  if ( b_sl == NULL )
    return -1;
  cnt = b_sl_GetCnt(b_sl);
  for( i = 0; i < cnt; i++ )
  {
    t = b_sl_GetVal(b_sl,i);
    if ( t != NULL )
      if ( strcmp(t, s) == 0 )
        return i;
  }
  return -1;
}

int b_sl_ImportByStr(b_sl_type b_sl, const char *s, const char *delim, const char *term)
{
  size_t l;
  int pharsed_characters = 0;
  
  l = 0;
  for(;;)
  {
    if ( strchr(delim, (int)(unsigned char)s[l]) == NULL )
      break;
    if ( strchr(term, (int)(unsigned char)s[l]) != NULL )
      break;
    l++;
  }
  s += l; pharsed_characters += l;
  if ( strchr(term, (int)*s) != NULL )
    return pharsed_characters;
  for(;;)
  {
    l = 0;
    for(;;)
    {
      if ( strchr(term, (int)(unsigned char)s[l]) != NULL )
        break;
      if ( strchr(delim, (int)(unsigned char)s[l]) != NULL )
        break;
      l++;
    }

    if ( l == 0 )
      break;
    if ( b_sl_AddRange(b_sl, s, 0, l) < 0 )
      return -1;
    s += l; 
    pharsed_characters += l;
    l = 0;
    for(;;)
    {
      if ( strchr(term, (int)(unsigned char)s[l]) != NULL )
        break;
      if ( strchr(delim, (int)(unsigned char)s[l]) == NULL )
        break;
      l++;
    }
    s += l; pharsed_characters += l;
    if ( strchr(term, (int)*s) != NULL )
      return pharsed_characters;
  }
  
  return pharsed_characters;
}

int b_sl_ExportToFP(b_sl_type b_sl, FILE *fp, const char *delim, const char *last)
{
  int i, cnt;
  cnt = b_sl_GetCnt(b_sl);
  for( i = 0; i < cnt; i++ )
  {
    if ( fprintf(fp, "%s", b_sl_GetVal(b_sl,i)) < 0 )
      return 0;
    if ( i+1 < cnt )
      if ( fprintf(fp, "%s", delim) < 0 )
        return 0;
  }  
  if ( fprintf(fp, "%s", last) < 0 )
    return 0;
  return 1;
}

const char *b_sl_ToStr(b_sl_type b_sl, const char *delim)
{
  int i, cnt;
  static char s[1024*3];		// overflow risk... shoule be improved once...
  s[0] = '\0';
  cnt = b_sl_GetCnt(b_sl);
  assert(cnt < 30);
  for( i = 0; i < cnt; i++ )
  {
    sprintf(s+strlen(s), "%s", b_sl_GetVal(b_sl,i));
    if ( i+1 < cnt )
      sprintf(s+strlen(s), "%s", delim);
  }  
  return s;
}

