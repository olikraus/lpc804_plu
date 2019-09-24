/*

  b_ff.c
  
  find file
  
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

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#ifdef HAVE_CONFIG_H
#include "config.h"
#include "mwc.h"
#endif /* HAVE_CONFIG_H */

static char *file_exists(const char *pre, const char *path, const char *name, const char *ext)
{
  char *s;
  size_t len = 0;
  FILE *fp;

  if ( pre != NULL )
    len += strlen(pre)+1;
  if ( path != NULL )
    len += strlen(path)+1;
  if ( name != NULL )
    len += strlen(name);
  if ( ext != NULL )
    len += strlen(ext);

  s = (char *)malloc(len+1);
  if ( s == NULL )
    return NULL;
    
  *s = '\0';
  if ( pre != NULL )
  {
    strcat(s, pre);
    strcat(s, "/");
  }
  if ( path != NULL )
  {
    strcat(s, path);
    strcat(s, "/");
  }
  if ( name != NULL )
    strcat(s, name);
  if ( ext != NULL )
    strcat(s, ext);
  if ( strlen(s) == 0 )
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
    
  fclose(fp);
  return s;
}

char *b_ff(const char *name, const char *default_name, const char *default_ext)
{
  char *s;
  s = file_exists(NULL, NULL, name, NULL); 
  if ( s != NULL ) 
    return s;
  
  if ( default_ext != NULL )
  {
    s = file_exists(NULL, NULL, name, default_ext); 
    if ( s != NULL ) 
      return s;
  }

#ifdef PACKAGE
  {
    char *home;
    char *home_sub_dir = "." PACKAGE;
    
    home = getenv("HOME");
    if ( home != NULL )
    {
      s = file_exists(home, home_sub_dir, name, NULL); 
      if ( s != NULL ) 
        return s;

      if ( default_ext != NULL )
      {
        s = file_exists(home, home_sub_dir, name, default_ext); 
        if ( s != NULL ) 
          return s;
      }

      if ( default_name != NULL )
      {
        s = file_exists(home, home_sub_dir, default_name, NULL); 
        if ( s != NULL ) 
          return s;
      }
    }
  }
#endif /* PACKAGE */
  
#ifdef DATA_DIR
  s = file_exists(NULL, DATA_DIR, name, NULL); 
  if ( s != NULL ) 
    return s;

  if ( default_ext != NULL )
  {
    s = file_exists(NULL, DATA_DIR, name, default_ext); 
    if ( s != NULL ) 
      return s;
  }

  if ( default_name != NULL )
  {
    s = file_exists(NULL, DATA_DIR, default_name, NULL); 
    if ( s != NULL ) 
      return s;
  }
  
#endif /* DATA_DIR */


  return NULL;  
}

char *b_get_ff_searchpath(void)
{
  size_t len = 2;
#ifdef PACKAGE
  char *home = getenv("HOME");
#endif /* PACKAGE */
  char *s;
#ifdef DATA_DIR
  len += strlen(DATA_DIR)+1;
#endif /* DATA_DIR */
  
#ifdef PACKAGE
  if ( home != NULL )
    len += strlen(home)+2+strlen(PACKAGE)+1;
#endif /* PACKAGE */

  s = (char *)malloc(len+1);
  if ( s == NULL )
    return NULL;
  
  *s = '\0';
  strcat(s, ". ");
  
#ifdef PACKAGE
  if ( home != NULL )
  {
    strcat(s, home);
    strcat(s, "/.");
    strcat(s, PACKAGE);
    strcat(s, " ");
  }
#endif /* PACKAGE */

#ifdef DATA_DIR
  strcat(s, DATA_DIR);
  strcat(s, " ");
#endif /* DATA_DIR */

  return s;
}

FILE *b_fopen(const char *name, const char *default_name, const char *default_ext, const char *mode)
{
  char *s;
  FILE *fp;
  s = b_ff(name, default_name, default_ext);
  if ( s == NULL )
    return NULL;
  fp = fopen(s, mode);
  free(s);
  return fp;
}

