/*

  CMDLINE.H

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
   
   On unix machines, use -DUNIX
   
*/

#ifndef _CMDLINE_H
#define _CMDLINE_H

#ifdef _MSC_VER
#pragma warning(disable:4131)
#ifndef DO_PROTO
#define DO_PROTO
#endif
#endif

#include <stdlib.h>
#ifndef _MAX_PATH
#define _MAX_PATH 1024
#endif

#define CL_LINE_LEN 2048

#define CL_ERR_STRING "\1"

#define CL_TYP_NONE   0
#define CL_TYP_ON     1   /* int */
#define CL_TYP_OFF    2   /* int */
#define CL_TYP_LONG   3   /* long */
#define CL_TYP_PATH   4   /* char * */
#define CL_TYP_STRING 5   /* char * */
#define CL_TYP_SET    6   /* long */
#define CL_TYP_ESCSEQ 7   /* char * */
#define CL_TYP_GROUP  8   /* nothing, only for help text */
#define CL_TYP_NAME   9   /* char *, same as CL_TYP_STRING but different help */

#define CL_ENTRY_LAST { CL_TYP_NONE, NULL, NULL, 0 }

struct _cl_entry_struct
{
   int typ;
   char *name;
   void *var_ptr;
   size_t var_size;
};
typedef struct _cl_entry_struct cl_entry_struct;

#define CL_FILE_MAX 64
#define CL_HELP_TEXT_DELIMITER '-'

extern char cl_file_list[CL_FILE_MAX][_MAX_PATH];
extern int cl_file_cnt;

#ifdef DO_PROTO
int cl_Do(cl_entry_struct *list, int argc, char *argv[]);
int cl_String(cl_entry_struct *list, char *s);
void cl_OutHelp(cl_entry_struct *entry, FILE *fp, char *prefix, int fl1);
#else
extern int cl_Do();
extern int cl_String();
extern void cl_OutHelp();
#endif

#endif
