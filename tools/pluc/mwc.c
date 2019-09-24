/*

  Memory Watch and Control

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



---example mwc.cfg file-----------------------------------------------------
; memory watch and control: config file
lower_space    8        ; should be multible of 8
upper_space    8        ; should be multible of 8
malloc_fill   -2        ; 0..255 value, -1 random, -2 none
free_fill      0        ; 0..255 value, -1 random, -2 none
space_fill     170      ; 0..255 value, -1 random
level          3        ; 0 (errors only) .. 3 (every error and warning)
rand_start     0        ; startvalue for pseudo random generator
alloc_fail    -1        ; fail specified alloc request (-1: never fail)
alloc_fail_cnt 1        ; nummber of allocs to fail
write_fail    -1        ; fail specified write request (-1: never fail)
write_fail_cnt 100000   ; nummber of writes to fail
read_fail     -1        ; fail specified read request (-1: never fail)
read_fail_cnt  100000   ; nummber of reads to fail
fail_on_reduce 0        ; 0 do not fail if size of block is reduced
stdout         1        ; 0 do not print report to stdout
report         0        ; 0 no output, 1 normal, 2 open, write, close
report_clear   1        ; 0 do not clear report file at startup
description    0        ; 0 do not show additional description
----------------------------------------------------------------------------


*/

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>     /* va_start, etc... */
#include <string.h>     /* memset */
#include <ctype.h>
#include <time.h>
#include <assert.h>
#include <errno.h>      /* errno variable */

struct _mwcei_struct
{
   size_t size;            /* memory, allocated by the user */
   long malloc_cnt;        /* counter for malloc */
   long realloc_cnt;       /* counter for realloc */
   const char *file_ptr;   /* filename */
   const char *arg_ptr;    /* optional arg */
   int line;               /* line number of sourcefile */
   int fill;
};
typedef struct _mwcei_struct mwcei_struct;

struct _mwce_struct
{
   unsigned short crc;   /* CRC16 */
   mwcei_struct i;
};
typedef struct _mwce_struct mwce_struct;

#define mwc_RandInit(x,a) \
   ((a)=(unsigned long)(a)*8UL+5UL,(x)=(unsigned long)(x)*4UL+1UL)

#define mwc_RandNext(x,a) \
   ((x)=(unsigned long)((unsigned long)(a)*(unsigned long)(x)),(x)>>2)


#define MWC_EXPAND 32
#define MWC_MAXPATH 1024
struct _mwc_struct
{
   long curr_alloc;
   long max_alloc;

   long alloc_cnt;
   long alloc_fail;
   long alloc_fail_cnt;
   int is_fail_on_reduce;
   long malloc_cnt;

   long write_cnt;
   long write_fail;
   long write_fail_cnt;

   long read_cnt;
   long read_fail;
   long read_fail_cnt;

   mwce_struct **list;
   size_t cnt;
   size_t search_start;

   char report_name[MWC_MAXPATH];

   FILE *fp;

   size_t mem_offset;

   /*---parameter---*/
   size_t lower_space;
   size_t upper_space;

   unsigned long crc_rand_a;
   unsigned long crc_rand_x;

   int malloc_filler;
   int free_filler;
   int space_filler;

   int is_report_to_stdout;
   int file_report;
   int is_report_clear;
   int is_show_description;

   int level;

   char buf[40];
};
typedef struct _mwc_struct mwc_struct;
typedef struct _mwc_struct *mwc_type;

void mwc_Printf(mwc_type mwc, int level, char *fmt, ...);
void mwc_PrintMemList(mwc_type mwc, int level);

/*-------------------------------------------------------------------------*/


mwc_type global_mwc = NULL;
int global_mwc_is_closed = 0;

/*--crc16------------------------------------------------------------------*/

/* the CRC polynomial 0x1021 */

static unsigned short crctab[1<<8] =
{
    0x0000,  0x1021,  0x2042,  0x3063,  0x4084,  0x50a5,  0x60c6,  0x70e7,
    0x8108,  0x9129,  0xa14a,  0xb16b,  0xc18c,  0xd1ad,  0xe1ce,  0xf1ef,
    0x1231,  0x0210,  0x3273,  0x2252,  0x52b5,  0x4294,  0x72f7,  0x62d6,
    0x9339,  0x8318,  0xb37b,  0xa35a,  0xd3bd,  0xc39c,  0xf3ff,  0xe3de,
    0x2462,  0x3443,  0x0420,  0x1401,  0x64e6,  0x74c7,  0x44a4,  0x5485,
    0xa56a,  0xb54b,  0x8528,  0x9509,  0xe5ee,  0xf5cf,  0xc5ac,  0xd58d,
    0x3653,  0x2672,  0x1611,  0x0630,  0x76d7,  0x66f6,  0x5695,  0x46b4,
    0xb75b,  0xa77a,  0x9719,  0x8738,  0xf7df,  0xe7fe,  0xd79d,  0xc7bc,
    0x48c4,  0x58e5,  0x6886,  0x78a7,  0x0840,  0x1861,  0x2802,  0x3823,
    0xc9cc,  0xd9ed,  0xe98e,  0xf9af,  0x8948,  0x9969,  0xa90a,  0xb92b,
    0x5af5,  0x4ad4,  0x7ab7,  0x6a96,  0x1a71,  0x0a50,  0x3a33,  0x2a12,
    0xdbfd,  0xcbdc,  0xfbbf,  0xeb9e,  0x9b79,  0x8b58,  0xbb3b,  0xab1a,
    0x6ca6,  0x7c87,  0x4ce4,  0x5cc5,  0x2c22,  0x3c03,  0x0c60,  0x1c41,
    0xedae,  0xfd8f,  0xcdec,  0xddcd,  0xad2a,  0xbd0b,  0x8d68,  0x9d49,
    0x7e97,  0x6eb6,  0x5ed5,  0x4ef4,  0x3e13,  0x2e32,  0x1e51,  0x0e70,
    0xff9f,  0xefbe,  0xdfdd,  0xcffc,  0xbf1b,  0xaf3a,  0x9f59,  0x8f78,
    0x9188,  0x81a9,  0xb1ca,  0xa1eb,  0xd10c,  0xc12d,  0xf14e,  0xe16f,
    0x1080,  0x00a1,  0x30c2,  0x20e3,  0x5004,  0x4025,  0x7046,  0x6067,
    0x83b9,  0x9398,  0xa3fb,  0xb3da,  0xc33d,  0xd31c,  0xe37f,  0xf35e,
    0x02b1,  0x1290,  0x22f3,  0x32d2,  0x4235,  0x5214,  0x6277,  0x7256,
    0xb5ea,  0xa5cb,  0x95a8,  0x8589,  0xf56e,  0xe54f,  0xd52c,  0xc50d,
    0x34e2,  0x24c3,  0x14a0,  0x0481,  0x7466,  0x6447,  0x5424,  0x4405,
    0xa7db,  0xb7fa,  0x8799,  0x97b8,  0xe75f,  0xf77e,  0xc71d,  0xd73c,
    0x26d3,  0x36f2,  0x0691,  0x16b0,  0x6657,  0x7676,  0x4615,  0x5634,
    0xd94c,  0xc96d,  0xf90e,  0xe92f,  0x99c8,  0x89e9,  0xb98a,  0xa9ab,
    0x5844,  0x4865,  0x7806,  0x6827,  0x18c0,  0x08e1,  0x3882,  0x28a3,
    0xcb7d,  0xdb5c,  0xeb3f,  0xfb1e,  0x8bf9,  0x9bd8,  0xabbb,  0xbb9a,
    0x4a75,  0x5a54,  0x6a37,  0x7a16,  0x0af1,  0x1ad0,  0x2ab3,  0x3a92,
    0xfd2e,  0xed0f,  0xdd6c,  0xcd4d,  0xbdaa,  0xad8b,  0x9de8,  0x8dc9,
    0x7c26,  0x6c07,  0x5c64,  0x4c45,  0x3ca2,  0x2c83,  0x1ce0,  0x0cc1,
    0xef1f,  0xff3e,  0xcf5d,  0xdf7c,  0xaf9b,  0xbfba,  0x8fd9,  0x9ff8,
    0x6e17,  0x7e36,  0x4e55,  0x5e74,  0x2e93,  0x3eb2,  0x0ed1,  0x1ef0
};

#define DO1 crc = (unsigned short)((crc<<8) ^ crctab[(crc>>8) ^ *buf++]);
#define DO2 DO1 DO1
#define DO4 DO2 DO2
#define DO8 DO4 DO4

static unsigned short mwc_crc16(unsigned short crc, unsigned char *buf, size_t len )
{
   while( len >= 8 )
   {
      DO8
      len -= 8;
   }
   if ( len != 0 )
   do
   {
      DO1
      len--;
   } while( len != 0 );
   return( crc );
}

/*--expr-------------------------------------------------------------------*/

char mwc_lower_space[] =   "lower_space";
char mwc_upper_space[] =   "upper_space";
char mwc_malloc_filler[] = "malloc_fill";
char mwc_free_filler[] =   "free_fill";
char mwc_space_filler[] =  "space_fill";
char mwc_rand_start[] =    "rand_start";
char mwc_level[] =         "level";
char mwc_alloc_fail[] =    "alloc_fail";
char mwc_alloc_fail_cnt[] ="alloc_fail_cnt";
char mwc_fail_on_reduce[] ="fail_on_reduce";

char mwc_write_fail[] =    "write_fail";
char mwc_write_fail_cnt[] ="write_fail_cnt";
char mwc_read_fail[] =     "read_fail";
char mwc_read_fail_cnt[] = "read_fail_cnt";

char mwc_is_report_to_stdout[] = "stdout";
char mwc_file_report[] =   "report";
char mwc_is_report_clear[] = "report_clear";
char mwc_is_show_description[] = "description";



long mwc_expr(char **s);

void mwc_skipspace(char **s)
{
   if ( **s == '\0' )
      return;
   while(isspace(**s))
      (*s)++;
   if ( (**s) == ';' )
   {
      while(**s != '\n' && **s != '\0')
         (*s)++;
   }
}

char *mwc_getid(char **s)
{
   static char t[1024];
   int i = 0;
   while( **s >= 'A' )
   {
      t[i] = **s;
      (*s)++;
      i++;
   }
   t[i] = '\0';
   return t;
}

long mwc_getval(char **s)
{
   long result = 0;
   while( isdigit(**s) ) 
   {
      result = result*10L+(long)(**s)-(long)'0';
      (*s)++;
   }
   mwc_skipspace(s);
   return result;
}

long mwc_getatom(char **s)
{
   long result, minus = 1L;
   if ( (**s) == '-' )
   {
      minus = -1;
      (*s)++; mwc_skipspace(s);
   }
   if ( (**s) == '(' )
   {
      (*s)++; mwc_skipspace(s);
      result = mwc_expr(s);
      if ( (**s) != ')' )
         puts("missing ')'");
      (*s)++; mwc_skipspace(s);
   }
   else
   {
      if ( **s >= '0' && **s <= '9' )
      {
         result = mwc_getval(s);
      }
      else
      {
         char *id;
         id = mwc_getid(s);
         result = 0L;
         if ( global_mwc != NULL )
         {
            if ( strcmp(id, mwc_lower_space ) == 0 )
               result = (long)global_mwc->lower_space;
            else if ( strcmp(id, mwc_lower_space ) == 0 )
               result = (long)global_mwc->upper_space;
            else if ( strcmp(id, mwc_malloc_filler ) == 0 )
               result = (long)global_mwc->malloc_filler;
            else if ( strcmp(id, mwc_free_filler ) == 0 )
               result = (long)global_mwc->free_filler;
            else if ( strcmp(id, mwc_space_filler ) == 0 )
               result = (long)global_mwc->space_filler;
            else if ( strcmp(id, mwc_rand_start ) == 0 )
               result = (long)(global_mwc->crc_rand_a-5L)/8;
            else if ( strcmp(id, mwc_level ) == 0 )
               result = (long)global_mwc->level;
            else if ( strcmp(id, mwc_alloc_fail_cnt ) == 0 )
               result = (long)global_mwc->alloc_fail_cnt;
            else if ( strcmp(id, mwc_alloc_fail ) == 0 )
               result = (long)global_mwc->alloc_fail;
            else if ( strcmp(id, mwc_write_fail ) == 0 )
               result = (long)global_mwc->write_fail;
            else if ( strcmp(id, mwc_write_fail_cnt ) == 0 )
               result = (long)global_mwc->write_fail_cnt;
            else if ( strcmp(id, mwc_read_fail ) == 0 )
               result = (long)global_mwc->read_fail;
            else if ( strcmp(id, mwc_read_fail_cnt ) == 0 )
               result = (long)global_mwc->read_fail_cnt;
            else if ( strcmp(id, mwc_fail_on_reduce ) == 0 )
               result = (long)global_mwc->is_fail_on_reduce;
            else if ( strcmp(id, mwc_is_report_to_stdout ) == 0 )
               result = (long)global_mwc->is_report_to_stdout;
            else if ( strcmp(id, mwc_file_report ) == 0 )
               result = (long)global_mwc->file_report;
            else if ( strcmp(id, mwc_is_report_clear ) == 0 )
               result = (long)global_mwc->is_report_clear;
            else if ( strcmp(id, mwc_is_show_description ) == 0 )
               result = (long)global_mwc->is_show_description;
            else
            {
               printf("unknown variable '%s'\n", id);
            }
         }
      }
   }
   return result*minus;
}

long mwc_muldiv(char **s)
{
   long v1,v2;
   char op;
   v1 = mwc_getatom(s);
   for(;;)
   {
      op = (**s);
      if ( op != '*' && op != '/' )
         break;
      (*s)++; mwc_skipspace(s);
      v2 = mwc_getatom(s);
      switch(op)
      {
         case '*': v1 *= v2; break;
         case '/': v1 /= v2; break;
      }
   }
   return v1;
}

long mwc_plusminus(char **s)
{
   long v1,v2;
   char op;
   v1 = mwc_muldiv(s);
   for(;;)
   {
      op = (**s);
      if ( op != '+' && op != '-' )
         break;
      (*s)++; mwc_skipspace(s);
      v2 = mwc_muldiv(s);
      switch(op)
      {
         case '+': v1 += v2; break;
         case '-': v1 -= v2; break;
      }
   }
   return v1;
}

long mwc_expr(char **s)
{
   mwc_skipspace(s);
   return mwc_plusminus(s);
}

int mwc_assignment(char **s)
{
   char *id;
   long result;
   mwc_skipspace(s);
   id = mwc_getid(s);
   if ( id == NULL )
      return 1;
   if ( id[0] == '\0' )
      return 1;

   mwc_skipspace(s);
   result = mwc_expr(s);

   if ( strcmp(id, mwc_lower_space ) == 0 )
      global_mwc->lower_space = (size_t)result;
   else if ( strcmp(id, mwc_upper_space ) == 0 )
      global_mwc->upper_space = (size_t)result;
   else if ( strcmp(id, mwc_malloc_filler ) == 0 )
      global_mwc->malloc_filler = (int)result;
   else if ( strcmp(id, mwc_free_filler ) == 0 )
      global_mwc->free_filler = (int)result;
   else if ( strcmp(id, mwc_space_filler ) == 0 )
      global_mwc->space_filler = (int)result;
   else if ( strcmp(id, mwc_rand_start ) == 0 )
      global_mwc->crc_rand_a = result*8L+5L;
   else if ( strcmp(id, mwc_level ) == 0 )
      global_mwc->level = (int)result;
   else if ( strcmp(id, mwc_alloc_fail_cnt ) == 0 )
      global_mwc->alloc_fail_cnt = (long)result;
   else if ( strcmp(id, mwc_alloc_fail ) == 0 )
      global_mwc->alloc_fail = (long)result;
   else if ( strcmp(id, mwc_write_fail_cnt ) == 0 )
      global_mwc->write_fail_cnt = (long)result;
   else if ( strcmp(id, mwc_write_fail ) == 0 )
      global_mwc->write_fail = (long)result;
   else if ( strcmp(id, mwc_read_fail_cnt ) == 0 )
      global_mwc->read_fail_cnt = (long)result;
   else if ( strcmp(id, mwc_read_fail ) == 0 )
      global_mwc->read_fail = (long)result;
   else if ( strcmp(id, mwc_fail_on_reduce ) == 0 )
      global_mwc->is_fail_on_reduce = (int)result;
   else if ( strcmp(id, mwc_is_report_to_stdout ) == 0 )
      global_mwc->is_report_to_stdout = (int)result;
   else if ( strcmp(id, mwc_file_report ) == 0 )
      global_mwc->file_report = (int)result;
   else if ( strcmp(id, mwc_is_report_clear ) == 0 )
      global_mwc->is_report_clear = (int)result;
   else if ( strcmp(id, mwc_is_show_description ) == 0 )
      global_mwc->is_show_description = (int)result;
   else
   {
      printf("variable '%s' unknown\n", id);
      return 0;
   }
   return 1;
}

int mwc_read_config(char *filename)
{
   static char s[256];
   char *t;
   FILE *fp;

   if ( global_mwc == NULL )
      return 0;

   fp = fopen(filename, "r");
   if ( fp == NULL )
      return 0;
   for(;;)
   {
      t = fgets(s, 256, fp);
      if ( t == NULL )
         break;
      mwc_assignment(&t);
   }
   fclose(fp);
   return 1;
}

/*--mwc--------------------------------------------------------------------*/


mwc_type mwc_Open(void)
{
   size_t i;
   mwc_type mwc = (mwc_type)malloc(sizeof(mwc_struct));
   if ( mwc != NULL )
   {
      strcpy(mwc->report_name, "mwc.out");
      mwc->curr_alloc = 0L;
      mwc->max_alloc = 0L;

      mwc->alloc_cnt = 0L;
      mwc->alloc_fail = -1L;
      mwc->alloc_fail_cnt = 1L;
      mwc->write_cnt = 0L;
      mwc->write_fail = -1L;
      mwc->write_fail_cnt = 1L;
      mwc->read_cnt = 0L;
      mwc->read_fail = -1L;
      mwc->read_fail_cnt = 1L;
      mwc->is_fail_on_reduce = 0;
      mwc->malloc_cnt = 0L;
      mwc->search_start = 0;
      mwc->list = (mwce_struct **)malloc( sizeof(mwce_struct *)*MWC_EXPAND );
      if ( mwc->list != NULL )
      {
         for( i = 0; i < MWC_EXPAND; i++ )
            mwc->list[i] = NULL;
         mwc->cnt = MWC_EXPAND;
         mwc->fp = NULL;
         mwc->file_report = 1;
         mwc->is_report_clear = 1;
         mwc->is_report_to_stdout = 0;
         mwc->is_show_description = 0;
         mwc->mem_offset = (size_t)((sizeof(mwce_struct)+3)&~3);

         mwc->lower_space = 8;
         mwc->upper_space = 8;

         mwc->crc_rand_a = 0;
         mwc->crc_rand_x = 0;
         mwc->malloc_filler = -2;      /* -1 rand, -2 none */
         mwc->free_filler = 0;         /* -1 rand, -2 none */
         mwc->space_filler = 0x0aa;    /* -1 rand */

         mwc->level = 3;
         mwc_RandInit(mwc->crc_rand_x, mwc->crc_rand_a);
         return mwc;
      }
      free(mwc);
   }
   return NULL;
}

void mwc_Close(mwc_type mwc)
{
   if ( mwc->fp != NULL )
      fclose(mwc->fp);
   free(mwc->list);
   free(mwc);
}

void mwc_exit(void)
{
   global_mwc_is_closed = 1;
   if ( global_mwc != NULL )
   {
      time_t t;
      time(&t);
      mwc_PrintMemList(global_mwc, 0);
      mwc_Printf(global_mwc, 0, "malloc/calloc: %ld  ", global_mwc->malloc_cnt );
      mwc_Printf(global_mwc, 0, "malloc/calloc/realloc: %ld\n", global_mwc->alloc_cnt );
      mwc_Printf(global_mwc, 0, "max allocated: %ld  ", global_mwc->max_alloc );
      mwc_Printf(global_mwc, 0, "current allocated: %ld\n", global_mwc->curr_alloc );
      mwc_Printf(global_mwc, 0, "write operations: %ld\n", global_mwc->write_cnt );
      mwc_Printf(global_mwc, 0, "mwc terminated %s\n", ctime(&t) );
      mwc_Close(global_mwc);
      global_mwc = NULL;
   }
   fflush(stdout);
}

mwc_type mwc_init(void)
{
   if ( global_mwc != NULL )
      return global_mwc;
   if ( global_mwc_is_closed != 0 )
      return NULL;
   global_mwc = mwc_Open();
   if ( global_mwc != NULL )
   {
      time_t t;
      time(&t);
      mwc_read_config("mwc.cfg");
      mwc_read_config("mwc_2nd.cfg");
      if ( global_mwc->is_report_clear != 0 )
         remove(global_mwc->report_name);
      mwc_Printf(global_mwc, 0, "mwc started %s", ctime(&t) );
      atexit( mwc_exit );
   }
   return global_mwc;
}



void mwc_Printf(mwc_type mwc, int level, char *fmt, ...)
{
   va_list v;
   if ( level <= mwc->level )
   {
      if ( mwc->file_report != 0 )
      {
         if ( mwc->fp == NULL )
         {
            mwc->fp = fopen(mwc->report_name, "a");
         }
         if ( mwc->fp != NULL )
         {
            va_start( v, fmt );     
            vfprintf(mwc->fp, fmt, v);
            va_end( v );            
            if ( mwc->file_report > 1 )
            {
               fclose(mwc->fp);
               mwc->fp = NULL;
            }
         }
      }
      if ( mwc->is_report_to_stdout != 0 )
      {
         va_start( v, fmt );     
         vfprintf(stdout, fmt, v);
         va_end( v );
         fflush(stdout);
      }
   }
}

void mwc_PrintFile(mwc_type mwc, int level, const char *file, int line, const char *org)
{
   mwc_Printf(mwc, level, "%s:%d (%s)", file, line, org);
}

void mwc_PrintPos(mwc_type mwc, int level, const char *fn, const char *arg, const char *file, int line, const char *org)
{
   mwc_PrintFile(mwc, level, file, line, org);
   mwc_Printf(mwc, level, " '%s'", arg);
}

void mwc_PrintPosSize(mwc_type mwc, int level, const char *fn, const char *arg, size_t size, const char *file, int line, const char *org)
{
   mwc_PrintPos(mwc, level, fn, arg, file, line, org);
   mwc_Printf(mwc, level, " %lu bytes", (unsigned long)size);
}

void mwc_PrintPosSizePtr(mwc_type mwc, int level, const char *fn, const char *arg, size_t size, const void *ptr, const char *file, int line, const char *org)
{
   mwc_PrintPos(mwc, level, fn, arg, file, line, org);
   mwc_Printf(mwc, level, " %lu bytes at adr %p", (unsigned long)size, ptr);
}

void mwc_PrintPosPtr(mwc_type mwc, int level, const char *fn, const char *arg, const void *ptr, const char *file, int line, const char *org)
{
   mwc_PrintPos(mwc, level, fn, arg, file, line, org);
   mwc_Printf(mwc, level, " adr %p", ptr);
}

void mwc_PrintPosMsg(mwc_type mwc, int level, const char *fn, const char *arg, const char *desc, const char *file, int line, const char *org)
{
   mwc_Printf(mwc, level, "%s: ", fn);
   mwc_PrintPos(mwc, level, fn, arg, file, line, org);
}

void mwc_PrintPosSizeMsg(mwc_type mwc, int level, const char *fn, const char *arg, size_t size, const char *desc, const char *file, int line, const char *org)
{
   mwc_Printf(mwc, level, "%s: ", fn);
   mwc_PrintPosSize(mwc, level, fn, arg, size, file, line, org);
   if ( mwc->is_show_description != 0 )
      mwc_Printf(mwc, level, " (%s)", desc);
   mwc_Printf(mwc, level, "\n");
}

void mwc_PrintPosSizePtrMsg(mwc_type mwc, int level, const char *fn, const char *arg, size_t size, const void *ptr, const char *desc, const char *file, int line, const char *org)
{
   mwc_Printf(mwc, level, "%s: ", fn);
   mwc_PrintPosSizePtr(mwc, level, fn, arg, size, ptr, file, line, org);
   if ( mwc->is_show_description != 0 )
      mwc_Printf(mwc, level, " (%s)", desc);
   mwc_Printf(mwc, level, "\n");
}

void mwc_PrintPosPtrMsg(mwc_type mwc, int level, const char *fn, const char *arg, const void *ptr, const char *desc, const char *file, int line, const char *org)
{
   mwc_Printf(mwc, level, "%s: ", fn);
   mwc_PrintPosPtr(mwc, level, fn, arg, ptr, file, line, org);
   if ( mwc->is_show_description != 0 )
      mwc_Printf(mwc, level, " (%s)", desc);
   mwc_Printf(mwc, level, "\n");
}


void mwc_PrintMemoryAllocation(mwc_type mwc, int level, mwce_struct *mwce)
{
   mwc_Printf(mwc, level, "alloc info (%ld): ", mwce->i.malloc_cnt);
   mwc_Printf(mwc, level, "%s:%d ", mwce->i.file_ptr, mwce->i.line);
   mwc_Printf(mwc, level, "'%s'=%lu bytes ", mwce->i.arg_ptr, (unsigned long)mwce->i.size);
   if ( mwce->i.realloc_cnt != 0L )
      mwc_Printf(mwc, level, "realloc nr. %ld\n", mwce->i.realloc_cnt);
   else
      mwc_Printf(mwc, level, "\n");
}

void mwc_PrintInfo(mwc_type mwc, int level, mwce_struct *mwce,
   const char *prefix, const char *arg, void *ptr, const char *file, int line)
{
   mwc_Printf(mwc, level, "%s info %s:%d '%s' at adr %p\n", prefix, file, line, arg, ptr);
   if ( mwce != NULL )
      mwc_PrintMemoryAllocation(mwc, level, mwce);
   else
      mwc_Printf(mwc, level, "unknown ptr %p\n", ptr);
}


int mwc_expand(mwc_type mwc)
{
   void *ptr;
   size_t i;
   ptr = realloc(mwc->list, sizeof(mwce_struct *)*(MWC_EXPAND+mwc->cnt));
   if ( ptr == NULL )
      return 0;
   mwc->list = (mwce_struct **)ptr;

   i = mwc->cnt;
   mwc->cnt += MWC_EXPAND;

   while( i < mwc->cnt)
      mwc->list[i++] = NULL;

   return 1;
}

size_t mwc_GetNewEntryPtr(mwc_type mwc)
{
   size_t i, s;
   s = mwc->search_start;
   for( i = 0; i < mwc->cnt; i++ )
   {
      if ( mwc->list[s] == NULL )
         return s;
      s++;
      if ( s >= mwc->cnt )
         s = 0;
   }
   mwc->search_start = s;
   s = mwc->cnt;
   if ( mwc_expand(mwc) == 0 )
      return mwc->cnt;
   return s;
}

void mwc_InitMemory(mwc_type mwc, mwce_struct *mwce)
{
   char fill;
   size_t i;
   if ( mwc->space_filler < 0 )
      fill = (char)(mwc_RandNext(mwc->crc_rand_x, mwc->crc_rand_a) >> 3);
   else
      fill = (char)mwc->space_filler;

   for( i = 0; i < mwc->lower_space; i++ )
      ((char *)mwce)[mwc->mem_offset+i] = fill;
   for( i = 0; i < mwc->upper_space; i++ )
      ((char *)mwce)[mwc->mem_offset+mwc->lower_space+mwce->i.size+i] = fill;

   {
      int f;
      if ( mwc->malloc_filler > -2 )
      {
         if ( mwc->malloc_filler < 0 )
            f = (int)(mwc_RandNext(mwc->crc_rand_x, mwc->crc_rand_a) >> 3);
         else
            f = mwc->malloc_filler;
         memset( ((char *)mwce)+mwc->mem_offset+mwc->lower_space,
            f&255, mwce->i.size );
      }
   }

   mwce->i.fill = (int)fill;
}

int mwc_IsInternalOK(mwc_type mwc, mwce_struct *mwce)
{
   if ( mwce->crc ==
      mwc_crc16(0, (unsigned char *)&(mwce->i), sizeof(mwcei_struct)) )
      return 1;
   return 0;
}

int mwc_IsLowerOK(mwc_type mwc, mwce_struct *mwce)
{
   char fill = (char)mwce->i.fill;
   size_t i;

   for( i = 0; i < mwc->lower_space; i++ )
      if ( ((char *)mwce)[mwc->mem_offset+i] != fill )
         return 0;

   return 1;
}

int mwc_IsUpperOK(mwc_type mwc, mwce_struct *mwce)
{
   char fill = (char)mwce->i.fill;
   size_t i;

   for( i = 0; i < mwc->upper_space; i++ )
      if ( ((char *)mwce)[mwc->mem_offset+mwc->lower_space+mwce->i.size+i] != fill )
         return 0;

   return 1;
}

void mwc_CheckMWCE(mwc_type mwc, mwce_struct *mwce, const char *arg, const char *file, int line, const char *org)
{
   if ( mwc_IsInternalOK(mwc, mwce) == 0 )
   {
      mwc_PrintPosSizePtrMsg(mwc,
         0,
         "write below",
         arg,
         mwce->i.size,
         (void *)(mwc->mem_offset+mwc->lower_space+(char *)mwce),
         "below ptr check failed (crc)",
         file,
         line,
         org);
      mwc_PrintMemoryAllocation(mwc, 0, mwce);
      return;
   }
   if ( mwc_IsLowerOK(mwc, mwce) == 0 )
   {
      mwc_PrintPosSizePtrMsg(mwc,
         0,
         "write below",
         arg,
         mwce->i.size,
         (void *)(mwc->mem_offset+mwc->lower_space+(char *)mwce),
         "below ptr check failed",
         file,
         line,
         org);
      mwc_PrintMemoryAllocation(mwc, 0, mwce);
   }
   if ( mwc_IsUpperOK(mwc, mwce) == 0 )
   {
      mwc_PrintPosSizePtrMsg(mwc,
         0,
         "write above",
         arg,
         mwce->i.size,
         (void *)(mwc->mem_offset+mwc->lower_space+(char *)mwce),
         "above ptr check failed",
         file,
         line,
         org);
      mwc_PrintMemoryAllocation(mwc, 0, mwce);
   }
}

void mwc_PrintMemList(mwc_type mwc, int level)
{
   size_t i;
   for( i = 0; i < mwc->cnt; i++ )
   {
      if ( mwc->list[i] != NULL )
         break;
   }
   if ( i >= mwc->cnt )
   {
      mwc_Printf(mwc, level, "no memory block allocated\n");
      return;
   }

   mwc_Printf(mwc, level,
   "---list-of-allocated-memory-blocks-----------------------------------\n");
   mwc_Printf(mwc, level,
   "errors: 'c' internal crc, 'l' lower write, 'u' upper write\n");
   mwc_Printf(mwc, level,
   "nr.  size    clu argument\n");
   for( i = 0; i < mwc->cnt; i++ )
   {
      if ( mwc->list[i] == NULL )
         continue;

      mwc_Printf(mwc, level,
         "%4ld ", mwc->list[i]->i.malloc_cnt );
      mwc_Printf(mwc, level,
         "%7ld ", (long)mwc->list[i]->i.size );
      if ( mwc_IsInternalOK(mwc, mwc->list[i]) == 0 )
      {
         mwc_Printf(mwc, level, "c??");
      }
      else
      {
         mwc_Printf(mwc, level, "-");
         if ( mwc_IsLowerOK(mwc, mwc->list[i]) == 0 )
            mwc_Printf(mwc, level, "l");
         else
            mwc_Printf(mwc, level, "-");
         if ( mwc_IsUpperOK(mwc, mwc->list[i]) == 0 )
            mwc_Printf(mwc, level, "u");
         else
            mwc_Printf(mwc, level, "-");
      }
      mwc_Printf(mwc, level,
         " '%s' ", (long)mwc->list[i]->i.arg_ptr );
      mwc_Printf(mwc, level,
         "%s:%d ", mwc->list[i]->i.file_ptr, mwc->list[i]->i.line );
      mwc_Printf(mwc, level, "\n");
   }
}

size_t mwc_FindMWCEPos(mwc_type mwc, void *ptr)
{
   size_t i;
   for( i = 0; i < mwc->cnt; i++ )
   {
      if ( mwc->list[i] != NULL )
         if ( mwc->mem_offset+mwc->lower_space+(char *)(mwc->list[i]) == (char *)ptr )
            return i;
   }
   return mwc->cnt;
}

mwce_struct *mwc_FindMWCE(mwc_type mwc, void *ptr)
{
   size_t pos;
   pos = mwc_FindMWCEPos(mwc, ptr);
   if( pos >= mwc->cnt )
      return NULL;
   return mwc->list[pos];
}

void *mwc_malloc(size_t size, const char *arg, const char *file, int line)
{
   mwc_type mwc = mwc_init();
   mwce_struct *mwce;
   void *ptr;
   size_t pos;
   size_t os_size;

   if ( mwc == NULL )
      return NULL;

   if ( mwc->alloc_cnt >= mwc->alloc_fail && mwc->alloc_cnt < mwc->alloc_fail+mwc->alloc_fail_cnt )
   {
      sprintf(mwc->buf, "force NULL (%ld)", mwc->alloc_cnt);
      mwc_PrintPosSizeMsg(
         mwc,
         1,
         mwc->buf,
         arg,
         size,
         "NULL pointer is returned (alloc_fail)",
         file,
         line,
         "malloc");
      mwc->alloc_cnt++;
      mwc->malloc_cnt++;
      return NULL;
   }

   pos = mwc_GetNewEntryPtr(mwc);
   if ( pos >= mwc->cnt )
   {
      mwc_PrintPosSizeMsg(
         mwc,
         0,
         "malloc failed",
         arg,
         size,
         "out of memory within internal structures",
         file,
         line,
         "malloc");
      return NULL;
   }

   os_size = (size_t)(mwc->mem_offset+mwc->lower_space+size+mwc->upper_space);

   ptr = malloc(os_size);
   if ( ptr == NULL )
   {
      mwc_PrintPosSizeMsg(
         mwc,
         0,
         "malloc failed",
         arg,
         size,
         "no memory from os",
         file,
         line,
         "malloc");
      return NULL;
   }

   mwc->list[pos] = (mwce_struct *)ptr;
   mwce = mwc->list[pos];

   mwce->i.file_ptr = file;
   mwce->i.arg_ptr = arg;
   mwce->i.line = line;
   mwce->i.size = size;
   mwc->malloc_cnt++;
   mwc->curr_alloc += (long)size;
   if ( mwc->max_alloc < mwc->curr_alloc )
      mwc->max_alloc = mwc->curr_alloc;
   mwce->i.malloc_cnt = mwc->alloc_cnt++;
   mwce->i.realloc_cnt = 0L;

   mwc_InitMemory(mwc, mwce);

   mwce->crc =
      mwc_crc16(0, (unsigned char *)&(mwce->i), sizeof(mwcei_struct));

   mwc_PrintInfo(mwc, 3, mwce,
      "malloc", arg, (void *)(mwc->mem_offset+mwc->lower_space+(char *)mwce), file, line);

   return (void *)(mwc->mem_offset+mwc->lower_space+(char *)mwce);
}

char *mwc_strdup(const char *str, const char *arg, const char *file, int line)
{
   mwc_type mwc = mwc_init();
   mwce_struct *mwce;
   void *ptr;
   size_t pos;
   size_t size;
   size_t os_size;

   if ( mwc == NULL )
      return NULL;
      
   if ( str == NULL )
   {
      mwc_PrintPosPtrMsg(
         mwc,
         1,
         "NULL ptr",
         arg,
         str,
         "NULL pointer as string",
         file,
         line,
         "strdup");
      return NULL;
   }

   size = strlen(str)+1;

   if ( mwc->alloc_cnt >= mwc->alloc_fail && mwc->alloc_cnt < mwc->alloc_fail+mwc->alloc_fail_cnt )
   {
      sprintf(mwc->buf, "force NULL (%ld)", mwc->alloc_cnt);
      mwc_PrintPosSizeMsg(
         mwc,
         1,
         mwc->buf,
         arg,
         size,
         "NULL pointer is returned (alloc_fail)",
         file,
         line,
         "strdup");
      mwc->alloc_cnt++;
      mwc->malloc_cnt++;
      return NULL;
   }

   pos = mwc_GetNewEntryPtr(mwc);
   if ( pos >= mwc->cnt )
   {
      mwc_PrintPosSizeMsg(
         mwc,
         0,
         "malloc failed",
         arg,
         size,
         "out of memory within internal structures",
         file,
         line,
         "strdup");
      return NULL;
   }

   os_size = (size_t)(mwc->mem_offset+mwc->lower_space+size+mwc->upper_space);

   ptr = malloc(os_size);
   if ( ptr == NULL )
   {
      mwc_PrintPosSizeMsg(
         mwc,
         0,
         "malloc failed",
         arg,
         size,
         "no memory from os",
         file,
         line,
         "strdup");
      return NULL;
   }

   mwc->list[pos] = (mwce_struct *)ptr;
   mwce = mwc->list[pos];

   mwce->i.file_ptr = file;
   mwce->i.arg_ptr = arg;
   mwce->i.line = line;
   mwce->i.size = size;
   mwc->malloc_cnt++;
   mwc->curr_alloc += (long)size;
   if ( mwc->max_alloc < mwc->curr_alloc )
      mwc->max_alloc = mwc->curr_alloc;
   mwce->i.malloc_cnt = mwc->alloc_cnt++;
   mwce->i.realloc_cnt = 0L;

   mwc_InitMemory(mwc, mwce);

   mwce->crc =
      mwc_crc16(0, (unsigned char *)&(mwce->i), sizeof(mwcei_struct));

   mwc_PrintInfo(mwc, 3, mwce,
      "strdup", arg, (void *)(mwc->mem_offset+mwc->lower_space+(char *)mwce), file, line);

   strcpy((mwc->mem_offset+mwc->lower_space+(char *)mwce), str);

   return (mwc->mem_offset+mwc->lower_space+(char *)mwce);
}

void *mwc_calloc(size_t num, size_t size, const char *arg, const char *file, int line)
{
   mwc_type mwc = mwc_init();
   mwce_struct *mwce;
   void *ptr;
   size_t pos;
   size_t os_size;

   if ( mwc == NULL )
      return NULL;

   if ( mwc->alloc_cnt >= mwc->alloc_fail && mwc->alloc_cnt < mwc->alloc_fail+mwc->alloc_fail_cnt )
   {
      sprintf(mwc->buf, "force NULL (%ld)", mwc->alloc_cnt);
      mwc_PrintPosSizeMsg(
         mwc,
         1,
         mwc->buf,
         arg,
         size,
         "NULL pointer is returned (alloc_fail)",
         file,
         line,
         "calloc");
      mwc->alloc_cnt++;
      mwc->malloc_cnt++;
      return NULL;
   }


   if ( sizeof(size_t) < 4 )
   {
      if ( (long)num*(long)size > 65500L-
         (long)sizeof(mwc->mem_offset)-
         (long)mwc->lower_space-
         (long)mwc->upper_space )
      {
         mwc_PrintPosSizeMsg(
            mwc,
            0,
            "calloc failed",
            arg,
            0,
            "can not simulate calloc call",
            file,
            line,
            "calloc");
         return NULL;
      }
   }


   pos = mwc_GetNewEntryPtr(mwc);
   if ( pos >= mwc->cnt )
   {
      mwc_PrintPosSizeMsg(
         mwc,
         0,
         "calloc failed",
         arg,
         size*num,
         "out of memory within internal structures",
         file,
         line,
         "calloc");
      return NULL;
   }

   os_size =
      (size_t)(mwc->mem_offset+mwc->lower_space+size*num+mwc->upper_space);

   ptr = malloc(os_size);
   if ( ptr == NULL )
   {
      mwc_PrintPosSizeMsg(
         mwc,
         0,
         "calloc failed",
         arg,
         size,
         "no memory from os",
         file,
         line,
         "calloc");
      return NULL;
   }

   memset(ptr, 0, os_size);
   mwc->list[pos] = (mwce_struct *)ptr;
   mwce = mwc->list[pos];

   mwce->i.file_ptr = file;
   mwce->i.arg_ptr = arg;
   mwce->i.line = line;
   mwce->i.size = size;
   mwc->malloc_cnt++;
   mwc->curr_alloc += (long)size;
   if ( mwc->max_alloc < mwc->curr_alloc )
      mwc->max_alloc = mwc->curr_alloc;
   mwce->i.malloc_cnt = mwc->alloc_cnt++;
   mwce->i.realloc_cnt = 0L;

   mwc_InitMemory(mwc, mwce);

   mwce->crc =
      mwc_crc16(0, (unsigned char *)&(mwce->i), sizeof(mwcei_struct));

   mwc_PrintInfo(mwc, 3, mwce,
      "calloc", arg, (void *)(mwc->mem_offset+mwc->lower_space+(char *)mwce), file, line);

   return (void *)(mwc->mem_offset+mwc->lower_space+(char *)mwce);
}

void _mwc_ptr_check(void *ptr, const char *arg, const char *file, int line)
{
   mwc_type mwc = mwc_init();
   mwce_struct *mwce = NULL;

   if ( mwc == NULL )
      return;

   mwce = mwc_FindMWCE(mwc, ptr);
   if ( mwce == NULL )
   {
      mwc_PrintPosPtrMsg(
         mwc,
         1,
         "unknown ptr",
         arg,
         (void *)(mwc->mem_offset+mwc->lower_space+(char *)mwce),
         "pointer for check is unknown",
         file,
         line,
         "mwc_check");
   }
   else
   {
      mwc_CheckMWCE(mwc, mwce, arg, file, line, "mwc_check");
   }
}

void *mwc_realloc(void *old_ptr, size_t new_len, const char *arg, const char *file, int line)
{
   mwc_type mwc = mwc_init();
   mwce_struct *mwce = NULL;
   void *new_ptr = NULL;
   size_t os_size;
   size_t pos;

   if ( mwc == NULL )
      return NULL;


   if ( old_ptr == NULL )
   {


      /*------------- realloc used as malloc -------------*/


      if ( mwc->alloc_cnt >= mwc->alloc_fail && mwc->alloc_cnt < mwc->alloc_fail+mwc->alloc_fail_cnt )
      {
         sprintf(mwc->buf, "force NULL (%ld)", mwc->alloc_cnt);
         mwc_PrintPosSizeMsg(
            mwc,
            1,
            mwc->buf,
            arg,
            new_len,
            "NULL pointer is returned (alloc_fail)",
            file,
            line,
            "realloc");
         mwc->alloc_cnt++;
         return NULL;
      }

      mwc_PrintPosSizeMsg(
         mwc,
         2,
         "NULL-realloc",
         arg,
         new_len,
         "pointer for realloc is NULL",
         file,
         line,
         "realloc");
   }
   else
   {

      /*------------- non NULL pointer to realloc -------------*/

      pos = mwc_FindMWCEPos(mwc, old_ptr);
      if ( pos >= mwc->cnt )
         mwce = NULL;
      else
      {
         mwce = mwc->list[pos];
      }

      if ( mwce == NULL )
      {
         if ( mwc->alloc_cnt >= mwc->alloc_fail && mwc->alloc_cnt < mwc->alloc_fail+mwc->alloc_fail_cnt )
         {
            sprintf(mwc->buf, "force NULL (%ld)", mwc->alloc_cnt);
            mwc_PrintPosSizeMsg(
               mwc,
               1,
               mwc->buf,
               arg,
               new_len,
               "NULL pointer is returned (alloc_fail)",
               file,
               line,
               "realloc");
            mwc->alloc_cnt++;
            return NULL;
         }

         mwc_PrintPosSizePtrMsg(
            mwc,
            1,
            "unknown realloc",
            arg,
            new_len,
            (void *)(mwc->mem_offset+mwc->lower_space+(char *)mwce),
            "pointer for realloc is unknown",
            file,
            line,
            "realloc");
      }
      else
      {
         if ( mwc->is_fail_on_reduce != 0 || mwce->i.size < new_len )
         {
            if ( mwc->alloc_cnt >= mwc->alloc_fail && mwc->alloc_cnt < mwc->alloc_fail+mwc->alloc_fail_cnt )
            {
               sprintf(mwc->buf, "force NULL (%ld)", mwc->alloc_cnt);
               mwc_PrintPosSizeMsg(
                  mwc,
                  1,
                  mwc->buf,
                  arg,
                  new_len,
                  "NULL pointer is returned (alloc_fail)",
                  file,
                  line,
                  "realloc");
               mwc->alloc_cnt++;
               return NULL;
            }
         }
         mwc_CheckMWCE(mwc, mwce, arg, file, line, "realloc");
         mwc->curr_alloc -= (long)mwce->i.size;
      }
   }

   os_size = (size_t)(mwc->mem_offset+mwc->lower_space+new_len+mwc->upper_space);

   if ( mwce == NULL )
   {
      pos = mwc_GetNewEntryPtr(mwc);
      if ( pos >= mwc->cnt )
      {
         mwc_PrintPosSizeMsg(
            mwc,
            0,
            "realloc failed",
            arg,
            new_len,
            "out of memory within internal structures",
            file,
            line,
            "realloc");
         return NULL;
      }

      new_ptr = realloc(old_ptr, os_size);
      if ( new_ptr == NULL )
      {
         mwc_PrintPosSizeMsg(
            mwc,
            0,
            "realloc failed",
            arg,
            new_len,
            "system realloc returns NULL",
            file,
            line,
            "realloc");
         return NULL;
      }
      mwc->list[pos] = (mwce_struct *)new_ptr;
      mwce = mwc->list[pos];

      mwce->i.file_ptr = file;
      mwce->i.arg_ptr = arg;
      mwce->i.line = line;
      mwce->i.size = new_len;
      mwc->malloc_cnt++;
      mwc->curr_alloc += (long)new_len;
      if ( mwc->max_alloc < mwc->curr_alloc )
         mwc->max_alloc = mwc->curr_alloc;
      mwce->i.malloc_cnt = mwc->alloc_cnt++;
      mwce->i.realloc_cnt = 0L;

      mwc_InitMemory(mwc, mwce);

      mwce->crc =
         mwc_crc16(0, (unsigned char *)&(mwce->i), sizeof(mwcei_struct));

      mwc_PrintInfo(mwc, 3, mwce,
         "realloc", arg, (void *)(mwc->mem_offset+mwc->lower_space+(char *)mwce),
         file, line);

      return (void *)(mwc->mem_offset+mwc->lower_space+(char *)mwce);
   }
   assert( pos < mwc->cnt );

   new_ptr = realloc((void *)mwce, os_size);
   if ( new_ptr == NULL )
   {
      mwc_PrintPosSizeMsg(
         mwc,
         0,
         "realloc failed",
         arg,
         new_len,
         "system realloc returns NULL",
         file,
         line,
         "realloc");
      return NULL;
   }
   mwc->list[pos] = (mwce_struct *)new_ptr;
   mwce = mwc->list[pos];

   mwc->alloc_cnt++;
   mwce->i.realloc_cnt++;
   mwce->i.file_ptr = file;
   mwce->i.arg_ptr = arg;
   mwce->i.line = line;
   mwce->i.size = new_len;
   mwc_InitMemory(mwc, mwce);
   mwce->crc =
      mwc_crc16(0, (unsigned char *)&(mwce->i), sizeof(mwcei_struct));
   mwc->curr_alloc += (long)mwce->i.size;
   if ( mwc->max_alloc < mwc->curr_alloc )
      mwc->max_alloc = mwc->curr_alloc;

   mwc_PrintInfo(mwc, 3, mwce,
      "realloc", arg, (void *)(mwc->mem_offset+mwc->lower_space+(char *)mwce),
      file, line);

   return (void *)(mwc->mem_offset+mwc->lower_space+(char *)mwce);
}

void mwc_free(void *ptr, const char *arg, const char *file, int line)
{
   mwc_type mwc = mwc_init();
   size_t pos;
   mwce_struct *mwce = NULL;

   if ( mwc == NULL )
      return;

   pos = mwc_FindMWCEPos(mwc, ptr);
   if ( pos >= mwc->cnt )
   {
      mwc_PrintPosPtrMsg(
         mwc,
         1,
         "unknown free",
         arg,
         ptr,
         "pointer for free is unknown",
         file,
         line,
         "free");
      free(ptr);
   }
   else
   {
      mwce = mwc->list[pos];

      mwc_CheckMWCE(mwc, mwce, arg, file, line, "free");
      mwc_PrintInfo(mwc, 3, mwce, "free", arg, ptr, file, line);
      mwc->curr_alloc -= (long)mwc->list[pos]->i.size;

      {
         int fill;
         size_t os_size = (size_t)(mwc->mem_offset+
            mwc->lower_space+mwce->i.size+mwc->upper_space);
         if ( mwc->free_filler > -2 )
         {
            if ( mwc->free_filler < 0 )
               fill = (int)(mwc_RandNext(mwc->crc_rand_x, mwc->crc_rand_a) >> 3);
            else
               fill = mwc->free_filler;
            memset( (void *)mwce, fill&255, os_size );
         }
      }

      free(mwc->list[pos]);
      mwc->list[pos] = NULL;
   }

}

int mwc_fputc(int c, FILE *stream, const char *arg, const char *file, int line)
{
   mwc_type mwc = mwc_init();

   if ( mwc->write_cnt >= mwc->write_fail && mwc->write_cnt < mwc->write_fail+mwc->write_fail_cnt )
   {
      sprintf(mwc->buf, "force EOF (%ld)", mwc->write_cnt);
      
      mwc_PrintPosMsg(
        mwc, 
        1, 
        mwc->buf, 
        arg, 
        "EOF returned (write_fail)", 
        file, 
        line, 
        "fputc");
        
      mwc->write_cnt++;
      errno = ENOMEM;     /* simulate "Insufficient storage space" */
      return EOF;
   }
  
   mwc->write_cnt++;
   return fputc(c, stream);
}

int mwc_fwrite(const void *ptr, size_t size,  size_t  nitems,
     FILE *stream, const char *arg, const char *file, int line)
{
   mwc_type mwc = mwc_init();

   if ( mwc->write_cnt >= mwc->write_fail && mwc->write_cnt < mwc->write_fail+mwc->write_fail_cnt )
   {
      sprintf(mwc->buf, "force EOF (%ld)", mwc->write_cnt);
      
      mwc_PrintPosMsg(
        mwc, 
        1, 
        mwc->buf, 
        arg, 
        "EOF returned (write_fail)", 
        file, 
        line, 
        "fwrite");
        
      mwc->write_cnt++;
      errno = ENOMEM;     /* simulate "Insufficient storage space" */
      return 0;
   }
  
   mwc->write_cnt++;
   return fwrite(ptr, size, nitems, stream);
}


