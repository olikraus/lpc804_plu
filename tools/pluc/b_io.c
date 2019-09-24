/*
  
  b_io.c
  
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

#include "b_io.h"
#include <string.h>
#include <stdlib.h>

#define B_IO_BYTE_ARRAY_MAX 70

int b_io_WriteChar(FILE *fp, char c)
{
  int cc = ((int)c)&255;
  if ( putc(cc, fp) != cc )
    return 0;
  return 1;
}

int b_io_ReadChar(FILE *fp, char *c)
{
  int cc;  
  cc = getc(fp);
  if ( cc < 0 )
    return 0;
  *c = cc;
  return 1;
}

int b_io_Write(FILE *fp, int cnt, const unsigned char *ptr)
{
  if ( fwrite(ptr, cnt, 1, fp) != 1 )
    return 0;
  return 1;
}

int b_io_Read(FILE *fp, int cnt, unsigned char *ptr)
{
  if ( fread(ptr, cnt, 1, fp) != 1 )
    return 0;
  return 1;
}

int b_io_WriteLengthByteArray(FILE *fp, unsigned char *ptr)
{
  int cnt = (*ptr & 63);
  if (cnt >= B_IO_BYTE_ARRAY_MAX)
    return 0;
  if ( putc((int)*ptr, fp) != *ptr )
    return 0;
  ptr++;
  while( cnt > 0 )
  {
    cnt--;
    if ( putc((int)*ptr, fp) != *ptr )
      return 0;
    ptr++;
  }
  return 1;
}

int b_io_ReadLengthByteArray(FILE *fp, unsigned char *ptr)
{
  int v, cnt;
  cnt = getc(fp);
  if ( cnt == EOF )
    return 0;
  *ptr = (unsigned char)cnt;
  cnt = (cnt & 63);
  if ( cnt >= B_IO_BYTE_ARRAY_MAX )
    return 0;
  ptr++;
  while( cnt > 0 )
  {
    cnt--;
    v = getc(fp);
    if ( v == EOF )
      return 0;
    *ptr = (unsigned char)v;
    ptr++;
  }
  return 1;
}

int b_io_WriteUnsignedInt(FILE *fp, unsigned val)
{
  unsigned char b[B_IO_BYTE_ARRAY_MAX+1];
  int i;
  for( i = 1; i < B_IO_BYTE_ARRAY_MAX; i++ )
  {
    if ( val == 0 )
      break;
    b[i] = (val&255);
    val>>=8;
  }
  b[0] = i-1;
  return b_io_WriteLengthByteArray(fp, b);
}

int b_io_ReadUnsignedInt(FILE *fp, unsigned *val)
{
  unsigned char b[B_IO_BYTE_ARRAY_MAX+1];
  int i, cnt;
  if ( b_io_ReadLengthByteArray(fp, b) == 0 )
    return 0;
  *val = 0;
  cnt = (int)(unsigned)b[0];
  for( i = 0; i < cnt; i++ )
  {
    *val <<= 8;
    *val |= (unsigned)b[cnt-1-i+1];
  }
  return 1;
}

static int b_io_write_int(FILE *fp, int val, int flag)
{
  unsigned char b[B_IO_BYTE_ARRAY_MAX+1];
  int i;
  unsigned char is_not = 0;
  if ( val < 0 )
  {
    is_not = 128;
    val = ~val;
  }
    
  for( i = 1; i < B_IO_BYTE_ARRAY_MAX; i++ )
  {
    if ( val == 0 )
      break;
    b[i] = (val&255);
    val>>=8;
  }
  b[0] = ((unsigned char)(i-1))|is_not;
  if ( flag != 0 )
    b[0] |= 64;
  return b_io_WriteLengthByteArray(fp, b);
}

int b_io_WriteInt(FILE *fp, int val)
{
  return b_io_write_int(fp, val, 0);
}


static int b_io_read_int(FILE *fp, int *val, int *flag)
{
  unsigned char b[B_IO_BYTE_ARRAY_MAX+1];
  int i, cnt;
  unsigned char is_not = 0;
  if ( b_io_ReadLengthByteArray(fp, b) == 0 )
    return 0;
  *flag = 0;
  *val = 0;
  cnt = ((int)(unsigned)b[0])&63;
  is_not = (b[0]&128);
  if ( (b[0]&64) != 0 )
    *flag = 1;
  for( i = 0; i < cnt; i++ )
  {
    *val <<= 8;
    *val |= (unsigned)b[cnt-1-i+1];
  }
  if ( is_not != 0 )
    *val = ~*val;
  return 1;
}

int b_io_ReadInt(FILE *fp, int *val)
{
  int flag;
  return b_io_read_int(fp, val, &flag);
}


int b_io_WriteString(FILE *fp, const char *s)
{
  int len;
  if ( s == NULL )
  {
    if ( b_io_WriteInt(fp, -1) == 0 )
      return 0;
    return 1;
  }
  len = strlen(s);
  if ( b_io_WriteInt(fp, len) == 0 )
    return 0;
  if ( fwrite(s, len, 1, fp) != 1 )
    return 0;
  return 1;
}

int b_io_ReadAllocString(FILE *fp, char **s_ptr)
{
  int len;
  char *s;
  if ( b_io_ReadInt(fp, &len) == 0 )
    return 0;
  if ( len < 0 )
  {
    *s_ptr = NULL;
    return 1;
  }
  s = (char *)malloc(len+1);
  if ( s == NULL )
    return 0;
  if ( fread(s, len, 1, fp) != 1 )
    return 0;
  s[len] = '\0';
  *s_ptr = s;
  return 1;
}

int b_io_WriteIntArray(FILE *fp, int cnt, int *values)
{
  int i;
  for( i = 0; i < cnt; i++)
    if ( b_io_WriteInt(fp, values[i]) == 0 )
      return 0;
  return 1;
}

int b_io_ReadIntArray(FILE *fp, int cnt, int *values)
{
  int i;
  for( i = 0; i < cnt; i++)
    if ( b_io_ReadInt(fp, values+i) == 0 )
      return 0;
  return 1;
}

/*----------------------------------------------------------------------------*/

#include <math.h>
#include "mwc.h"

int b_io_WriteDouble(FILE *fp, double x)
{
  int a[sizeof(double)];
  int cnt, max = sizeof(double);
  int bcnt;
  double exp;
  int flag = 0;

  if ( x < 0.0 )
  {
    x = -x;
    flag = 1;
  }

  exp = ceil(log(x)/log(2.0));
  x = x*pow(2.0, -exp);
  
  if ( b_io_write_int(fp, (int)exp, flag) == 0 )
    return 0;
    
  for( cnt = 0; cnt < max; cnt++ )
  {
    a[cnt] = 0;
    for( bcnt = 0; bcnt < 8; bcnt++ )
    {
      x *= 2.0;
      if ( x >= 1.0 )
      {
        a[cnt] |= 1<<(7-bcnt);
        x -= 1.0;
      }
    }
  }
  
  if ( putc(max, fp) != max )
    return 0;
  for( cnt = 0; cnt < max; cnt++ )
    if ( putc(a[max - 1 - cnt], fp) != a[max - 1 - cnt] )
      return 0;
    
  return 1;
}

int b_io_ReadDouble(FILE *fp, double *x)
{
  int flag;
  int exp;
  int val;
  int cnt, max;

  if ( b_io_read_int(fp, &exp, &flag) == 0 )
    return 0;

  *x = 1.0;
  
  max = getc(fp);
  if ( max < 0 )
    return 0;
  for( cnt = 0; cnt < max; cnt++ )
  {
    val = getc(fp);
    if ( val < 0 )
      return 0;
    *x += (double)val;
    *x /= 256.0;
    /*
    for( bcnt = 0; bcnt < 8; bcnt++ )
    {
      if ( (val & (1<<bcnt)) != 0 )
        *x += 1;
      *x /= 2.0;
    }
    */
  }

  *x *= pow(2.0, (double)exp);
  if ( flag != 0 )
    *x = - *x;
  return 1;
}


/*
void val(double x)
{
  unsigned char a[10];
  int cnt, max = 10;
  int bcnt;
  double exp, nx;
  
  exp = ceil(log(x)/log(2));
  nx = x*pow(2, -exp);
  
  for( cnt = 0; cnt < max; cnt++ )
  {
    a[cnt] = 0;
    for( bcnt = 0; bcnt < 8; bcnt++ )
    {
      nx *= 2.0;
      if ( nx >= 1.0 )
      {
        a[cnt] |= 1<<bcnt;
        nx -= 1.0;
      }
    }
    printf("%02x ", a[cnt]);
  }
  
  nx = 1.0;
  
  for( cnt = 0; cnt < max; cnt++ )
  {
    for( bcnt = 0; bcnt < 8; bcnt++ )
    {
      if ( (a[max - 1 - cnt] & (1<<(7-bcnt))) != 0 )
        nx += 1.0;
      nx /= 2.0;
    }
  }

  nx = nx*pow(2, exp);
  
  printf("%.30lg  %.30lg\n", x, nx);
}

void test(double x)
{
  FILE *fp;
  double y;
  fp = fopen("xyz", "w");
  b_io_WriteDouble(fp, x);
  fclose(fp);
  
  fp = fopen("xyz", "r");
  b_io_ReadDouble(fp, &y);
  fclose(fp);
  
  printf("%.20lg %.20lg\n", x, y);
}

int main()
{
  test(1.0);
  test(-1.0);
  test(12.34);
  test(-12.34);
  test(M_PI);
  test(1.23e234);
  test(-1.23e234);
  test(1.23e-234);
  test(-1.23e-234);
}
*/
