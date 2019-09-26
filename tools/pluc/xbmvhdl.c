/*

  xbmvhdl.c

  finite state machine for eXtended burst mode machines
  
  generate some VHDL code

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

#include "xbm.h"
#include <assert.h>

int internal_xbm_set_str(char **s, const char *name);

char xbm_vhdl_key_configuration[] = "CONFIGURATION";
char xbm_vhdl_key_of[] = "OF";
char xbm_vhdl_key_is[] = "IS";
char xbm_vhdl_key_for[] = "FOR";
char xbm_vhdl_key_end[] = "END";
char xbm_vhdl_key_entity[] = "ENTITY";
char xbm_vhdl_key_library[] = "LIBRARY";
char xbm_vhdl_key_use[] = "USE";
char xbm_vhdl_key_architecture[] = "ARCHITECTURE";
char xbm_vhdl_key_component[] = "COMPONENT";
char xbm_vhdl_key_port[] = "PORT";
char xbm_vhdl_key_in[] = "IN";
char xbm_vhdl_key_out[] = "OUT";
char xbm_vhdl_key_std_logic[] = "STD_LOGIC";
char xbm_vhdl_key_signal[] = "SIGNAL";
char xbm_vhdl_key_shared_variable[] = "SHARED VARIABLE";
char xbm_vhdl_key_std_logic_vector[] = "STD_LOGIC_VECTOR";
char xbm_vhdl_key_map[] = "MAP";
char xbm_vhdl_key_process[] = "PROCESS";
char xbm_vhdl_key_begin[] = "BEGIN";
char xbm_vhdl_key_if[] = "IF";
char xbm_vhdl_key_or[] = "OR";
char xbm_vhdl_key_then[] = "THEN";
char xbm_vhdl_key_assert[] = "ASSERT";
char xbm_vhdl_key_report[] = "REPORT";
char xbm_vhdl_key_severity_error[] = "SEVERITY ERROR";
char xbm_vhdl_key_wait[] = "WAIT";

char xbm_vhdl_key_ns[] = "ns";

int xbm_vhdl_init(xbm_type x)
{
  int is_ok = 1;
  
  is_ok &= internal_xbm_set_str(&(x->tb_entity_name), "testbench");
  is_ok &= internal_xbm_set_str(&(x->tb_arch_name), "test");
  is_ok &= internal_xbm_set_str(&(x->tb_conf_name), "cfg_testbench");
  is_ok &= internal_xbm_set_str(&(x->tb_component_name), "mydesign");
  is_ok &= internal_xbm_set_str(&(x->tb_clr_name), "clr");
  is_ok &= internal_xbm_set_str(&(x->tb_clk_name), "clk");
  
  return is_ok;
}

int xbm_SetVHDLComponentName(xbm_type x, const char *name)
{
  return internal_xbm_set_str(&(x->tb_component_name), name);
}

void xbm_vhdl_destroy(xbm_type x)
{
  internal_xbm_set_str(&(x->tb_entity_name), NULL);
  internal_xbm_set_str(&(x->tb_arch_name), NULL);
  internal_xbm_set_str(&(x->tb_conf_name), NULL);
  internal_xbm_set_str(&(x->tb_component_name), NULL);
  internal_xbm_set_str(&(x->tb_clr_name), NULL);
  internal_xbm_set_str(&(x->tb_clk_name), NULL);

}

int xbm_vhdl_indent(xbm_type x, FILE *fp, int cnt)
{
  while( cnt > 0 )
  {
    if ( fprintf(fp, " ") < 0 )
      return 0;
    cnt--;
  }
  return 1;
}

/*
  LIBRARY ieee;
  USE ieee.std_logic_1164.all;
*/
static int xbm_vhdltb_library(xbm_type x, FILE *fp)
{
  int r;
  if ( xbm_vhdl_indent(x, fp, 0) == 0 ) return 0;
  r = fprintf(fp, "%s %s;\n", xbm_vhdl_key_library, "ieee");
  if ( r < 0 ) return 0;

  if ( xbm_vhdl_indent(x, fp, 0) == 0 ) return 0;
  r = fprintf(fp, "%s %s;\n\n", xbm_vhdl_key_use, "ieee.std_logic_1164.all");
  if ( r < 0 ) return 0;
  
  return 1;
}



/*
  ENTITY testbench IS
  END testbench;
*/

static int xbm_vhdltb_entity(xbm_type x, FILE *fp)
{
  int r;

  if ( xbm_vhdl_indent(x, fp, 0) == 0 ) return 0;
  r = fprintf(fp, "%s %s %s\n", 
    xbm_vhdl_key_entity,
    x->tb_entity_name,
    xbm_vhdl_key_is);
  if ( r < 0 ) return 0;
  
  if ( xbm_vhdl_indent(x, fp, 0) == 0 ) return 0;
  r = fprintf(fp, "%s %s;\n\n", 
    xbm_vhdl_key_end,
    x->tb_entity_name);
  if ( r < 0 ) return 0;
  
  return 1;
}

/*
  CONFIGURATION cfg_testbench OF testbench IS
    FOR test
    END FOR;
  END cfg_testbench;
*/

static int xbm_vhdltb_configuration(xbm_type x, FILE *fp)
{
  int r;
  if ( xbm_vhdl_indent(x, fp, 0) == 0 ) return 0;
  r = fprintf(fp, "%s %s %s %s %s\n",
    xbm_vhdl_key_configuration,
    x->tb_conf_name,
    xbm_vhdl_key_of,
    x->tb_entity_name,
    xbm_vhdl_key_is);
  if ( r < 0 ) return 0;
  
  if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
  r = fprintf(fp, "%s %s\n", xbm_vhdl_key_for, x->tb_arch_name);
  if ( r < 0 ) return 0;

  if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
  r = fprintf(fp, "%s %s;\n", xbm_vhdl_key_end, xbm_vhdl_key_for);
  if ( r < 0 ) return 0;

  if ( xbm_vhdl_indent(x, fp, 0) == 0 ) return 0;
  r = fprintf(fp, "%s %s;\n", xbm_vhdl_key_end, x->tb_conf_name);
  if ( r < 0 ) return 0;
    
  return 1;
}

/*
  ARCHITECTURE test OF testbench IS
*/
static int xbm_vhdltb_arch_head(xbm_type x, FILE *fp)
{
  int r;
  if ( xbm_vhdl_indent(x, fp, 0) == 0 ) return 0;
  r = fprintf(fp, "%s %s %s %s %s\n",
    xbm_vhdl_key_architecture,
    x->tb_arch_name,
    xbm_vhdl_key_of,
    x->tb_entity_name,
    xbm_vhdl_key_is);
  if ( r < 0 ) return 0;

  return 1;
}

/*
  END test;
*/
static int xbm_vhdltb_arch_end(xbm_type x, FILE *fp)
{
  int r;
  if ( xbm_vhdl_indent(x, fp, 0) == 0 ) return 0;
  r = fprintf(fp, "%s %s;\n\n",
    xbm_vhdl_key_end,
    x->tb_arch_name);
  if ( r < 0 ) return 0;

  return 1;
}

/*
  COMPONENT mydesign
  PORT(StartDMARcv, ReqInN, DRAckNormN, DRAckLastN : IN STD_LOGIC;
    zo0, zo1, zo2, zo3, zo4, zo5, zi0, zi1, zi2, zi3, zi4, zi5, EndDMAInt, DRQ, AckOutN : OUT STD_LOGIC;
    clr, clk : IN STD_LOGIC);
  END COMPONENT;
*/
static int xbm_vhdltb_arch_component(xbm_type x, FILE *fp, int is_clr, int is_clk)
{
  int r = 0;
  int i, j;
  b_sl_type in_sl;
  b_sl_type out_sl;

  in_sl = pinfoGetInLabelList(x->pi_machine);
  out_sl = pinfoGetOutLabelList(x->pi_machine);
  
  if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
  r = fprintf(fp, "%s %s\n",
    xbm_vhdl_key_component,
    x->tb_component_name);
  if ( r < 0 ) return 0;

  if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
  r = fprintf(fp, "%s(\n",
    xbm_vhdl_key_port);
  if ( r < 0 ) return 0;

  if ( xbm_vhdl_indent(x, fp, 2) == 0 ) return 0;
  for( i = 0; i < x->inputs; i++ )
  {
    r = fprintf(fp, "%s", b_sl_GetVal(in_sl, i));
    if ( r < 0 ) return 0;
    if ( i < x->inputs-1 )
    {
      r = fprintf(fp, ", ");
      if ( r < 0 ) return 0;
    }
  }
  r = fprintf(fp, " : %s %s;\n", xbm_vhdl_key_in, xbm_vhdl_key_std_logic);
  if ( r < 0 ) return 0;

  /* state input */    
  if ( xbm_vhdl_indent(x, fp, 2) == 0 ) return 0;
  for( i = x->inputs; i < b_sl_GetCnt(in_sl); i++ )
  {
    r = fprintf(fp, "%s, ", b_sl_GetVal(in_sl, i));
    if ( r < 0 ) return 0;
  }
  
  /* state output + output variables */    
  for( i = 0; i < b_sl_GetCnt(out_sl); i++ )
  {
    r = fprintf(fp, "%s", b_sl_GetVal(out_sl, i));
    if ( r < 0 ) return 0;
    if ( i < b_sl_GetCnt(out_sl)-1 )
    {
      r = fprintf(fp, ", ");
      if ( r < 0 ) return 0;
    }
  }

  r = fprintf(fp, " : %s %s%s\n", xbm_vhdl_key_out, xbm_vhdl_key_std_logic,
    (is_clr!=0||is_clk!=0)?";":");");
  if ( r < 0 ) return 0;

  if ( is_clr!=0 || is_clk!=0 )
  {
    if ( xbm_vhdl_indent(x, fp, 2) == 0 ) return 0;
    if ( is_clr != 0 && is_clk != 0 )
      r = fprintf(fp, "%s, %s", x->tb_clr_name, x->tb_clk_name);
    else if ( is_clr != 0 )
      r = fprintf(fp, "%s", x->tb_clr_name);
    else if ( is_clk != 0 )
      r = fprintf(fp, "%s", x->tb_clk_name);
    if ( r < 0 ) return 0;
    r = fprintf(fp, " : %s %s);\n", xbm_vhdl_key_in, xbm_vhdl_key_std_logic);
    if ( r < 0 ) return 0;
  }

  if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
  r = fprintf(fp, "%s %s;\n",
    xbm_vhdl_key_end,
    xbm_vhdl_key_component);
  if ( r < 0 ) return 0;
  
  return 1;
}

/*
  SIGNAL x : STD_LOGIC_VECTOR(0 to 3);
  SIGNAL zy : STD_LOGIC_VECTOR(0 to 2);
  SHARED VARIABLE gzy : STD_LOGIC_VECTOR(0 to 2);
  SIGNAL zi : STD_LOGIC_VECTOR(0 to 5);
  SHARED VARIABLE gzi : STD_LOGIC_VECTOR(0 to 5);
  SIGNAL clr : STD_LOGIC;
  SIGNAL clk : STD_LOGIC;
*/
static int xbm_vhdltb_arch_signals(xbm_type x, FILE *fp, int is_clr, int is_clk)
{
  int r = 0;
  
  if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
  r = fprintf(fp, "%s x : %s(0 to %d);\n", 
    xbm_vhdl_key_signal, xbm_vhdl_key_std_logic_vector, x->inputs-1);
  if ( r < 0 ) return 0;

  if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
  r = fprintf(fp, "%s zi : %s(0 to %d);\n", 
    xbm_vhdl_key_signal, xbm_vhdl_key_std_logic_vector, x->pi_machine->in_cnt-x->inputs-1);
  if ( r < 0 ) return 0;
  
  if ( x->is_sync == 0 )
  {  
    if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
    r = fprintf(fp, "%s gzi : %s(0 to %d);\n", 
      xbm_vhdl_key_shared_variable, xbm_vhdl_key_std_logic_vector, x->pi_machine->in_cnt-x->inputs-1);
    if ( r < 0 ) return 0;
  }
  
  if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
  r = fprintf(fp, "%s zy : %s(0 to %d);\n", 
    xbm_vhdl_key_signal, xbm_vhdl_key_std_logic_vector, x->pi_machine->out_cnt-1);
  if ( r < 0 ) return 0;

  if ( x->is_sync == 0 )
  {  
    if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
    r = fprintf(fp, "%s gzy : %s(0 to %d);\n", 
      xbm_vhdl_key_shared_variable, xbm_vhdl_key_std_logic_vector, x->pi_machine->out_cnt-1);
    if ( r < 0 ) return 0;
  }

  if ( is_clr != 0 )
  {
    if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
    r = fprintf(fp, "%s %s : %s;\n", 
      xbm_vhdl_key_signal, x->tb_clr_name, xbm_vhdl_key_std_logic);
    if ( r < 0 ) return 0;
  }

  if ( is_clk != 0 )
  {
    if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
    r = fprintf(fp, "%s %s : %s;\n", 
      xbm_vhdl_key_signal, x->tb_clk_name, xbm_vhdl_key_std_logic);
    if ( r < 0 ) return 0;
  }
  
  return 1;
}

/*
  DUT: mydesign PORT MAP(x(0), x(1), x(2), x(3), z(0), zi(1), zi(2), zi(3), zi(4), zi(5), zy(0), zy(1), zy(2), zy(3), zy(4), zy(5), zy(6), zy(7), zy(8), clr, clk);
*/
static int xbm_vhdltb_arch_dut(xbm_type x, FILE *fp, int is_clr, int is_clk)
{
  int i;
  int r = 0;
  if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
  r = fprintf(fp, "DUT: %s %s %s(", 
    x->tb_component_name, xbm_vhdl_key_port, xbm_vhdl_key_map);
  if ( r < 0 ) return 0;
  for( i = 0; i < x->inputs; i++ )
  {
    r = fprintf(fp, "x(%d), ", i);
    if ( r < 0 ) return 0;
  }
  for( i = 0; i < x->pi_machine->in_cnt-x->inputs; i++ )
  {
    r = fprintf(fp, "zi(%d), ", i);
    if ( r < 0 ) return 0;
  }

  for( i = 0; i < x->pi_machine->out_cnt; i++ )
  {
    if ( is_clr == 0 && is_clk == 0 && i == x->pi_machine->out_cnt-1)
      r = fprintf(fp, "zy(%d));\n", i);
    else
      r = fprintf(fp, "zy(%d), ", i);
    if ( r < 0 ) return 0;
  }

  if ( is_clr != 0 && is_clk != 0 )
    r = fprintf(fp, "%s, %s);\n", x->tb_clr_name, x->tb_clk_name);
  else if ( is_clr != 0 )
    r = fprintf(fp, "%s);\n", x->tb_clr_name);
  else if ( is_clk != 0 )
    r = fprintf(fp, "%s);\n", x->tb_clk_name);
  if ( r < 0 ) return 0;

  return 1;
}

/*
  PROCESS(y(0))
  BEGIN
    IF ( y(0) = '0' OR y(0) = '1' ) THEN
      ASSERT ( glitch_y(0) /= '1' ) REPORT "Glitch on output y(0) found." SEVERITY ERROR;
      glitch_y(0) := '1';
    END IF;
  END PROCESS;
*/
static int xbm_vhdltb_arch_glitch_process(xbm_type x, FILE *fp, const char *sn, const char *gn, int idx)
{
  int r = 0;

  if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
  r = fprintf(fp, "%s(%s(%d))\n", xbm_vhdl_key_process, sn, idx);
  if ( r < 0 ) return 0;
  
  if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
  r = fprintf(fp, "%s\n", xbm_vhdl_key_begin);
  if ( r < 0 ) return 0;

  if ( xbm_vhdl_indent(x, fp, 2) == 0 ) return 0;
  r = fprintf(fp, "%s ( %s(%d) = '0' %s %s(%d) = '1' ) %s\n", xbm_vhdl_key_if, sn, idx, xbm_vhdl_key_or, sn, idx, xbm_vhdl_key_then);
  if ( r < 0 ) return 0;

  if ( xbm_vhdl_indent(x, fp, 3) == 0 ) return 0;
  r = fprintf(fp, "%s ( %s(%d) /= '1' ) %s \"glitch\" %s;\n", 
    xbm_vhdl_key_assert, gn, idx, xbm_vhdl_key_report, xbm_vhdl_key_severity_error);
  if ( r < 0 ) return 0;

  if ( xbm_vhdl_indent(x, fp, 3) == 0 ) return 0;
  r = fprintf(fp, "%s(%d) := '1';\n", gn, idx); 
  if ( r < 0 ) return 0;

  if ( xbm_vhdl_indent(x, fp, 2) == 0 ) return 0;
  r = fprintf(fp, "%s %s;\n", xbm_vhdl_key_end, xbm_vhdl_key_if); 
  if ( r < 0 ) return 0;

  if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
  r = fprintf(fp, "%s %s;\n", xbm_vhdl_key_end, xbm_vhdl_key_process); 
  if ( r < 0 ) return 0;

  return 1;
}

static int xbm_vhdltb_arch_glitch_processes(xbm_type x, FILE *fp)
{
  int i;
  
  if ( x->is_sync != 0 )
    return 1;
  
  for( i = 0; i < x->pi_machine->in_cnt-x->inputs; i++ )
    if ( xbm_vhdltb_arch_glitch_process(x, fp, "zi", "gzi", i) == 0 )
      return 0;

  for( i = 0; i < x->pi_machine->out_cnt; i++ )
    if ( xbm_vhdltb_arch_glitch_process(x, fp, "zy", "gzy", i) == 0 )
      return 0;

  return 1;  
}

static int xbm_vhdltb_walk_async_cb(xbm_type x, void *data, int tr_pos, dcube *cs, dcube *ct, int is_reset, int is_unique)
{
  FILE *fp = (FILE *)data;
  int i;
  int c;
  int r;

  /* reset glitch variables */

  if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
  r = fprintf(fp, "gzi := \"");
  if ( r < 0 ) return 0;
  for( i = 0; i < x->pi_machine->in_cnt-x->inputs; i++ )
    if ( putc((int)'0', fp) != '0' )
      return 0;
  r = fprintf(fp, "\";\n");
  if ( r < 0 ) return 0;
  
  if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
  r = fprintf(fp, "gzy := \"");
  if ( r < 0 ) return 0;
  for( i = 0; i < x->pi_machine->out_cnt; i++ )
    if ( putc((int)'0', fp) != '0' )
      return 0;
  r = fprintf(fp, "\";\n");
  if ( r < 0 ) return 0;


  if ( is_reset != 0 && x->tb_reset_type != XBM_RESET_NONE)
  {

    /* apply reset state */

    /* XBM_RESET_LOW: low active */
    if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
    r = fprintf(fp, "%s <= '%d';\n", 
      x->tb_clr_name,
      (x->tb_reset_type==XBM_RESET_LOW)?0:1);
    if ( r < 0 ) return 0;

    if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
    r = fprintf(fp, "%s %s %d %s;\n", 
      xbm_vhdl_key_wait, 
      xbm_vhdl_key_for, 
      x->tb_wait_time_ns, 
      xbm_vhdl_key_ns);
    if ( r < 0 ) return 0;

    /* reset glitch variables */

    if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
    r = fprintf(fp, "gzi := \"");
    if ( r < 0 ) return 0;
    for( i = 0; i < x->pi_machine->in_cnt-x->inputs; i++ )
      if ( putc((int)'0', fp) != '0' )
        return 0;
    r = fprintf(fp, "\";\n");
    if ( r < 0 ) return 0;

    if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
    r = fprintf(fp, "gzy := \"");
    if ( r < 0 ) return 0;
    for( i = 0; i < x->pi_machine->out_cnt; i++ )
      if ( putc((int)'0', fp) != '0' )
        return 0;
    r = fprintf(fp, "\";\n");
    if ( r < 0 ) return 0;
    
    /* go to start position */

    if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
    r = fprintf(fp, "x <= \"");
    if ( r < 0 ) return 0;
    for( i = 0; i < x->inputs; i++ )
    {
      c = "x01-"[dcGetIn(cs, i)];
      if ( putc(c, fp) != c )
        return 0;
    }
    r = fprintf(fp, "\";\n");
    if ( r < 0 ) return 0;

    if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
    r = fprintf(fp, "%s %s %d %s;\n", 
      xbm_vhdl_key_wait, 
      xbm_vhdl_key_for, 
      x->tb_wait_time_ns, 
      xbm_vhdl_key_ns);
    if ( r < 0 ) return 0;

    /* reset glitch variables */

    if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
    r = fprintf(fp, "gzi := \"");
    if ( r < 0 ) return 0;
    for( i = 0; i < x->pi_machine->in_cnt-x->inputs; i++ )
      if ( putc((int)'0', fp) != '0' )
        return 0;
    r = fprintf(fp, "\";\n");
    if ( r < 0 ) return 0;

    if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
    r = fprintf(fp, "gzy := \"");
    if ( r < 0 ) return 0;
    for( i = 0; i < x->pi_machine->out_cnt; i++ )
      if ( putc((int)'0', fp) != '0' )
        return 0;
    r = fprintf(fp, "\";\n");
    if ( r < 0 ) return 0;

    /* enable normal operation */

    if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
    r = fprintf(fp, "%s <= '%d';\n",
      x->tb_clr_name,
      (x->tb_reset_type==XBM_RESET_LOW)?1:0);
    if ( r < 0 ) return 0;
  }
  else
  {

    /* go to start position */

    if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
    r = fprintf(fp, "x <= \"");
    if ( r < 0 ) return 0;
    for( i = 0; i < x->inputs; i++ )
    {
      c = "x01-"[dcGetIn(cs, i)];
      if ( putc(c, fp) != c )
        return 0;
    }
    r = fprintf(fp, "\";\n");
    if ( r < 0 ) return 0;

  }

  if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
  r = fprintf(fp, "%s %s %d %s;\n", 
    xbm_vhdl_key_wait, 
    xbm_vhdl_key_for, 
    x->tb_wait_time_ns, 
    xbm_vhdl_key_ns);
  if ( r < 0 ) return 0;


  /* reset glitch variables */

  if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
  r = fprintf(fp, "gzi := \"");
  if ( r < 0 ) return 0;
  for( i = 0; i < x->pi_machine->in_cnt-x->inputs; i++ )
    if ( putc((int)'0', fp) != '0' )
      return 0;
  r = fprintf(fp, "\";\n");
  if ( r < 0 ) return 0;

  if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
  r = fprintf(fp, "gzy := \"");
  if ( r < 0 ) return 0;
  for( i = 0; i < x->pi_machine->out_cnt; i++ )
    if ( putc((int)'0', fp) != '0' )
      return 0;
  r = fprintf(fp, "\";\n");
  if ( r < 0 ) return 0;

  /* do the transition to the next state */
  
  if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
  r = fprintf(fp, "x <= \"");
  if ( r < 0 ) return 0;
  for( i = 0; i < x->inputs; i++ )
  {
    c = "x01-"[dcGetIn(ct, i)];
    if ( putc(c, fp) != c )
      return 0;
  }
  r = fprintf(fp, "\";\n");
  if ( r < 0 ) return 0;

  if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
  r = fprintf(fp, "%s %s %d %s;\n", 
    xbm_vhdl_key_wait, 
    xbm_vhdl_key_for, 
    x->tb_wait_time_ns, 
    xbm_vhdl_key_ns);
  if ( r < 0 ) return 0;

  /* check for correct transition */

  if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
  r = fprintf(fp, "%s (zy = \"", xbm_vhdl_key_assert);
  if ( r < 0 ) return 0;
  for( i = 0; i < x->pi_machine->out_cnt; i++ )
  {
    c = "01"[dcGetOut(ct, i)];
    if ( putc(c, fp) != c )
      return 0;
  }
  r = fprintf(fp, "\") %s \"transition %s -> %s failed\" %s;\n", 
    xbm_vhdl_key_report, 
    xbm_GetStNameStr(x, xbm_GetTrSrcStPos(x, tr_pos)),
    xbm_GetStNameStr(x, xbm_GetTrDestStPos(x, tr_pos)),
    xbm_vhdl_key_severity_error);
  if ( r < 0 ) return 0;
  
  return 1;
}

static int xbm_vhdltb_walk_sync_cb(xbm_type x, void *data, int tr_pos, dcube *cs, dcube *ct, int is_reset, int is_unique)
{
  FILE *fp = (FILE *)data;
  int i;
  int c;
  int r;

  if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
  r = fprintf(fp, "x <= \"");
  if ( r < 0 ) return 0;
  for( i = 0; i < x->inputs; i++ )
  {
    c = "x01-"[dcGetIn(ct, i)];
    if ( putc(c, fp) != c )
      return 0;
  }
  r = fprintf(fp, "\";\n");
  if ( r < 0 ) return 0;

  if ( is_reset != 0 && x->tb_reset_type != XBM_RESET_NONE)
  {

    /* apply reset state */

    /* XBM_RESET_LOW: low active */
    if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
    r = fprintf(fp, "%s <= '%d';\n", 
      x->tb_clr_name,
      (x->tb_reset_type==XBM_RESET_LOW)?0:1);
    if ( r < 0 ) return 0;

    if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
    r = fprintf(fp, "%s %s %d %s;\n", 
      xbm_vhdl_key_wait, 
      xbm_vhdl_key_for, 
      x->tb_wait_time_ns, 
      xbm_vhdl_key_ns);
    if ( r < 0 ) return 0;

    if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
    r = fprintf(fp, "%s <= '0';\n", 
      x->tb_clk_name);
    if ( r < 0 ) return 0;

    if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
    r = fprintf(fp, "%s %s %d %s;\n", 
      xbm_vhdl_key_wait, 
      xbm_vhdl_key_for, 
      x->tb_wait_time_ns, 
      xbm_vhdl_key_ns);
    if ( r < 0 ) return 0;

    if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
    r = fprintf(fp, "%s <= '1';\n", 
      x->tb_clk_name);
    if ( r < 0 ) return 0;

    if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
    r = fprintf(fp, "%s %s %d %s;\n", 
      xbm_vhdl_key_wait, 
      xbm_vhdl_key_for, 
      x->tb_wait_time_ns, 
      xbm_vhdl_key_ns);
    if ( r < 0 ) return 0;

    /* enable normal operation */

    if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
    r = fprintf(fp, "%s <= '%d';\n", 
      x->tb_clr_name,
      (x->tb_reset_type==XBM_RESET_LOW)?1:0);
    if ( r < 0 ) return 0;
  }

  if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
  r = fprintf(fp, "%s %s %d %s;\n", 
    xbm_vhdl_key_wait, 
    xbm_vhdl_key_for, 
    x->tb_wait_time_ns, 
    xbm_vhdl_key_ns);
  if ( r < 0 ) return 0;

  /* do the transition to the next state */

  if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
  r = fprintf(fp, "%s %s %d %s;\n", 
    xbm_vhdl_key_wait, 
    xbm_vhdl_key_for, 
    x->tb_wait_time_ns, 
    xbm_vhdl_key_ns);
  if ( r < 0 ) return 0;

  if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
  r = fprintf(fp, "%s <= '0';\n", 
    x->tb_clk_name);
  if ( r < 0 ) return 0;

  if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
  r = fprintf(fp, "%s %s %d %s;\n", 
    xbm_vhdl_key_wait, 
    xbm_vhdl_key_for, 
    x->tb_wait_time_ns, 
    xbm_vhdl_key_ns);
  if ( r < 0 ) return 0;

  if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
  r = fprintf(fp, "%s <= '1';\n", 
    x->tb_clk_name);
  if ( r < 0 ) return 0;

  if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
  r = fprintf(fp, "%s %s %d %s;\n", 
    xbm_vhdl_key_wait, 
    xbm_vhdl_key_for, 
    x->tb_wait_time_ns, 
    xbm_vhdl_key_ns);
  if ( r < 0 ) return 0;


  /* check for correct transition */

  if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
  r = fprintf(fp, "%s (zy = \"", xbm_vhdl_key_assert);
  if ( r < 0 ) return 0;
  for( i = 0; i < x->pi_machine->out_cnt; i++ )
  {
    c = "01"[dcGetOut(ct, i)];
    if ( putc(c, fp) != c )
      return 0;
  }
  r = fprintf(fp, "\") %s \"transition %s -> %s failed \"%s;\n", 
    xbm_vhdl_key_report, 
    xbm_GetStNameStr(x, xbm_GetTrSrcStPos(x, tr_pos)),
    xbm_GetStNameStr(x, xbm_GetTrDestStPos(x, tr_pos)),
    xbm_vhdl_key_severity_error);
  if ( r < 0 ) return 0;
  
  return 1;
}

static int xbm_vhdltb_walk(xbm_type x, FILE *fp)
{
  int r = 0;

  if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
  r = fprintf(fp, "%s\n", xbm_vhdl_key_process);
  if ( r < 0 ) return 0;
  
  if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
  r = fprintf(fp, "%s\n", xbm_vhdl_key_begin);
  if ( r < 0 ) return 0;

  if ( x->is_sync != 0 )
  {
    if ( xbm_DoWalk(x, xbm_vhdltb_walk_sync_cb, (void *)fp) == 0 )
      return 0;
  }
  else
  {
    if ( xbm_DoWalk(x, xbm_vhdltb_walk_async_cb, (void *)fp) == 0 )
      return 0;
  }

  if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
  r = fprintf(fp, "%s;\n", xbm_vhdl_key_wait); 
  if ( r < 0 ) return 0;
  
  if ( xbm_vhdl_indent(x, fp, 1) == 0 ) return 0;
  r = fprintf(fp, "%s %s;\n", xbm_vhdl_key_end, xbm_vhdl_key_process); 
  if ( r < 0 ) return 0;

  return 1;
}

static int xbm_vhdltb_architecture(xbm_type x, FILE *fp)
{
  int r;
  int is_clr = (x->tb_reset_type != XBM_RESET_NONE)?1:0;
  int is_clk = x->is_sync;

  if ( xbm_vhdltb_arch_head(x, fp) == 0 ) return 0;
  if ( xbm_vhdltb_arch_component(x, fp, is_clr, is_clk) == 0 ) return 0;
  if ( xbm_vhdltb_arch_signals(x, fp, is_clr, is_clk) == 0 ) return 0;

  if ( xbm_vhdl_indent(x, fp, 0) == 0 ) return 0;
  r = fprintf(fp, "%s\n", xbm_vhdl_key_begin);
  if ( r < 0 ) return 0;

  if ( xbm_vhdltb_arch_dut(x, fp, is_clr, is_clk) == 0 ) return 0;
  if ( xbm_vhdltb_arch_glitch_processes(x, fp) == 0 ) return 0;

  if ( xbm_vhdltb_walk(x, fp) == 0 ) return 0;

  if ( xbm_vhdltb_arch_end(x, fp) == 0 ) return 0;

  return 1;  
}

static int xbm_vhdltb_all(xbm_type x, FILE *fp, const char *name)
{
  int r;
  if ( xbm_vhdl_indent(x, fp, 0) == 0 ) return 0;
  r = fprintf(fp, "-- file: %s\n\n", name);
  if ( r < 0 ) return 0;

  if ( x->name != NULL )
  {
    if ( xbm_vhdl_indent(x, fp, 0) == 0 ) return 0;
    r = fprintf(fp, "-- machine: %s\n\n", x->name);
    if ( r < 0 ) return 0;
  }


  if ( xbm_vhdltb_library(x, fp) == 0 ) return 0;
  if ( xbm_vhdltb_entity(x, fp) == 0 ) return 0;
  if ( xbm_vhdltb_architecture(x, fp) == 0 ) return 0;
  if ( xbm_vhdltb_configuration(x, fp) == 0 ) return 0;
  return 1;
}


int xbm_WriteTestbenchVHDL(xbm_type x, const char *name)
{
  FILE *fp;
  int r;
  fp = fopen(name, "w");
  if ( fp == NULL )
  {
    xbm_Error(x, "XBM: Can not create testbench '%s'.", name);
    return 0;
  }
  r = xbm_vhdltb_all(x, fp, name);
  if ( r == 0 )
    xbm_Error(x, "XBM: Error with testbench '%s'.", name);
  fclose(fp);
  xbm_Log(x, 4, "XBM: Testbench '%s' created.", name);
  return r;
}

