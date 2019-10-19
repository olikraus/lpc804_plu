/*

  pluc.c
  
  f(..., x, ...) = x && f(..., 1, ... ) || !x && f(..., 0, ... )
  
  
  init	
    setup of all the data structures
  read
    parse files and merge everyting into a single huge boolean problem
  map
    cut down the boolean problem and map the problem into the available luts
  
*/

/*

  Limitations
    - Route Algo will search for the first valid path: This is ok, as long as the 
      LVLSHFT connections are not included. If the LVLSHFT connections
      are present, then we need to search for the shortest path
      For the user, this means, that a route between two GPIOs is routed
      via a LUT (and not via LVLSHFT).  Nevertheless GPIO routes can be
      routed manually via LVLSHFT.
    - There is no output function reuse or optimization. Not sure whether this
       is possible with LUTs... 
    

*/


#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include "dcube.h"
#include "fsm.h"
#include "fsmenc.h"
#include "cmdline.h"

/*==================================================*/
struct _pluc_lut_struct
{
  pinfo pi;			/* the label of the output will be the name of expected signal */
  dclist dcl;
  /* obsolete, the LUT name can always be derived from the position in the list */
  // char *user_out_name;	/* the original output signal name, given by the user */
  int is_placed;			/* whether this LUT is placed */
};
typedef struct _pluc_lut_struct pluc_lut_t;


#define PLUC_REGOP_NOT_USED 0
#define PLUC_REGOP_ASSIGN_OR 1
#define PLUC_REGOP_AND 2
#define PLUC_REGOP_NAND 3
#define PLUC_REGOP_OR 4
#define PLUC_REGOP_NOR 5
#define PLUC_REGOP_AND_OR 6
#define PLUC_REGOP_NAND_OR 7

#define PLUC_BASE_PLU 0
#define PLUC_BASE_SWM 1



struct _pluc_regop_struct
{
  int8_t op;			/* 0: not used, 1: assign "or value" */
					/* 2: "&=" "and_value", 3: "&= ~" "and_value", 4: "|=" "or_value", 5: "|= ~" "or_value" */
					/* 6: "&=and_value" "|=or_value", 7: "&=~and_value |=or_value */ 
  int8_t base;			/* not really a base address,but a index for the base address, 0: PLU, 1: SWM  */
					/* 0: 0x40028000 PLU   */
					/* 1: 0x4000c000 SWD  */
  uint32_t idx;			/* index for base (byte offset)*/
  uint32_t and_value;	/* and value */
  uint32_t or_value;		/* or value */
  
};

typedef struct _pluc_regop_struct pluc_regop_t;


#define PLUC_WIRE_REGOP_CNT 2
struct _pluc_wire_struct
{
  const char *from;
  const char *to;  
  pluc_regop_t regop[PLUC_WIRE_REGOP_CNT];
  int8_t is_visited;
  int8_t is_rd_block;			/* this connection requires read blocking */
  int8_t is_used;
  int8_t is_wr_blocked;		/* shared resourced can be blocked for writing, once they are used */
  int8_t is_rd_blocked;		/* shared resourced can be blocked for reading, once they are used, will be used only if is_rd_block is 1 */
  int8_t is_lut_in;
  int8_t is_lut_out;
  
};
typedef struct _pluc_wire_struct pluc_wire_t;


/*==================================================*/
/* global variables */

int pluc_is_FF_used = 0;		// whether FF are used or not

char c_file_name[1024] = "";
long cmdline_clkdiv = 1;
int cmdline_lposc = 0;
int cmdline_listmap = 0;
int cmdline_listkeywords = 0;
int cmdline_xrst = 0;		// generate reset for IOCON, GPIO und SWM
char cmdline_output[1024] = "";
char cmdline_input[1024] = "";
char cmdline_procname[1024] = "plu";
int cmdline_dbg = 0;
int cmdline_mis = 0;

FILE *c_fp = NULL;

/* 
 read
    parse files and merge everyting into a single huge boolean problem
*/
pinfo pi, pi2;
dclist cl_on, cl_dc, cl2_on, cl2_dc;
  
fsm_type fsm = NULL;

b_sl_type pluc_keywords = NULL;


/*
  map
    cut down the boolean problem and map the problem into the available luts
*/
#define PLUC_LUT_MAX 128
#define PLUC_FF_MAX 4
pluc_lut_t pluc_lut_list[PLUC_LUT_MAX];
int pluc_lut_cnt = PLUC_FF_MAX;
int pluc_ff_cnt = 0;

int pluc_internal_cnt = 0;


/*==================================================*/
/* LPC804 routing information */
/* 
  Routing is based on the signal/wire name 
  LUTx_INPy			y input pin y for lut xx
  LUTx				x lut output
  FFx				x flip flip output, note that LUT0 output is automatically mapped to FF0
  
*/
#define PLUC_WIRE_STRING(x) #x

#define PLUC_WIRE_LUT_OUT(x)  PLUC_WIRE_STRING(LUT ## x)
#define PLUC_WIRE_FF_OUT(x)  PLUC_WIRE_STRING(FF ## x)
#define PLUC_WIRE_LUT_IN(lut, in) PLUC_WIRE_STRING( LUT ## lut ## _INP ## in)
#define PLUC_WIRE_PLU_IN(in) PLUC_WIRE_STRING( PLUINPUT ## in)
#define PLUC_WIRE_PLU_OUT(out) PLUC_WIRE_STRING( PLUOUT ## out)


#define PLUC_CONNECT_PLU_IN_LUT_IN(from, lut, in, idx, v) \
 { PLUC_WIRE_PLU_IN(from), PLUC_WIRE_LUT_IN(lut, in), {{1, 0, idx, 0, v}, {0, 0, 0, 0, 0}}, 0, 0, 0, 0, 0, 1, 0 } ,

#define LPC804_CONNECT_PLU_OUT_GPIO(out, gpio, o) \
 { PLUC_WIRE_PLU_OUT(out), #gpio, {{7, 1, 0x180, 3<<(out*2+12), o<<(out*2+12)}, {0, 0, 0, 0, 0}}, 0, 0, 0, 0, 0, 0, 0 } ,

#define PLUC_CONNECT_LUT_OUT_LUT_IN(from, lut, in, idx, v) \
 { PLUC_WIRE_LUT_OUT(from), PLUC_WIRE_LUT_IN(lut, in), {{1, 0, idx, 0, v}, {0, 0, 0, 0, 0}}, 0, 0, 0, 0, 0, 1, 0 } ,

#define LPC804_CONNECT_LUT_OUT_PLU_OUT(lutout, pluout) \
 { PLUC_WIRE_LUT_OUT(lutout), PLUC_WIRE_PLU_OUT(pluout), {{1, 0, 0xc00+pluout*4, 0, lutout}, {0, 0, 0, 0, 0}}, 0, 0, 0, 0, 0, 0, 1 } ,

#define PLUC_CONNECT_FF_OUT_LUT_IN(from, lut, in, idx, v) \
 { PLUC_WIRE_FF_OUT(from), PLUC_WIRE_LUT_IN(lut, in), {{1, 0, idx, 0, v}, {0, 0, 0, 0, 0}}, 0, 0, 0, 0, 0, 1, 0 } ,

#define LPC804_CONNECT_FF_OUT_PLU_OUT(ffout, pluout) \
 { PLUC_WIRE_FF_OUT(ffout), PLUC_WIRE_PLU_OUT(pluout), {{1, 0, 0xc00+pluout*4, 0, ffout+26}, {0, 0, 0, 0, 0}}, 0, 0, 0, 0, 0, 0, 0 } ,

#define LPC804_CONNECT_GPIO_PLU_IN(gpio, in, o) \
 { #gpio, PLUC_WIRE_PLU_IN(in), {{7, 1, 0x180, 3<<(in*2), o<<(in*2)}, {0, 0, 0, 0, 0}}, 0, 0, 0, 0, 0, 0, 0 } ,

#define LPC804_CONNECT_LUT_OUT_FF_OUT(lut, ff) \
 { PLUC_WIRE_LUT_OUT(lut), PLUC_WIRE_FF_OUT(ff), {{0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}}, 0, 0, 0, 0, 0, 0, 1	 } ,

/* o: offset, pos: 0..3 */

#define LPC804_CONNECT_GPIO_TO_FN(gpio_no, no, fn, idx, pos) \
 { "PIO0" #gpio_no, #fn, { {7, 1, idx, 255UL<<(pos*8), no<<(pos*8) }, {0, 0, 0, 0, 0}}, 0, 0, 0, 0, 0, 0, 0 },

#define LPC804_CONNECT_FN_TO_GPIO(gpio_no, no, fn, idx, pos) \
 { #fn, "PIO0" #gpio_no, { {7, 1, idx, 255UL<<(pos*8), no<<(pos*8) }, {0, 0, 0, 0, 0}}, 0, 1, 0, 0, 0, 0, 0 },

#define LPC804_CONNECT_FN_TO_FN(fn1, fn2) \
 { #fn1, #fn2, { {0, 0, 0, 0, 0 }, {0, 0, 0, 0, 0}}, 0, 0, 0, 0, 0, 0, 0 },


#define PLUC_CONNECT_NONE() \
 { NULL, NULL, {{0, 0, 0, 0, 0}, {0, 0, 0, 0, 0}}, 0, 0, 0, 0, 0, 0, 0 }

/* from LUT, dest-LUT, input of dest-LUT, index, mux value */

#define LPC804_CONNECT_LUT_IN(lut, in, idx) \
  PLUC_CONNECT_PLU_IN_LUT_IN(0, lut, in, idx, 0) \
  PLUC_CONNECT_PLU_IN_LUT_IN(1, lut, in, idx, 1) \
  PLUC_CONNECT_PLU_IN_LUT_IN(2, lut, in, idx, 2) \
  PLUC_CONNECT_PLU_IN_LUT_IN(3, lut, in, idx, 3) \
  PLUC_CONNECT_PLU_IN_LUT_IN(4, lut, in, idx, 4) \
  PLUC_CONNECT_PLU_IN_LUT_IN(5, lut, in, idx, 5) \
  PLUC_CONNECT_LUT_OUT_LUT_IN(0, lut, in, idx, 6) \
  PLUC_CONNECT_LUT_OUT_LUT_IN(1, lut, in, idx, 7) \
  PLUC_CONNECT_LUT_OUT_LUT_IN(2, lut, in, idx, 8)  \
  PLUC_CONNECT_LUT_OUT_LUT_IN(3, lut, in, idx, 9)  \
  PLUC_CONNECT_LUT_OUT_LUT_IN(4, lut, in, idx, 10)  \
  PLUC_CONNECT_LUT_OUT_LUT_IN(5, lut, in, idx, 11)  \
  PLUC_CONNECT_LUT_OUT_LUT_IN(6, lut, in, idx, 12)  \
  PLUC_CONNECT_LUT_OUT_LUT_IN(7, lut, in, idx, 13)  \
  PLUC_CONNECT_LUT_OUT_LUT_IN(8, lut, in, idx, 14)  \
  PLUC_CONNECT_LUT_OUT_LUT_IN(9, lut, in, idx, 15)  \
  PLUC_CONNECT_LUT_OUT_LUT_IN(10, lut, in, idx, 16)  \
  PLUC_CONNECT_LUT_OUT_LUT_IN(11, lut, in, idx, 17)  \
  PLUC_CONNECT_LUT_OUT_LUT_IN(12, lut, in, idx, 18)  \
  PLUC_CONNECT_LUT_OUT_LUT_IN(13, lut, in, idx, 19)  \
  PLUC_CONNECT_LUT_OUT_LUT_IN(14, lut, in, idx, 20)  \
  PLUC_CONNECT_LUT_OUT_LUT_IN(15, lut, in, idx, 21)  \
  PLUC_CONNECT_LUT_OUT_LUT_IN(16, lut, in, idx, 22)  \
  PLUC_CONNECT_LUT_OUT_LUT_IN(17, lut, in, idx, 23)  \
  PLUC_CONNECT_LUT_OUT_LUT_IN(18, lut, in, idx, 24)  \
  PLUC_CONNECT_LUT_OUT_LUT_IN(19, lut, in, idx, 25)  \
  PLUC_CONNECT_LUT_OUT_LUT_IN(20, lut, in, idx, 26)  \
  PLUC_CONNECT_LUT_OUT_LUT_IN(21, lut, in, idx, 27)  \
  PLUC_CONNECT_LUT_OUT_LUT_IN(22, lut, in, idx, 28)  \
  PLUC_CONNECT_LUT_OUT_LUT_IN(23, lut, in, idx, 29)  \
  PLUC_CONNECT_LUT_OUT_LUT_IN(24, lut, in, idx, 30)  \
  PLUC_CONNECT_LUT_OUT_LUT_IN(25, lut, in, idx, 31) \
  PLUC_CONNECT_FF_OUT_LUT_IN(0, lut, in, idx, 32) \
  PLUC_CONNECT_FF_OUT_LUT_IN(1, lut, in, idx, 33) \
  PLUC_CONNECT_FF_OUT_LUT_IN(2, lut, in, idx, 34) \
  PLUC_CONNECT_FF_OUT_LUT_IN(3, lut, in, idx, 35)

/* dest-LUT, input of dest-LUT, index input mux */
#define LPC804_CONNECT_LUT(lut) \
  LPC804_CONNECT_LUT_IN(lut, 0, lut*0x020+0x000) \
  LPC804_CONNECT_LUT_IN(lut, 1, lut*0x020+0x004) \
  LPC804_CONNECT_LUT_IN(lut, 2, lut*0x020+0x008) \
  LPC804_CONNECT_LUT_IN(lut, 3, lut*0x020+0x00c) \
  LPC804_CONNECT_LUT_IN(lut, 4, lut*0x020+0x010) \
  LPC804_CONNECT_LUT_OUT_PLU_OUT(lut, 0) \
  LPC804_CONNECT_LUT_OUT_PLU_OUT(lut, 1) \
  LPC804_CONNECT_LUT_OUT_PLU_OUT(lut, 2) \
  LPC804_CONNECT_LUT_OUT_PLU_OUT(lut, 3) \
  LPC804_CONNECT_LUT_OUT_PLU_OUT(lut, 4) \
  LPC804_CONNECT_LUT_OUT_PLU_OUT(lut, 5) \
  LPC804_CONNECT_LUT_OUT_PLU_OUT(lut, 6) \
  LPC804_CONNECT_LUT_OUT_PLU_OUT(lut, 7)


#define LPC804_CONNECT_FF(ff) \
  LPC804_CONNECT_FF_OUT_PLU_OUT(ff, 0) \
  LPC804_CONNECT_FF_OUT_PLU_OUT(ff, 1) \
  LPC804_CONNECT_FF_OUT_PLU_OUT(ff, 2) \
  LPC804_CONNECT_FF_OUT_PLU_OUT(ff, 3) \
  LPC804_CONNECT_FF_OUT_PLU_OUT(ff, 4) \
  LPC804_CONNECT_FF_OUT_PLU_OUT(ff, 5) \
  LPC804_CONNECT_FF_OUT_PLU_OUT(ff, 6) \
  LPC804_CONNECT_FF_OUT_PLU_OUT(ff, 7) \
  LPC804_CONNECT_LUT_OUT_FF_OUT(ff, ff)


#define LPC804_CONNECT_TO_FN(fn, idx, pos) \
  LPC804_CONNECT_GPIO_TO_FN(_30, 30, fn, idx, pos) \
  LPC804_CONNECT_GPIO_TO_FN(_29, 29, fn, idx, pos) \
  LPC804_CONNECT_GPIO_TO_FN(_28, 28, fn, idx, pos) \
  LPC804_CONNECT_GPIO_TO_FN(_27, 27, fn, idx, pos) \
  LPC804_CONNECT_GPIO_TO_FN(_26, 26, fn, idx, pos) \
  LPC804_CONNECT_GPIO_TO_FN(_25, 25, fn, idx, pos) \
  LPC804_CONNECT_GPIO_TO_FN(_24, 24, fn, idx, pos) \
  LPC804_CONNECT_GPIO_TO_FN(_23, 23, fn, idx, pos) \
  LPC804_CONNECT_GPIO_TO_FN(_22, 22, fn, idx, pos) \
  LPC804_CONNECT_GPIO_TO_FN(_21, 21, fn, idx, pos) \
  LPC804_CONNECT_GPIO_TO_FN(_20, 20, fn, idx, pos) \
  LPC804_CONNECT_GPIO_TO_FN(_19, 19, fn, idx, pos) \
  LPC804_CONNECT_GPIO_TO_FN(_18, 18, fn, idx, pos) \
  LPC804_CONNECT_GPIO_TO_FN(_17, 17, fn, idx, pos) \
  LPC804_CONNECT_GPIO_TO_FN(_16, 16, fn, idx, pos) \
  LPC804_CONNECT_GPIO_TO_FN(_15, 15, fn, idx, pos) \
  LPC804_CONNECT_GPIO_TO_FN(_14, 14, fn, idx, pos) \
  LPC804_CONNECT_GPIO_TO_FN(_13, 13, fn, idx, pos) \
  LPC804_CONNECT_GPIO_TO_FN(_12, 12, fn, idx, pos) \
  LPC804_CONNECT_GPIO_TO_FN(_11, 11, fn, idx, pos) \
  LPC804_CONNECT_GPIO_TO_FN(_10, 10, fn, idx, pos) \
  LPC804_CONNECT_GPIO_TO_FN(_9, 9, fn, idx, pos) \
  LPC804_CONNECT_GPIO_TO_FN(_8, 8, fn, idx, pos) \
  LPC804_CONNECT_GPIO_TO_FN(_7, 7, fn, idx, pos) \
  LPC804_CONNECT_GPIO_TO_FN(_5, 5, fn, idx, pos) \
  LPC804_CONNECT_GPIO_TO_FN(_4, 4, fn, idx, pos) \
  LPC804_CONNECT_GPIO_TO_FN(_3, 3, fn, idx, pos) \
  LPC804_CONNECT_GPIO_TO_FN(_2, 2, fn, idx, pos) \
  LPC804_CONNECT_GPIO_TO_FN(_1, 1, fn, idx, pos) \
  LPC804_CONNECT_GPIO_TO_FN(_0, 0, fn, idx, pos) 


#define LPC804_CONNECT_FROM_FN(fn, idx, pos) \
  LPC804_CONNECT_FN_TO_GPIO(_30, 30, fn, idx, pos) \
  LPC804_CONNECT_FN_TO_GPIO(_29, 29, fn, idx, pos) \
  LPC804_CONNECT_FN_TO_GPIO(_28, 28, fn, idx, pos) \
  LPC804_CONNECT_FN_TO_GPIO(_27, 27, fn, idx, pos) \
  LPC804_CONNECT_FN_TO_GPIO(_26, 26, fn, idx, pos) \
  LPC804_CONNECT_FN_TO_GPIO(_25, 25, fn, idx, pos) \
  LPC804_CONNECT_FN_TO_GPIO(_24, 24, fn, idx, pos) \
  LPC804_CONNECT_FN_TO_GPIO(_23, 23, fn, idx, pos) \
  LPC804_CONNECT_FN_TO_GPIO(_22, 22, fn, idx, pos) \
  LPC804_CONNECT_FN_TO_GPIO(_21, 21, fn, idx, pos) \
  LPC804_CONNECT_FN_TO_GPIO(_20, 20, fn, idx, pos) \
  LPC804_CONNECT_FN_TO_GPIO(_19, 19, fn, idx, pos) \
  LPC804_CONNECT_FN_TO_GPIO(_18, 18, fn, idx, pos) \
  LPC804_CONNECT_FN_TO_GPIO(_17, 17, fn, idx, pos) \
  LPC804_CONNECT_FN_TO_GPIO(_16, 16, fn, idx, pos) \
  LPC804_CONNECT_FN_TO_GPIO(_15, 15, fn, idx, pos) \
  LPC804_CONNECT_FN_TO_GPIO(_14, 14, fn, idx, pos) \
  LPC804_CONNECT_FN_TO_GPIO(_13, 13, fn, idx, pos) \
  LPC804_CONNECT_FN_TO_GPIO(_12, 12, fn, idx, pos) \
  LPC804_CONNECT_FN_TO_GPIO(_11, 11, fn, idx, pos) \
  LPC804_CONNECT_FN_TO_GPIO(_10, 10, fn, idx, pos) \
  LPC804_CONNECT_FN_TO_GPIO(_9, 9, fn, idx, pos) \
  LPC804_CONNECT_FN_TO_GPIO(_8, 8, fn, idx, pos) \
  LPC804_CONNECT_FN_TO_GPIO(_7, 7, fn, idx, pos) \
  LPC804_CONNECT_FN_TO_GPIO(_5, 5, fn, idx, pos) \
  LPC804_CONNECT_FN_TO_GPIO(_4, 4, fn, idx, pos) \
  LPC804_CONNECT_FN_TO_GPIO(_3, 3, fn, idx, pos) \
  LPC804_CONNECT_FN_TO_GPIO(_2, 2, fn, idx, pos) \
  LPC804_CONNECT_FN_TO_GPIO(_1, 1, fn, idx, pos) \
  LPC804_CONNECT_FN_TO_GPIO(_0, 0, fn, idx, pos) 


pluc_wire_t lpc804_wire_table[] = {
  LPC804_CONNECT_GPIO_PLU_IN(PIO0_0, 0, 0)
  LPC804_CONNECT_GPIO_PLU_IN(PIO0_8, 0, 1)
  LPC804_CONNECT_GPIO_PLU_IN(PIO0_17, 0, 2)
  
  LPC804_CONNECT_GPIO_PLU_IN(PIO0_1, 1, 0)
  LPC804_CONNECT_GPIO_PLU_IN(PIO0_9, 1, 1)
  LPC804_CONNECT_GPIO_PLU_IN(PIO0_18, 1, 2)
  
  LPC804_CONNECT_GPIO_PLU_IN(PIO0_2, 2, 0)
  LPC804_CONNECT_GPIO_PLU_IN(PIO0_10, 2, 1)
  LPC804_CONNECT_GPIO_PLU_IN(PIO0_19, 2, 2)
  
  LPC804_CONNECT_GPIO_PLU_IN(PIO0_3, 3, 0)
  LPC804_CONNECT_GPIO_PLU_IN(PIO0_11, 3, 1)
  LPC804_CONNECT_GPIO_PLU_IN(PIO0_20, 3, 2)
  
  LPC804_CONNECT_GPIO_PLU_IN(PIO0_4, 4, 0)
  LPC804_CONNECT_GPIO_PLU_IN(PIO0_12, 4, 1)
  LPC804_CONNECT_GPIO_PLU_IN(PIO0_21, 4, 2)
  
  LPC804_CONNECT_GPIO_PLU_IN(PIO0_5, 5, 0)
  LPC804_CONNECT_GPIO_PLU_IN(PIO0_13, 5, 1)
  LPC804_CONNECT_GPIO_PLU_IN(PIO0_22, 5, 2)

  LPC804_CONNECT_PLU_OUT_GPIO(0, PIO0_7, 0)
  LPC804_CONNECT_PLU_OUT_GPIO(0, PIO0_14, 1)
  LPC804_CONNECT_PLU_OUT_GPIO(0, PIO0_23, 2)

  LPC804_CONNECT_PLU_OUT_GPIO(1, PIO0_8, 0)
  LPC804_CONNECT_PLU_OUT_GPIO(1, PIO0_15, 1)
  LPC804_CONNECT_PLU_OUT_GPIO(1, PIO0_24, 2)

  LPC804_CONNECT_PLU_OUT_GPIO(2, PIO0_9, 0)
  LPC804_CONNECT_PLU_OUT_GPIO(2, PIO0_16, 1)
  LPC804_CONNECT_PLU_OUT_GPIO(2, PIO0_25, 2)

  LPC804_CONNECT_PLU_OUT_GPIO(3, PIO0_10, 0)
  LPC804_CONNECT_PLU_OUT_GPIO(3, PIO0_17, 1)
  LPC804_CONNECT_PLU_OUT_GPIO(3, PIO0_26, 2)

  LPC804_CONNECT_PLU_OUT_GPIO(4, PIO0_11, 0)
  LPC804_CONNECT_PLU_OUT_GPIO(4, PIO0_18, 1)
  LPC804_CONNECT_PLU_OUT_GPIO(4, PIO0_27, 2)

  LPC804_CONNECT_PLU_OUT_GPIO(5, PIO0_12, 0)
  LPC804_CONNECT_PLU_OUT_GPIO(5, PIO0_19, 1)
  LPC804_CONNECT_PLU_OUT_GPIO(5, PIO0_28, 2)

  LPC804_CONNECT_PLU_OUT_GPIO(6, PIO0_13, 0)
  LPC804_CONNECT_PLU_OUT_GPIO(6, PIO0_20, 1)
  LPC804_CONNECT_PLU_OUT_GPIO(6, PIO0_29, 2)

  LPC804_CONNECT_PLU_OUT_GPIO(7, PIO0_14, 0)
  LPC804_CONNECT_PLU_OUT_GPIO(7, PIO0_21, 1)
  LPC804_CONNECT_PLU_OUT_GPIO(7, PIO0_30, 2)

  LPC804_CONNECT_LUT_OUT_PLU_OUT(0, 0)
  LPC804_CONNECT_LUT_OUT_PLU_OUT(1, 0)
  LPC804_CONNECT_LUT_OUT_PLU_OUT(2, 0)
  LPC804_CONNECT_LUT_OUT_PLU_OUT(3, 0)
  LPC804_CONNECT_LUT_OUT_PLU_OUT(4, 0)
  LPC804_CONNECT_LUT_OUT_PLU_OUT(5, 0)
  LPC804_CONNECT_LUT_OUT_PLU_OUT(6, 0)
  LPC804_CONNECT_LUT_OUT_PLU_OUT(7, 0)
  LPC804_CONNECT_LUT_OUT_PLU_OUT(8, 0)
  LPC804_CONNECT_LUT_OUT_PLU_OUT(10, 0)

  LPC804_CONNECT_LUT(0)
  LPC804_CONNECT_LUT(1)
  LPC804_CONNECT_LUT(2)
  LPC804_CONNECT_LUT(3)
  LPC804_CONNECT_LUT(4)
  LPC804_CONNECT_LUT(5)
  LPC804_CONNECT_LUT(6)
  LPC804_CONNECT_LUT(7)
  LPC804_CONNECT_LUT(8)
  LPC804_CONNECT_LUT(9)
  LPC804_CONNECT_LUT(10)
  LPC804_CONNECT_LUT(11)
  LPC804_CONNECT_LUT(12)
  LPC804_CONNECT_LUT(13)
  LPC804_CONNECT_LUT(14)
  LPC804_CONNECT_LUT(15)
  LPC804_CONNECT_LUT(16)
  LPC804_CONNECT_LUT(17)
  LPC804_CONNECT_LUT(18)
  LPC804_CONNECT_LUT(19)
  LPC804_CONNECT_LUT(20)
  LPC804_CONNECT_LUT(21)
  LPC804_CONNECT_LUT(22)
  LPC804_CONNECT_LUT(23)
  LPC804_CONNECT_LUT(24)
  LPC804_CONNECT_LUT(25)
  
  LPC804_CONNECT_FF(0)
  LPC804_CONNECT_FF(1)
  LPC804_CONNECT_FF(2)
  LPC804_CONNECT_FF(3)
  
  
  LPC804_CONNECT_TO_FN(PLU_CLKIN, 0x01c, 3)
  LPC804_CONNECT_TO_FN(LVLSHFT_IN0, 0x018, 1)
  LPC804_CONNECT_TO_FN(LVLSHFT_IN1, 0x018, 2)
  LPC804_CONNECT_TO_FN(T0_CAP0, 0x00c, 1)
  LPC804_CONNECT_TO_FN(T0_CAP1, 0x00c, 2)
  LPC804_CONNECT_TO_FN(T0_CAP2, 0x00c, 3)
  
  LPC804_CONNECT_FROM_FN(T0_MAT0,  0x010, 0)
  LPC804_CONNECT_FROM_FN(T0_MAT1,  0x010, 1)
  LPC804_CONNECT_FROM_FN(T0_MAT2,  0x010, 2)
  LPC804_CONNECT_FROM_FN(T0_MAT3,  0x010, 3)

  LPC804_CONNECT_FROM_FN(LVLSHFT_OUT0,  0x018, 3)
  LPC804_CONNECT_FROM_FN(LVLSHFT_OUT1,  0x01c, 0)
  LPC804_CONNECT_FROM_FN(COMP0_OUT,  0x014, 2)		// ACMP_O
  LPC804_CONNECT_FROM_FN(CLKOUT,  0x014, 3)
  LPC804_CONNECT_FROM_FN(GPIO_INT_BMAT,  0x018, 0)

  /* adding the LVLSHFT connection does not make sense as long as we do not find the shortest path in pluc_calc_from_to() */
  
  // LPC804_CONNECT_FN_TO_FN(LVLSHFT_IN0, LVLSHFT_OUT0)
  // LPC804_CONNECT_FN_TO_FN(LVLSHFT_IN1, LVLSHFT_OUT1)
  
  PLUC_CONNECT_NONE()
};


/*==================================================*/
/* forward declarations */
int pluc_calc_from_to(const char *from, const char *to);
void pluc_mark_route_chain_wire_list(void);


/*==================================================*/
void pluc_log(const char *format, ...)
{
  va_list va;
  va_start(va, format);
  vprintf(format, va);
  puts("");
  va_end(va);
}

void pluc_err(const char *format, ...)
{
  va_list va;
  va_start(va, format);
  vprintf(format, va);
  puts("");
  va_end(va);
}

/*==================================================*/
/*
source:
https://en.wikibooks.org/wiki/Algorithm_Implementation/Strings/Levenshtein_distance
Creative Commons Attribution-ShareAlike License
*/
#define MIN3(a, b, c) ((a) < (b) ? ((a) < (c) ? (a) : (c)) : ((b) < (c) ? (b) : (c)))

int levenshtein(const char *s1, const char *s2) 
{
    unsigned int s1len, s2len, x, y, lastdiag, olddiag;
    s1len = strlen(s1);
    s2len = strlen(s2);
    unsigned int column[s1len+1];
    for (y = 1; y <= s1len; y++)
        column[y] = y;
    for (x = 1; x <= s2len; x++)
    {
        column[0] = x;
        for (y = 1, lastdiag = x-1; y <= s1len; y++)
	{
            olddiag = column[y];
            column[y] = MIN3(column[y] + 1, column[y-1] + 1, lastdiag + (s1[y-1] == s2[x-1] ? 0 : 1));
            lastdiag = olddiag;
        }
    }
    return(column[s1len]);
}

/*==================================================*/
/* remove a don't care column, which may appear during the replace and map algorithm */
void pluc_remove_dc(pinfo *pi, dclist cl)
{
  int i;
  
  i = 0;
  while( i < pinfoGetInCnt(pi) && pinfoGetInCnt(pi) >= 2 )
  //while( i < pinfoGetInCnt(pi)  )
  {
    if ( dclIsDCInVar(pi, cl, i) != 0 )
    {
      dclDeleteIn(pi, cl, i);
      pinfoDeleteInLabel(pi, i);      
    }
    else
    {
      i++;
    }    
  }
}


/*==================================================*/
/*=== init ===*/
/*==================================================*/

static void pluc_fsm_log_fn(void *data, int ll, char *fmt, va_list va)
{
  vprintf(fmt, va);
  puts("");
}

int pluc_init_keywords(void)
{
  int i;
  pluc_keywords = b_sl_Open();
  if ( pluc_keywords == NULL )
    return 0;
  
  i = 0;
  while( lpc804_wire_table[i].from != NULL )
  {
    if ( b_sl_Find(pluc_keywords, lpc804_wire_table[i].from) < 0 )
      if ( b_sl_Add(pluc_keywords, lpc804_wire_table[i].from) < 0 )
	return 0;

    if ( b_sl_Find(pluc_keywords, lpc804_wire_table[i].to) < 0 )
      if ( b_sl_Add(pluc_keywords, lpc804_wire_table[i].to) < 0 )
	return 0;
    i++;
  }
  return 1;
}

void pluc_show_keywords(void)
{
  int i;
  for( i = 0; i < b_sl_GetCnt(pluc_keywords); i++ )
  {
    printf("%s ", b_sl_GetVal(pluc_keywords, i));
  }
  puts("");
}


int pluc_init(void)
{
  int i;
  pluc_log("Init");  
  
  /*  read */
  
  if ( pinfoInit(&pi) == 0 )
    return 0;
  if ( pinfoInit(&pi2) == 0 )
    return 0;
  if ( dclInitVA(4, &cl_on, &cl_dc, &cl2_on, &cl2_dc) == 0 )
    return 0;

  
  fsm_state_out_signal = "_FF_as_output_";
  fsm_state_in_signal = "_FF_as_input_";		/* different name than out_signal, otherwise merge will not work */
  
  
  fsm = fsm_Open();
  if ( fsm == NULL )
    return 0;
  fsm_PushLogFn(fsm, pluc_fsm_log_fn, NULL, 0);

  /* map */

  for( i = 0; i < PLUC_LUT_MAX; i++ )
  {
    if ( pinfoInit(&(pluc_lut_list[i].pi) ) == 0 )
      return 0;
    if ( dclInit( &(pluc_lut_list[i].dcl) ) == 0 )
      return 0;
    //pluc_lut_list[i].user_out_name = NULL;
    pluc_lut_list[i].is_placed = 0;
  }
  
  pluc_lut_cnt = PLUC_FF_MAX;		// we could assign 0, if there is no state machine
    
  if ( pluc_init_keywords() == 0 )
    return 0;
  
  return 1;
}

/*==================================================*/
/*=== read ===*/
/*==================================================*/


int pluc_check_signal(const char *s, int is_error)
{
  int distance_min = 30000;
  const char *keyword_min = NULL;
  int distance_curr;
  int i;
  
  if ( strncmp(s, fsm_state_out_signal, strlen(fsm_state_out_signal)) == 0 )
    return 1;

  if ( strncmp(s, fsm_state_in_signal, strlen(fsm_state_in_signal)) == 0 )
    return 1;
  
  for( i = 0; i < b_sl_GetCnt(pluc_keywords); i++ )
  {
    if ( strcmp(b_sl_GetVal(pluc_keywords, i), s) == 0 )
      return 1;
    if ( is_error )
    {
      distance_curr = levenshtein(b_sl_GetVal(pluc_keywords, i), s);
      if ( distance_min > distance_curr )
      {
	distance_min = distance_curr;
	keyword_min = b_sl_GetVal(pluc_keywords, i);
      }
    }
  }
  if ( is_error )
  {
    if ( keyword_min != NULL )
    {
      pluc_err("Read: Signal '%s' invalid. Do you mean '%s'? Use 'pluc -listkeywords' to list all valid signal names. ", s, keyword_min);
    }
  }
  return 0;
}

/*
  There a two global problem descriptions
  pi, cl_on, cl_dc and pi2, cl2_on, cl2_dc   
*/


int pluc_build_fsm(void)
{
  /*
    is_fbo: is not used any more
    is_old: If the new state minimizer fails, use the old method
  */
  
  if ( cmdline_mis )
    if( fsm_MinimizeStates(fsm, /* is_fbo=*/ 0, /* is_old=*/ 0) == 0 )	
      return 0;

  if ( fsm_GetGroupCnt(fsm) > 0 )
  {
    if ( fsm_GetNodeCnt(fsm) > fsm_GetGroupCnt(fsm) )
    {
      pluc_log("FSM: State reduction %d -> %d", fsm_GetNodeCnt(fsm), fsm_GetGroupCnt(fsm));
    }
    else
    {
      pluc_log("FSM: State count = %d", fsm_GetNodeCnt(fsm));
    }
  }
  
  /*
    encode is one of :
    encode = FSM_ENCODE_FAN_IN; 
    encode = FSM_ENCODE_IC_ALL; 
    encode = FSM_ENCODE_IC_PART;
    encode = FSM_ENCODE_SIMPLE; 
  */
  
  if ( fsm_BuildClockedMachine(fsm, FSM_ENCODE_SIMPLE) == 0 )
  {
    return 0;
  }
  
  /*
  result:
	fsm->pi_machine, 
      fsm->cl_machine, 
      fsm_GetCodeWidth(fsm), 
      state signals have the format zo# and zi#
  */
  
  
  return 1;
}

int pluc_is_extension(const char *filename, const char *ext)
{
  size_t flen = strlen(filename);
  size_t elen = strlen(ext);
  if ( flen < elen )
    return 0;
  if ( strcmp(filename+flen-elen, ext) != 0 )
    return 0;
  return 1;	/* match */
}

/* merge file into pi/cl_on */
int pluc_read_file(const char *filename)
{
    pinfoDestroy(&pi2);
    if ( pinfoInit(&pi2) == 0 )
      return 0;
    dclClear(cl2_on);
    
    if ( pluc_is_extension(filename, ".kiss") != 0 )
    {
      fsm_Clear(fsm);

      fsm_ReadKISS(fsm, filename);
      pluc_build_fsm();
      //dclShow(fsm->pi_machine, fsm->cl_machine);
      pluc_log("Read (KISS): FSM state bits=%d in=%d out=%d", fsm->code_width, fsm->in_cnt, fsm->out_cnt);
      //pluc_log("Read: FSM state bits=%d in=%d out=%d", fsm->code_width, fsm->pi_machine->in_cnt, fsm->pi_machine->out_cnt);
      pinfoMerge(&pi, cl_on, fsm->pi_machine, fsm->cl_machine);
      //dclShow(&pi, cl_on);

      if ( cmdline_dbg )
      {
	dclShow(fsm->pi_machine, fsm->cl_machine);
      }
      
    }
    else if ( pluc_is_extension(filename, ".bms") != 0 )
    {
      fsm_Clear(fsm);

      //fsm_Import(fsm, filename);

      fsm_ReadBMS(fsm, filename);
      pluc_build_fsm();
      //dclShow(fsm->pi_machine, fsm->cl_machine);
      pluc_log("Read (BMS): FSM state bits=%d in=%d out=%d", fsm->code_width, fsm->in_cnt, fsm->out_cnt);
      
      if ( cmdline_dbg )
      {
	dclShow(fsm->pi_machine, fsm->cl_machine);
      }
      
      //pluc_log("Read: FSM state bits=%d in=%d out=%d", fsm->code_width, fsm->pi_machine->in_cnt, fsm->pi_machine->out_cnt);
      pinfoMerge(&pi, cl_on, fsm->pi_machine, fsm->cl_machine);
      //dclShow(&pi, cl_on);
      
    }
    else if ( pluc_is_extension(filename, ".bex") != 0 )
    {
      if ( dclReadBEX(&pi2, cl2_on, cl2_dc, filename) == 0 )    /* dcex.c */
      {
	pluc_err("Read: Error with file '%s'", filename);
	return 0;
      }
      pluc_log("Read (BEX): File %s imported, in=%d out=%d", filename, pi2.in_cnt, pi2.out_cnt);
      if ( cmdline_dbg )
      {
	dclShow(&pi2, cl2_on);
      }
      
      pinfoMerge(&pi, cl_on, &pi2, cl2_on);      
    }
    else if ( dclImport(&pi2, cl2_on, cl2_dc, filename) != 0 )
    {
      /* combine both functions, consider the case where the output of cl2 is input of cl */
      //puts("pluc_read_file");
      //dclShow(&pi2, cl2_on);
      pinfoMerge(&pi, cl_on, &pi2, cl2_on);
      //dclShow(&pi, cl_on);
      
      pluc_log("Read (generic): File %s imported, in=%d out=%d", filename, pi2.in_cnt, pi2.out_cnt);
      if ( cmdline_dbg )
      {
	dclShow(&pi2, cl2_on);
      }
    }
    else
    {
      pluc_err("Read: Error with file/expression '%s'.", filename);
      return 0;
    }

    
    return 1;
}

int pluc_read(void)
{
  int i;
  volatile b_sl_type sl;
  pluc_log("Read (files: %d)", cl_file_cnt);
  
  pinfoDestroy(&pi);
  if ( pinfoInit(&pi) == 0 )
    return 0;
  dclClear(cl_on);
  
  for( i = 0; i < cl_file_cnt; i++ )
  {
    if ( pluc_read_file(cl_file_list[i]) == 0 )
      return 0;
  }


  /* try to replace unknown signal names */
  if ( cmdline_dbg )
  {
    pluc_log("Read: File read & merge done");
    dclShow(&pi, cl_on);
  }
  
  sl = pinfoGetOutLabelList(&pi);
  i = 0;
  while( i <  b_sl_GetCnt(sl) )
  {
    if ( pluc_check_signal(b_sl_GetVal(sl, i), 0) == 0 )
    {
      pluc_log("Read: Resolving variable '%s'", b_sl_GetVal(sl, i)); 
      switch( dclReplaceInOut(&pi, cl_on, i)  )
      {
	case 0:
	  pluc_log("Read: Variable '%s' resolve failed.", b_sl_GetVal(sl, i)); 
	  i++;
	  break;
	case -1:
	  pluc_err("Read: Illegal cyclic dependency for variable '%s'.", b_sl_GetVal(sl, i)); 
	  i++;
	  break;
	case -2:
	  pluc_err("Read: Variable '%s' is used, but is not written/defined.", b_sl_GetVal(sl, i)); 
	  i++;
	  break;
	default:
	  // do not increment i, because column i was deleted
	  break;
      }
    }
    else
    {
      i++;
    }
  }
  pluc_remove_dc(&pi, cl_on);
  if ( cmdline_dbg )
  {
    dclShow(&pi, cl_on);
  }
  

  sl = pinfoGetOutLabelList(&pi);
  for( i = 0; i < b_sl_GetCnt(sl); i++ )
    if ( pluc_check_signal(b_sl_GetVal(sl, i), 1) == 0 )
      return 0;
  sl = pinfoGetInLabelList(&pi);
  for( i = 0; i < b_sl_GetCnt(sl); i++ )
    if ( pluc_check_signal(b_sl_GetVal(sl, i), 1) == 0 )
      return 0;
  //b_sl_type pinfoGetOutLabelList(pinfo *pi);

  
  pluc_log("Read: Input: %s", b_sl_ToStr(pinfoGetInLabelList(&pi), ", "));
  pluc_log("Read: Output: %s", b_sl_ToStr(pinfoGetOutLabelList(&pi), ", "));
  pluc_log("Read: Done (overall problem in=%d out=%d)", pi.in_cnt, pi.out_cnt);

  
  //dclShow(&pi, cl_on);

  return 1;
}

/*==================================================*/
/*=== map ===*/
/*==================================================*/

/* return an internal unique name for the LUT, the internal name starts with a "." */
const char *pluc_get_lut_output_internal_name(int pos)
{
  static char s[32];
  sprintf(s, ".LUT%02d", pos);
  return s;
}

/* return the LUT name, as used in the wire table...  */
const char *pluc_get_lut_output_name(int pos)
{
  static char s[32];
  sprintf(s, "LUT%d", pos);
  return s;
}

const char *pluc_get_ff_output_name(int pos)
{
  static char s[32];
  sprintf(s, "FF%d", pos);
  return s;
}



/*
  add a (reduced) problem into the lut list and increase the number of
  luts in the list.

  All don't cares are removed before the LUT is added (pluc_remove_dc). 
*/
int pluc_add_lut(pinfo *pi, dclist cl)
{
  const char *out_signal;
  
  out_signal = pinfoGetOutLabel(pi, 0);
  if ( strncmp(out_signal, fsm_state_out_signal, strlen(fsm_state_out_signal)) != 0 )
  {
    /* not a FF, add this problem to the LUT table */
    
    if ( pinfoCopy( &(pluc_lut_list[pluc_lut_cnt].pi), pi) == 0 )
      return 0;

    if ( dclCopy(pi, pluc_lut_list[pluc_lut_cnt].dcl, cl) == 0 )
      return 0;

    pluc_remove_dc(&(pluc_lut_list[pluc_lut_cnt].pi), pluc_lut_list[pluc_lut_cnt].dcl);

    //if ( pluc_lut_list[pluc_lut_cnt].user_out_name != NULL )
    //  free(pluc_lut_list[pluc_lut_cnt].user_out_name);
    //pluc_lut_list[pluc_lut_cnt].user_out_name = NULL;
    pluc_lut_list[pluc_lut_cnt].is_placed = 0;

    pluc_log("Map: Added LUT%d, inputs: %d (%s), output cnt: %d", pluc_lut_cnt, pluc_lut_list[pluc_lut_cnt].pi.in_cnt, b_sl_ToStr(pinfoGetInLabelList(&(pluc_lut_list[pluc_lut_cnt].pi)), ", "), pluc_lut_list[pluc_lut_cnt].pi.out_cnt);

    pluc_lut_cnt++;
  }
  else
  {
    /* FF, add this problem to the LUT table with has a FF output */

    if ( pinfoCopy( &(pluc_lut_list[pluc_ff_cnt].pi), pi) == 0 )
      return 0;

    if ( dclCopy(pi, pluc_lut_list[pluc_ff_cnt].dcl, cl) == 0 )
      return 0;

    pluc_remove_dc(&(pluc_lut_list[pluc_ff_cnt].pi), pluc_lut_list[pluc_ff_cnt].dcl);

    pluc_lut_list[pluc_ff_cnt].is_placed = 0;

    pluc_log("Map: Added FF%d, inputs: %d (%s), output cnt: %d", pluc_ff_cnt, pi->in_cnt, b_sl_ToStr(pinfoGetInLabelList(pi), ", "), pi->out_cnt);
    
    pluc_ff_cnt++;
    
    pluc_is_FF_used = 1;		/* yes, a FF will be used */
  }
  return 1;
}

int pluc_map_cof(pinfo *pi, dclist cl, dcube *cof, int depth)
{
  int i;
  int none_dc_cnt;
  
  dclist cl_left, cl_right;
  int new_left_lut;
  int new_right_lut;
  
  /* get two cubes for the later split (if required) */
  dcube *cofactor_left = &(pi->stack1[depth]);
  dcube *cofactor_right = &(pi->stack2[depth]);
  
  /* calculate the number of variabels which are not fully DC */
  /* Variables which are DC in all cubes must be ignored (infact they could be deleted) */
  none_dc_cnt = 0;
  for( i = 0; i < pi->in_cnt; i++ )
  {
    if ( dclIsDCInVar(pi, cl, i) == 0 )
      none_dc_cnt++;
  }
  
  
  /* if the number of variables (which are not DC) is lower than 6, then we are done */
  if ( none_dc_cnt <= 5 )
  {
    //dclShow(pi, cl);
    assert( none_dc_cnt > 0 );
    if ( dclCnt(cl) == 1 )
    {
      if ( dcGetIn( dclGet(cl, 0), 0 ) == 2 )		/* identity: input equals output */
      {
	if ( pi->in_cnt == 1 && pi->out_cnt == 1 )
	{
	  if ( pluc_calc_from_to(pinfoGetInLabel(pi, 0), pinfoGetOutLabel(pi, 0)) != 0 )
	  {
	    pluc_mark_route_chain_wire_list();
	    return 1;
	  }
	  else
	  {
	    pluc_log("Map: No route for identity %s --> %s", pinfoGetInLabel(pi, 0), pinfoGetOutLabel(pi, 0));	  
	  }
	}
	else
	{
	  pluc_log("Map: in or out not 1, fallback to LUT construction");	  
	}
      }
      else if (dcGetIn( dclGet(cl, 0), 0 ) == 1 ) /* inverted: output <= !input */
      {
	/* could be implemented with GPIO read value inversion, if there is a GPIO used */
      }
    }
    
    if ( pluc_add_lut(pi, cl) == 0 )
      return 0;
    //pluc_log("Map: Leaf fn (in-cnt %d) added to LUT table (index %d), output '%s'", none_dc_cnt, pluc_lut_cnt-1, pinfoGetOutLabel(pi, 0));
    
    //printf("leaf (depth=%d)\n", depth);
    //dclShow(pi, cl);
    return 1;
  }
  
  /* find a suitable variable for splitting */
  for( i = 0; i < pi->in_cnt; i++ )
  {
    if ( dclIsDCInVar(pi, cl, i) == 0 )
    {
      break;
    }
  }
  
  if ( pinfoCntDCList(pi, cl, pi->tmp+0) == 0 )
    return 0;		/* memory issue */
  i = pinfoGetSplittingInVar(pi);


  if ( i >= pi->in_cnt )
    return 0;
  
  
  /* for a full split we need two more luts: get the number for these luts */
  new_left_lut = pluc_internal_cnt+0;
  new_right_lut = pluc_internal_cnt+1;
  pluc_internal_cnt += 2;
  
  
  /* create a new boolean function which connects the later functions */
  {
    pinfo *pi_connect = pinfoOpen();
    dclist cl_connect;

    if ( pinfoAddOutLabel(pi_connect, pinfoGetOutLabel(pi, 0)) < 0 )
      return pinfoClose(pi_connect), 0;
    
    if ( pinfoAddInLabel(pi_connect, pinfoGetInLabel(pi, i)) < 0 )
      return pinfoClose(pi_connect), 0;
    
    if ( pinfoAddInLabel(pi_connect, pluc_get_lut_output_internal_name(new_left_lut)) < 0 )
      return pinfoClose(pi_connect), 0;

    if ( pinfoAddInLabel(pi_connect, pluc_get_lut_output_internal_name(new_right_lut)) < 0 )
      return pinfoClose(pi_connect), 0;

    if ( dclInit(&cl_connect) == 0 )
      return pinfoClose(pi_connect), 0;
      
    
    dcSetTautology(pi, pi_connect->tmp+17);
    dcSetIn(pi_connect->tmp+17, 0, 2);
    dcSetIn(pi_connect->tmp+17, 1, 2);
    if ( dclAdd(pi_connect, cl_connect, pi_connect->tmp+17) < 0 )
      return pinfoClose(pi_connect), dclDestroy(cl_connect), 0;
    
    dcSetTautology(pi, pi_connect->tmp+17);
    dcSetIn(pi_connect->tmp+17, 0, 1);
    dcSetIn(pi_connect->tmp+17, 2, 1);
    if ( dclAdd(pi_connect, cl_connect, pi_connect->tmp+17) < 0 )
      return pinfoClose(pi_connect), dclDestroy(cl_connect), 0;

    //printf("connector (depth=%d)\n", depth);
    //dclShow(pi_connect, cl_connect);
    
    if ( pluc_add_lut(pi_connect, cl_connect) == 0 )
      return pinfoClose(pi_connect), dclDestroy(cl_connect), 0;

    pinfoClose(pi_connect);
    dclDestroy(cl_connect);
  }
  
  /* construct the cofactor cubes: split happens against variable i (as derived above) */
  dcCopy(pi, cofactor_left, cof);
  dcCopy(pi, cofactor_right, cof);
  dcInSetAll(pi, cofactor_left, CUBE_IN_MASK_DC);
  dcInSetAll(pi, cofactor_right, CUBE_IN_MASK_DC);
  dcSetIn(cofactor_left, i, 2);
  dcSetIn(cofactor_right, i, 1);
  
  
  //int dcGetBinateInVarCofactor(pinfo *pi, dcube *r, dcube *rinv, dclist cl, dcube *cof)
  //if ( dcGetNoneDCInVarCofactor(pi, cofactor_left, cofactor_right, cl, cof) == 0 )
  //  return 0;

  if ( dclInitVA(2, &cl_left, &cl_right) == 0 )
    return 0;
  
  if ( dclSCCCofactor(pi, cl_left, cl, cofactor_left) == 0 )
    return dclDestroyVA(2, cl_left, cl_right), 0;
    
  if ( dclSCCCofactor(pi, cl_right, cl, cofactor_right) == 0 )
    return dclDestroyVA(2, cl_left, cl_right), 0;

  pinfoSetOutLabel(pi, 0, pluc_get_lut_output_internal_name(new_left_lut));
  if ( pluc_map_cof(pi, cl_left, cofactor_left, depth+1) == 0 )
    return dclDestroyVA(2, cl_left, cl_right), 0;
  
  pinfoSetOutLabel(pi, 0, pluc_get_lut_output_internal_name(new_right_lut));
  if ( pluc_map_cof(pi, cl_right, cofactor_right, depth+1) == 0 )
    return dclDestroyVA(2, cl_left, cl_right), 0;
  
  return dclDestroyVA(2, cl_left, cl_right), 1;
  
}


int pluc_map(void)
{
  int out_var;
  pluc_log("Map (output variables: %d)", pi.out_cnt);
  
  for( out_var = 0; out_var < pi.out_cnt; out_var++)
  {
    
    pinfoDestroy(&pi2);
    dclDestroy(cl2_on);
    if ( pinfoInitFromOutVar(&pi2, &cl2_on, &pi, cl_on, out_var) == 0 )
    {
      pluc_log("Map failed (pinfoInitFromOutVar)");
      return 0;
    }
    assert(pi2.out_cnt == 1);
    dcube *cof = pi2.tmp+2;
    dcSetTautology(&pi2, cof);
    
    /* remove dc column, so that we can more easier detect direct routes */
    pluc_remove_dc(&pi2, cl2_on);
    
    if ( dclCnt(cl2_on) == 0 )
    {
      pluc_log("Map WARNING: Unspecified/empty output variable %s found.", pinfoGetOutLabel(&pi, out_var));
    }      
    else
    {
      pluc_log("Map: Output variable '%s'", pinfoGetOutLabel(&pi, out_var));
      //dclShow(&pi2, cl2_on);
      /* start the simplification */
      if ( pluc_map_cof(&pi2, cl2_on, cof, 0) == 0 )
      {
	pluc_log("Map failed (pluc_map_cof)");
	return 0;
      }
    }
  }
  return 1;
}

/*==================================================*/
/*=== route ===*/
/*==================================================*/



/*==================================================*/
/* Wire replace procedures */

int pluc_pinfo_replace_in_label(pinfo *pi, const char *search, const char *replace)
{
  int pos;
  pos = pinfoFindInLabelPos(pi, search);
  if ( pos < 0 )
    return 1;
  return pinfoSetInLabel(pi, pos, replace);
}

int pluc_replace_all_lut_in(const char *search, const char *replace)
{
  int i;
  //pluc_log("Route: Replace internal signal %s --> %s", search, replace);
  for( i = 0; i < pluc_lut_cnt; i++ )
  {
    if ( pluc_pinfo_replace_in_label( &(pluc_lut_list[i].pi), search, replace ) == 0 )
      return 0;
  }
  return 1;
}

int pluc_replace_internal_lut_signals(void)
{
  int i;
  const char *s;
  
  pluc_log("Route: Replace internal signal names");
  
  for( i = 0; i < pluc_lut_cnt; i++ )
  {
    s = pinfoGetOutLabel(&(pluc_lut_list[i].pi), 0);
    if ( s != NULL )
    {
      if ( s[0] == '.' )
      {
	if ( pluc_replace_all_lut_in(s, pluc_get_lut_output_name(i)) == 0 )
	  return 0;
	if ( pinfoSetOutLabel(&(pluc_lut_list[i].pi), 0, pluc_get_lut_output_name(i)) == 0 )
	  return 0;
      }
    }
  }
  return 1;
}

/*
  during read and map, ff output and ff input have different internal names
  additionally the ff number (postfix) is defined by the reader, but must be
  aligned with the position in the lut table.
*/
int pluc_replace_ff_signals(void)
{
  int i;
  const char *s;
  const char *post;
  char buf[64];
  
  pluc_log("Route: Replace ff signal names (ff cnt=%d)", pluc_ff_cnt);

  for( i = 0; i < pluc_ff_cnt; i++ )
  {
    s = pinfoGetOutLabel(&(pluc_lut_list[i].pi), 0);
    if ( s != NULL )
    {
      
      if ( strncmp( s, fsm_state_out_signal, strlen(fsm_state_out_signal)) == 0 )
      {
	post = s + strlen(fsm_state_out_signal);
	sprintf(buf, "%s%s", fsm_state_in_signal, post);
	//pluc_log("Route: Replace %s -> %s", buf, pluc_get_ff_output_name(i));
	if ( pluc_replace_all_lut_in(buf, pluc_get_ff_output_name(i)) == 0 )
	  return 0;
	if ( pinfoSetOutLabel(&(pluc_lut_list[i].pi), 0, pluc_get_ff_output_name(i)) == 0 )
	  return 0;
      }
    }
  }
  return 1;
}

/*==================================================*/
/* Route find procedures */

/* array, which stores the current route chain, generated by  "pluc_calc_route_chain" */
#define PLUC_ROUTE_CHAIN_MAX 16
int pluc_route_chain_list[PLUC_ROUTE_CHAIN_MAX];		/* points into wire table */
int pluc_route_chain_cnt = 0;

/* mark the current list in the wire table */
void pluc_mark_route_chain_wire_list(void)
{
  int i, j;
  for( i = 0; i < pluc_route_chain_cnt; i++ )
  {
    /* mark the route as used */
    lpc804_wire_table[pluc_route_chain_list[i]].is_used = 1;
    
    /* mark all other routes, leading to the same resource, as blocked */
    j = 0;
    while( lpc804_wire_table[j].from != NULL )
    {
      if ( j != pluc_route_chain_list[i] )
      {
	if ( strcmp(lpc804_wire_table[pluc_route_chain_list[i]].to, lpc804_wire_table[j].to) == 0 )
	{
	  lpc804_wire_table[j].is_wr_blocked = 1;
	  //pluc_log("Route: Write blocked %s -> %s", lpc804_wire_table[j].from, lpc804_wire_table[j].to);
	}
      }
      j++;
    }

    /* if the input (read block) also needs to be blocked, then do so... */
    if ( lpc804_wire_table[pluc_route_chain_list[i]].is_rd_block )
    {
      j = 0;
      while( lpc804_wire_table[j].from != NULL )
      {
	if ( j != pluc_route_chain_list[i] )
	{
	  if ( strcmp(lpc804_wire_table[pluc_route_chain_list[i]].from, lpc804_wire_table[j].from) == 0 )
	  {
	    lpc804_wire_table[j].is_rd_blocked = 1;
	    //pluc_log("Route: Read blocked %s -> %s", lpc804_wire_table[j].from, lpc804_wire_table[j].to);
	  }
	}
	j++;
      } // while	
    } // if is_rd_block
    
  } // for chain
}

int pluc_find_to(const char *s)
{
    int i = 0;
    while( lpc804_wire_table[i].from != NULL )
    {
      if ( strcmp(lpc804_wire_table[i].to, s) == 0 )
	return i;
      i++;
    }
    return -1;
}

int pluc_find_from(const char *s)
{
    int i = 0;
    while( lpc804_wire_table[i].from != NULL )
    {
      if ( strcmp(lpc804_wire_table[i].from, s) == 0 )
	return i;
      i++;
    }
    return -1;
}



/*

  OBSOLETE

  calculate a route from the provided input or output node 
  result is stored in
    - pluc_route_chain_list
    - pluc_route_chain_cnt

  Args:
    s:	port name
    is_in_to_out: route direction 0: from out to in, 1: from in to out
*/
int pluc_calc_route_chain(const char *s, int is_in_to_out)
{
  int i;
  
  
  pluc_route_chain_cnt = 0;
  i = 0;
  while( i < PLUC_ROUTE_CHAIN_MAX )
  {
    if ( is_in_to_out ) 
      pluc_route_chain_list[i] = pluc_find_from(s);
    else
      pluc_route_chain_list[i] = pluc_find_to(s);
    if ( pluc_route_chain_list[i] < 0 )
    {
      /* not found */
      pluc_err("Route: Signal '%s' unknown", s);
      return 0;
    }
    pluc_log("Route: From %s to %s", lpc804_wire_table[pluc_route_chain_list[i]].from, lpc804_wire_table[pluc_route_chain_list[i]].to);

    if ( is_in_to_out ) 
    {
      if ( lpc804_wire_table[pluc_route_chain_list[i]].is_lut_in != 0 )
      {
	i++;
	break;
      }
    }
    else
    {
      if ( lpc804_wire_table[pluc_route_chain_list[i]].is_lut_out != 0 )
      {
	i++;
	break;
      }
    }

    
    
    if ( is_in_to_out ) 
      s = lpc804_wire_table[pluc_route_chain_list[i]].to;
    else
      s = lpc804_wire_table[pluc_route_chain_list[i]].from;
    i++;
  }

  pluc_route_chain_cnt = i;  
  return 1;
}

int _pluc_calc_from_to_sub(const char *from, const char *to, int depth)
{
  int i = 0;
  const char *f;
  //pluc_log("from_to: %s-->%s depth: %d", from, to, depth);

  //pluc_log("from_to: %s-->%s", from, to);
  f = lpc804_wire_table[i].from;
  while( f != NULL )
  {
    if ( *f == *from )  // performance optimization
    {
      if ( lpc804_wire_table[i].is_visited == 0 && lpc804_wire_table[i].is_rd_blocked == 0 && lpc804_wire_table[i].is_wr_blocked == 0  )
      {
	if ( strcmp(f, from) == 0 )
	{
	  if ( strcmp(lpc804_wire_table[i].to, to) == 0 )
	  {
	    /* found leaf node */
	    pluc_route_chain_list[pluc_route_chain_cnt++] = i;
	    //pluc_log("found from_to: %s-->%s", from, to);
	    return 1;
	  }
	  lpc804_wire_table[i].is_visited = 1;
	  //pluc_log("consider from_to: %s-->%s", lpc804_wire_table[i].from, lpc804_wire_table[i].to);
	  if ( _pluc_calc_from_to_sub(lpc804_wire_table[i].to, to, depth+1) != 0 )
	  {
	    pluc_route_chain_list[pluc_route_chain_cnt++] = i;
	    //pluc_log("found sub from_to: %s-->%s", from, to);
	    return 1;
	  }
	  lpc804_wire_table[i].is_visited = 0;
	}
      }
    }
    i++;
    f = lpc804_wire_table[i].from;
  }
  return 0;
}

/*
  calculate the chain from the given wire to the provided wire.
  fills the pluc_route_chain_list array and also updates pluc_route_chain_cnt 
  During the calculation any is_used connections are ignored.

  pluc_route_chain_cnt is updated.
  After successfull call to this function, also call pluc_mark_route_chain_wire_list();

  TODO: implement shortest path
*/
int pluc_calc_from_to(const char *from, const char *to)
{
  int i = 0;

  while( lpc804_wire_table[i].from != NULL )
  {
    lpc804_wire_table[i].is_visited = 0;
    i++;
  }
  
  pluc_route_chain_cnt = 0;
  return _pluc_calc_from_to_sub(from, to, 0);
}

/*
  All required luts are stored in pluc_lut_list.
  However luts are not yet placed and connected.
  This function will place the external luts (those which are 
  connected to a none-internal signal) in the correct lut.
  Internl signals names start with a ".".
*/
int pluc_route_external_connected_luts(void)
{
  int i;
  const char *s;
  pluc_log("Route: LUTs with external signals (LUT cnt=%d)", pluc_lut_cnt);
  for( i = 0; i < pluc_lut_cnt; i++ )
  {
    if ( pluc_lut_list[i].is_placed == 0 )
    {
      s = pinfoGetOutLabel( &(pluc_lut_list[i].pi), 0);
      if ( s != NULL )
      {
	if ( s[0] != '.' )
	{	  
	  if ( strcmp( pluc_get_lut_output_name(i), pinfoGetOutLabel(&(pluc_lut_list[i].pi), 0) ) != 0 )
	  {
	    if ( pluc_calc_from_to(pluc_get_lut_output_name(i), pinfoGetOutLabel(&(pluc_lut_list[i].pi), 0) ) != 0 )
	    {
	      
	      pluc_log("Route: Path found from LUT%d to %s ", i, pinfoGetOutLabel(&(pluc_lut_list[i].pi), 0));
	      /* TODO: We must search from LUT i to output pinfoGetOutLabel(&(pluc_lut_list[i].pi), 0) */
	      //if ( pluc_calc_route_chain(s, 0) == 0 )	// WRONG function call
	      //{
	      //  pluc_err("No route found from '%s' to any LUT", s);
	      //  return 0;
	      //}
	      pluc_mark_route_chain_wire_list();		/* route found.. mark it! */
	      //if ( pluc_lut_list[i].user_out_name != NULL )
	      //  free(pluc_lut_list[i].user_out_name);
	      //pluc_lut_list[i].user_out_name = strdup(s);
	      //if ( pluc_lut_list[i].user_out_name == NULL )
	      //  return pluc_err("memory error"), 0;
	      //pluc_log("Output '%s' connected to LUT%d", pinfoGetOutLabel(&(pluc_lut_list[i].pi), 0), i);
	      pluc_lut_list[i].is_placed = 1;			/* mark the LUT as done */
	      pluc_log("Route: LUT%d marked as placed", i);
	    }
	    else
	    {
	      pluc_err("Route: No route found from LUT%d to %s", i, pinfoGetOutLabel(&(pluc_lut_list[i].pi), 0));
	    }
	  } // LUT output is identical to LUT name
	  else
	  {
	    /* routing will be done via the input, but still the placement flag has to be set */
	    pluc_lut_list[i].is_placed = 1;			/* mark the LUT as done */
	      pluc_log("Route: Sub-LUT%d marked as placed", i);
	    
	    //pluc_log("identical names for lut and output %s", pluc_get_lut_output_name(i));
	  }
	} // external connection
	else
	{
	  //pluc_log("internal LUT with output %s", s);
	}
      } // s != NULL
    } // is_placed == 0
  }
  return 1;
}

/*
  works, but...:
  TODO:
  This procedure must be more clever. 
  
  PIO0_12    -> PLUINPUT4 --> LUT5
  But later, ther should be another connection
  PIO0_12    -> PLUINPUT4 --> LUT6
  which will fail, because PIO0_12    -> PLUINPUT4 is already used.
  However, this is wrong, because the source has not changed and PIO0_12    -> PLUINPUT4 can be reused.

  Eigentlich müsste es doch so sein: Wenn es mehrere übergänge zu dem selben Ziel gibt, müssten diese unkenntlich gemacht werden
  also alles andere ausser 
    PIO0_12    -> PLUINPUT4
  das ebenfalls nach PLUINPUT4 geht, müsste deaktiviert werden (darf nicht mehr verwendet werden)

  ist das noch relevant???
  no, this seems to work:
  ./pluc 'PIO0_9 <= PIO0_2; PIO0_15 <= PIO0_2;'
  in this case PLUINPUT2 is reused for both LUTs
*/

int pluc_route_lut_input(void)
{
  int i, j;
  char in[32];
  pinfo *pi;

  pluc_log("Route: LUT input route");
  
  for( i = 0; i < pluc_lut_cnt; i++ )
  {
    pi = &(pluc_lut_list[i].pi);
    pluc_log("Route: LUT%d with input(s) %s", i, b_sl_ToStr(pinfoGetInLabelList(pi), ", "));
    for( j = 0; j < pinfoGetInCnt(pi); j++ )
    {
	
      //sprintf(in, "%s_INP%d", pluc_get_lut_output_name(i), pinfoGetInCnt(pi)-j-1);	/* the signal at pos 0 in pinfo becomes the highest input in the LUT */
      sprintf(in, "%s_INP%d", pluc_get_lut_output_name(i), j);	
      if ( pluc_calc_from_to(pinfoGetInLabel(pi, j), in ) != 0 )
      {
	  pluc_log("Route: LUT input path found from %s to %s ", pinfoGetInLabel(pi, j), in);
	  pluc_mark_route_chain_wire_list();		/* route found.. mark it! */
      }
      else
      {
	pluc_err("Route: No LUT input path found from %s to %s", pinfoGetInLabel(pi, j), in);
	return 0;
      }
    }
  }
  return 1;
}


int pluc_route(void)
{
  /* rename the internal signal name to normal LUT names */
  /* this is actually a result of the previous step, but is done here instead */
  if ( pluc_replace_internal_lut_signals() == 0 )
    return 0;
  /* do the same with the ff luts */
  if ( pluc_replace_ff_signals() == 0 )
    return 0;


  /* connect LUT output signal, ignore LUTs where the output signal is identical with the LUT name */
  if ( pluc_route_external_connected_luts() == 0 )
    return 0;

  if ( pluc_route_lut_input() == 0 )
    return 0;
  
  /*
  for( int i = 0; i < pluc_lut_cnt; i++ )
  {
    printf("pluc_lut_list entry %d:\n", i);
    dclShow(&(pluc_lut_list[i].pi), pluc_lut_list[i].dcl);    
  }
  */
  
  
  return 1;
}

/*==================================================*/
/* code generation */
/*

  Return the configuration value for the LUT.
  All unused colums had been previously removed (see pluc_remove_dc), so that the LUT has five or lesser inputs.


*/

void pluc_out(const char *s)
{
  if ( c_fp == NULL )
    printf("%s", s);
  else
    fprintf(c_fp, "%s", s);
}

void pluc_codegen_pre(void)
{
  pluc_out("/* LPC804 PLU setup file, generated by 'pluc' */\n");
  pluc_out("#include <stdint.h>\n");
  pluc_out("\n");
  pluc_out("void ");
  pluc_out(cmdline_procname);
  pluc_out("(void)\n");
  pluc_out("{\n");
  //pluc_out("\tuint32_t clkctrl0;\n");
  //pluc_out("\tuint32_t clkctrl1;\n");
  //pluc_out("\tclkctrl0 = *(uint32_t *)0x40048080UL;   /* backup SYSAHBCLKCTRL0 */\n");
  //pluc_out("\tclkctrl1 = *(uint32_t *)0x40048084UL;   /* backup SYSAHBCLKCTRL1 */\n");
}

void pluc_codegen_post(void)
{
  //pluc_out("\t*(uint32_t *)0x40048080UL = clkctrl0;   /* restore SYSAHBCLKCTRL0 */\n");
  //pluc_out("\t*(uint32_t *)0x40048084UL = clkctrl1;   /* restore SYSAHBCLKCTRL1 */\n");
  //pluc_out("\t/* Note: Ensure to enable the PLU clock for read operation on PLU output registers */\n");
  pluc_out("}\n");
}


void pluc_codegen_init(void)
{
  
  //pluc_out("\t*(uint32_t *)0x40048084UL &= ~(1UL<<5);   /* Disable PLU Clock */\n");
  pluc_out("\t*(uint32_t *)0x40048084UL |= (1UL<<5);   /* Enable PLU Clock */\n");
  pluc_out("\t*(uint32_t *)0x4004808CUL &= ~(1UL<<5);   /* Reset PLU */\n");
  pluc_out("\t*(uint32_t *)0x4004808CUL |= (1UL<<5);   /* Clear Reset PLU */\n");
  
  pluc_out("\t*(uint32_t *)0x40048080UL |= (1UL<<6);   /* Enable GPIO0 Clock */\n");
  if ( cmdline_xrst )
  {
    pluc_out("\t*(uint32_t *)0x40048088UL &= ~(1UL<<6);   /* Reset GPIO0 */\n");
    pluc_out("\t*(uint32_t *)0x40048088UL |= (1UL<<6);   /* Clear Reset GPIO0 */\n");
  }
  pluc_out("\t*(uint32_t *)0x40048080UL |= (1UL<<7);   /* Enable SWM Clock */\n");
  if ( cmdline_xrst )
  {
    pluc_out("\t*(uint32_t *)0x40048088UL &= ~(1UL<<7);   /* Reset SWM */\n");
    pluc_out("\t*(uint32_t *)0x40048088UL |= (1UL<<7);   /* Clear Reset SWM */\n");
  }
  pluc_out("\t*(uint32_t *)0x40048080UL |= (1UL<<18);   /* Enable IOCON Clock */\n");
  if ( cmdline_xrst )
  {
    pluc_out("\t*(uint32_t *)0x40048088UL &= ~(1UL<<18);   /* Reset IOCON */\n");
    pluc_out("\t*(uint32_t *)0x40048088UL |= (1UL<<18);   /* Clear Reset IOCON */\n");
  }
  
  
  
  
}

uint32_t lpc804_pin_to_iocon_offset[] = {
  0x44,	// 0
  0x2C,	// 1
  0x18,	// 2
  0x14,	// 3
  0x10,	// 4
  0x0C,	// 5
  0x80,	// (6)
  0x3C,	// 7
  0x38,	// 8
  0x34,	// 9
  0x20,	// 10
  0x1C,	// 11
  0x08,	// 12
  0x04,	// 13
  0x48,	// 14
  0x28,	// 15
  0x24,	// 16
  0x00,	// 17
  0x74,	// 18
  0x60,	// 19
  0x58,	// 20
  0x30,	// 21
  0x70,	// 22
  0x6C,	// 23
  0x68,	// 24
  0x64,	// 25
  0x54,	// 26
  0x50,	// 27
  0x4C,	// 28
  0x40,	// 29
  0x5C	// 30
};

void pluc_codegen_gpio_output(int pin)
{
  char s[512];
  sprintf(s, "\t*(uint32_t *)0xA0002380UL |= (1UL<<%d);    /* PIO0_%d GPIO DIRSETP: Setup as output */\n", pin, pin);
  pluc_out(s);
  sprintf(s, "\t*(uint32_t *)0x%08xUL &= ~(3UL<<3);    /* PIO0_%d IOCON: Clear mode, deactivate any pull-up/pulldown */\n", 0x040044000+lpc804_pin_to_iocon_offset[pin], pin);
  pluc_out(s);
}

void pluc_codegen_gpio_input(int pin)
{  
  char s[512];
  sprintf(s, "\t*(uint32_t *)0x%08xUL &= ~(3UL<<3);    /* PIO0_%d IOCON: Clear mode */\n", 0x040044000+lpc804_pin_to_iocon_offset[pin], pin);
  pluc_out(s);
  sprintf(s, "\t*(uint32_t *)0x%08xUL |= (2UL<<3);    /* PIO0_%d IOCON: Enable pull-up */\n", 0x040044000+lpc804_pin_to_iocon_offset[pin], pin);
  pluc_out(s);  
  //sprintf(s, "\t*(uint32_t *)0x%08xUL |= (1UL<<5);    /* PIO0_%d IOCON: Enable Hysteresis */\n", 0x040044000+lpc804_pin_to_iocon_offset[pin], pin);
  //pluc_out(s);  
  sprintf(s, "\t*(uint32_t *)0xA0002400UL |= (1UL<<%d);    /* PIO0_%d GPIO DIRCLRP: Setup as input */\n", pin, pin);  
  pluc_out(s);
}

void pluc_codegen_gpio(void)
{
  int i;
  int pin;
  int is_input;
  int is_output;
  char s[16];
  for( pin = 0; pin <= 30; pin++ )
  {
    sprintf(s, "PIO0_%d", pin);
    is_input = 0;
    is_output = 0;
    i = 0;
    while( lpc804_wire_table[i].from != NULL )
    {
      if ( lpc804_wire_table[i].is_used )
      {
	if ( strcmp(lpc804_wire_table[i].from, s) == 0 )
	{
	  is_input = 1;
	}
	if ( strcmp(lpc804_wire_table[i].to, s) == 0 )
	{
	  is_output = 1;
	}
      }
      i++;
    }  

    /* if a GPIO pin is input for another internal function, then it will be forced to be an input pin */
    /* in other words: If a GPIO is input or both (input and output), there is no choice, it must be */
    /* configired as input: https://github.com/olikraus/lpc804_plu/tree/master/lpc804/plu_sw_input */

    if ( is_input )	// also for both in&output
      pluc_codegen_gpio_input(pin);
    else if ( is_output )
      pluc_codegen_gpio_output(pin);
  }
    
}

void pluc_out_regop(pluc_regop_t *regop)
{
  static char s[1024];
  const char *base = "";
  uint32_t adr = 0L;
  switch(regop->base)
  {
    case 0:	adr = 0x40028000;	base = "PLU"; break;
    case 1:	adr = 0x4000c000;	base = "SWM"; break;
    default: assert(0); break;
  }
  adr += regop->idx;
  
  assert((regop->idx & 3) == 0);
  
  switch(regop->op)
  {
    case 0:
      s[0] = '\0';
      break;
    case 1:
      sprintf(s, "*(uint32_t *)0x%08x = 0x%08xUL; /*%s*/", adr, regop->or_value, base);
      break;
    case 2:
      sprintf(s, "*(uint32_t *)0x%08x &= 0x%08xUL; /*%s*/", adr, regop->and_value, base);
      break;
    case 3:
      sprintf(s, "*(uint32_t *)0x%08x &= ~0x%08xUL; /*%s*/", adr, regop->and_value, base);
      break;
    case 4:
      sprintf(s, "*(uint32_t *)0x%08x |= 0x%08xUL; /*%s*/", adr, regop->or_value, base);
      break;
    case 5:
      sprintf(s, "*(uint32_t *)0x%08x |= ~0x%08xUL; /*%s*/", adr, regop->or_value, base);
      break;
    case 6:
      sprintf(s, "*(uint32_t *)0x%08x &= 0x%08xUL; *(uint32_t *)0x%08x |= 0x%08xUL; /*%s*/", adr, regop->and_value, adr, regop->or_value, base);
      break;
    case 7:
      sprintf(s, "*(uint32_t *)0x%08x &= ~0x%08xUL; *(uint32_t *)0x%08x |= 0x%08xUL; /*%s*/", adr, regop->and_value, adr, regop->or_value, base);
      break;
  }

  if ( s[0] != '\0' )
  {
    pluc_out("\t");
    pluc_out(s);
    pluc_out("\n");
  }
}

void pluc_out_wire(pluc_wire_t *wire)
{
  static char s[1024];
  int i;
  sprintf(s, "\t/* %s --> %s */\n", wire->from, wire->to);
  pluc_out(s);
  for( i = 0; i < PLUC_WIRE_REGOP_CNT; i++ )
  {
    pluc_out_regop(wire->regop+i);
  }
}

void pluc_codegen_wire(void)
{
  int i = 0;
  while( lpc804_wire_table[i].from != NULL )
  {
    if ( lpc804_wire_table[i].is_used )
    {
      pluc_out_wire(lpc804_wire_table+i);
    }
    i++;
  }  
}


/*
  0x40028000+0x800+lut*4
*/
uint32_t pluc_get_lut_config_value(int lut)
{
  dcube *input;
  uint32_t result;
  int in_cnt;
  int i;
  int bit_cnt;
  
  //dclShow(&(pluc_lut_list[lut].pi), pluc_lut_list[lut].dcl);
  
  in_cnt = pinfoGetInCnt(&(pluc_lut_list[lut].pi));
  assert( in_cnt > 0 );
  assert( in_cnt <= 5 );
  
  bit_cnt = 1<<in_cnt;
  
  input = &(pluc_lut_list[lut].pi.tmp[10]);
  
  /* prepare the counter, all DC except for the inputs (which are set to 0) */
  dcInSetAll(&(pluc_lut_list[lut].pi), input, CUBE_IN_MASK_DC);
  for( i = 0; i < in_cnt; i++ )
    dcSetIn(input, i, 1);
  
  /* calculate the truth table for each input combination */
  result = 0;
  for( i = 0; i < bit_cnt; i++ )
  {
    dclResult(&(pluc_lut_list[lut].pi), input, pluc_lut_list[lut].dcl);
    
    if ( dcGetOut(input, 0) != 0 )
      result |= (1<<i);
    
    //pluc_log("%02d: %s", i, dcToStr(&(pluc_lut_list[lut].pi), input, " ", ""));
    
    /*
    result >>= 1;
    if ( dcGetOut(input, 0) != 0 )
      result |= 1<<(in_cnt-1);
    */
    
    dcInc(&(pluc_lut_list[lut].pi), input);
  }
  //pluc_log("LUT%d config value %08x, incnt=%d, bitcnt=%d", lut, result, bit_cnt, in_cnt);
  return result;
}

void pluc_out_lut_config_value(int lut)
{
  static char s[1024];
  uint32_t config_value;
    
  sprintf(s, "\t/* LUT %d Config Value */\n", lut);
  pluc_out(s);
  
  
  {
    int i, cnt = dclCnt(pluc_lut_list[lut].dcl);
    
    sprintf(s, "\t/* %s|", b_sl_ToStr(pluc_lut_list[lut].pi.in_sl, " "));
    sprintf(s+strlen(s), "%s */\n", b_sl_ToStr(pluc_lut_list[lut].pi.out_sl, " "));
    pluc_out(s);
    
    for( i = 0; i < cnt; i++ )
    {
      sprintf(s, "\t/* %s */\n", dcToStr(&(pluc_lut_list[lut].pi), dclGet(pluc_lut_list[lut].dcl, i), "|", ""));
      pluc_out(s);
    }
  }
  
  
  config_value = pluc_get_lut_config_value(lut);
  
  sprintf(s, "\t*(uint32_t *)0x%08x = 0x%08xUL; /*PLU*/\n", 0x40028000+0x800+lut*4, config_value);
  pluc_out(s);
  
}

void pluc_codegen_lut(void)
{
  int i;
  for( i = 0; i < PLUC_LUT_MAX; i++ )
  {
    if ( pluc_lut_list[i].is_placed )
    {
      pluc_out_lut_config_value(i);
    }
  }
}

int pluc_codegen_is_clock_ok(void)
{
  int i;
  if ( pluc_is_FF_used )
  {
    i = 0;
    while( lpc804_wire_table[i].from != NULL )
    {
      if ( lpc804_wire_table[i].is_used )
      {
	if ( strcmp(lpc804_wire_table[i].to, "PLU_CLKIN") == 0 )
	{
	  break;
	}
      }
      i++;
    }
    if ( lpc804_wire_table[i].from == NULL )
    {
      /* FF is used, but PLU_CLKIN is not wired */
      return 0;
    }
  }
  return 1;
}

void pluc_codegen_clock(void)
{
  char s[1024];
    
  int i;
  
  
  /* check whether CLKOUT is used and setup SYSCON CLKOUT register */
  i = 0;
  while( lpc804_wire_table[i].from != NULL )
  {
    if ( lpc804_wire_table[i].is_used )
    {
      if ( strcmp(lpc804_wire_table[i].from, "CLKOUT") == 0 )
      {
	break;
      }
    }
    i++;
  }
  if ( lpc804_wire_table[i].from != NULL )
  {
    
    if ( cmdline_lposc == 0 )
    {
      sprintf(s, "\t/* CLKOUT clock source select register: Select main clock */\n");
      pluc_out(s);
      sprintf(s, "\t*(uint32_t *)0x400480F0UL = 1;  /*SYSCON CLKOUTSEL*/\n");
      pluc_out(s);
    }
    else
    {
      sprintf(s, "\t/* Enable LP-OSC and select LP OSC for CLKOUT */\n");
      pluc_out(s);
      sprintf(s, "\t*(uint32_t *)0x40048238UL &= ~(1<<6); /* Clear bit 6 in SYSCON PDRUNCFG to enable LP-OSC */\n");
      pluc_out(s);
      sprintf(s, "\t*(uint32_t *)0x400480F0UL = 4;  /* SYSCON CLKOUTSEL: Select LP-OSC */\n");
      pluc_out(s);
    }
    
    sprintf(s, "\t/* CLKOUT clock divider register: Divide by %ld */\n", cmdline_clkdiv);
    pluc_out(s);
    sprintf(s, "\t*(uint32_t *)0x400480F4UL = %ld;  /*SYSCON CLKOUTDIV*/\n", cmdline_clkdiv);
    pluc_out(s);
  }
  
}


int pluc_codegen(void)
{
  if ( pluc_codegen_is_clock_ok() == 0 )
  {
    pluc_err("Error: PLU_CLKIN is not connected.");
    return 0;
  }

  
  
  if ( c_file_name[0] != '\0' )
  {
    c_fp = fopen(c_file_name, "w");
  }
  
  pluc_codegen_pre();
  pluc_codegen_init();
  pluc_codegen_gpio();
  pluc_codegen_lut();
  pluc_codegen_wire();
  pluc_codegen_clock();
  pluc_codegen_post();
  
  if ( c_fp != NULL )
  {
    fclose(c_fp);
  }
  return 1;
}

/*==================================================*/


int pluc_read_and_merge(void)	// obsolete
{
  int i;
  pluc_log("Read (files: %d)", cl_file_cnt);
  //assert(0);
  
  for( i = 0; i < cl_file_cnt; i++ )
  {
    pinfoDestroy(&pi);
    if ( pinfoInit(&pi) == 0 )
      return 0;
    dclClear(cl_on);
    
    if ( pluc_read_file(cl_file_list[i]) == 0 )
      return 0;
    if ( pluc_map() == 0 )		// assumes problen in pl, cl_on
      return 0;
  }
  return 1;
}



int pluc(int is_merge)
{
  
  if ( pluc_init() == 0 )
    return 0;
  
  if ( is_merge != 0 )
  {
    if ( pluc_read() == 0 )		// read into in pl, cl_on
      return 0;
    if ( pluc_map() == 0 )		// assumes problen in pl, cl_on
      return 0;
  }
  else
  {
    if ( pluc_read_and_merge() == 0 )   // obsolete
      return 0;
  }
  if ( pluc_route() == 0 )
    return 0;
  
  pluc_log("Synthesis done.");
  
  if ( pluc_codegen() == 0 )
    return 0;

  //dclShow(&pi, cl_on);
  //dclShow(&pi2, cl2_on);
  return 1;
}




/*==================================================*/
/* commandline handling & help */

cl_entry_struct cl_list[] =
{
  { CL_TYP_GROUP, "Code Generation", NULL, 0},
  { CL_TYP_STRING,  "oc-write C code", c_file_name, 1024 },
  { CL_TYP_STRING,  "fn-name of the generated c function", cmdline_procname, 1024 },
  { CL_TYP_ON,      "mis-activate state minization (may not work with some KISS files)", &cmdline_mis,  0 },
  { CL_TYP_LONG,  "clkdiv-flip flop clock division (CLKOUTDIV, 1..255)", &cmdline_clkdiv, 0 },
  { CL_TYP_ON,      "xrst-generate reset code for SWM, IOCON and GPIO", &cmdline_xrst,  0 },
  { CL_TYP_ON,      "lposc-use low power osc instead of main_clk as clock for any flip-flops", &cmdline_lposc,  0 },
  { CL_TYP_GROUP, "Information", NULL, 0},
  { CL_TYP_ON,      "listkeywords-list allowed signal names", &cmdline_listkeywords,  0 },
  { CL_TYP_GROUP, "PLUC Internals", NULL, 0},
  { CL_TYP_ON,      "dbg-output additional debug information", &cmdline_dbg,  0 },
  { CL_TYP_ON,      "listmap-list wire mapping", &cmdline_listmap,  0 },
  { CL_TYP_STRING,  "testoutroute-Find a route from given output to a LUT", cmdline_output, 1024 },
  { CL_TYP_STRING,  "testinroute-Find a route from given input to a LUT", cmdline_input, 1024 },
  CL_ENTRY_LAST
};

void help(const char *pn)
{
    printf("Usage: %s [options] <input files> \n", pn);
    puts("options:");
    cl_OutHelp(cl_list, stdout, " ", 20);
}


/*==================================================*/
/* main procedure */

int main(int argc, char **argv)
{
  if ( cl_Do(cl_list, argc, argv) == 0 )
  {
    puts("cmdline pluc_error");
    exit(1);
  }
  
  if ( cmdline_listmap != 0 )
  {
    int i = 0;
    while( lpc804_wire_table[i].from != NULL )
    {
      printf("%05d: %c/%c %-10s -> %-10s\n", i, lpc804_wire_table[i].is_lut_in?'i':'-', lpc804_wire_table[i].is_lut_out?'o':'-', lpc804_wire_table[i].from, lpc804_wire_table[i].to );
      i++;
    }
    exit(0);
  }
  
  if ( cmdline_listkeywords != 0 )
  {
    pluc_init();
    printf("Allowed keywords:\n");
    pluc_show_keywords();
    exit(0);
  }

  
  if ( cmdline_output[0] != '\0' )
  {
    pluc_calc_route_chain(cmdline_output, 0);
    exit(1);
  }

  if ( cmdline_input[0] != '\0' )
  {
    pluc_calc_route_chain(cmdline_input, 1);
    exit(1);
  }
  
  
  
  if ( cl_file_cnt < 1 )
  {
    help(argv[0]);
  }
  else
  {
    pluc(1);
  }
}

