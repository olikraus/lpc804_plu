/*

  flash.c

  LPC804 Project 

  Copyright (c) 2020, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification, 
  are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice, this list 
    of conditions and the following disclaimer.
    
  * Redistributions in binary form must reproduce the above copyright notice, this 
    list of conditions and the following disclaimer in the documentation and/or other 
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.  
  

  Code based on AN12125_LPC802
  https://www.nxp.com/docs/en/application-note/AN12125_LPC802.zip

  Usefull location for "dest_flash_adr" could be 0x7f00
  Note: 0x7f80 / 0x7fc0 should not be used.

*/


#include "flash.h"

#define FLASH_PAGE_SIZE       (64)
#define FLASH_SECTOR_SIZE         (1024)

typedef void (*iap_fntype) (uint32_t *, uint32_t *);
#define call_iap(c,s) ((iap_fntype) 0x0F001FF1)(c, s)



/*
  Description:
    Clear a page in flash ROM. According to the datasheet, the last
    two pages of the flash area can not be erased.
    
  Arguments:
    dest_flash_adr:		Destination address of the page, 64 byte boundary, 0x0000-0x7F7F
    
  Return:
    0, in case of any error

*/
int flash_erase_page(uint32_t dest_flash_adr)
{
  register uint32_t sector;
  register uint32_t page;
  
  uint32_t command_param[5];
  uint32_t status_result[5];



  /* prepare sectior */

  sector = dest_flash_adr / FLASH_SECTOR_SIZE;

  command_param[0] = 50;					// prepare sector cmd
  command_param[1] = sector;
  command_param[2] = sector;
  __disable_irq();
  call_iap(command_param, status_result);
  __enable_irq();
  if ( status_result[0] != 0 )
      return 0;                    // error


  /* erase page */

  page = dest_flash_adr/FLASH_PAGE_SIZE;

  command_param[0] = 59; 					// page erase
  command_param[1] = page;
  command_param[2] = page;
  command_param[3] = 0;
  __disable_irq();
  call_iap(command_param, status_result);
  __enable_irq();
  if ( status_result[0] != 0 )
      return 0;                    // error

  return 1;	// ok
}




/*
  Description:
    Write the content of one page (64 bytes) to target location

  Arguments:
    dest_flash_adr:		Destination address, 64 byte boundary, 0x0000-0x7F7F
    src_ram_adr:		Source location of the page, word boundary

  Return:
    0, in case of any error

*/
int flash_write_page(uint32_t dest_flash_adr, const uint8_t *src_ram_adr)
{
  register uint32_t sector;
  
  uint32_t command_param[5];
  uint32_t status_result[5];



  /* prepare sectior */

  sector = dest_flash_adr / FLASH_SECTOR_SIZE; 

  command_param[0] = 50; 					// prepare sector cmd
  command_param[1] = sector;
  command_param[2] = sector;
  __disable_irq();
  call_iap(command_param, status_result);
  __enable_irq();
  if ( status_result[0] != 0 )
      return 0;                    					// error


  /* write page */

  command_param[0] = 51;					// copy RAM to flash cmd
  command_param[1] = dest_flash_adr;
  command_param[2] = (uint32_t)src_ram_adr;
  command_param[3] = FLASH_PAGE_SIZE;
  command_param[4] = 0;
  __disable_irq();
  call_iap(command_param, status_result);
  __enable_irq();
  if ( status_result[0] != 0 )
    return 0;								// error

  return 1;	// ok
}

/*

  Description:
    Write the content of one page (64 bytes) to target location
    Executes erase and write operation.
    On a LPC804 with 15MHz, this call requires about 3ms
    The LPC804 probably required 3ms (most subsystems active)
    --> cap time constant = 3.3V/3ms * 5uF = 5ms

  Arguments:
    dest_flash_adr:		Destination address, 64 byte boundary, 0x0000-0x7F7F
    src_ram_adr:		Source location of the page, word boundary

  Return:
    0, in case of any error

*/
int flash_page(uint32_t dest_flash_adr, const uint8_t *src_ram_adr)
{
  if ( flash_erase_page(dest_flash_adr) == 0 )
    return 0;
  return flash_write_page(dest_flash_adr, src_ram_adr);
}
