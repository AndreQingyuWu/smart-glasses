/******************************************************************************

  Copyright (C), 2001-2011, DCN Co., Ltd.

 ******************************************************************************
  File Name     : glass_flash.h
  Version       : Initial Draft
  Author        : x
  Created       : 2019/12/19
  Last Modified :
  Description   : GLASS  FLASH    manager
  Function List :
  History       :
  1.Date        : 2019/12/19
    Author      : x
    Modification: Created file

******************************************************************************/
#ifndef   GLASS_FLASH_H
#define   GLASS_FLASH_H

#include <stdint.h>
#include "app_error.h"
#include "glass_clock.h"
#include "pstorage_platform.h"

#define   FLASH_POOL_NUM          (1)
#define   DAY_OF_BLOCK            (3)
#define   FLASH_BUFFER_SIZE       (20)
#define   FLASH_RECORD_SIZE       (512)


typedef  struct   _st_flash_buffer
{
   time_union_t     data_mark;
   unsigned  char   data_buffer[16];

}flash_buffer;



typedef  __packed struct   _st_flash_record
{
	time_union_t       data_mark;   
    unsigned  int      data_w_offset;
    unsigned  int      data_r_offset;
    unsigned  int      data_left;  
    unsigned  char     day_calculate[48];  
    
}flashrecord;

typedef  struct  _st_block_pool 
{   

    int                 id;
    int                 cur_block;
    flashrecord         flash_record;
    pstorage_handle_t   flash_record_t;
    pstorage_handle_t   flash_handle_t[DAY_OF_BLOCK];

}st_block_pool;



typedef  struct  _st_flash_manager
{
   
   pstorage_handle_t    handle_record;
   st_block_pool        flash_pool[FLASH_POOL_NUM];
   
}st_flash_manager;


#define RECORD_VALID(data)              \
do                                      \
{                                       \
	const uint32_t LOCAL_DATA = (data); \
    if (LOCAL_DATA ==0XFFFFFFFF){       \
           data= 0X00000000;            \
    }                                   \
} while (0)


#define   system_debug(X)\
do{                      \
	const uint32_t LOCAL_ERR_CODE = (X);       \
    if(LOCAL_ERR_CODE!=NRF_SUCCESS)            \
    {                                          \
        while(1){                              \
			printf("system run  error at  file  [%s]  line  [%d] err[%d]\r\n", (uint8_t*) __FILE__,__LINE__,X);\
			nrf_delay_ms(1000);                                                                         \
		}                                                                                               \
    }                                                                                                   \
}while(0)                                                                                               \


#endif
