/******************************************************************************

  Copyright (C), 2001-2011, DCN Co., Ltd.

 ******************************************************************************
  File Name     : glass_flash.c
  Version       : Initial Draft
  Author        : x
  Created       : 2019/12/19
  Last Modified :
  Description   : flash manager.c
  Function List :
  History       :
  1.Date        : 2019/12/19
    Author      : x
    Modification: Created file

******************************************************************************/
#include <stdio.h>
#include "pstorage.h"
#include "nrf_error.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "glass_flash.h"
#include "glass_clock.h"

extern    UTCTime               RTCSTATE;
static    pstorage_handle_t     ghandle;
static    unsigned int          opcode;
static    volatile int          sync;
static    st_flash_manager      flash_manager;


/*****************************************************************************
 Prototype    : glass_flash_sync
 Description  : 设置等待标志
 Input        : pstorage_handle_t  handle  
                uint8_t op_code            
 Output       : None
 Return Value : static
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/12/20
    Author       : x
    Modification : Created function

*****************************************************************************/
static  void  glass_flash_sync(pstorage_handle_t  handle,uint8_t op_code)
{
   opcode            = op_code;
   ghandle.block_id  = handle.block_id;
   ghandle.module_id = handle.module_id;
}
/*****************************************************************************
 Prototype    : glass_wait_sync
 Description  : 等待sysnc信号
 Input        : None
 Output       : None
 Return Value : static
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/12/20
    Author       : x
    Modification : Created function

*****************************************************************************/
static  int   glass_wait_sync()
{
   int  i = 0;
   int  m = 1000;
   
   while(m--)
   {
      if(!sync)
      {
          nrf_delay_ms(10);
      }else{
           break;
      }
   }   

   sync  =  0;
   
   return  m > 0 ? NRF_SUCCESS : NRF_ERROR_INVALID_STATE;
   
}
/*****************************************************************************
 Prototype    : glass_flash_callback
 Description  : flash 操作回调函数
 Input        : pstorage_handle_t * handle  
                uint8_t op_code             
                uint32_t result            
                uint8_t * p_data            
                uint32_t data_len           
 Output       : None
 Return Value : static
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/12/19
    Author       : x
    Modification : Created function

*****************************************************************************/
static void   glass_flash_callback(pstorage_handle_t * handle,uint8_t op_code, uint32_t result,uint8_t * p_data, uint32_t data_len)
{  
	if(opcode == op_code&&result == NRF_SUCCESS &&handle->block_id == ghandle.block_id&&handle->module_id == ghandle.module_id)
    {
         sync = 1;
    }else{
	     printf("opcode  error  %d,result  %d\r\n",op_code,result);
	}
}
/*****************************************************************************
 Prototype    : flash_pool_earse
 Description  : flash pool  擦除
 Input        : st_block_pool *ppool  
 Output       : None
 Return Value : static
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/12/25
    Author       : x
    Modification : Created function

*****************************************************************************/
static  int  flash_pool_earse(st_block_pool *ppool)
{

   int  err_code = 0;
   pstorage_handle_t   dest_block_id;

   printf("ppool->id  %d\r\n",ppool->id);
   
   err_code =  pstorage_block_identifier_get(&(flash_manager.handle_record),ppool->id,&dest_block_id);	
   system_debug(err_code);

   glass_flash_sync(dest_block_id,PSTORAGE_CLEAR_OP_CODE);
   err_code = pstorage_clear(&dest_block_id,512);
   system_debug(err_code);

   err_code = glass_wait_sync();
   system_debug(err_code);

   for(int   i = 0; i  <  DAY_OF_BLOCK; i++){
    
         err_code =  pstorage_block_identifier_get(&(ppool->flash_handle_t[i]),ppool->id,&dest_block_id);	
         system_debug(err_code);

        
         glass_flash_sync(dest_block_id,PSTORAGE_CLEAR_OP_CODE);
         err_code = pstorage_clear(&dest_block_id,PSTORAGE_MAX_BLOCK_SIZE);
         system_debug(err_code);

         err_code = glass_wait_sync();
         system_debug(err_code);
   }

   return   err_code;
   
}
/*****************************************************************************
 Prototype    : glass_flash_search
 Description  : 获得当前写入的数据池
 Input        : time_union_t  data  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/12/19
    Author       : x
    Modification : Created function

*****************************************************************************/
static  int  flash_pool_search(time_union_t  data)
{

    int        id     = 0;
    int        poolid = 0;
    uint32_t   err_code;

    time_union_t   time;
    data  =  flash_manager.flash_pool[0].flash_record.data_mark;

    /*find  the same  and pool and  the  pool is not  full*/
    for(int id = 0;  id  < FLASH_POOL_NUM ;id++)
    {
       if(flash_manager.flash_pool[id].flash_record.data_mark.data == data.data) 
       {
          if(flash_manager.flash_pool[id].flash_record.data_w_offset <  \
                                           DAY_OF_BLOCK*PSTORAGE_MAX_BLOCK_SIZE)
          {
              poolid  =    id;
              return   poolid;
          }
       }
    }

    /*find  pool  is  not  already  use  and  empty*/
    for(id = 0;  id  < FLASH_POOL_NUM;id++)
    {
       if(flash_manager.flash_pool[id].flash_record.data_mark.data == 0x00000000) 
       {
             poolid  =    id;
             return   poolid;
       }
    }

    /*find  the odlest  pool and  we will erase  the pool  later*/
    for(id =0 ;id < FLASH_POOL_NUM;id++)
    {
       if(time.data <  flash_manager.flash_pool[id].flash_record.data_mark.data)
       {
           poolid   =  id;
           time.data=  flash_manager.flash_pool[id].flash_record.data_mark.data;
           
       }
    }

    
    /*if program run hera ,flash  write  full  and  we  can  find  a  older earse*/
    err_code =  flash_pool_earse(&(flash_manager.flash_pool[poolid]));
    system_debug(err_code);

    /*earse  and  reset  pool w/r  offset*/
    _flash_updata_record(&flash_manager.flash_pool[poolid],poolid);
    
    return     poolid;
    
}

/*****************************************************************************
 Prototype    : flash_pool_write
 Description  : 向特定内存池中写
 Input        : int  id               
                flash_buffer  buffer  
 Output       : None
 Return Value : static
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/12/19
    Author       : x
    Modification : Created function

*****************************************************************************/
static  int   flash_pool_write(st_block_pool *ppool,flash_buffer  buffer)
{

     int  blockid  = 0;
     int  offset   = 0;
     int  err_code = 0;
     pstorage_handle_t   dest_block_id;

     if(!ppool)
        return   NRF_ERROR_INVALID_PARAM;
     
     offset =  ppool->flash_record.data_w_offset;
     blockid = offset/PSTORAGE_MAX_BLOCK_SIZE;
     offset  = offset-(PSTORAGE_MAX_BLOCK_SIZE*blockid);

     printf("we  will  write  pool  [%d] offset  [%d]\r\n",ppool->id,ppool->flash_record.data_w_offset);
     err_code =  pstorage_block_identifier_get(&(ppool->flash_handle_t[blockid]),  0,   &dest_block_id);	
     system_debug(err_code);
    
     
     glass_flash_sync(dest_block_id,PSTORAGE_UPDATE_OP_CODE);
        
     err_code = pstorage_update(&dest_block_id,buffer.data_buffer,16,offset); 
     system_debug(err_code);
     
     err_code  = glass_wait_sync();
     system_debug(err_code);
     
     return   err_code;
     
}
/*****************************************************************************
 Prototype    : flah_write_buffer
 Description  : 写flash 操作接口
 Input        : flash_buffer buffer  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/12/19
    Author       : x
    Modification : Created function

*****************************************************************************/
int   glass_flash_write(char *buf,int  len)
{ 

    int      poolid = 0;
    uint32_t   err_code;
    flash_buffer     buffer;
    UTCTimeStruct*   pdata = NULL;
    
    if(len != 16)
        return   NRF_ERROR_INVALID_LENGTH;

    if(!RTCSTATE)
        return   NRF_ERROR_FORBIDDEN;
    
    pdata = get_wall_clock_time();
    memcpy(buffer.data_buffer,buf,len);   
    buffer.data_mark.time.year   = pdata->year;
    buffer.data_mark.time.month  = pdata->month;
    buffer.data_mark.time.day    = pdata->day;
    buffer.data_mark.time.hours  = pdata->hour;
    buffer.data_mark.time.minute = pdata->minutes;
    buffer.data_mark.time.seconds= pdata->seconds;

#if  1
    poolid   =  flash_pool_search(buffer.data_mark);

    err_code =  flash_pool_write(&(flash_manager.flash_pool[poolid]),buffer);
    system_debug(err_code);
#endif
    
    
    flash_manager.flash_pool[poolid].flash_record.data_w_offset += 16;
    flash_manager.flash_pool[poolid].flash_record.data_mark.data = buffer.data_mark.data;
    err_code = flash_sync_record(&(flash_manager.flash_pool[poolid]),buffer);
    system_debug(err_code);
    
#if  1
    printf("flash  write  ok\r\n");
#endif

    return  err_code;
    
}
/*****************************************************************************
 Prototype    : flash_sync_read
 Description  : sync  flash
 Input        : None
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/12/25
    Author       : x
    Modification : Created function

*****************************************************************************/
int  flash_sync_read(st_block_pool *ppool)
{
    uint32_t   err_code;
    
    pstorage_handle_t   dest_block_id;

   
    err_code =  pstorage_block_identifier_get(&flash_manager.handle_record,ppool->id, &dest_block_id);	
    system_debug(err_code);

    glass_flash_sync(dest_block_id,PSTORAGE_UPDATE_OP_CODE);
   
    err_code = pstorage_update(&dest_block_id,(uint8_t  *)&(ppool->flash_record),sizeof(flashrecord),0); 
    system_debug(err_code);

    err_code  = glass_wait_sync();
    system_debug(err_code);    
   
    return  NRF_SUCCESS;


}
/*****************************************************************************
 Prototype    : glass_flash_read
 Description  : 从flash 中读取当天的一条数据
 Input        : time_union_t  data  
                char *buf           
                int*  len           
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/12/25
    Author       : x
    Modification : Created function

*****************************************************************************/
int  glass_flash_read(st_block_pool *ppool, char *buf,int*  len)
{

   int  blockid  = 0;
   int  offset   = 0;
   int  err_code = 0;
   pstorage_handle_t   dest_block_id;
  
   if(ppool->flash_record.data_r_offset >= ppool->flash_record.data_w_offset )
   {
       return   NRF_ERROR_FORBIDDEN;
   }

   if(buf == NULL)
        return   NRF_ERROR_FORBIDDEN;

   
   offset =   ppool->flash_record.data_r_offset;
   blockid = offset/PSTORAGE_MAX_BLOCK_SIZE;
   offset  = offset-(PSTORAGE_MAX_BLOCK_SIZE*blockid);
   
   err_code =  pstorage_block_identifier_get(&(ppool->flash_handle_t[blockid]),0,&dest_block_id);	
   system_debug(err_code);

           
    glass_flash_sync(dest_block_id,PSTORAGE_LOAD_OP_CODE);
    
    pstorage_load(buf, &dest_block_id, 16,0);
    system_debug(err_code);

    err_code  = glass_wait_sync();
    system_debug(err_code);

    ppool->flash_record.data_r_offset += 16; 


}
/*****************************************************************************
 Prototype    : flash_updata_record_flash
 Description  : 更新数据偏移到flash
 Input        : None
 Output       : None
 Return Value : static
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/12/19
    Author       : x
    Modification : Created function

*****************************************************************************/
static   int   flash_sync_record(st_block_pool *ppool,flash_buffer  buffer)
{ 
    int  err_code = 0;
   
    pstorage_handle_t   dest_block_id;

    err_code =  pstorage_block_identifier_get(&flash_manager.handle_record, ppool->id, &dest_block_id);	
    system_debug(err_code);

    
    glass_flash_sync(dest_block_id,PSTORAGE_UPDATE_OP_CODE);
   
    err_code = pstorage_update(&dest_block_id,(uint8_t  *)&(ppool->flash_record),sizeof(flashrecord),0); 
    system_debug(err_code);

    err_code  = glass_wait_sync();
    system_debug(err_code);

    return  err_code;

}

/*****************************************************************************
 Prototype    : glass_falsh_updatainfo
 Description  : 更新磁盘记录到RAM
 Input        : flashrecord  *precord   
                flashhandlet*  phandle  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/12/19
    Author       : x
    Modification : Created function

*****************************************************************************/
static   int     _flash_updata_record(st_block_pool*  pflashpool,int  id)
{

    uint32_t   err_code;
    
    pstorage_handle_t   dest_block_id;

   
    err_code =  pstorage_block_identifier_get(&flash_manager.handle_record, id, &dest_block_id);	
    system_debug(err_code);

         
    glass_flash_sync(dest_block_id,PSTORAGE_LOAD_OP_CODE);
    
    pstorage_load((void *)&(pflashpool->flash_record), &dest_block_id, sizeof(struct _st_flash_record),0);
    system_debug(err_code);

    err_code  = glass_wait_sync();
    system_debug(err_code);
 
    RECORD_VALID(pflashpool->flash_record.data_w_offset);
    RECORD_VALID(pflashpool->flash_record.data_r_offset);
    RECORD_VALID(pflashpool->flash_record.data_mark.data);

  #if  1
    printf("Pool  [%d]  Offset  [%d]\r\n",id,pflashpool->flash_record.data_w_offset);

  #endif

  
    return  NRF_SUCCESS;

}
/*****************************************************************************
 Prototype    : Alloc_Flash_Pool
 Description  : 创建一个存储仓库
 Input        : st_block_pool*  pflashpool  
                int  id                     
 Output       : None
 Return Value : static
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/12/19
    Author       : x
    Modification : Created function

*****************************************************************************/
static   int   _flash_create_pool(st_block_pool *ppool,int   id)
{
     uint32_t                   err_code;
     pstorage_module_param_t    module_param;
     
     ppool->id                  = id;

     for(int i = 0; i< DAY_OF_BLOCK; i++)
     {
         module_param.block_count   = 1;
         module_param.block_size    = PSTORAGE_MAX_BLOCK_SIZE;
         module_param.cb            = glass_flash_callback;

         err_code =  pstorage_register(&module_param,&(ppool->flash_handle_t[i]));
         system_debug(err_code);
     }
     
     return   err_code;

}
/*****************************************************************************
 Prototype    : flash_create_pool
 Description  : 创建flash 存储池
 Input        : st_block_pool *ppool  
                int   id              
 Output       : None
 Return Value : static
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/12/24
    Author       : x
    Modification : Created function

*****************************************************************************/
static   int  flash_create_pool()
{

   uint32_t err_code;
	
   for(int  i = 0;  i < FLASH_POOL_NUM;i++){
        err_code =  _flash_create_pool(&(flash_manager.flash_pool[i]),i);
   }

   return  err_code;
}
/*****************************************************************************
 Prototype    : flash_updata_record
 Description  : flash 更新记录到内
 Input        : st_block_pool*  pflashpool  
                int  id                     
 Output       : None
 Return Value : static
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/12/24
    Author       : x
    Modification : Created function

*****************************************************************************/
static   int  flash_updata_record( )
{
     uint32_t err_code;

     for(int  i = 0;  i  <  FLASH_POOL_NUM ;i++){

        err_code =  _flash_updata_record(&(flash_manager.flash_pool[i]),i);
     }

     return  err_code;
 
}
/*****************************************************************************
 Prototype    : flash_create_record
 Description  : 生成存储记录
 Input        : None
 Output       : None
 Return Value : static
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/12/24
    Author       : x
    Modification : Created function

*****************************************************************************/
static   int   flash_create_record()
{
     uint32_t                   err_code;
     pstorage_module_param_t    module_param;
     
     module_param.block_count   = FLASH_POOL_NUM;
     module_param.block_size    = FLASH_RECORD_SIZE;
     module_param.cb            = glass_flash_callback;
     
     err_code =  pstorage_register(&module_param,&flash_manager.handle_record);
     system_debug(err_code);   

     return   err_code;

}
/*****************************************************************************
 Prototype    : glass_flash_dump
 Description  : 获得当前的flash信息
 Input        : st_flash_manager *  pflash_manager  
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/12/25
    Author       : x
    Modification : Created function

*****************************************************************************/
st_flash_manager*  glass_flash_dump()
{
    return    &flash_manager;
}
/*****************************************************************************
 Prototype    : glass_flash_init
 Description  : flash  内存初始化
 Input        : None
 Output       : None
 Return Value : 
 Calls        : 
 Called By    : 
 
  History        :
  1.Date         : 2019/12/19
    Author       : x
    Modification : Created function

*****************************************************************************/
int   glass_flash_init()
{
     uint32_t      err_code;
     st_block_pool*   ppool;
     
     //err_code =  pstorage_init();   
     //system_debug (err_code);

     err_code =  flash_create_record();
     system_debug (err_code);

     err_code =  flash_create_pool  ();
     system_debug (err_code);

     err_code =  flash_updata_record();
     system_debug (err_code); 
   
     return   err_code;
     
}



#if  0
int  glash_debug()
{


    
	uint32_t                   err_code;
    unsigned  char             buf[128] =  {0};
    pstorage_handle_t          dest_block_id_[7];
    pstorage_handle_t          dest_record_id;
    pstorage_handle_t          blockid;

    err_code =  pstorage_block_identifier_get(&flash_manager.handle_record, 0, &dest_record_id);	
    if(err_code!=NRF_SUCCESS)
    {
        printf("pstorage_block_identifier_get  error\r\n");
        return  err_code;
    }
	



    glass_flash_sync(dest_record_id,PSTORAGE_LOAD_OP_CODE);
	
    pstorage_load(buf,&dest_record_id,sizeof(struct _st_flash_record),0);
    
    err_code  = glass_wait_sync();
    if(err_code == NRF_SUCCESS)
    {
        for(int  i = 0; i  < sizeof(struct _st_flash_record);i++)
         {
           printf("buf[%d]  %X\r\n",i,buf[i] );
         }
    }else{
       printf("read  buf  error\r\n");
    }

    

    for(int  i= 0;i <  sizeof(struct _st_flash_record);  i ++){

       buf[i]  =  i;
    }

    
    glass_flash_sync(dest_record_id,PSTORAGE_UPDATE_OP_CODE);
    err_code = pstorage_update(&dest_record_id,buf,16,0); 
    err_code  = glass_wait_sync();
    if(err_code == NRF_SUCCESS)
    {
       printf("write   buf  ok\r\n");
    }else{
       printf("write  buf    error\r\n");
    }

#if  0


     pstorage_module_param_t    module_param;
     
	 //err_code = pstorage_init();   
     //system_debug (err_code);
    
     module_param.block_count   = 1;
     module_param.block_size    = 16;
     module_param.cb            = glass_flash_callback;
     
     err_code =  pstorage_register(&module_param,&blockid);
     system_debug(err_code);
     


     
     for(int i = 0; i< 56; i++)
     {
           module_param.block_count   = 1;
           module_param.block_size    = PSTORAGE_MAX_BLOCK_SIZE;
           module_param.cb            = glass_flash_callback;

           err_code =  pstorage_register(&module_param,&dest_block_id_[i]);
           system_debug(err_code);
     }


    err_code =  pstorage_block_identifier_get(&blockid, 0, &dest_record_id);	
    if(err_code!=NRF_SUCCESS)
    {
        printf("pstorage_block_identifier_get  error\r\n");
        return  err_code;
    }
	



    glass_flash_sync(dest_record_id,PSTORAGE_LOAD_OP_CODE);
	
    pstorage_load(buf,&dest_record_id,sizeof(struct _st_flash_record),0);
    
    err_code  = glass_wait_sync();
    if(err_code == NRF_SUCCESS)
    {
        for(int  i = 0; i  < sizeof(struct _st_flash_record);i++)
         {
           printf("buf[%d]  %X\r\n",i,buf[i] );
         }
    }else{
       printf("read  buf  error\r\n");
    }

    

    for(int  i= 0;i <  sizeof(struct _st_flash_record);  i ++){

       buf[i]  =  i;
    }

    
    glass_flash_sync(dest_record_id,PSTORAGE_UPDATE_OP_CODE);
    err_code = pstorage_update(&dest_record_id,buf,16,0); 
    err_code  = glass_wait_sync();
    if(err_code == NRF_SUCCESS)
    {
       printf("write   buf  ok\r\n");
    }else{
       printf("write  buf    error\r\n");
    }



#endif
}

#endif
