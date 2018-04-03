#line 1 "core\\malloc.c"
#line 1 "core\\malloc.h"
















typedef unsigned long  u32;
typedef unsigned short u16;
typedef unsigned char  u8;	  

















		 
		 

struct _m_mallco_dev
{
	void (*init)(u8);					
	u8 (*perused)(u8);		  	    	
	u8 	*membase[2];					
	u16 *memmap[2]; 					
	u8  memrdy[2]; 						
};
extern struct _m_mallco_dev mallco_dev;	 

void mymemset(void *s,u8 c,u32 count);	 
void mymemcpy(void *des,void *src,u32 n);
void mem_init(u8 memx);					 
u32 mem_malloc(u8 memx,u32 size);		 
u8 mem_free(u8 memx,u32 offset);		 
u8 mem_perused(u8 memx);				 


void myfree(u8 memx,void *ptr);  			
void *mymalloc(u8 memx,u32 size);			
void *myrealloc(u8 memx,void *ptr,u32 size);














#line 2 "core\\malloc.c"


__align(4) u8 mem1base[40*1024];													
__align(4) u8 mem2base[200*1024] __attribute__((at(0X68000000)));					

u16 mem1mapbase[40*1024/32];													
u16 mem2mapbase[200*1024/32] __attribute__((at(0X68000000+200*1024)));	

const u32 memtblsize[2]={40*1024/32,200*1024/32};		
const u32 memblksize[2]={32,32};					
const u32 memsize[2]={40*1024,200*1024};							



struct _m_mallco_dev mallco_dev=
{
	mem_init,				
	mem_perused,			
	mem1base,mem2base,		
	mem1mapbase,mem2mapbase,
	0,0,  					
};





void mymemcpy(void *des,void *src,u32 n)  
{  
    u8 *xdes=des;
	u8 *xsrc=src; 
    while(n--)*xdes++=*xsrc++;  
}  




void mymemset(void *s,u8 c,u32 count)  
{  
    u8 *xs = s;  
    while(count--)*xs++=c;  
}	   


void mem_init(u8 memx)  
{  
    mymemset(mallco_dev.memmap[memx], 0,memtblsize[memx]*2);
	mymemset(mallco_dev.membase[memx], 0,memsize[memx]);	
	mallco_dev.memrdy[memx]=1;								
}  



u8 mem_perused(u8 memx)  
{  
    u32 used=0;  
    u32 i;  
    for(i=0;i<memtblsize[memx];i++)  
    {  
        if(mallco_dev.memmap[memx][i])used++; 
    } 
    return (used*100)/(memtblsize[memx]);  
}  




u32 mem_malloc(u8 memx,u32 size)  
{  
    signed long offset=0;  
    u16 nmemb;	
	u16 cmemb=0;
    u32 i;  
    if(!mallco_dev.memrdy[memx])mallco_dev.init(memx);
    if(size==0)return 0XFFFFFFFF;
    nmemb=size/memblksize[memx];  	
    if(size%memblksize[memx])nmemb++;  
    for(offset=memtblsize[memx]-1;offset>=0;offset--)
    {     
		if(!mallco_dev.memmap[memx][offset])cmemb++;
		else cmemb=0;								
		if(cmemb==nmemb)							
		{
            for(i=0;i<nmemb;i++)  					
            {  
                mallco_dev.memmap[memx][offset+i]=nmemb;  
            }  
            return (offset*memblksize[memx]);
		}
    }  
    return 0XFFFFFFFF;
}  




u8 mem_free(u8 memx,u32 offset)  
{  
    int i;  
    if(!mallco_dev.memrdy[memx])
	{
		mallco_dev.init(memx);    
        return 1;
    }  
    if(offset<memsize[memx])
    {  
        int index=offset/memblksize[memx];			
        int nmemb=mallco_dev.memmap[memx][index];	
        for(i=0;i<nmemb;i++)  						
        {  
            mallco_dev.memmap[memx][index+i]=0;  
        }  
        return 0;  
    }else return 2;
}  



void myfree(u8 memx,void *ptr)  
{  
	u32 offset;  
    if(ptr==0)return;
 	offset=(u32)ptr-(u32)mallco_dev.membase[memx];  
    mem_free(memx,offset);
}  




void *mymalloc(u8 memx,u32 size)  
{  
    u32 offset;  									      
	offset=mem_malloc(memx,size);  	   				   
    if(offset==0XFFFFFFFF)return 0;  
    else return (void*)((u32)mallco_dev.membase[memx]+offset);  
}  





void *myrealloc(u8 memx,void *ptr,u32 size)  
{  
    u32 offset;  
    offset=mem_malloc(memx,size);  
    if(offset==0XFFFFFFFF)return 0;     
    else  
    {  									   
	    mymemcpy((void*)((u32)mallco_dev.membase[memx]+offset),ptr,size);	
        myfree(memx,ptr);  											  		
        return (void*)((u32)mallco_dev.membase[memx]+offset);  				
    }  
}












