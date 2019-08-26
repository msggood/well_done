#include <stdio.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>

#include "reg_defined.h"

static int fd;
static uint32_t page_size;
static uint32_t map_base,mapped_size;

int reg_init(uint32_t base_addr,uint32_t size)
{
    page_size = getpagesize();
    mapped_size = size;
    fd = open("/dev/mem", (O_RDWR | O_SYNC));
    if(fd<0)
    {
        printf("open /dev/mem error\n");
    }
    map_base = mmap(NULL,mapped_size,(PROT_READ | PROT_WRITE), MAP_SHARED,fd, base_addr & ~(off_t)(page_size - 1));
    if(map_base==NULL)
        printf("mmap error\n");
}
void writew(uint32_t offset,uint32_t value)
{
     *(volatile uint32_t*)(map_base+offset) = value;
}

uint32_t readw(uint32_t offset)
{
     return *(volatile uint32_t*)(map_base+offset);
}

void reg_exit()
{
    if (munmap(map_base, mapped_size) == -1)
        printf("munmap error\n");
    close(fd);
}
