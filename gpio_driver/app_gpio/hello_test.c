#include <stdio.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/ioctl.h>

int main()
{
    int ret;
    int fd =open("/dev/hello",O_RDWR);
    if(fd<0)
    {
       printf("open /dev/hello error!\n");
       return 0;
    }
    char r_buf[4];
    char w_buf[3]="wri";
    memset(r_buf,'\0',4);
    ret=read(fd,r_buf,3);
    printf("r_buf=%s,ret=%d\n",r_buf,ret);
    sleep(1);
    write(fd,w_buf,3);
//    if(ioctl(fd,1,2)!=0)
//	printf("1,2 error\n");
    if((ret=ioctl(fd,2,3))!=0)
	printf("2,3 error ret=%d\n",ret);
//    if(ioctl(fd,3,4)!=0)
//	printf("1,2 error\n");
    return 0;
}
