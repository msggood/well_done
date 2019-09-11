#include <sys/types.h>
#include <sys/socket.h>
#include <sys/epoll.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>

#define MAX_EVENT_NUM           1024
#define BUFFER_SIZE             10

void perr_exit(char *arg)
{
    printf("%s\n",arg);
    exit(-1);
}

int start_ser(char *ipaddr, char *port)
{
    int sock = socket(AF_INET, SOCK_STREAM, 0);

    struct sockaddr_in serveraddr;
    bzero(&serveraddr, sizeof(serveraddr));
    serveraddr.sin_family = AF_INET;
    serveraddr.sin_port = htons(atoi(port));
    inet_pton(AF_INET, ipaddr, &serveraddr.sin_addr);

    bind(sock, (struct sockaddr *)&serveraddr, sizeof(serveraddr));

    listen(sock, 128);

    return sock;
}

int setnonblocking(int fd)
{
    int old_opt = fcntl(fd, F_GETFD);
    int new_opt = old_opt | O_NONBLOCK;
    fcntl(fd, F_SETFD, new_opt);

    return old_opt;
}//将文件描述符设置为非阻塞的

void addfd(int epollfd, int fd, int enable_et)
{
    struct epoll_event event;
    event.data.fd = fd;
    event.events = EPOLLIN;
    if(enable_et){
        event.events |= EPOLLET;
    }
    printf("add fd=%d\n",fd);
    epoll_ctl(epollfd, EPOLL_CTL_ADD, fd, &event);
    //      setnonblocking(fd);
}//将文件描述符fd的EPOLLIN注册到epollfd指示的epoll内核事件表中，enable_et表示是否对fd启用ET模式

void process_event(struct epoll_event *events, int num, int epollfd, int listenfd)
{
    char buf[BUFFER_SIZE];
    printf("event nums=%d\n",num);
    for(int i = 0; i < num; i++){
        printf("fd=%d\n",events[i].data.fd);
        printf("ev =0x%.8x\n",events[i].events);
        if(events[i].data.fd == listenfd){
            struct sockaddr clientaddr;
            int sockaddr_len=sizeof(clientaddr);
            int connfd = accept(listenfd, &clientaddr,&sockaddr_len);
            addfd(epollfd, connfd, false);//对connfd使用默认的lt模式
        }else if(events[i].events & EPOLLIN){//只要socket读缓存中还有未读的数据，这段代码就会触发
           // printf("event trigger fd=%d\n",events[i].data.fd);
            memset(buf, '\0', BUFFER_SIZE);
            if(events[i].events & EPOLLET)
            {
                printf("et event\n");
            }
            int ret = recv(events[i].data.fd , buf, BUFFER_SIZE-1, 0);
            if(ret <= 0){
                close(events[i].data.fd );
                continue;
            }
            printf("recv:%d,%s\n", ret, buf);
        }else{
            printf("something else happened\n");
        }
    }
}

int main(int argc, char *argv[])
{

    int listenfd = start_ser("192.168.0.222", "6666");
    struct epoll_event events[MAX_EVENT_NUM];
    int epollfd = epoll_create(1);
    if(epollfd < 0){
        perr_exit("epoll_create err");
    }
    addfd(epollfd, listenfd, true);
    while(1){
        int ret = epoll_wait(epollfd, events, MAX_EVENT_NUM, -1);
        if(ret < 0){
            printf("epoll failure\n");
            break;
        }

        printf("----------------------------------------------------\n");
        process_event(events, ret, epollfd, listenfd);//lt模式
        //et(events, ret, epollfd, listenfd);//et模式
        printf("\n\n");
    }
    close(listenfd);
    return 0;
}
