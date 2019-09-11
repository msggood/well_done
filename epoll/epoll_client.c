#include <sys/types.h>
#include <sys/socket.h>
#include <sys/epoll.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>



int main(int argc, char *argv[])
{
        int connfd;
        struct sockaddr_in serveraddr;
        char buf[1024];

        connfd = socket(AF_INET, SOCK_STREAM, 0);

        bzero(&serveraddr, sizeof(serveraddr));
        serveraddr.sin_family = AF_INET;
        serveraddr.sin_port = htons(atoi(argv[2]));
        inet_pton(AF_INET, argv[1], &serveraddr.sin_addr);

        while(connect(connfd, (struct sockaddr *)&serveraddr, sizeof(serveraddr)));

        while(fgets(buf, 1024, stdin) != NULL){
                write(connfd, buf, strlen(buf));
        }

        close(connfd);
        return 0;
}
