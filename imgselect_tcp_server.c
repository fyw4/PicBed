#include <stdio.h>
#include <string.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <signal.h>
#include <sys/wait.h>
#include <sys/types.h>

#include "wrap.h"

#define PORT 8000
int main(int argc, char* argv[])
{
	//创建套接字
	int lfd = tcp4bind(PORT, NULL);
	//监听
	Listen(lfd, 128);
	int maxfd = lfd; //最大的文件描述符
	fd_set oldset, rset;
	FD_ZERO(&oldset);
	FD_ZERO(&rset);
	//将lfd添加到oldset集合中
	FD_SET(lfd,&oldset);
	while(1)
	{
		//select监听
		rset = oldset;//将oldset赋值给需要监听的集合rset
		int n = select(maxfd + 1, &rset, NULL, NULL, 0);
		if(n < 0)
		{
			perror(" ");
			break;
		}
		else if(n == 0)
		{
			continue;
		}
		else //监听到了文件描述符
		{
			//lfd变化，有新连接 
			if(FD_ISSET(lfd, &rset))
			{
				struct sockaddr_in cliaddr;
				socklen_t len = sizeof(cliaddr);
				char ip[16] = "";
				
				//提取新的连接
				int cfd = Accept(lfd, (struct sockaddr*)&cliaddr, &len);
				printf("new client ip = %s port = %d\n", inet_ntop(AF_INET, &cliaddr.sin_addr.s_addr, ip, 16), ntohs(cliaddr.sin_port));
				
				//将cfd添加至oldset集合中，以下次监听
				FD_SET(cfd, &oldset);
				
				//更新maxfd
				if(cfd > maxfd)
					maxfd = cfd;
				
				//如果只有lfd变化，continue
				if(--n == 0)
					continue;
			}
			
			//cfd 遍历lfd之后的文件描述符是否在rset集合中，如果在cfd变化
			for(int i = lfd + 1; i <= maxfd; i++)
			{
				//如果定义的i描述符在rset集合中
				if(FD_ISSET(i, &rset))
				{
					char buf[1500] = "";
					int ret = Read(i, buf, sizeof(buf));
					if(ret < 0)//出错将cfd关闭，从oldset中删除cfd
					{
						perror("");
						Close(i);
						FD_CLR(i, &oldset);
						continue;
					}
					else if(ret == 0)
					{
						printf("client Close\n");
						Close(i);
						FD_CLR(i, &oldset);
					}
					else
					{
						printf("%s\n", buf);
						Write(i, buf, ret);
					}
				}
			}
		}
		
	}
	
	
	return 0;
}
