/*
 * =====================================================================================
 *
 *       Filename:  udp_server.c
 *
 *    Description:  Simple UDP Server
 *
 *        Version:  1.0
 *        Created:  11/07/2013 08:48:44 AM
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (), 
 *   Organization:  
 *
 * =====================================================================================
 */

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#define BUFLEN 512
#define PORT 9930
 
void err(char *str)
{
    perror(str);
    exit(1);
}

// Get the CRC-8 value from dataIn when using x^8 + x^7 + x^2 + x + 1
uint8_t FindCrc8(const uint8_t* dataIn, int len)
{
    const uint8_t* dataPtr = dataIn;
    unsigned crc = 0;
    int i, j;

    for (i = len; i; i--, dataPtr++) {
        crc ^= (*dataPtr << 8);

        for(j = 8; j; j--) {
            if (crc & 0x8000)
                crc ^= (0x1870 << 3);
            crc <<= 1;
        }
    }
    
    return (uint8_t)(crc >> 8);
}
 
int main(void)
{
    struct sockaddr_in my_addr, cli_addr;
    int sockfd, i;
    socklen_t slen=sizeof(cli_addr);
    char buf[BUFLEN];
    uint8_t* buf2;
    uint8_t crcIn;
    uint8_t crcCalculated;
 
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
      err("socket");
    else
      printf("Server : Socket() successful\n");
 
    bzero(&my_addr, sizeof(my_addr));
    my_addr.sin_family = AF_INET;
    my_addr.sin_port = htons(PORT);
    my_addr.sin_addr.s_addr = htonl(INADDR_ANY);
     
    if (bind(sockfd, (struct sockaddr* ) &my_addr, sizeof(my_addr))==-1)
      err("bind");
    else
      printf("Server : bind() successful\n");
 
    while(1)
    {
        if (recvfrom(sockfd, buf, BUFLEN, 0, (struct sockaddr*)&cli_addr, &slen)==-1)
            err("recvfrom()");
        printf("Received packet from %s:%d\nData: %s\n",
               inet_ntoa(cli_addr.sin_addr), ntohs(cli_addr.sin_port), buf);
        
        // Set buf2 to be the string
        buf2 = (uint8_t*)buf;
        buf2++;
        crcIn = (uint8_t)buf[0];
        crcCalculated = FindCrc8(buf2, strlen((char *)buf2));
        
    	if (crcIn == crcCalculated)
    	{
    		printf("CRC received matches CRC expected: 0x%x vs 0x%x\n\n", crcIn, crcCalculated);
    	}
    	else
    	{
    		printf("Error: CRC received does not match CRC expected: 0x%x vs 0x%x\n\n", crcIn, crcCalculated);
    	}

        memset(buf, '\0', BUFLEN);
    }
 
    close(sockfd);
    return 0;
}
