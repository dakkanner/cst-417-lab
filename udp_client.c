//UDPClient.c
 
/*
 * gcc -o client UDPClient.c
 * ./client <server-ip>
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
 
void err(char *s)
{
    perror(s);
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
 
int main(int argc, char** argv)
{
    struct sockaddr_in serv_addr;
    int sockfd, i, slen=sizeof(serv_addr);
    char buf[BUFLEN];
    char buf2[BUFLEN];
    uint8_t crc8Value;
 
    if(argc != 2)
    {
      printf("Usage : %s <Server-IP>\n",argv[0]);
      exit(0);
    }
 
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
        err("socket");
 
    bzero(&serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);
    if (inet_aton(argv[1], &serv_addr.sin_addr)==0)
    {
        fprintf(stderr, "inet_aton() failed\n");
        exit(1);
    }
 
    while(1)
    {
        printf("\nEnter data to send(Type exit and press enter to exit) : ");
        scanf("%[^\n]",buf);
        getchar();
        if(strcmp(buf,"exit") == 0)
          exit(0);
        
        // Prepend the CRC-8 value to the string
        crc8Value = FindCrc8((uint8_t *)buf, strlen(buf));
        buf2[0] = (char)crc8Value;
        buf2[1] = '\0';
        strcat(buf2, buf);
        printf("Sending message '(0x%x)%s'", crc8Value, buf2);
 
        if (sendto(sockfd, buf2, strlen(buf2), 0, (struct sockaddr*)&serv_addr, slen)==-1)
            err("sendto()");
        
        for(i = 0; i < BUFLEN; i++)
        {
            buf[i] = '\0';
            buf2[i] = '\0';
        }
    }
 
    close(sockfd);
    return 0;
}