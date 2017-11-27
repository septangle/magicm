#include "MySocket.h"
#include <WINSOCK2.H>
#include <stdio.h>
#pragma  comment(lib,"ws2_32.lib")

CMySocket::CMySocket()
{
}


CMySocket::~CMySocket()
{
}

void CMySocket::Send(char* pContent, int nSize)
{
    WORD sockVersion = MAKEWORD(2, 2);
    WSADATA data;
    if (WSAStartup(sockVersion, &data) != 0)
    {
        return;
    }

    SOCKET sclient = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sclient == INVALID_SOCKET)
    {
        printf("invalid socket !");
        return;
    }

    sockaddr_in serAddr;
    serAddr.sin_family = AF_INET;
    serAddr.sin_port = htons(9000);
    serAddr.sin_addr.S_un.S_addr = inet_addr("192.168.1.111");
    if (connect(sclient, (sockaddr *)&serAddr, sizeof(serAddr)) == SOCKET_ERROR)
    {
        printf("connect error !");
        closesocket(sclient);
        return;
    }
    int n = send(sclient, pContent, nSize, 0);

    closesocket(sclient);
    WSACleanup();
}