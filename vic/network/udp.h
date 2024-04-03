//#pragma once
//
//#include <atomic>
//#include <iostream>
//#include <stdio.h>
//#include <string>
//#include <thread>
//#include <windows.h>
//#include <winsock2.h>
//#include <ws2tcpip.h>
//
//namespace vic
//{
//namespace network
//{
//namespace udp
//{
//
//class WSAContext
//{
//public:
//    WSAContext()
//    {
//        WSADATA data;
//        WORD version = MAKEWORD(2, 2);
//        int wsOk = WSAStartup(version, &data);
//        if(wsOk != 0)
//        {
//            std::cout << "WSAContext: unable to Startup! " << WSAGetLastError() << std::endl;
//            throw std::runtime_error("WSAContext: unable to Startup!");
//        }
//    }
//    ~WSAContext() { WSACleanup(); }
//};
//
//// NOTE: in order to use these classes, link: wsock32.lib, Ws2_32.lib
//// project (rmb)-> properties -> configuration properties -> linker -> input -> additional dependencies
//
//class UdpServer
//{
//public:
//    UdpServer(u_short port)
//        : mPort(port)
//    {
//        Setup();
//    }
//
//    void Setup()
//    {
//        mIn = socket(AF_INET, SOCK_DGRAM, 0);
//        if(mIn == INVALID_SOCKET)
//        {
//            throw std::runtime_error("UdpServer: invalid socket");
//        }
//
//        timeval timeout;
//        timeout.tv_sec = 0;
//        timeout.tv_usec = 1000;
//        setsockopt(mIn, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(struct timeval));
//
//        sockaddr_in serverHint;
//        serverHint.sin_addr.S_un.S_addr = ADDR_ANY;
//        serverHint.sin_family = AF_INET;
//        serverHint.sin_port = htons(mPort); // little to big endian
//
//        if(bind(mIn, (sockaddr*)&serverHint, sizeof(serverHint)) == SOCKET_ERROR)
//        {
//            std::cout << "UdpServer: Cannot bind to socket! error: " << WSAGetLastError() << std::endl;
//            throw std::runtime_error("UdpServer: Cannot bind to socket!");
//        }
//        mClientLenght = sizeof(mClient);
//        ZeroMemory(&mClient, mClientLenght);
//        ZeroMemory(mBuffer, 1024); // buffer where message gets received
//    }
//
//    template <typename Lambda>
//    void Run(Lambda onMessage)
//    {
//        std::size_t i = 0;
//        while(true)
//        {
//            if(!mRunning)
//            {
//                break;
//            }
//            i++;
//
//            ZeroMemory(mBuffer, 1024);
//            int bytesIn = recvfrom(mIn, mBuffer, 1024, 0, (sockaddr*)&mClient, &mClientLenght);
//            if(bytesIn == SOCKET_ERROR)
//            {
//                // std::cout << "Error when recieving from client! error: " << WSAGetLastError() << std::endl;
//                // throw std::runtime_error("Error when recieving from client!");
//                std::this_thread::yield();
//                break;
//            }
//            else if(bytesIn == 0)
//            {
//                continue;
//            }
//
//            // read message
//            constexpr std::size_t clientIpSize = 256;
//            char clientIp[clientIpSize];
//            ZeroMemory(clientIp, clientIpSize);
//
//            inet_ntop(AF_INET, &mClient.sin_addr, clientIp, clientIpSize);
//
//            // do something with the message
//            std::string message{mBuffer};
//            onMessage(mBuffer);
//        }
//
//        Cleanup();
//    }
//
//    void Stop() { mRunning = false; }
//
//    ~UdpServer() { Cleanup(); }
//
//    void Cleanup()
//    {
//        if(mIn != INVALID_SOCKET)
//            closesocket(mIn);
//        mIn = INVALID_SOCKET;
//    }
//
//private:
//    SOCKET mIn{INVALID_SOCKET};
//    std::atomic<bool> mRunning{true};
//
//    u_short mPort;
//    sockaddr_in mClient;
//    int mClientLenght;
//    char mBuffer[1024];
//};
//
//class UdpClient
//{
//public:
//    UdpClient(u_short port)
//        : mPort(port)
//    {
//        Setup();
//    }
//
//    void Setup()
//    {
//        // create hint structure for the server
//        mServer.sin_family = AF_INET;
//        mServer.sin_port = htons(mPort);
//        inet_pton(AF_INET, "127.0.0.1", &mServer.sin_addr);
//
//        // Socket creation
//        mOut = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
//        if(mOut == INVALID_SOCKET)
//        {
//            throw std::runtime_error("UdpClient invalid socket");
//        }
//    }
//
//    void Send(std::string_view message)
//    {
//        // write out to that socket
//        std::string somePackage{message}; // make a local copy of the message
//        int sendOk = sendto(mOut, somePackage.c_str(), int(somePackage.size() + 1), 0, (sockaddr*)&mServer, int(sizeof(mServer)));
//
//        if(sendOk == SOCKET_ERROR)
//        {
//            std::cout << "Error in client.Send()! error: " << WSAGetLastError() << std::endl;
//        }
//    }
//
//    ~UdpClient() { Cleanup(); }
//
//    void Cleanup()
//    {
//        if(mOut != INVALID_SOCKET)
//            closesocket(mOut);
//        mOut = INVALID_SOCKET;
//    }
//
//private:
//    u_short mPort;
//
//    SOCKET mOut{INVALID_SOCKET};
//    sockaddr_in mServer;
//};
//} // namespace udp
//} // namespace network
//} // namespace vic