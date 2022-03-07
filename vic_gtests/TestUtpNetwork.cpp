#include "pch.h"

#pragma comment (lib, "ws2_32.lib")
#pragma comment (lib, "wsock32.lib")

#include "vic/network/udp.h"
#include <chrono>
#include <thread>

using namespace vic;

TEST(TestUdpNetwork, TestHappyFlow)
{
	vic::network::udp::WSAContext wsaContext{};

	u_short port = 54005;
	vic::network::udp::UdpServer server{ port };

	{
		vic::network::udp::UdpClient client{ port };
		// std::this_thread::sleep_for(std::chrono::seconds(1));

		std::atomic<int> received = 0;
		auto onMessage = [&](const std::string& message) {
			received += 1;
		};

		std::thread serverThread([&]() { server.Run(onMessage); });

		std::this_thread::sleep_for(std::chrono::milliseconds(100));

		client.Send("Hello");
		client.Send("World!");

		// TODO(vicdie): this sleep breaks the test
		// std::this_thread::sleep_for(std::chrono::milliseconds(100));

		server.Stop();
		std::this_thread::sleep_for(std::chrono::milliseconds(100));

		if (serverThread.joinable())
			serverThread.join();

		EXPECT_EQ(received, 2);
	}
}

// TODO(vicdie): how do you test a protocol that does not give a guarantee for _anything_?