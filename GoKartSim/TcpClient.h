#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <boost/asio.hpp>
#include <queue>

using boost::asio::ip::tcp;

class TcpClient
{
public:
    TcpClient() = delete;
    TcpClient(const std::string& ip, const std::string& port);
    ~TcpClient();
    void enqueue(const std::string& data);
    void processQueue();
private:
    void connect();
    void run();

    boost::asio::io_context io_context_;
    tcp::socket socket_;
    tcp::resolver resolver_;
    std::string ip_;
    std::string port_;
    bool connected_;
    std::queue<std::string> queue_;
    std::thread sender_{ &TcpClient::processQueue, this };
};