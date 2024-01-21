#include <iostream>
#include <string>
#include <chrono>
#include <thread>
#include <boost/asio.hpp>

using boost::asio::ip::tcp;

std::string data = { "$14822963501AA00D104FFA200" };

class TcpClient
{
public:
    TcpClient() = delete;
    TcpClient(const std::string& ip, const std::string& port);
    ~TcpClient();
private:
    void connect();
    void start();

    boost::asio::io_context io_context_;
    tcp::socket socket_;
    tcp::resolver resolver_;
    std::string ip_;
    std::string port_;
    bool connected_;
};