#include "TcpClient.h"

TcpClient::TcpClient(const std::string& ip, const std::string& port) :
    socket_{ io_context_ },
    resolver_{ io_context_ },
    ip_{ ip },
    port_{ port },
    connected_{ false }
{
    connect();
}

TcpClient::~TcpClient()
{
    boost::system::error_code ec;
    socket_.shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
    socket_.close();
}

void TcpClient::connect()
{
    boost::system::error_code error;

    auto endpoint_iterator = resolver_.resolve({ ip_, port_ });

    while (!connected_) {
        try
        {
            boost::asio::connect(socket_, endpoint_iterator, error);
            if (error) {
                std::cerr << "Unable to connect" << std::endl;
                std::cout << "Trying again in 1 second." << std::endl;
            }
            else {
                std::cerr << "Connected." << std::endl;
                connected_ = true;
                start();
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << "Exception: " << e.what() << std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
}

void TcpClient::start()
{
    boost::system::error_code error;

    while (connected_) {
        try
        {
            std::cout << "Sending data: '" << data << "'" << std::endl;
            boost::asio::write(socket, boost::asio::buffer(data + "\n", data.length() + 1), error);

            if (error) {
                connect();
            }
            else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << "Exception: " << e.what() << std::endl;
        }
    }
}