#include "TcpClient.h"

TcpClient::TcpClient(const std::string& ip, const std::string& port) :
    socket_{ io_context_ },
    resolver_{ io_context_ },
    ip_{ ip },
    port_{ port },
    connected_{ false }
{
    run();
}

TcpClient::~TcpClient()
{
    boost::system::error_code ec;
    socket_.shutdown(boost::asio::ip::tcp::socket::shutdown_both, ec);
    socket_.close();
}

void TcpClient::run()
{
    sender_.join();
    connect();
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
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }
            else {
                std::cerr << "Connected." << std::endl;
                connected_ = true;
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << "Exception: " << e.what() << std::endl;
        }
    }

    run();
}

void TcpClient::enqueue(const std::string& data)
{
    queue_.push(data);
}

void TcpClient::processQueue()
{
    boost::system::error_code error;

    while (connected_) {
        try
        {
            if (!queue_.empty())
            {
                std::cout << "Sending data: '" << queue_.front() << "'" << std::endl;
                boost::asio::write(socket_, boost::asio::buffer(queue_.front() + "\n", queue_.front().length() + 1), error);
                queue_.pop();
            }

            if (error) {
                connected_ = false;
            }
            else {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
        catch (const std::exception& e)
        {
            std::cerr << "Exception: " << e.what() << std::endl;
        }
    }
}