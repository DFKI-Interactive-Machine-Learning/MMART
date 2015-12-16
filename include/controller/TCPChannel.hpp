/*
 * This file is part of the AV-NUI project.
 * Copyright (C) 2014 DFKI GmbH. All rights reserved.
 *
 * Disclaimer:
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDER AND
 * CONTRIBUTORS ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 *
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */
#pragma once
#include "controller/ControlChannel.hpp"
#include "core/common.hpp"

#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/enable_shared_from_this.hpp>
#include <boost/thread/thread.hpp>
#include <boost/asio.hpp>
#include <iostream>

#define TCP_PROXY_BUFFER_SIZE 32767       // >  9   AND   < (2^31 - 9) = 2147483639
#define TCP_PROXY_ACCEPT_TIMEOUT 5000     // in  milliseconds and  VALUE < 2 147 483
#define TCP_PROXY_CONNECT_TIMEOUT 4000    // in  milliseconds and  VALUE < 2 147 483
#define TCP_PROXY_SEND_TIMEOUT 5000       // in  milliseconds and  VALUE < 2 147 483
#define TCP_PROXY_RECV_TIMEOUT 5000       // in  milliseconds and  VALUE < 2 147 483

/**
 * class TcpClientEntry
 * Implements a client end-point needed by TCPStreamChannel.
 */
class TcpClientEntry: public boost::enable_shared_from_this<TcpClientEntry> {
public:
	TcpClientEntry(boost::asio::io_service& io) :
			m_socket(io) {
	}

	inline void close()  {
		m_socket.close();
	}

	inline bool alive() {
		return m_socket.is_open();
	}

	inline boost::asio::ip::tcp::socket& socket() {
		return m_socket;
	}
	/**
	 * \brief Send data to client.
	 * \param message_type - type of message
	 * \param data - char data
	 * \param len  - of data
	 */
	ssize_t write(const char* data, const size_t len);

private:
	/**
	 * Boost socket.
	 */
	boost::asio::ip::tcp::socket m_socket;
};

typedef std::vector<boost::shared_ptr<TcpClientEntry> > ClientList;
/**
 * @class TcpServer
 * Implements tcp server functionality using boost asio library.
 * The class is capable of handling serveral clients.
 */
class TCPStreamChannel: public StreamChannel {
public:
	TCPStreamChannel(const unsigned short port, bool noDelayOption = true);
	virtual ~TCPStreamChannel() {
	};
	/**
	 * \brief Sending stringstream.
	 * \param message_type - type of message
	 * \param stream       - string stream
	 */
	virtual void send(const int32_t message_type, const std::string&);

	virtual void send(const int message_type, const google::protobuf::Message&);
	/**
	 * \brief Open stream.
	 */
	void open();
	/**
	 * \brief Close stream.
	 */
	void close();
	/**
	 * \brief Returns the number of connected clients.
	 * \returns number of clients
	 */
	unsigned int noOfClients();

protected:

	ClientList m_clients;
	unsigned short m_port;
	unsigned int m_bufferSize;
	bool m_noDelayOption;
	char* m_header;
	char* m_buf;
	bool m_open;
private:
	boost::system::error_code m_err;
	boost::asio::io_service m_io;
	boost::asio::ip::tcp::acceptor* m_acceptor;

	void removeDeadClients();
	void handleAccept();
};

/**
 * @class TcpServer
 * Implements tcp server functionality using boost asio library.
 * The class is capable of handling serveral clients.
 */
class TCPControlChannel: public ControlChannel {
public:
	TCPControlChannel(const unsigned short port) :
			ControlChannel(), m_socket(m_io_incoming),
			m_ptr_incoming_thread(NULL), m_acceptor_incoming(NULL),
			m_open_incoming(false),
			m_i_port(port), state(WAIT_FOR_HEADER) {
	}
	virtual ~TCPControlChannel();
	void open();
	void close();
	unsigned int noOfClients();

protected:
	/**
	 * Internal connection state.
	 */
	enum ConnectionState {
		WAIT_FOR_HEADER, WAIT_FOR_DATA, PACKET_COMPLETE
	};
	bool m_open_incoming;
	unsigned short m_i_port;

	void incomingCalls();
private:
	boost::system::error_code m_err_feedback;
	boost::asio::io_service m_io_incoming;
	boost::asio::ip::tcp::acceptor* m_acceptor_incoming;
	boost::asio::ip::tcp::socket m_socket;
	boost::thread* m_ptr_incoming_thread;
	ConnectionState state;
};
