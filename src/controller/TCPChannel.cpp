#include "controller/TCPChannel.hpp"
#include "core/common.hpp"
#include <boost/thread/thread.hpp>
#include <google/protobuf/io/coded_stream.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>

namespace gpb = google::protobuf::io;
using boost::asio::ip::tcp;
using namespace nui::events;

ssize_t TcpClientEntry::write(const char* data, const size_t len) {
	boost::system::error_code err;
	std::stringstream ss;
	boost::asio::write(m_socket, boost::asio::buffer(data, len),
			boost::asio::transfer_all(), err);
	if (err.value() != 0) {
		close();
		return -1;
	}
	return static_cast<ssize_t>(len);
}

TCPStreamChannel::TCPStreamChannel(const unsigned short port,
		bool noDelayOption) :
		StreamChannel(), m_noDelayOption(noDelayOption) {
	m_header = new char[10];
	m_acceptor = NULL;
	m_open = false;
	memcpy(m_header, "AV-NUI", 6);
	m_buf = 0;
	m_port = port;
	m_bufferSize = 3000000;
}

TCPControlChannel::~TCPControlChannel() {
	close();
}

void TCPStreamChannel::open() {
	if (!m_open) {
		assert(m_bufferSize > 0);
		m_buf = new char[m_bufferSize];
		tcp::endpoint endpoint(tcp::v4(), static_cast<unsigned short>(m_port));
		// setup acceptor manually to set reuse_address option
		m_acceptor = new tcp::acceptor(m_io);
		m_acceptor->open(endpoint.protocol());
		m_acceptor->set_option(
				boost::asio::ip::tcp::acceptor::reuse_address(true));
		try 
		{
			m_acceptor->bind(endpoint);
			m_acceptor->listen();
		} 
		catch(const std::exception& e)
		{
			BOOST_LOG_TRIVIAL(error)<< "Failed to open server port. [exception] : " << e.what();
		}
		BOOST_LOG_TRIVIAL(info)<< "Open TCP server connection on port: " << I2Str(m_port);
		boost::shared_ptr<TcpClientEntry> tmp(new TcpClientEntry(m_io));
		m_clients.push_back(tmp);
		// Reset io service to prepare for upcoming poll (needed when server is restarted)
		m_io.reset();
		// Setup asynchronous acceptor
		m_acceptor->async_accept(m_clients.back()->socket(),
				boost::bind(&TCPStreamChannel::handleAccept, this));
		m_open = true;
	}
}

void TCPStreamChannel::removeDeadClients() {
	// remove all clients that are not alive (i.e. socket closed)
	bool clientRemoved = false;
	do {
		std::vector<boost::shared_ptr<TcpClientEntry> >::iterator it =
				m_clients.begin();
		for (; it != (m_clients.end() - 1); ++it) {
			if (!(*it)->socket().is_open()) {
				m_clients.erase(it);
				clientRemoved = true;
				break;
			}
		}
	} while (!clientRemoved);
}

void TCPStreamChannel::send(const int32_t message_type, const std::string& data) {
	m_io.poll(); // execute ready handlers (i.e. the accept handler)
	boost::uint32_t dataLength;
	dataLength = data.size();
	// Header: | "AV-NUI" | message type | dataLen |
	// Byte:   |  0-5     |         6-9  | 10-13   |
	memcpy(m_header + 6,  (const char*) &message_type, 4); // write package message type
	memcpy(m_header + 10, (const char*) &dataLength,   4); // write package length
	bool clientDied = false;
	for (unsigned int i = 0; i < m_clients.size() - 1; ++i) {
		if (m_clients[i]->alive()) {
			// make sure that data buffer pointer is reset for successive clients

			if (m_clients[i]->write(m_header, 14) < 0)
			{
				clientDied = true;
			}
			if (m_clients[i]->write(data.c_str(), dataLength) < 0)
			{
				clientDied = true;
			}
		}
	}
	// remove dead clients, if some
	if (clientDied)
	{
		removeDeadClients();
	}
}
void TCPStreamChannel::send(const int32_t message_type, const google::protobuf::Message& message) {
    m_io.poll(); // execute ready handlers (i.e. the accept handler)
	int32_t dataLength = message.ByteSize();
	char * buffer = new char[dataLength + 8];
	gpb::ZeroCopyOutputStream * os = new gpb::ArrayOutputStream(buffer, message.ByteSize() + 8);
	gpb::CodedOutputStream * cos   = new gpb::CodedOutputStream(os);
	cos->WriteVarint32(message_type);
	cos->WriteVarint32(dataLength);
	message.SerializeToCodedStream(cos);
	int32_t byteCount = cos->ByteCount();
	delete cos;
	delete os;
	bool clientDied = false;
	for (unsigned int i = 0; i < m_clients.size() - 1; ++i) {
		if (m_clients[i]->alive()) {
			if (m_clients[i]->write(buffer, byteCount) < 0)
			{
				clientDied = true;
			}
		}
	}
	// remove dead clients, if some
	if (clientDied)
	{
		removeDeadClients();
	}
}
unsigned int TCPStreamChannel::noOfClients() {
	return static_cast<unsigned int>(m_clients.size()) - 1;
}

void TCPStreamChannel::handleAccept() {
	// on new accept: create a new socket and listen again
	if (m_noDelayOption) {
		boost::asio::ip::tcp::no_delay option(true);
		m_clients.back()->socket().set_option(option);
#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
		SOCKET native_sock = m_clients.back()->socket().native();
		int result = 0;

		if (INVALID_SOCKET != native_sock)
		{
			timeval tv; 
			memset( &tv, 0, sizeof(tv) ); 
			tv.tv_sec = 2; 
			result = setsockopt(native_sock, SOL_SOCKET, SO_SNDTIMEO, (char *)&tv, sizeof(tv) ); 
			if(result != 0)
			{
				BOOST_LOG_TRIVIAL(info)<< "TCP connection timeout can not be set. [ERROR] :  " << I2Str(result);
			}
		}
#endif
	}
	boost::shared_ptr<TcpClientEntry> tmp(new TcpClientEntry(m_io));
	m_clients.push_back(tmp);
	m_acceptor->async_accept(m_clients.back()->socket(),
			boost::bind(&TCPStreamChannel::handleAccept, this));
}

void TCPControlChannel::close() {
	if (m_open_incoming) {
		m_io_incoming.stop();
		if (m_acceptor_incoming && m_acceptor_incoming->is_open()) {
			m_acceptor_incoming->close();
		}
		m_open_incoming = false;
		delete m_acceptor_incoming;
		m_acceptor_incoming = NULL;
	}
}

void TCPStreamChannel::close() {
	if (m_open) {
		for (unsigned int i = 0; i < m_clients.size(); i++)
			m_clients[i]->socket().close();
		m_clients.clear();
		if (m_acceptor && m_acceptor->is_open())
			m_acceptor->close();
		m_io.stop();

		delete m_acceptor;
		m_acceptor = NULL;
		m_open = false;
	}
}

void TCPControlChannel::open() {
	if (!m_open_incoming) {
		tcp::endpoint endpoint(tcp::v4(),
				static_cast<unsigned short>(m_i_port));
		// setup acceptor manually to set reuse_address option
		m_acceptor_incoming = new tcp::acceptor(m_io_incoming);
		m_acceptor_incoming->open(endpoint.protocol());
		m_acceptor_incoming->set_option(
				boost::asio::ip::tcp::acceptor::reuse_address(true));
		m_acceptor_incoming->bind(endpoint);
		m_acceptor_incoming->listen();
		BOOST_LOG_TRIVIAL(info)<< "Incoming TCP server connection on port: " << I2Str(m_i_port);
		// Reset io serive to prepare for upcoming poll (needed when server is restarted)
		m_io_incoming.reset();
		m_open_incoming = true;
		boost::thread t(boost::bind(&TCPControlChannel::incomingCalls, this));
		m_ptr_incoming_thread = &t;
	}
}

void TCPControlChannel::incomingCalls() {
	ConnectionState state = WAIT_FOR_HEADER;
	try {
		m_acceptor_incoming->accept(m_socket);
	} catch (...) {
		return;
	}
	bool endpointValid = false;
	int32_t dataLength = 0;
	std::stringstream dataBuf;
	char buf[10];
	size_t len = 0;
	size_t received = 0;

	while (m_open_incoming) {
		switch (state) {
		case WAIT_FOR_HEADER:
			// make sure to read a datagram that has sizeof(header) at max
			len = m_socket.read_some(boost::asio::buffer(buf, 10),
					m_err_feedback);

			if (m_err_feedback.value() != 0) {
				BOOST_LOG_TRIVIAL(error)<< "Error occurred [CODE]: " << I2Str(m_err_feedback.value());
				return;
			}
			//check if we have a header
			if (memcmp(buf, "AV-NUI", 6) == 0)
			{
				memcpy(&dataLength, buf + 6, 4);
				received = 0;
				//proceed to next state
				state = WAIT_FOR_DATA;
			}
			break;
			case WAIT_FOR_DATA:
			len = m_socket.read_some(boost::asio::buffer(buf, dataLength - received), m_err_feedback);
			if (m_err_feedback.value() != 0)
			{
				BOOST_LOG_TRIVIAL(error) << "Error occurred [CODE]: " << I2Str(m_err_feedback.value());
				return;
			}
			received += len;
			dataBuf.write(buf, len);

			// only proceed if data is complete
			if (received == dataLength)
			{
				state = PACKET_COMPLETE;
			}

			break;
			case PACKET_COMPLETE:
			FeedbackEvent* fe = new FeedbackEvent();
			fe->command = dataBuf.str();
			fe->timestamp = time(NULL);
			fireReceivedCommand(FEEDBACKEVENT_PTR(fe));
			dataBuf.str("");
			dataBuf.clear();
			state = WAIT_FOR_HEADER;
			break;

		}
	}
}
