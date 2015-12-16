/*
 * This file is part of the AV-NUI project.
 * Copyright (C) 2012 DFKI GmbH. All rights reserved.
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
#include "core/common.hpp"
#include "eventmanager/Events.hpp"
#include <google/protobuf/message.h>
#include <boost/function.hpp>

typedef boost::function1<void, nui::events::FeedbackEventPtr> ReceivedCommandFunc;

/**
 * Abstract controller channel.
 */
class StreamChannel {
public:
	/**
	 * \brief Destructor.
	 */
	virtual ~StreamChannel() {};
	/**
	 * Sends a command using the opened channel.
	 *  \param[in] stringstream with message
	 */
	virtual void send(const int message_type, const std::string&) = 0;
	/**
	 * Sends a command using the opened channel.
	 *  \param[in] - type
	 *  \param[in] stringstream with message
	 */
	virtual void send(const int message_type, const google::protobuf::Message&) = 0;
	/**
	 * Opens the controller channel
	 */
	virtual void open() = 0;
	/**
	 * Closes the controller channel
	 */
	virtual void close() = 0;
};
/**
 * Abstract controller channel.
 */
class ControlChannel {
public:
	/**
	 * Constructor
	 */
	ControlChannel()
 	{
	};
	virtual ~ControlChannel(){};
	/**
	 * Opens the controller channel
	 */
	virtual void open() = 0;
	/**
	 * Closes the controller channel
	 */
	virtual void close() = 0;
	/**
	 * \brief Received command register function for callback.
	 *
	 * \param[in] function pointer
	 */
	void RegisterReceivedCommandFunc(ReceivedCommandFunc func) {
		m_received_command_func = func;
	}
protected:
	/**
	 * Fires a command to the listener function.
	 *  \param[in] string - command
	 */
	void fireReceivedCommand(nui::events::FeedbackEventPtr command) {
		if (m_received_command_func) {
			m_received_command_func(command);
		}
	}
private:
	/** Received command callback function. */
	ReceivedCommandFunc m_received_command_func;
};
