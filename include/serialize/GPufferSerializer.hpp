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
#include "core/common.hpp"
#include "eventmanager/Events.hpp"
#include "interface.pb.h"
#include <queue>

/**
 * \brief Using Google Protocol buffers to serialize.
 * \param evt - Leap motion event
 * \param path - path to store
 */
EVENTMANAGER_EXPORT void  gbuff_serialize(nui::events::LeapTrackingEvent& evt, const bfs::path path);

/**
 * \brief Using Google Protocol buffers to serialize.
 * \param evt - Leap motion event
 * \param stream - output stream
 */
EVENTMANAGER_EXPORT void  gbuff_serialize(nui::events::LeapTrackingEvent& evt, std::ostream* stream);

/**
 * \brief Using Google Protocol buffers to serialize.
 * \param queue - Leap motion event
 * \param path - path to store
 */
EVENTMANAGER_EXPORT void  gbuff_serialize(std::queue<nui::events::LeapTrackingEvent>& evt, const bfs::path path);

/**
 * \brief Using Google Protocol buffers to serialize.
 * \param queue - Leap motion event
 * \param stream - output stream
 */
EVENTMANAGER_EXPORT void  gbuff_serialize(std::queue<nui::events::LeapTrackingEvent>& evt, std::ostream* stream);

/**
 * \brief Using Google Protocol buffers to serialize.
 * \param queue - Leap motion event
 * \param stream - output stream
 */
EVENTMANAGER_EXPORT void gbuff_serialize(std::queue<nui::events::CircleGestureEvent>& evtQueue,
		const bfs::path path);

/**
 * \brief Using Google Protocol buffers to serialize.
 * \param queue - Leap gestures event
 * \param stream - output stream
 */
EVENTMANAGER_EXPORT void gbuff_serialize(std::queue<nui::events::CircleGestureEvent>& evtQueue,
		std::ostream* stream);

EVENTMANAGER_EXPORT void gbuff_serialize(std::queue<nui::events::LeapActionGestureEvent>& evtQueue,
        const bfs::path path);
EVENTMANAGER_EXPORT void gbuff_serialize(std::queue<nui::events::LeapActionGestureEvent>& evtQueue,
        std::ostream* stream);
EVENTMANAGER_EXPORT void gbuff_serialize(std::queue<nui::events::SwipeGestureEvent>& evtQueue,
        const bfs::path path);
EVENTMANAGER_EXPORT void gbuff_serialize(std::queue<nui::events::SwipeGestureEvent>& evtQueue,
        std::ostream* stream);
EVENTMANAGER_EXPORT void gbuff_serialize(std::queue<nui::events::KeyTapGestureEvent>& evtQueue,
        const bfs::path path);
EVENTMANAGER_EXPORT void gbuff_serialize(std::queue<nui::events::KeyTapGestureEvent>& evtQueue,
        std::ostream* stream);
EVENTMANAGER_EXPORT void gbuff_serialize(std::queue<nui::events::ScreenTapGestureEvent>& evtQueue,
        const bfs::path path);
EVENTMANAGER_EXPORT void gbuff_serialize(std::queue<nui::events::ScreenTapGestureEvent>& evtQueue,
        std::ostream* stream);
EVENTMANAGER_EXPORT void gbuff_serialize(std::queue<nui::events::PinchGestureEvent>& evtQueue,
        const bfs::path path);
EVENTMANAGER_EXPORT void gbuff_serialize(std::queue<nui::events::PinchGestureEvent>& evtQueue,
        std::ostream* stream);
EVENTMANAGER_EXPORT void gbuff_serialize(std::queue<nui::events::GrabGestureEvent>& evtQueue,
        const bfs::path path);
EVENTMANAGER_EXPORT void gbuff_serialize(std::queue<nui::events::GrabGestureEvent>& evtQueue,
        std::ostream* stream);


/**
 * \brief Using Google Protocol buffers to serialize.
 * \param queue - Attention event queue
 * \param path - path to store
 */
EVENTMANAGER_EXPORT void  gbuff_serialize(std::queue<nui::events::UserAttentionEvent>& evtQueue, const bfs::path path);

/**
 * \brief Using Google Protocol buffers to serialize.
 * \param queue - Attention event queue
 * \param stream - output stream
 */
EVENTMANAGER_EXPORT void gbuff_serialize(std::queue<nui::events::UserAttentionEvent>& evtQueue,
        std::ostream* stream);

/**
 * \brief Using Google Protocol buffers to serialize.
 * \param queue - Attention myo queue
 * \param path - path to store
 */
EVENTMANAGER_EXPORT void  gbuff_serialize(std::queue<nui::events::MyoEvent>& evtQueue, const bfs::path path);

/**
 * \brief Using Google Protocol buffers to serialize.
 * \param queue - Attention myo queue
 * \param stream - output stream
 */
EVENTMANAGER_EXPORT void gbuff_serialize(std::queue<nui::events::MyoEvent>& evtQueue,
        std::ostream* stream);