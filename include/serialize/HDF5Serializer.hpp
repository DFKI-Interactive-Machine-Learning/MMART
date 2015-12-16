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
#include "interface.pb.h"
#include <boost/filesystem.hpp>
#include <queue>


/**
 * \brief Configures the serializer.
 * \param config - configuration of NUI
 */
EVENTMANAGER_EXPORT void reconfigure_serializer(NUIconfig* config);

/**
 * \brief Serializes the events to a hdf5 file.
 * \param queue - Queue of Leap motion event
 * \param path  - path to store
 */
EVENTMANAGER_EXPORT void hdf5_serialize(std::queue<nui::events::LeapTrackingEvent>& evt, const bfs::path path);

/**
 * \brief Serializes the events to a hdf5 file.
 * \param queue - Queue of Leap motion event
 * \param path  - path to store
 */
EVENTMANAGER_EXPORT void hdf5_serialize(std::queue<nui::stream::Frame>& evt, const bfs::path path);

/**
 * \brief Serializes the events to a hdf5 file.
 * \param leap - Queue of Leap motion event
 * \param gesture - Queue of Leap motion event
 * \param attention - Queue of Leap motion event
 * \param path  - path to store
 */
EVENTMANAGER_EXPORT void hdf5_serialize(std::queue<nui::stream::Frame>& ,
		                              std::queue<nui::stream::Gesture>& ,
		                              std::queue<nui::stream::UserAttention>&,
		                              const bfs::path path);

/**
 * \brief Serializes the events to a hdf5 file.
 * \param queue - myo event
 * \param path  - path to store
 */
EVENTMANAGER_EXPORT void hdf5_serialize(std::queue<nui::events::MyoEvent>& evt, const bfs::path path);

/**
 * \brief Serializes the events to a hdf5 file.
 * \param queue - Queue of Myo events
 * \param path  - path to store
 */
EVENTMANAGER_EXPORT void hdf5_serialize(std::queue<nui::stream::MyoFrame>& evt, const bfs::path path);

/**
 * \brief Serializes serialized cached events to a hdf5 file.
 * \param queue - Queue of cache file events
 * \param path  - path to store
 */
EVENTMANAGER_EXPORT void hdf5_gbuff_cache_files(std::queue<bfs::path>& evt, const bfs::path path);

