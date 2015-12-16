#include "serialize/GPufferSerializer.hpp"
#include <fstream>
#include <iostream>

using namespace nui::stream;
using namespace nui::events;

void gbuff_serialize(LeapTrackingEvent& evt, const bfs::path path) {
	std::fstream output(path.string().c_str(),
			std::ios::out | std::ios::trunc | std::ios::binary);
	gbuff_serialize(evt, &output);
}

void gbuff_serialize(LeapTrackingEvent& evt, std::ostream* stream) {
	FrameList list;
	std::queue<LeapTrackingEvent>* qu = new std::queue<LeapTrackingEvent>();
	qu->push(evt);
    gbuff_serialize(*qu, stream);
	delete qu;

}

void gbuff_serialize(std::queue<LeapTrackingEvent>& evtQueue,
		const bfs::path path) {
	std::fstream output(path.string().c_str(),
			std::ios::out | std::ios::trunc | std::ios::binary);
	gbuff_serialize(evtQueue, &output);
}

void gbuff_serialize(std::queue<LeapTrackingEvent>& evtQueue,
		std::ostream* stream) {
	FrameList list;
	while (!evtQueue.empty()) {
		LeapTrackingEvent evt = evtQueue.front();
		evtQueue.pop();
		Frame* f = list.add_frames();
		f->set_timestamp(evt.timestamp);
		leap2frame(evt, f);
	}
	if (!list.SerializeToOstream(stream)) {
		BOOST_LOG_TRIVIAL(error) << "FAILED TO WRITE TO CACHE!!" << std::endl;
	}
}

void gbuff_serialize(std::queue<CircleGestureEvent>& evtQueue,
        const bfs::path path) {
    std::fstream output(path.string().c_str(),
            std::ios::out | std::ios::trunc | std::ios::binary);
    gbuff_serialize(evtQueue, &output);
}

void gbuff_serialize(std::queue<CircleGestureEvent>& evtQueue,
        std::ostream* stream) {
    GestureList list;
    while (!evtQueue.empty()) {
        CircleGestureEvent evt = evtQueue.front();
        circle2gesture(evt, list.add_gestures());
        evtQueue.pop();
    }
    if (!list.SerializeToOstream(stream)) {
        BOOST_LOG_TRIVIAL(error) << "FAILED TO WRITE TO CACHE!!" << std::endl;
    }
}
void gbuff_serialize(std::queue<LeapActionGestureEvent>& evtQueue,
        const bfs::path path) {
    std::fstream output(path.string().c_str(),
            std::ios::out | std::ios::trunc | std::ios::binary);
    gbuff_serialize(evtQueue, &output);
}

void gbuff_serialize(std::queue<LeapActionGestureEvent>& evtQueue,
        std::ostream* stream) {
    GestureList list;
    while (!evtQueue.empty()) {
        LeapActionGestureEvent evt = evtQueue.front();
        action2gesture(evt, list.add_gestures());
        evtQueue.pop();
    }
    if (!list.SerializeToOstream(stream)) {
        BOOST_LOG_TRIVIAL(error) << "FAILED TO WRITE TO CACHE!!" << std::endl;
    }
}

void gbuff_serialize(std::queue<SwipeGestureEvent>& evtQueue,
        const bfs::path path) {
    std::fstream output(path.string().c_str(),
            std::ios::out | std::ios::trunc | std::ios::binary);
    gbuff_serialize(evtQueue, &output);
}

void gbuff_serialize(std::queue<SwipeGestureEvent>& evtQueue,
        std::ostream* stream) {
    GestureList list;
    while (!evtQueue.empty()) {
        SwipeGestureEvent evt = evtQueue.front();
        swipe2gesture(evt, list.add_gestures());
        evtQueue.pop();
    }
    if (!list.SerializeToOstream(stream)) {
        BOOST_LOG_TRIVIAL(error)<< "FAILED TO WRITE TO CACHE!!" << std::endl;
    }
}
void gbuff_serialize(std::queue<KeyTapGestureEvent>& evtQueue,
        const bfs::path path) {
    std::fstream output(path.string().c_str(),
            std::ios::out | std::ios::trunc | std::ios::binary);
    gbuff_serialize(evtQueue, &output);
}

void gbuff_serialize(std::queue<KeyTapGestureEvent>& evtQueue,
        std::ostream* stream) {
    GestureList list;
    while (!evtQueue.empty()) {
        KeyTapGestureEvent evt = evtQueue.front();
        //TODO
        evtQueue.pop();
    }
    if (!list.SerializeToOstream(stream)) {
        BOOST_LOG_TRIVIAL(error)<< "FAILED TO WRITE TO CACHE!!" << std::endl;
    }
}
void gbuff_serialize(std::queue<ScreenTapGestureEvent>& evtQueue,
        const bfs::path path) {
    std::fstream output(path.string().c_str(),
            std::ios::out | std::ios::trunc | std::ios::binary);
    gbuff_serialize(evtQueue, &output);
}

void gbuff_serialize(std::queue<ScreenTapGestureEvent>& evtQueue,
        std::ostream* stream) {
    GestureList list;
    while (!evtQueue.empty()) {
        ScreenTapGestureEvent evt = evtQueue.front();
        //TODO
        evtQueue.pop();
    }
    if (!list.SerializeToOstream(stream)) {
        BOOST_LOG_TRIVIAL(error)<< "FAILED TO WRITE TO CACHE!!" << std::endl;
    }
}

void gbuff_serialize(std::queue<PinchGestureEvent>& evtQueue,
        const bfs::path path) {
    std::fstream output(path.string().c_str(),
            std::ios::out | std::ios::trunc | std::ios::binary);
    gbuff_serialize(evtQueue, &output);
}

void gbuff_serialize(std::queue<PinchGestureEvent>& evtQueue,
        std::ostream* stream) {
    GestureList list;
    while (!evtQueue.empty()) {
        PinchGestureEvent evt = evtQueue.front();
        pinch2gesture(evt, list.add_gestures());
        evtQueue.pop();
    }
    if (!list.SerializeToOstream(stream)) {
        BOOST_LOG_TRIVIAL(error) << "FAILED TO WRITE TO CACHE!!" << std::endl;
    }
}

void gbuff_serialize(std::queue<GrabGestureEvent>& evtQueue,
        const bfs::path path) {
    std::fstream output(path.string().c_str(),
            std::ios::out | std::ios::trunc | std::ios::binary);
    gbuff_serialize(evtQueue, &output);
}

void gbuff_serialize(std::queue<GrabGestureEvent>& evtQueue,
        std::ostream* stream) {
    GestureList list;
    while (!evtQueue.empty()) {
        GrabGestureEvent evt = evtQueue.front();
        grab2gesture(evt, list.add_gestures());
        evtQueue.pop();
    }
    if (!list.SerializeToOstream(stream)) {
        BOOST_LOG_TRIVIAL(error)<< "FAILED TO WRITE TO CACHE!!" << std::endl;
    }
}

void gbuff_serialize(std::queue<UserAttentionEvent>& evtQueue,
        const bfs::path path) {
    std::fstream output(path.string().c_str(),
            std::ios::out | std::ios::trunc | std::ios::binary);
    gbuff_serialize(evtQueue, &output);
}

void gbuff_serialize(std::queue<nui::events::UserAttentionEvent>& evtQueue,
        std::ostream* stream) {
    AttentionList list;
    while (!evtQueue.empty()) {
        UserAttentionEvent evt = evtQueue.front();
        evtQueue.pop();
        UserAttention* a = list.add_attentions();
        a->set_timestamp(evt.timestamp);
        attention2attention(evt, a);
    }
    if (!list.SerializeToOstream(stream)) {
        BOOST_LOG_TRIVIAL(error) << "FAILED TO WRITE TO CACHE!!" << std::endl;
    }
}

void  gbuff_serialize(std::queue<nui::events::MyoEvent>& evtQueue, const bfs::path path)
{
	std::fstream output(path.string().c_str(),
            std::ios::out | std::ios::trunc | std::ios::binary);
    gbuff_serialize(evtQueue, &output);
}

void gbuff_serialize(std::queue<nui::events::MyoEvent>& evtQueue,
        std::ostream* stream)
{
	MyoList list;
	while (!evtQueue.empty()) {
		MyoEvent evt = evtQueue.front();
		evtQueue.pop();
		MyoFrame* a = list.add_frames();
		myo2myoframe(evt, a);
	}
	try {
		if (!list.SerializeToOstream(stream)) {
			BOOST_LOG_TRIVIAL(error) << "FAILED TO WRITE TO CACHE!!" << std::endl;
		}
	} catch(std::exception e) {
		BOOST_LOG_TRIVIAL(error) << e.what() << std::endl;
	}
}