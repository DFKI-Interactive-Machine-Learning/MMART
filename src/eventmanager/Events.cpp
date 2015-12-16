#include "eventmanager/Events.hpp"

using namespace nui::stream;
using namespace nui::events;

MyoFrame_MyoState convertMyoState(const MyoState st) {
	switch (st) {
	case NOT_CONNECT:
		return MyoFrame_MyoState_NOT_CONNECT;
	case CONNECTION_ESTABLISHED:
		return MyoFrame_MyoState_CONNECTION_ESTABLISHED;
	case CONNECTION_LOST:
		return MyoFrame_MyoState_CONNECTION_LOST;
	case BATTERY_STATUS_LOW:
		return MyoFrame_MyoState_BATTERY_STATUS_LOW;
	case CONNECTION_GOOD:
		return MyoFrame_MyoState_CONNECTION_GOOD;
	case CONNECTION_POOR:
		return MyoFrame_MyoState_CONNECTION_POOR;
	}
	return MyoFrame_MyoState_NOT_CONNECT;
}

MyoFrame_MyoPose convertMyoPose(const MyoPose st) {
	switch (st) {
	case UNKNOWN:
		return MyoFrame_MyoPose_UNKNOWN;
	case REST_GESTURE:
		return MyoFrame_MyoPose_REST_GESTURE;
	case FIST_GESTURE:
		return MyoFrame_MyoPose_FIST_GESTURE;
	case WAVE_IN_GESTURE:
		return MyoFrame_MyoPose_WAVE_IN_GESTURE;
	case WAVE_OUT_GESTURE:
		return MyoFrame_MyoPose_WAVE_OUT_GESTURE;
	case FINGERS_SPREAD_GESTURE:
		return MyoFrame_MyoPose_FINGERS_SPREAD_GESTURE;
	case DOUBLE_TAB_GESTURE:
		return MyoFrame_MyoPose_DOUBLE_TAB_GESTURE;
	}
	return MyoFrame_MyoPose_UNKNOWN;
}

Gesture_GestureState convertGestureState(const LeapGestureState st) {
	switch (st) {
	case STATE_INVALID:
		return Gesture_GestureState_STATE_INVALID;
	case STATE_START:
		return Gesture_GestureState_STATE_START;
	case STATE_UPDATE:
		return Gesture_GestureState_STATE_UPDATE;
	case STATE_STOP:
		return Gesture_GestureState_STATE_STOP;
	}
	return Gesture_GestureState_STATE_INVALID;
}

Bone_BoneName convertBoneName(const BoneName bn) {
	switch (bn) {
	case TYPE_METACARPAL:
		return Bone_BoneName_METACARPAL;
	case TYPE_PROXIMAL:
		return Bone_BoneName_PROXIMAL;
	case TYPE_INTERMEDIATE:
		return Bone_BoneName_INTERMEDIATE;
	case TYPE_DISTAL:
		return Bone_BoneName_DISTAL;
	}
	return Bone_BoneName_METACARPAL;
}

Finger_FingerName convertFingerName(const FingerName bn) {
	switch (bn) {
	case TYPE_THUMB:
		return Finger_FingerName_THUMB;
	case TYPE_INDEX:
		return Finger_FingerName_INDEX;
	case TYPE_MIDDLE:
		return Finger_FingerName_MIDDLE;
	case TYPE_RING:
		return Finger_FingerName_RING;
	case TYPE_PINKY:
		return Finger_FingerName_PINKY;
	}
	return Finger_FingerName_THUMB;
}


SystemMessage_MessageType convertMessageType(const MessageType t) {
	switch (t) {
	case MESSAGETYPE_COMMAND:
		return SystemMessage_MessageType_COMMAND;
	case MESSAGETYPE_INFO:
		return SystemMessage_MessageType_INFO;
	case MESSAGETYPE_CONFIGURATION:
		return SystemMessage_MessageType_CONFIGURATION;
	}
	return SystemMessage_MessageType_INFO;
}

void bone2bone(BoneTypePtr b, Bone* b2) {
	Vector3f* dir = new Vector3f();
	Vector3f* center = new Vector3f();
	dir->set_x(b.get()->direction.x());
	dir->set_y(b.get()->direction.y());
	dir->set_z(b.get()->direction.z());
	b2->set_allocated_direction(dir);
	center->set_x(b.get()->center.x());
	center->set_y(b.get()->center.y());
	center->set_z(b.get()->center.z());
	b2->set_allocated_center(center);
	b2->set_length(b->length);
	b2->set_width(b->width);
	b2->set_name(convertBoneName(b->name));
}

void finger2finger(FingerTypePtr f, Finger* f2) {
	Vector3f* dir = new Vector3f();
	dir->set_x(f.get()->direction.x());
	dir->set_y(f.get()->direction.y());
	dir->set_z(f.get()->direction.z());
	f2->set_allocated_direction(dir);

	Vector3f* tipPosition = new Vector3f();
	tipPosition->set_x(f.get()->tipPosition.x());
	tipPosition->set_y(f.get()->tipPosition.y());
	tipPosition->set_z(f.get()->tipPosition.z());
	f2->set_allocated_tipposition(tipPosition);

	Vector3f* tipVelocity = new Vector3f();
	tipVelocity->set_x(f.get()->tipVelocity.x());
	tipVelocity->set_y(f.get()->tipVelocity.y());
	tipVelocity->set_z(f.get()->tipVelocity.z());
	f2->set_allocated_tipvelocity(tipVelocity);

	f2->set_length(f.get()->length);
	f2->set_width(f.get()->width);
	f2->set_name(convertFingerName(f->name));

	bone2bone(f.get()->bones[TYPE_METACARPAL], f2->add_bones());
	bone2bone(f.get()->bones[TYPE_PROXIMAL], f2->add_bones());
	bone2bone(f.get()->bones[TYPE_INTERMEDIATE], f2->add_bones());
	bone2bone(f.get()->bones[TYPE_DISTAL], f2->add_bones());
}

void hand2hand(HandTypePtr h, Hand* h2) {
	h2->set_id(h.get()->id);
	h2->set_side(h.get()->side == RIGHT ? Hand_Side_RIGHT : Hand_Side_LEFT);
	h2->set_sphereradius(h.get()->sphereRadius);
	h2->set_confidence(h.get()->confidence);
	h2->set_grabstrength(h.get()->grabStrength);
	h2->set_pinchstrength(h.get()->pinchStrength);
	Vector3f* dir = new Vector3f();
	dir->set_x(h.get()->direction.x());
	dir->set_y(h.get()->direction.y());
	dir->set_z(h.get()->direction.z());
	h2->set_allocated_direction(dir);

	Vector3f* palmPosition = new Vector3f();
	palmPosition->set_x(h.get()->palmPosition.x());
	palmPosition->set_y(h.get()->palmPosition.y());
	palmPosition->set_z(h.get()->palmPosition.z());
	h2->set_allocated_palmposition(palmPosition);

	Vector3f* palmNormal = new Vector3f();
	palmNormal->set_x(h.get()->palmNormal.x());
	palmNormal->set_y(h.get()->palmNormal.y());
	palmNormal->set_z(h.get()->palmNormal.z());
	h2->set_allocated_palmnormal(palmNormal);

	Vector3f* palmVelocity = new Vector3f();
	palmVelocity->set_x(h.get()->palmVelocity.x());
	palmVelocity->set_y(h.get()->palmVelocity.y());
	palmVelocity->set_z(h.get()->palmVelocity.z());
	h2->set_allocated_palmvelocity(palmVelocity);

	Vector3f* sphereCenter = new Vector3f();
	sphereCenter->set_x(h.get()->sphereCenter.x());
	sphereCenter->set_y(h.get()->sphereCenter.y());
	sphereCenter->set_z(h.get()->sphereCenter.z());
	h2->set_allocated_spherecenter(sphereCenter);

	finger2finger(h.get()->fingers[TYPE_THUMB], h2->add_fingers());
	finger2finger(h.get()->fingers[TYPE_INDEX], h2->add_fingers());
	finger2finger(h.get()->fingers[TYPE_MIDDLE], h2->add_fingers());
	finger2finger(h.get()->fingers[TYPE_RING], h2->add_fingers());
	finger2finger(h.get()->fingers[TYPE_PINKY], h2->add_fingers());
}

void nui::events::leap2frame(nui::events::LeapTrackingEvent& evt, nui::stream::Frame* frame) {
	frame->set_timestamp(evt.timestamp);
	for (HandVector::iterator it = evt.hands.begin(); it != evt.hands.end();
			++it) {
		HandTypePtr hand = static_cast<HandTypePtr>(*it);
		Hand* h = frame->add_hands();
		hand2hand(hand, h);
	}
}
void nui::events::swipe2gesture(nui::events::SwipeGestureEvent& evt, nui::stream::Gesture* g)
{
	Vector3f* dir = new Vector3f();
	dir->set_x(evt.direction.x());
	dir->set_y(evt.direction.y());
	dir->set_z(evt.direction.z());
	g->set_id(evt.id);
	g->set_state(convertGestureState(evt.state));
	g->set_side(Gesture_Side_RIGHT);
	g->set_timestamp(evt.event_timestamp());
	g->set_side(Gesture_Side_RIGHT);
	g->set_name("swipe");
	g->SetAllocatedExtension(Gesture::swipe_direction, dir);
	g->SetExtension(Gesture::swipe_speed, evt.speed);
}

void nui::events::circle2gesture(nui::events::CircleGestureEvent& evt, nui::stream::Gesture* g)
{
	g->set_id(evt.id);
	g->set_state(convertGestureState(evt.state));
	g->set_timestamp(evt.event_timestamp());
	g->set_side(Gesture_Side_RIGHT);
	g->set_name("circle");
	g->SetExtension(Gesture::circle_angle, evt.angle);
	g->SetExtension(Gesture::circle_progress, evt.progress);
	g->SetExtension(Gesture::circle_radius, evt.radius);
	g->SetExtension(Gesture::circle_clockwise, evt.clockwise);
}
void nui::events::screentap2gesture(nui::events::ScreenTapGestureEvent& evt, nui::stream::Gesture* g)
{
	Vector3f* dir = new Vector3f();
	dir->set_x(evt.direction.x());
	dir->set_y(evt.direction.y());
	dir->set_z(evt.direction.z());
	Vector3f* tapPosition = new Vector3f();
	tapPosition->set_x(evt.position.x());
	tapPosition->set_y(evt.position.y());
	tapPosition->set_z(evt.position.z());
	g->set_id(evt.id);
	g->set_state(convertGestureState(evt.state));
	g->set_timestamp(evt.event_timestamp());
	g->set_side(Gesture_Side_RIGHT);
	g->set_name("tap");
	g->SetAllocatedExtension(Gesture::tap_direction, dir);
	g->SetAllocatedExtension(Gesture::tap_position, tapPosition);
}

void nui::events::pinch2gesture(nui::events::PinchGestureEvent& evt, nui::stream::Gesture* g)
{
	g->set_id(evt.id);
	g->set_state(convertGestureState(evt.state));
	g->set_timestamp(evt.event_timestamp());
	g->set_side(Gesture_Side_RIGHT);
	g->set_name("pinch");
	g->SetExtension(Gesture::pinch_strength, evt.pinchStrength);
}

void nui::events::grab2gesture(nui::events::GrabGestureEvent& evt, nui::stream::Gesture* g)
{
	g->set_id(evt.id);
	g->set_state(convertGestureState(evt.state));
	g->set_timestamp(evt.event_timestamp());
	g->set_side(Gesture_Side_RIGHT);
	g->set_name("grab");
	g->SetExtension(Gesture::grab_strength, evt.grabStrength);
}

void nui::events::action2gesture(LeapActionGestureEvent& evt, nui::stream::Gesture* g)
{
	g->set_id(evt.id);
	g->set_state(convertGestureState(evt.state));
	g->set_timestamp(evt.event_timestamp());
	g->set_side(Gesture_Side_RIGHT);
	g->set_name(evt.action);
}

void nui::events::system2system(SystemEvent& evt, nui::stream::SystemMessage* g)
{
	g->set_id(evt.id);
	g->set_type(convertMessageType(evt.type));
	g->set_timestamp(evt.event_timestamp());
	g->set_category(evt.category);
	g->set_message(evt.message);
}

void nui::events::myo2myoframe(MyoEvent& h, nui::stream::MyoFrame* a)
{
	a->set_timestamp(h.event_timestamp());
	a->set_id(h.id);
	a->set_state(convertMyoState(h.currentState));
	a->set_onarm(h.onArm);
	a->set_whicharm( h.whichArm == NONE ? MyoFrame_Side_NONE : (h.whichArm == RIGHT ? MyoFrame_Side_RIGHT : MyoFrame_Side_LEFT));
	a->set_isunlocked(h.isUnlocked);
	a->set_batterylevel(h.batteryLevel);
	a->set_rssi(h.rssi);
	a->set_roll(h.roll_w);
	a->set_pitch(h.pitch_w);
	a->set_yaw(h.yaw_w);
	a->set_acc_x(h.acc_x);
	a->set_acc_y(h.acc_y);
	a->set_acc_z(h.acc_z);
	a->set_gyro_x(h.gyro_x);
	a->set_gyro_y(h.gyro_y);
	a->set_gyro_z(h.gyro_z);
	a->add_emgsamples(h.emgSamples[0]);
	a->add_emgsamples(h.emgSamples[1]);
	a->add_emgsamples(h.emgSamples[2]);
	a->add_emgsamples(h.emgSamples[3]);
	a->add_emgsamples(h.emgSamples[4]);
	a->add_emgsamples(h.emgSamples[5]);
	a->add_emgsamples(h.emgSamples[6]);
	a->add_emgsamples(h.emgSamples[7]);
}

void nui::events::attention2attention(UserAttentionEvent& evt, nui::stream::UserAttention* g)
{
	g->set_timestamp(evt.timestamp);
	for(std::vector<nui::events::AttentionArea>::size_type i = 0; i != evt.areas.size(); i++) {
		nui::stream::AttentionArea* ae = g->add_areas();
		ae->set_label(evt.areas[i].label);
		ae->set_active(evt.areas[i].active);
		ae->set_distance(evt.areas[i].distance);
		ae->set_probability(evt.areas[i].probability);
		Vector3f* point = new Vector3f();
		point->set_x(evt.areas[i].point.x());
		point->set_y(evt.areas[i].point.y());
		point->set_z(evt.areas[i].point.z());
		ae->set_allocated_point(point);
		Vector3f* vtop = new Vector3f();
		vtop->set_x(evt.areas[i].vector_top.x());
		vtop->set_y(evt.areas[i].vector_top.y());
		vtop->set_z(evt.areas[i].vector_top.z());
		ae->set_allocated_vector_top(vtop);
		Vector3f* vright = new Vector3f();
		vright->set_x(evt.areas[i].vector_right.x());
		vright->set_y(evt.areas[i].vector_right.y());
		vright->set_z(evt.areas[i].vector_right.z());
		ae->set_allocated_vector_right(vright);
		Vector3f* plane = new Vector3f();
		plane->set_x(evt.areas[i].plane_normal.x());
		plane->set_y(evt.areas[i].plane_normal.y());
		plane->set_z(evt.areas[i].plane_normal.z());
		ae->set_allocated_plane_normal(plane);
	}
}
