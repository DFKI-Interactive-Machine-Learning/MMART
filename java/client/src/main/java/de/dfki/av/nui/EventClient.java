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
package de.dfki.av.nui;

import java.util.List;

import org.apache.log4j.BasicConfigurator;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import de.dfki.av.event.LeapMotionGestureListener;
import de.dfki.av.event.LeapMotionListener;
import de.dfki.av.event.SystemEventListener;
import de.dfki.av.event.UserAttentionEventListener;
import de.dfki.av.math.Vector3f;
import de.dfki.av.nui.Interface.AttentionArea;
import de.dfki.av.nui.Interface.Gesture.GestureState;
import de.dfki.av.nui.Interface.Gesture.Side;
import de.dfki.av.nui.Interface.Hand;
import de.dfki.av.nui.Interface.SystemMessage.MessageType;

public class EventClient implements LeapMotionGestureListener,
		LeapMotionListener, SystemEventListener, UserAttentionEventListener {

	/**
	 * Logger.
	 */
	private static Logger log = LoggerFactory.getLogger(EventClient.class);

	public static void main(String[] args) {
		BasicConfigurator.configure();
		EventClient c = new EventClient();
		final Receiver r = new Receiver("localhost", 50111);
		r.addLeapMotionGestureListener(c);
		r.addLeapMotionListener(c);
		r.addUserAttentionListener(c);
		r.addSystemEventListener(c);
		r.start();

	}

	@Override
	public void handTrackingEvent(long timestamp, List<Hand> hands) {

	}

	@Override
	public final void swipeGesture(int id, long timestamp, Side side,
			GestureState state, float confidence, Vector3f dir, float speed) {
		log.debug("Swipe detected [id] : " + id + " [timestamp] :" + timestamp
				+ " [side] : " + side.toString() + " [state] : "
				+ state.toString() + " [confidence] : " + confidence
				+ " [direction] : " + dir.toString() + " [speed] : " + speed);
	}

	@Override
	public final void tapGesture(int id, long timestamp, Side side,
			GestureState state, float confidence, Vector3f dir, Vector3f pos) {
		log.debug("Tap detected [id] : " + id + " [timestamp] :" + timestamp
				+ " [side] : " + side.toString() + " [state] : "
				+ state.toString() + " [confidence] : " + confidence
				+ " [direction] : " + dir.toString() + " [position] : "
				+ pos.toString());
	}

	@Override
	public final void pinchGesture(int id, long timestamp, Side side,
			GestureState state, float confidence, float strength) {
		log.debug("Pinch detected [id] : " + id + " [timestamp] :" + timestamp
				+ " [side] : " + side.toString() + " [state] : "
				+ state.toString() + " [confidence] : " + confidence
				+ " [strength] : " + strength);
	}

	@Override
	public final void grabGesture(int id, long timestamp, Side side,
			GestureState state, float confidence, float strength) {

		log.debug("Grab detected [id] : " + id + " [timestamp] :" + timestamp
				+ " [side] : " + side.toString() + " [state] : "
				+ state.toString() + " [confidence] : " + confidence
				+ " [strength] : " + strength);

	}

	@Override
	public final void circleGesture(int id, long timestamp, Side side,
			GestureState state, float confidence, float radius, float progress,
			float angle, boolean clockwise) {
		log.debug("Circle detected [id] : " + id + " [timestamp] :" + timestamp
				+ " [side] : " + side.toString() + " [state] : "
				+ state.toString() + " [confidence] : " + confidence
				+ " [radius] : " + radius + " [progress] : " + progress
				+ " [angle] : " + angle + " [clockwise] : " + clockwise);

	}

	@Override
	public void gestureAction(int id, long timestamp, Side side,
			GestureState state, float confidence, String action) {
		log.debug("Circle detected [id] : " + id + " [timestamp] :" + timestamp
				+ " [side] : " + side.toString() + " [state] : "
				+ state.toString() + " [confidence] : " + confidence
				+ " [action] : " + action);
		
	}

	@Override
	public void userAttention(long timestamp, List<AttentionArea> area) {
		log.debug("User attention  [timestamp] :" + timestamp);
		for (final AttentionArea ae : area)
		{
			log.debug("   Attention area  [area] :" + ae.getLabel() + " [probability] : " + ae.getProbability());
		}
		
	}

	@Override
	public void systemEvent(int id, long timestamp, MessageType type,
			String category, String message) {
		log.debug("System event [id] : " + id + " [timestamp] :" + timestamp
				+ " [side] : " + type.toString() + " [category] : "
				+ category.toString() + " [message] : " + message);
		
	}

}
