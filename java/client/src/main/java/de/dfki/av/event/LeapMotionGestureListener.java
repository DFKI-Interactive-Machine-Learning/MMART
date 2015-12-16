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
package de.dfki.av.event;

import de.dfki.av.math.Vector3f;
import de.dfki.av.nui.Interface.Gesture.GestureState;
import de.dfki.av.nui.Interface.Gesture.Side;

/**
 * Gesture interface.
 * 
 * @author Markus Weber
 */
public interface LeapMotionGestureListener {
	/**
	 * Swipe gesture.
	 * 
	 * @param id
	 *            - id of event
	 * @param timestamp
	 *            - timestamp of event
	 * @param side
	 *            - side of the hand
	 * @param state
	 *            - state of gesture
	 * @param confidence
	 *            - confidence of recognizer
	 * 
	 * @param dir
	 *            - direction
	 * @param speed
	 *            - speed of swipe
	 */
	void swipeGesture(final int id, final long timestamp, final Side side,
			final GestureState state, final float confidence,
			final Vector3f dir, final float speed);

	/**
	 * Tap gesture.
	 * 
	 * @param id
	 *            - id of event
	 * @param timestamp
	 *            - timestamp of event
	 * @param side
	 *            - side of the hand
	 * @param state
	 *            - state of gesture
	 * @param confidence
	 *            - confidence of recognizer
	 * @param dir
	 *            - tap direction
	 * @param pos
	 *            - tap position
	 */
	void tapGesture(final int id, final long timestamp, final Side side,
			final GestureState state, final float confidence,
			final Vector3f dir, final Vector3f pos);

	/**
	 * Pinch gesture.
	 * 
	 * @param id
	 *            - id of event
	 * @param timestamp
	 *            - timestamp of event
	 * @param side
	 *            - side of the hand
	 * @param state
	 *            - state of gesture
	 * @param confidence
	 *            - confidence of recognizer
	 * @param strength
	 *            - strength of pinch
	 */
	void pinchGesture(final int id, final long timestamp, final Side side,
			final GestureState state, final float confidence,
			final float strength);

	/**
	 * Grab gesture.
	 * 
	 * @param id
	 *            - id of event
	 * @param timestamp
	 *            - timestamp of event
	 * @param side
	 *            - side of the hand
	 * @param state
	 *            - state of gesture
	 * @param confidence
	 *            - confidence of recognizer
	 * @param strength
	 *            - strenght of grab
	 */
	void grabGesture(final int id, final long timestamp, final Side side,
			final GestureState state, final float confidence,
			final float strength);

	/**
	 * Circle gesture.
	 * 
	 * @param id
	 *            - id of event
	 * @param timestamp
	 *            - timestamp of event
	 * @param side
	 *            - side of the hand
	 * @param state
	 *            - state of gesture
	 * @param confidence
	 *            - confidence of recognizer
	 * @param radius
	 *            - radius of circle
	 * @param progress
	 *            - progress of gesture
	 * @param angle
	 *            - angle of circle
	 * @param clockwise
	 *            - clockwise rotation
	 */
	void circleGesture(final int id, final long timestamp, final Side side,
			final GestureState state, final float confidence, float radius,
			float progress, float angle, boolean clockwise);
	
	/**
	 * Gesture action.
	 * @param id
	 *            - id of event
	 * @param timestamp
	 *            - timestamp of event
	 * @param side
	 *            - side of the hand
	 * @param state
	 *            - state of gesture
	 * @param confidence
	 *            - confidence of recognizer
	 * @param action - action
	 */
	void gestureAction(final int id, final long timestamp, final Side side,
			final GestureState state, final float confidence, final String action);
}
