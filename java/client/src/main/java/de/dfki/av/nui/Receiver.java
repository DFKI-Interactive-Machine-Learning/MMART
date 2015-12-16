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

import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.List;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import com.google.protobuf.CodedInputStream;
import com.google.protobuf.ExtensionRegistry;

import de.dfki.av.event.LeapMotionGestureListener;
import de.dfki.av.event.LeapMotionListener;
import de.dfki.av.event.SystemEventListener;
import de.dfki.av.event.UserAttentionEventListener;
import de.dfki.av.math.Vector3f;
import de.dfki.av.nui.Interface.AttentionArea;
import de.dfki.av.nui.Interface.Frame;
import de.dfki.av.nui.Interface.Gesture;
import de.dfki.av.nui.Interface.Gesture.GestureState;
import de.dfki.av.nui.Interface.Gesture.Side;
import de.dfki.av.nui.Interface.SystemMessage;
import de.dfki.av.nui.Interface.SystemMessage.MessageType;
import de.dfki.av.nui.Interface.UserAttention;

/**
 * Receiver of AV-NUI Events.
 *
 * @author Markus Weber
 */
public class Receiver extends Thread {
	/**
	 * Wait for one second before retry.
	 */
	private static final int WAIT_TIME = 1000;
	/* ------------------------- PACKAGE TYPES ----------------------------- */
	/**
	 * Constant TRANSMISSION_TYPE_LEAP_TRACKING.
	 */
	public static final int TRANSMISSION_TYPE_LEAP_TRACKING = 1;
	/**
	 * Constant TRANSMISSION_TYPE_LEAP_TRACKING.
	 */
	public static final int TRANSMISSION_TYPE_LEAP_SWIPE = TRANSMISSION_TYPE_LEAP_TRACKING + 1;
	/**
	 * Constant TRANSMISSION_TYPE_LEAP_CIRCLE.
	 */
	public static final int TRANSMISSION_TYPE_LEAP_CIRCLE = TRANSMISSION_TYPE_LEAP_SWIPE + 1;
	/**
	 * Constant TRANSMISSION_TYPE_LEAP_TAP.
	 */
	public static final int TRANSMISSION_TYPE_LEAP_TAP = TRANSMISSION_TYPE_LEAP_CIRCLE + 1;
	/**
	 * Constant TRANSMISSION_TYPE_LEAP_PINCH.
	 */
	public static final int TRANSMISSION_TYPE_LEAP_PINCH = TRANSMISSION_TYPE_LEAP_TAP + 1;
	/**
	 * Constant TRANSMISSION_TYPE_LEAP_GRAB.
	 */
	public static final int TRANSMISSION_TYPE_LEAP_GRAB = TRANSMISSION_TYPE_LEAP_PINCH + 1;
	/**
	 * Constant TRANSMISSION_TYPE_LEAP_GRAB.
	 */
	public static final int TRANSMISSION_TYPE_LEAP_ACTION_GESTURE = TRANSMISSION_TYPE_LEAP_GRAB + 1;
	/**
	 * Constant TRANSMISSION_TYPE_SYSTEM_EVENT.
	 */
	public static final int TRANSMISSION_TYPE_SYSTEM_EVENT = TRANSMISSION_TYPE_LEAP_ACTION_GESTURE + 1;
	/**
	 * Constant TRANSMISSION_TYPE_ATTENTION_EVENT.
	 */
	public static final int TRANSMISSION_TYPE_ATTENTION_EVENT = TRANSMISSION_TYPE_SYSTEM_EVENT + 1;
	/**
	 * Constant TRANSMISSION_TYPE_ATTENTION_EVENT.
	 */
	public static final int TRANSMISSION_TYPE_MYO_EVENT = TRANSMISSION_TYPE_ATTENTION_EVENT + 1;
	/**
	 * Extension registry.
	 */
	private ExtensionRegistry registry;
	/**
	 * Logger.
	 */
	private static Logger log = LoggerFactory.getLogger(Receiver.class);

	/**
	 * TCP socket listening for incoming data.
	 */
	private Socket client = null;
	/**
	 * Server host address.
	 */
	private final String host;
	/**
	 * Server port.
	 */
	private final int port;
	/**
	 * Flag if Receiver is active.
	 */
	private boolean active;
	/**
	 * Listener for raw leap events.
	 */
	private final List<LeapMotionListener> leaplistener = new ArrayList<>();
	/**
	 * Listener for raw leap gesture.
	 */
	private final List<LeapMotionGestureListener> gesturelistener = new ArrayList<>();

	/**
	 * Listener for system events.
	 */
	private final List<SystemEventListener> systemlistener = new ArrayList<>();

	/**
	 * Listener for user attention listener.
	 */
	private final List<UserAttentionEventListener> userattentionlistener = new ArrayList<>();

	/**
	 * Receiver with host and port.
	 *
	 * @param avhost
	 *            - host of AV-NUI server
	 * @param avport
	 *            - port of AV-NUI tracking server
	 */
	public Receiver(final String avhost, final int avport) {
		this.host = avhost;
		this.port = avport;
		registry = ExtensionRegistry.newInstance();
		Interface.registerAllExtensions(registry);
	}

	/**
	 * Connects with streaming service.
	 * 
	 * @return flag if connection is successfully established
	 * @throws UnknownHostException
	 *             - Server name or IP trouble
	 */
	public final boolean connect() throws UnknownHostException {
		try {
			client = new Socket(this.host, this.port);
		} catch (final IOException e) {
			log.error("Connection failed.", e);
			return false;
		}
		return client.isConnected();
	}

	@Override
	public final void run() {
		boolean wait = true;
		while (wait) {
			try {
				while (!connect()) {
					log.debug("Waiting for server. [host] :" + host
							+ " [port]: " + port);
					Thread.sleep(WAIT_TIME);
				}
			} catch (UnknownHostException | InterruptedException e1) {
				return;
			}
			active = true;
			try {
				InputStream in = client.getInputStream();
				while (active) {
					try {
						CodedInputStream cin = CodedInputStream.newInstance(in);
						int t = cin.readInt32();
						int l = cin.readInt32();
						int i = 0;
						if (l > 0) {
							byte[] buffer = new byte[l];
							while (i < l) {
								buffer[i] = cin.readRawByte();
								i++;
							}
							parseBuffer(t, buffer);
						}
					} catch (final Exception e) {
						log.error("Parsing of Protobuf message failed.", e);
					}
				}
			} catch (final IOException e) {
				log.error(e.getMessage());
			} finally {
				if (client != null) {
					try {
						client.close();
					} catch (IOException e) {
						log.error(e.getMessage());
					}
				}
			}
		}
	}

	/**
	 * Parsing buffer.
	 * 
	 * @param mtype
	 *            - message type
	 * @param buffer
	 *            - message buffer
	 */
	private void parseBuffer(final int mtype, final byte[] buffer) {
		Gesture g;
		switch (mtype) {
		case TRANSMISSION_TYPE_LEAP_TRACKING:
			Frame f;
			try {
				f = Frame.parseFrom(buffer);
				fireLeapFrame(f);
			} catch (final Exception e) {
				log.error("Parsing of leap frame failed.", e);
			}
			break;
		case TRANSMISSION_TYPE_LEAP_SWIPE:
			try {
				g = Gesture.parseFrom(new ByteArrayInputStream(buffer),
						registry);

				final de.dfki.av.nui.Interface.Vector3f dir = g
						.getExtension(Gesture.swipeDirection);

				fireSwipeGesture(g.getId(), g.getTimestamp(), g.getSide(),
						g.getState(), g.getConfidence(),
						new Vector3f(dir.getX(), dir.getY(), dir.getZ()),
						g.getExtension(Gesture.swipeSpeed));
			} catch (final Exception e) {
				log.error("Parsing of gesture frame failed. [Exception]", e);
			}
			break;
		case TRANSMISSION_TYPE_LEAP_CIRCLE:
			try {
				g = Gesture.parseFrom(new ByteArrayInputStream(buffer),
						registry);
				fireCircleGesture(g.getId(), g.getTimestamp(), g.getSide(),
						g.getState(), g.getConfidence(),
						g.getExtension(Gesture.circleRadius),
						g.getExtension(Gesture.circleProgress),
						g.getExtension(Gesture.circleAngle),
						g.getExtension(Gesture.circleClockwise));
			} catch (final Exception e) {
				log.error("Parsing of gesture frame failed. [Exception]", e);
			}
			break;
		case TRANSMISSION_TYPE_LEAP_TAP:
			try {
				g = Gesture.parseFrom(new ByteArrayInputStream(buffer),
						registry);
				final de.dfki.av.nui.Interface.Vector3f dir = g
						.getExtension(Gesture.tapDirection);
				final de.dfki.av.nui.Interface.Vector3f pos = g
						.getExtension(Gesture.tapPosition);
				fireScreenTapGesture(g.getId(), g.getTimestamp(), g.getSide(),
						g.getState(), g.getConfidence(),
						new Vector3f(dir.getX(), dir.getY(), dir.getZ()),
						new Vector3f(pos.getX(), pos.getY(), pos.getZ()));
			} catch (final Exception e) {
				log.error("Parsing of gesture frame failed. [Exception]", e);
			}
			break;
		case TRANSMISSION_TYPE_LEAP_PINCH:
			try {
				g = Gesture.parseFrom(new ByteArrayInputStream(buffer),
						registry);
				firePinchGesture(g.getId(), g.getTimestamp(), g.getSide(),
						g.getState(), g.getConfidence(),
						g.getExtension(Gesture.pinchStrength));
			} catch (final Exception e) {
				log.error("Parsing of gesture frame failed. [Exception]", e);
			}
			break;
		case TRANSMISSION_TYPE_LEAP_GRAB:
			try {
				g = Gesture.parseFrom(new ByteArrayInputStream(buffer),
						registry);
				fireGrabGesture(g.getId(), g.getTimestamp(), g.getSide(),
						g.getState(), g.getConfidence(),
						g.getExtension(Gesture.grabStrength));
			} catch (final Exception e) {
				log.error("Parsing of gesture frame failed. [Exception]", e);
			}
			break;
		case TRANSMISSION_TYPE_LEAP_ACTION_GESTURE:
			try {
				g = Gesture.parseFrom(new ByteArrayInputStream(buffer),
						registry);
				fireActionGesture(g.getId(), g.getTimestamp(), g.getSide(),
						g.getState(), g.getConfidence(), g.getName());
			} catch (final Exception e) {
				log.error("Parsing of gesture frame failed. [Exception]", e);
			}
			break;
		case TRANSMISSION_TYPE_SYSTEM_EVENT:
			try {
				final SystemMessage s = SystemMessage.parseFrom(
						new ByteArrayInputStream(buffer), registry);
				fireSystemEvent(s.getId(), s.getTimestamp(), s.getType(),
						s.getCategory(), s.getMessage());
				handleSystemEvent(s);
			} catch (final Exception e) {
				log.error("Parsing of system message failed. [Exception]", e);
			}
			break;
		case TRANSMISSION_TYPE_ATTENTION_EVENT:
			try {
				final UserAttention ue = UserAttention.parseFrom(
						new ByteArrayInputStream(buffer), registry);
				fireUserAttention(ue.getTimestamp(), ue.getAreasList());
			} catch (final Exception e) {
				log.error("Parsing of user attention failed. [Exception]", e);
			}
			break;
            case TRANSMISSION_TYPE_MYO_EVENT:
            try {
                final Interface.MyoFrame me = Interface.MyoFrame.parseFrom(
                        new ByteArrayInputStream(buffer), registry);
                fireMyoFrame(me.getTimestamp(), me);
            } catch (final Exception e) {
                log.error("Parsing of user attention failed. [Exception]", e);
            }
		default:
			break;
		}

	}

    private void fireMyoFrame(long timestamp, Interface.MyoFrame me) {
    }

    /**
	 * Handles system events
	 * 
	 * @param s
	 */
	private void handleSystemEvent(final SystemMessage s) {
		if ("EVENTMANAGER".equalsIgnoreCase(s.getCategory())
				&& "SHUTDOWN".equalsIgnoreCase(s.getMessage())) {
			active = false; // we shut down
		}

	}

	/**
	 * Fires leap frame event.
	 * 
	 * @param f
	 *            - leap frame
	 */
	private void fireLeapFrame(final Frame f) {
		for (final LeapMotionListener l : this.leaplistener) {
			l.handTrackingEvent(f.getTimestamp(), f.getHandsList());
		}
	}

	/**
	 * Fire swipe gesture.
	 * 
	 * @param id
	 *            - id
	 * @param timestamp
	 *            - recording timestamp
	 * @param side
	 *            - hand side
	 * @param state
	 *            - state of the gesture
	 * @param confidence
	 *            - confidence of the recognizer
	 * @param dir
	 *            - direction vector
	 * @param speed
	 *            - speed vector
	 */
	private void fireSwipeGesture(final int id, final long timestamp,
			final Side side, final GestureState state, final float confidence,
			final Vector3f dir, final float speed) {
		for (final LeapMotionGestureListener l : this.gesturelistener) {
			l.swipeGesture(id, timestamp, side, state, confidence, dir, speed);
		}
	}

	/**
	 * Fire screen tap gesture.
	 * 
	 * @param id
	 *            - id
	 * @param timestamp
	 *            - recording timestamp
	 * @param side
	 *            - hand side
	 * @param state
	 *            - state of the gesture
	 * @param confidence
	 *            - confidence of the recognizer
	 * @param dir
	 *            - direction vector
	 * @param pos
	 *            - position vector
	 */
	private void fireScreenTapGesture(final int id, final long timestamp,
			final Side side, final GestureState state, final float confidence,
			final Vector3f dir, final Vector3f pos) {
		for (final LeapMotionGestureListener l : this.gesturelistener) {
			l.tapGesture(id, timestamp, side, state, confidence, dir, pos);
		}
	}

	/**
	 * Fire pinch gesture.
	 * 
	 * @param id
	 *            - id
	 * @param timestamp
	 *            - recording timestamp
	 * @param side
	 *            - hand side
	 * @param state
	 *            - state of the gesture
	 * @param confidence
	 *            - confidence of the recognizer
	 * @param strength
	 *            - strength of pinch
	 */
	private void firePinchGesture(final int id, final long timestamp,
			final Side side, final GestureState state, final float confidence,
			final float strength) {
		for (final LeapMotionGestureListener l : this.gesturelistener) {
			l.pinchGesture(id, timestamp, side, state, confidence, strength);
		}
	}

	/**
	 * Fire action gesture.
	 * 
	 * @param id
	 *            - id
	 * @param timestamp
	 *            - recording timestamp
	 * @param side
	 *            - hand side
	 * @param state
	 *            - state of the gesture
	 * @param confidence
	 *            - confidence of the recognizer
	 * @param action
	 *            - action
	 */
	private void fireActionGesture(final int id, final long timestamp,
			final Side side, final GestureState state, final float confidence,
			final String action) {
		for (final LeapMotionGestureListener l : this.gesturelistener) {
			l.gestureAction(id, timestamp, side, state, confidence, action);
		}
	}

	/**
	 * Fire grab gesture.
	 * 
	 * @param id
	 *            - id
	 * @param timestamp
	 *            - recording timestamp
	 * @param side
	 *            - hand side
	 * @param state
	 *            - state of the gesture
	 * @param confidence
	 *            - confidence of the recognizer
	 * @param strength
	 *            - strength of grab
	 */
	private void fireGrabGesture(final int id, final long timestamp,
			final Side side, final GestureState state, final float confidence,
			final float strength) {
		for (final LeapMotionGestureListener l : this.gesturelistener) {
			l.grabGesture(id, timestamp, side, state, confidence, strength);
		}
	}

	/**
	 * Fire circle gesture.
	 * 
	 * @param id
	 *            - id
	 * @param timestamp
	 *            - recording timestamp
	 * @param side
	 *            - hand side
	 * @param state
	 *            - state of the gesture
	 * @param confidence
	 *            - confidence of the recognizer
	 * @param radius
	 *            - radius of circle
	 * @param progress
	 *            - progress of radius
	 * @param angle
	 *            - angle
	 * @param clockwise
	 *            - rotation is clockwise
	 */
	private void fireCircleGesture(final int id, final long timestamp,
			final Side side, final GestureState state, final float confidence,
			final float radius, final float progress, final float angle,
			final boolean clockwise) {
		for (final LeapMotionGestureListener l : this.gesturelistener) {
			l.circleGesture(id, timestamp, side, state, confidence, radius,
					progress, angle, clockwise);
		}
	}

	/**
	 * Fire User Attention events.
	 * 
	 * @param timestamp
	 * @param area
	 */
	private void fireUserAttention(final long timestamp,
			final List<AttentionArea> area) {
		for (final UserAttentionEventListener l : this.userattentionlistener) {
			l.userAttention(timestamp, area);
		}
	}

	/**
	 * Fire system events.
	 * 
	 * @param id
	 *            -- ID
	 * @param timestamp
	 *            -- time of event
	 * @param type
	 *            -- type of event
	 * @param category
	 *            -- category of message
	 * @param message
	 *            -- message
	 */
	private void fireSystemEvent(final int id, final long timestamp,
			final MessageType type, final String category, final String message) {
		for (final SystemEventListener l : this.systemlistener) {
			l.systemEvent(id, timestamp, type, category, message);
		}
	}

	/**
	 * Adds a LeapMotion hand listener.
	 *
	 * @param lis
	 *            - listener
	 */
	public final void addLeapMotionListener(final LeapMotionListener lis) {
		this.leaplistener.add(lis);
	}

	/**
	 * Adds a LeapMotion gesture listener.
	 *
	 * @param lis
	 *            - listener
	 */
	public final void addLeapMotionGestureListener(
			final LeapMotionGestureListener lis) {
		this.gesturelistener.add(lis);
	}

	/**
	 * Adds a LeapMotion hand listener.
	 *
	 * @param lis
	 *            - listener
	 */
	public final void removeLeapMotionListener(final LeapMotionListener lis) {
		this.leaplistener.remove(lis);
	}

	/**
	 * Adds a LeapMotion gesture listener.
	 *
	 * @param lis
	 *            - listener
	 */
	public final void removeLeapMotionGestureListener(
			final LeapMotionGestureListener lis) {
		this.gesturelistener.add(lis);
	}

	/**
	 * Adds a system event listener.
	 *
	 * @param lis
	 *            - listener
	 */
	public final void addSystemEventListener(final SystemEventListener lis) {
		this.systemlistener.add(lis);
	}

	/**
	 * Adds a user attention listener.
	 *
	 * @param lis
	 *            - listener
	 */
	public final void addUserAttentionListener(
			final UserAttentionEventListener lis) {
		this.userattentionlistener.add(lis);
	}

	/**
	 * Adds a system event listener.
	 *
	 * @param lis
	 *            - listener
	 */
	public final void removeSystemEventListener(final SystemEventListener lis) {
		this.systemlistener.remove(lis);
	}

	/**
	 * Adds a LeapMotion gesture listener.
	 *
	 * @param lis
	 *            - listener
	 */
	public final void removeUserAttentionListener(
			final UserAttentionEventListener lis) {
		this.userattentionlistener.add(lis);
	}

}
