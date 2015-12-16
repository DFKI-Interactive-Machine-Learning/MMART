/*
 * This file is part of the AV-NUI project.
 * Copyright (C) 2013 DFKI GmbH. All rights reserved.
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
#ifdef WITH_USER_INTERFACE
	#include <QtCore/QtGlobal>
#endif
//define dll export
#if defined (_WIN32) 
 
#if defined(cam_EXPORTS)
#define  WEBCAM_EXPORT __declspec(dllexport)
#else
#define  WEBCAM_EXPORT __declspec(dllimport)
#endif /*cam_EXPORTS */

#if defined(common_EXPORTS)
#define  COMMON_EXPORT __declspec(dllexport)
#else
#define  COMMON_EXPORT __declspec(dllimport)
#endif /* config_EXPORTS */

#if defined(classifier_EXPORTS)
#define  CLASSIFIER_EXPORT __declspec(dllexport)
#else
#define  CLASSIFIER_EXPORT __declspec(dllimport)
#endif /* classifier_EXPORTS */

#if defined(config_EXPORTS)
#define  CONFIG_EXPORT __declspec(dllexport)
#else
#define  CONFIG_EXPORT __declspec(dllimport)
#endif /* config_EXPORTS */

#if defined(controller_EXPORTS)
#define  CONTROLLER_EXPORT __declspec(dllexport)
#else
#define  CONTROLLER_EXPORT __declspec(dllimport)
#endif /* controller_EXPORTS */

#if defined(eventmanager_EXPORTS)
#define  EVENTMANAGER_EXPORT __declspec(dllexport)
#else
#define  EVENTMANAGER_EXPORT __declspec(dllimport)
#endif /* eventmanager_EXPORTS */

#if defined(common_EXPORTS)
#define  COMMON_EXPORT __declspec(dllexport)
#else
#define  COMMON_EXPORT __declspec(dllimport)
#endif /* common_EXPORTS */

#if defined(gesture_recognition_EXPORTS)
#define  GESTURERECOGNITION_EXPORT __declspec(dllexport)
#else
#define  GESTURERECOGNITION_EXPORT __declspec(dllimport)
#endif /* gesture_EXPORTS */

#if defined(kinectwrapper_EXPORTS)
#define  KINECTWRAPPER_EXPORT __declspec(dllexport)
#else
#define  KINECTWRAPPER_EXPORT __declspec(dllimport)
#endif /* kinectwrapper_EXPORTS */

#if defined(leapmotion_EXPORTS)
#define  LEAPMOTION_EXPORT __declspec(dllexport)
#else
#define  LEAPMOTION_EXPORT __declspec(dllimport)
#endif /* leapmotion_EXPORTS */

#if defined(myo_EXPORTS)
#define  MYO_EXPORT __declspec(dllexport)
#else
#define  MYO_EXPORT __declspec(dllimport)
#endif /* myo_EXPORTS */

#if defined(logwrapper_EXPORTS)
#define  LOGWRAPPER_EXPORT __declspec(dllexport)
#else
#define  LOGWRAPPER_EXPORT __declspec(dllimport)
#endif /* logwrapper_EXPORTS */

#if defined(pmd_EXPORTS)
#define  PMDNANO_EXPORT __declspec(dllexport)
#else
#define  PMDNANO_EXPORT __declspec(dllimport)
#endif /* pmd_EXPORTS */

#if defined(intelrealsense_EXPORTS)
#define  INTELREALSENSE_EXPORT __declspec(dllexport)
#else
#define  INTELREALSENSE_EXPORT __declspec(dllimport)
#endif /* intelrealsense_EXPORTS */

#if defined(depthsensecamera_EXPORTS)
#define  DEPTHSENSECAMERA_EXPORT __declspec(dllexport)
#else
#define  DEPTHSENSECAMERA_EXPORT __declspec(dllimport)
#endif /* intelrealsense_EXPORTS */
#if defined(depthsensecamera_EXPORTS)

#define  SERIALIZER_EXPORT __declspec(dllexport)
#else
#define  SERIALIZER_EXPORT __declspec(dllimport)
#endif /*serializer_EXPORTS */

#if defined(libavnui_EXPORTS)
#define  GUI_EXPORT Q_DECL_EXPORT 
#else
#define  GUI_EXPORT Q_DECL_IMPORT 
#endif /* libavnui_EXPORTS */

#if defined(libwizardui_EXPORTS)
#define  WIZARD_UI_EXPORT Q_DECL_EXPORT 
#else
#define  WIZARD_UI_EXPORT Q_DECL_IMPORT 
#endif /* libwizardui_EXPORTS */

#if defined(viz_EXPORTS)
#define  VISUALIZER_EXPORT __declspec(dllexport)
#else
#define  VISUALIZER_EXPORT __declspec(dllimport)
#endif /* pmd_EXPORTS */

/* defined (_WIN32) */
#elif __unix__    
#if defined(bodytracking_EXPORTS)
#define  BODYTRACKING_EXPORT __attribute__((__visibility__("default")))
#else
#define  BODYTRACKING_EXPORT
#endif /* bodytracking_EXPORTS */

#if defined(classifier_EXPORTS)
#define  CLASSIFIER_EXPORT __attribute__((__visibility__("default")))
#else
#define  CLASSIFIER_EXPORT __declspec(dllimport)
#endif /* classifier_EXPORTS */

#if defined(config_EXPORTS)
#define  CONFIG_EXPORT __attribute__((__visibility__("default")))
#else
#define  CONFIG_EXPORT
#endif /* config_EXPORTS */

#if defined(controller_EXPORTS)
#define  CONTROLLER_EXPORT __attribute__((__visibility__("default")))
#else
#define  CONTROLLER_EXPORT
#endif /* controller_EXPORTS */

#if defined(eventmanager_EXPORTS)
#define  EVENTMANAGER_EXPORT __attribute__((__visibility__("default")))
#else
#define  EVENTMANAGER_EXPORT
#endif /* eventmanager_EXPORTS */

#if defined(events_EXPORTS)
#define  EVENTS_EXPORT __attribute__((__visibility__("default")))
#else
#define  EVENTS_EXPORT
#endif /* events_EXPORTS */

#if defined(gesture_EXPORTS)
#define  GESTURE_EXPORT __attribute__((__visibility__("default")))
#else
#define  GESTURE_EXPORT
#endif /* gesture_EXPORTS */

#if defined(kinectwrapper_EXPORTS)
#define  KINECTWRAPPER_EXPORT __attribute__((__visibility__("default")))
#else
#define  KINECTWRAPPER_EXPORT
#endif /* kinectwrapper_EXPORTS */

#if defined(leapmotion_EXPORTS)
#define  LEAPMOTION_EXPORT __attribute__((__visibility__("default")))
#else
#define  LEAPMOTION_EXPORT 
#endif /* leapmotion_EXPORTS */

#if defined(libavnui_EXPORTS)
#define  GUI_EXPORT __attribute__((__visibility__("default")))
#else
#define  GUI_EXPORT 
#endif /* vistragui_EXPORTS */


#if defined(webcam_EXPORTS)
#define  WEBCAM_EXPORT __attribute__((__visibility__("default")))
#else
#define  WEBCAM_EXPORT
#endif /*webcam_EXPORTS */

#if defined(pmd_EXPORTS)
#define  PMDNANO_EXPORT __attribute__((__visibility__("default")))
#else
#define  PMDNANO_EXPORT
#endif /*webcam_EXPORTS */

#if defined(kinect_EXPORTS)
#define  KINECT_EXPORT __attribute__((__visibility__("default")))
#else
#define  KINECT_EXPORT
#endif /*webcam_EXPORTS */

#if defined(serializer_EXPORTS)
#define  SERIALIZER_EXPORT __attribute__((__visibility__("default")))
#else
#define  SERIALIZER_EXPORT
#endif /*serializer_EXPORTS */
#endif
