#include "attention/AttentionMonitoring.h"


NUISTATUS AttentionMonitoring::initialize()
{

    m_attention_model = new DGroegerCV::SpatialAttentionModel();

    //driving simulator
    m_attention_model->AddTargetArea(new DGroegerCV::TargetArea("Dash board mount", cv::Vec3d(650, 20, 13), cv::Vec3d(0, 0, -290), cv::Vec3d(-1443, 0, 0), false));
    m_attention_model->AddTargetArea(new DGroegerCV::TargetArea("Dash board", cv::Vec3d(630, 20, 18.8), cv::Vec3d(0, 0, -50), cv::Vec3d(-1403, 0, 0), false));
    m_attention_model->AddTargetArea(new DGroegerCV::TargetArea("Speedometer", cv::Vec3d(440, 150, 143), cv::Vec3d(0, -130, 0), cv::Vec3d(-360, 0, 0)));
    m_attention_model->AddTargetArea(new DGroegerCV::TargetArea("Windshield", cv::Vec3d(650, 0, -277), cv::Vec3d(0, -700, 0), cv::Vec3d(-800, 0, 0)));
    m_attention_model->AddTargetArea(new DGroegerCV::TargetArea("Infotainment Screen", cv::Vec3d(0, 380, 143), cv::Vec3d(0, -180, 0), cv::Vec3d(-220, 0, 0)));
    m_attention_model->AddTargetArea(new DGroegerCV::TargetArea("Rear view mirror", cv::Vec3d(0, -500, -140), cv::Vec3d(0, -60, 0), cv::Vec3d(-220, 0, 0)));
    m_attention_model->AddTargetArea(new DGroegerCV::TargetArea("left mirror", cv::Vec3d(850, -50, -250), cv::Vec3d(0, -70, 0), cv::Vec3d(-150, 0, 0)));

    m_process_thread = new boost::thread(boost::bind(&AttentionMonitoring::processEvent, this));

    return NUI_SUCCESS;
}

NUISTATUS AttentionMonitoring::unInitialize()
{
    if (m_process_thread) {
        m_process_thread->join();
    }
    return NUI_SUCCESS;
}

void AttentionMonitoring::process(nui::events::UserAttentionEvent&)
{
}

void AttentionMonitoring::process(nui::events::HeadTrackingEvent& evt)
{
    m_event_queue_mutex.lock();
    m_tracking_event_queue.push(evt);
    m_event_queue_mutex.unlock();
}

void AttentionMonitoring::processEvent()
{
    bool run = true;
    while (run) {
        bool got_event = false;
        nui::events::HeadTrackingEvent event;
        m_event_queue_mutex.lock();
        while (!m_tracking_event_queue.empty()) {
            event = m_tracking_event_queue.front();
            m_tracking_event_queue.pop();
            got_event = true;
        }
        m_event_queue_mutex.unlock();

        if (got_event) {

            //calculate ray
            //TODO

//            cv::Vec2d right_eye_center, left_eye_center, screenPos, calibPos;

//            bool initialized = m_gaze_estimator->initialized();

//            if (m_gaze_estimator->processFrame(frame, right_eye_center, left_eye_center, screenPos, calibPos)) {

//                if (!initialized) {
//                    initialized = true;
//                    DGroegerCV::CHM* chm = m_gaze_estimator->getHeadTracker()->getCHM();
//                    //TODO
////                    emit initializedHead(chm->getRadiusX(), chm->getHeight(), chm->getRotation(), chm->getTranslation(), right_eye_center, left_eye_center);
//                }

//                //get targets
//                std::vector<DGroegerCV::AreaHit*> hits = m_gaze_estimator->GetCurrentTargets();


//                nui::events::HeadTrackingEvent *evt = new nui::events::HeadTrackingEvent();

//                evt->rotation = m_gaze_estimator->getHeadTracker()->getCHM()->getRotation();
//                evt->translation = m_gaze_estimator->getHeadTracker()->getCHM()->getTranslation();

//                evt->eye_displacement_left = m_gaze_estimator->getHeadTracker()->getCHM()->getCurrentNormalizedDisplacementVectorLeft();
//                evt->eye_displacement_right = m_gaze_estimator->getHeadTracker()->getCHM()->getCurrentNormalizedDisplacementVectorRight();

//                fireEvent(evt);

                //TODO
//                emit trackedHead(rotation_vector, translation_vector, right_eye_center, left_eye_center);

//            }
        } else {
            SLEEP(100);
        }

        // EXIT condition: queue has to be empty and the state should be stopped
        run = !(m_state == STOPPED && m_tracking_event_queue.empty());
    }

}

