#include "eventmanager/Modality.hpp"
#include <boost/log/trivial.hpp>

using namespace nui::events;

void
IModality::run()
{
	boost::unique_lock<boost::mutex> lock(m_state_mutex);
	if(m_state == NOT_INITIALIZED)
	{
		if ( NUI_FAILED(initialize()) )
		{
			BOOST_LOG_TRIVIAL(error) << "FAILED to initialize modality [name] : " << m_name << " -> ABORTED";
			unInitialize();
			return; // abort thread
		}
	}
	lock.release()->unlock();
	BOOST_LOG_TRIVIAL(info) << "Modality [name] : " << m_name << " is now active.";
	m_state = RUNNING; // now running
	m_start = boost::posix_time::microsec_clock::universal_time();
	m_updates = 0;
	if(m_mode == NONBLOCKING)
	{
		while(m_state != STOPPED && m_executiontime > 0)// execution time == 0 -> selftriggered
		{
			// process data
			if(m_state == RUNNING )
			{
				// begin
				timestamp_t t1(boost::posix_time::microsec_clock::universal_time());
				process(m_eventqueue);
				while(!m_eventqueue.empty())
				{
					NUIEventPtr ptr = NUIEVENT_PTR(m_eventqueue.front());
					fireEvent(ptr);
					m_eventqueue.pop();
				}
				// end
				timestamp_t t2(boost::posix_time::microsec_clock::universal_time());
				// time needed for processing available data
				duration_t exe_time = t2 - t1;
				// check if we have to sleep
				if(exe_time.total_milliseconds() < m_executiontime)
				{
					SLEEP(m_executiontime - exe_time.total_milliseconds()); // trying to produce a constant frame rate
				}
				else
				{
					// add a little threshold 
					if (exe_time.total_milliseconds() + 10 < m_executiontime)
					{
						BOOST_LOG_TRIVIAL(warning) << "Execution took to long (" << I2Str(exe_time.total_milliseconds() - m_executiontime) << " ms)for modality [name:= " << name() << "].";
					}
				}
			}
			else
			{
				wait();
			}
			m_updates++;
		}
	} 
	else if (m_mode == BLOCKING)
	{
		process();
	}
}


void 
IModality::wait()
{
	boost::unique_lock<boost::mutex> lock(m_mutex);
	while(m_waiting)
	{
		m_wait_condition_variable.wait(lock);
	}
}

void 
IModality::wakeup()
{
	m_waiting = false;
	m_wait_condition_variable.notify_one();
};

const float  
IModality::fps() const	
{
	timestamp_t current(boost::posix_time::microsec_clock::universal_time());
	// time needed for processing avaibable data
	duration_t exe_time = current - m_start;
	return m_updates / static_cast<float>(exe_time.total_seconds());
}


NUISTATUS
IModality::start()
{
	if ((state() != RUNNING && state() != PAUSED)) {
		if(onStart() == NUI_SUCCESS)
		{
			m_thread = new boost::thread(boost::bind(&IModality::run, this));
			BOOST_LOG_TRIVIAL(info)<< m_name << " started.";
		}
		else
		{
			BOOST_LOG_TRIVIAL(error)<< m_name << " failed to start.";
			return NUI_FAILED;
		}
		return NUI_SUCCESS;
	}
	return ERR_WRONG_STATE;
};


NUISTATUS
IModality::pause()
{
	m_state = PAUSED;
	wait();
	return NUI_SUCCESS;
};

NUISTATUS
IModality::resume()
{
	m_state = RUNNING;
	wakeup();
	return NUI_SUCCESS;
};

NUISTATUS
IModality::stop()
{
	m_state = STOPPED;
	timestamp_t current(boost::posix_time::microsec_clock::universal_time());
	// time needed for processing available data
	duration_t exe_time = current - m_start;
	wakeup();
	onStop();
	if(m_thread)
	{
		m_thread->join();
		BOOST_LOG_TRIVIAL(info) << name() << " [RUNTIME] : " << I2Str(exe_time.total_seconds()) << " s "<< I2Str(exe_time.total_milliseconds() % 1000) << " ms "
		<< " [UPDATES] : " << I2Str(m_updates) << "  [FPS] : " << I2Str(fps());
	}
	return unInitialize();
};



