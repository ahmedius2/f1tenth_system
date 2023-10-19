#include <string>
#include <sstream>
#include <iostream>
#include <pthread.h>
#include <sched.h>
#include <sys/resource.h>
#include <time.h>
#include <stdlib.h>
#include <fstream>
#include <thread>
#include <chrono>
#include <list>
#include <mutex>
#include <atomic>
#include <malloc.h>
#include <sys/mman.h>

#include <rclcpp/rclcpp.h>
//#include <ros/callback_queue.h>
#include <rosgraph_msgs/msg/clock.hpp>
#include <assert.h>
#include <sched_server/sched_client.hpp>

#include "time_profiling_spinner/time_profiling_spinner.h"

TimeProfilingSpinner::TimeProfilingSpinner(
        OperationMode op_mode,
        double callbackCheckFrequency,
        bool useCompanionThread,
        std::function<void()> funcToCall,
        std::string fname_post)
    : cb_prof_nh(std::make_shared<rclcpp::Node>("callback_profiling")),
      timing_nh(std::make_shared<rclcpp::Node>("timing")),
      period_id(0),
      funcToCall_(funcToCall),
      fname_post_(fname_post)
{
    useCompanionThread = false; // ignore this for now

    bool startSynchronized;
    // Fetching parameters in ROS 2
    timing_nh->declare_parameter<bool>("sync_start", false);
	timing_nh->declare_parameter<double>("default_period_hz", 10.0);
    
    timing_nh->get_parameter("sync_start", startSynchronized, false);
    synchronizedStart_ = startSynchronized;

    op_mode_ = op_mode;
    if(op_mode_ == OperationMode::CHAIN_HEAD){
        op_mode_ = synchronizedStart_ ?
          OperationMode::PERIODIC : OperationMode::RUN_CB_ON_ARRIVAL;
    }

    double def_cb_chk_freq;
    // Fetching parameters in ROS 2
    timing_nh->get_parameter("default_period_hz", def_cb_chk_freq, 10.0);

    bool udef = (callbackCheckFrequency == USE_DEFAULT_CALLBACK_FREQ);
    callbackCheckFrequency_ = udef ? def_cb_chk_freq : callbackCheckFrequency;

    // Logging in ROS 2
    RCLCPP_INFO(timing_nh->get_logger(), "Callback frequency: %f", callbackCheckFrequency_);

    perfProfileInit();
}

void TimeProfilingSpinner::perfProfileInit()
{
    auto event_id = -1;

    struct perf_event_attr pe;
    std::memset(&pe, 0, sizeof(struct perf_event_attr));
    pe.size = sizeof(struct perf_event_attr);
    pe.disabled = 1;
    pe.exclude_kernel = 1;
    pe.exclude_hv = 1;
    // pe.pinned = 0;
    pe.read_format = PERF_FORMAT_GROUP | PERF_FORMAT_ID;

    // HARDWARE EVENTS
    pe.type = PERF_TYPE_HARDWARE;

    pe.config = PERF_COUNT_HW_CPU_CYCLES;
    perfGroupFd = perf_event_open(&pe, 0, -1, -1, 0);
    if(perfGroupFd == -1) {
        RCLCPP_ERROR(rclcpp::get_logger("TimeProfilingSpinner"), "Problem opening perf hw cpu cycles: %s", std::strerror(errno));
        // Consider throwing an exception or returning an error code here...
    }
    ioctl(perfGroupFd, PERF_EVENT_IOC_ID, &event_id);
    perfEventIDsMap.emplace(event_id, "cpu_cycles");

    pe.config = PERF_COUNT_HW_INSTRUCTIONS;
    auto fd = perf_event_open(&pe, 0, -1, perfGroupFd, 0);
    if(fd == -1)
        perror("Problem opening perf hw instructions");
    perfFileDescriptors.push_back(fd);
    ioctl(fd, PERF_EVENT_IOC_ID, &event_id);
    perfEventIDsMap.emplace(event_id, "instructions");

    pe.config = PERF_COUNT_HW_CACHE_REFERENCES;
    fd = perf_event_open(&pe, 0, -1, perfGroupFd, 0);
    if(fd == -1)
        perror("Problem opening perf hw cache ref");
    perfFileDescriptors.push_back(fd);
    ioctl(fd, PERF_EVENT_IOC_ID, &event_id);
    perfEventIDsMap.emplace(event_id, "cache_references");

    pe.config = PERF_COUNT_HW_CACHE_MISSES;
    fd = perf_event_open(&pe, 0, -1, perfGroupFd, 0);
    if(fd == -1)
        perror("Problem opening perf hw cache miss");
    perfFileDescriptors.push_back(fd);
    ioctl(fd, PERF_EVENT_IOC_ID, &event_id);
    perfEventIDsMap.emplace(event_id, "cache_misses");

    // SOFTWARE EVENTS
	pe.type = PERF_TYPE_SOFTWARE;

	pe.config = PERF_COUNT_SW_CPU_CLOCK;
	fd = perf_event_open(&pe, 0, -1, perfGroupFd, 0);
	if(fd == -1) {
		RCLCPP_ERROR(rclcpp::get_logger("TimeProfilingSpinner"), "Problem opening perf sw cpu clock: %s", std::strerror(errno));
		// Consider handling the error...
	}
	perfFileDescriptors.push_back(fd);
	ioctl(fd, PERF_EVENT_IOC_ID, &event_id);
	perfEventIDsMap.emplace(event_id, "cpu_clock_time_ns");

	pe.config = PERF_COUNT_SW_TASK_CLOCK;
	fd = perf_event_open(&pe, 0, -1, perfGroupFd, 0);
	if(fd == -1) {
		RCLCPP_ERROR(rclcpp::get_logger("TimeProfilingSpinner"), "Problem opening perf sw task clock: %s", std::strerror(errno));
		// Consider handling the error...
	}
	perfFileDescriptors.push_back(fd);
	ioctl(fd, PERF_EVENT_IOC_ID, &event_id);
	perfEventIDsMap.emplace(event_id, "task_clock_time_ns");

	pe.config = PERF_COUNT_SW_PAGE_FAULTS;
	fd = perf_event_open(&pe, 0, -1, perfGroupFd, 0);
	if(fd == -1) {
		RCLCPP_ERROR(rclcpp::get_logger("TimeProfilingSpinner"), "Problem opening perf sw page faults: %s", std::strerror(errno));
		// Consider handling the error...
	}
	perfFileDescriptors.push_back(fd);
	ioctl(fd, PERF_EVENT_IOC_ID, &event_id);
	perfEventIDsMap.emplace(event_id, "page_faults");


    // These can be added as well as SW:
    // PERF_COUNT_SW_CONTEXT_SWITCHES
    // PERF_COUNT_SW_CPU_MIGRATIONS

//    // HARDWARE CACHE EVENTS
//    pe.type = PERF_TYPE_HW_CACHE;
//    pe.config = (PERF_COUNT_HW_CACHE_LL) |
//            (PERF_COUNT_HW_CACHE_OP_READ << 8) |
//            (PERF_COUNT_HW_CACHE_RESULT_MISS << 16);
//    fd = perf_event_open(&pe, 0, -1, perfGroupFd, 0);
//    if(fd == -1)
//        perror("Problem opening perf hw llc cache read miss");
//    perfFileDescriptors.push_back(fd);
//    ioctl(fd, PERF_EVENT_IOC_ID, &event_id);
//    perfEventIDsMap.emplace(event_id, "LL_cache_r_misses");

//    pe.config = (PERF_COUNT_HW_CACHE_LL) |
//            (PERF_COUNT_HW_CACHE_OP_WRITE << 8) |
//            (PERF_COUNT_HW_CACHE_RESULT_MISS << 16);
//    fd = perf_event_open(&pe, 0, -1, perfGroupFd, 0);
//    if(fd == -1)
//        perror("Problem opening perf hw llc cache write miss");
//    perfFileDescriptors.push_back(fd);
//    ioctl(fd, PERF_EVENT_IOC_ID, &event_id);
//    perfEventIDsMap.emplace(event_id, "LL_cache_w_misses");

    assert(NUM_PERF_EVENTS == (perfFileDescriptors.size()+1));
    //PERF_FLAG_FD_OUTPUT

}

void TimeProfilingSpinner::perfProfileStart(){
    ioctl(perfGroupFd, PERF_EVENT_IOC_RESET, PERF_IOC_FLAG_GROUP);
    ioctl(perfGroupFd, PERF_EVENT_IOC_ENABLE, PERF_IOC_FLAG_GROUP);
}

void TimeProfilingSpinner::perfProfileEnd(sched_server::CallbackProfile& cb_prof_info){
    ioctl(perfGroupFd, PERF_EVENT_IOC_DISABLE, PERF_IOC_FLAG_GROUP);

    struct read_format rf;

    auto ret = read(perfGroupFd, &rf, sizeof(rf));
    if(ret == -1)
        RCLCPP_ERROR(rclcpp::get_logger("TimeProfilingSpinner"), "perf read error: %s", std::strerror(errno));

    for (auto i = 0; i < rf.nr; i++) {
      std::string str_of_id = perfEventIDsMap[rf.values[i].id];

      if (str_of_id.compare("cpu_cycles") == 0) {
          cb_prof_info.cpu_cycles = rf.values[i].value;
      } else if (str_of_id.compare("instructions") == 0) {
          cb_prof_info.instructions = rf.values[i].value;
      } else if (str_of_id.compare("cpu_clock_time_ns") == 0) {
          cb_prof_info.cpu_clock_time_ns = rf.values[i].value;
      } else if (str_of_id.compare("task_clock_time_ns") == 0) {
          cb_prof_info.task_clock_time_ns = rf.values[i].value;
      } else if (str_of_id.compare("page_faults") == 0) {
          cb_prof_info.page_faults = rf.values[i].value;
      } else if (str_of_id.compare("cache_references") == 0) {
          cb_prof_info.cache_references = rf.values[i].value;
      } else if (str_of_id.compare("cache_misses") == 0) {
          cb_prof_info.cache_misses = rf.values[i].value;
      }
    }

    cb_prof_info.period_id = period_id;
}

std::chrono::system_clock::time_point TimeProfilingSpinner::getInitTargetTime()
{
    auto clock_msg_future = std::make_shared<std::promise<rosgraph_msgs::msg::Clock>>();
    auto future_result = clock_msg_future->get_future();

    auto sub_callback = [clock_msg_future](const rosgraph_msgs::msg::Clock::SharedPtr msg) {
        clock_msg_future->set_value(*msg);
    };

    auto subscription = timing_nh->create_subscription<rosgraph_msgs::msg::Clock>(
        "period_init_time", 10, sub_callback);

    // Wait for a message to be received
    if(future_result.wait_for(std::chrono::seconds(3)) == std::future_status::timeout) {
        RCLCPP_ERROR(timing_nh->get_logger(), "Timeout waiting for 'period_init_time' message");
        // Handle timeout...
    }

    auto clk_cptr = future_result.get();

    return std::chrono::system_clock::time_point(
        std::chrono::milliseconds(
            static_cast<unsigned long>(
                clk_cptr.clock.sec*1000 + 5000)));
}


void TimeProfilingSpinner::spinAndProfileUntilShutdown(){
    RCLCPP_INFO(rclcpp::get_logger("TimeProfilingSpinner"), "Starting to initialize spinner.");

    SchedClient::ConfigureSchedOfCallingThread();

    cbProfPublisher = cb_prof_nh->create_publisher<sched_server::msg::CallbackProfile>(
    	cb_prof_nh->get_name(), 10);


    int policy = sched_getscheduler(0);
    bool privileged = (policy != SCHED_OTHER);
    if(privileged){
        // real-time system memory settings
        mallopt(M_MMAP_MAX, 0);
        mallopt(M_TRIM_THRESHOLD, -1);
        mlockall(MCL_CURRENT | MCL_FUTURE);
    }

    rclcpp::spin(cb_prof_nh);
}


	// perf events setup
	auto period = std::chrono::milliseconds(
		        static_cast<unsigned long>(1000.0/callbackCheckFrequency_));

	std::chrono::system_clock::time_point target;
	if(synchronizedStart_){
		target = getInitTargetTime();
		std::this_thread::sleep_until(target);
	}
	else{
		target = std::chrono::system_clock::now() + period;
	}

	// priorities should be descending through chains
	if(op_mode_ == OperationMode::RUN_CB_ON_ARRIVAL){
		auto cbWait = std::chrono::milliseconds(1000) / callbackCheckFrequency_;
		while(rclcpp::ok()){
		    callAvailableOneByOne(target, cbWait);
		    if(target < std::chrono::system_clock::now()){
		        target += period;
		        ++period_id;
		    }
		}
	}
	else{ // periodic
		bool func_available = static_cast<bool>(funcToCall_);

		while(rclcpp::ok())
		{
		    callAvailableOneByOne(target += period);
		    if(rclcpp::ok() && func_available){
		        sched_server::CallbackProfile cp;
		        perfProfileStart();
		        funcToCall_();
		        perfProfileEnd(cp);
		        cbProfPublisher->publish(cp);
		    }
		    std::this_thread::sleep_until(target);
		    ++period_id;
		}
	}

	RCLCPP_INFO(rclcpp::get_logger("TimeProfilingSpinner"), "Stopped spinning.");


int TimeProfilingSpinner::callAvailableOneByOne(
        std::chrono::system_clock::time_point timeout,
        std::chrono::milliseconds cbWaitTime)
{
    std::vector<sched_server::CallbackProfile> cbProfs;
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(cb_prof_nh);

    while(rclcpp::ok() &&
          std::chrono::system_clock::now() < timeout &&
          executor.get_queue_size() > 0)
    {
        sched_server::CallbackProfile cp;
        perfProfileStart();

        // Spin node once to process a single callback
        executor.spin_node_once(cb_prof_nh, cbWaitTime);

        perfProfileEnd(cp);
        cbProfs.push_back(cp);
    }

    for(auto& cp : cbProfs)
        cbProfPublisher->publish(cp);

    return cbProfs.size();
}


void TimeProfilingSpinner::signalHandler(int sig)
{
    RCLCPP_INFO(rclcpp::get_logger("TimeProfilingSpinner"), "Signal has come.");
    rclcpp::shutdown();
}


TimeProfilingSpinner::~TimeProfilingSpinner(){
    for(long fd : perfFileDescriptors)
        close(fd);
    close(perfGroupFd);
}

