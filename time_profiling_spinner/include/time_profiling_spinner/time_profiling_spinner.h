// header
//

#ifndef SCHED_SERVICE_H
#define SCHED_SERVICE_H

#include <list>
#include <chrono>
#include <map>
#include <functional>
#include <atomic>
#include <condition_variable>
#include <semaphore.h>
#include <linux/perf_event.h>
#include <linux/hw_breakpoint.h>
#include <sys/ioctl.h>
#include <linux/perf_event.h>
#include <asm/unistd.h>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include "callback_profile/msg/callback_profile.hpp"

#define NUM_PERF_EVENTS 7

#define USE_DEFAULT_CALLBACK_FREQ 0

class TimeProfilingSpinner
{
public:
    enum class OperationMode {
        CHAIN_HEAD,
        RUN_CB_ON_ARRIVAL,
        PERIODIC
    };

    TimeProfilingSpinner(std::shared_ptr<rclcpp::Node> node_handle);

    void perfProfileInit();

    void perfProfileStart();

    void perfProfileEnd(callback_profile::msg::CallbackProfile& cb_prof_info);

    void spinAndProfileUntilShutdown();

    static void signalHandler(int sig);

    static long
    perf_event_open(struct perf_event_attr *hw_event, pid_t pid,
                    int cpu, int group_fd, unsigned long flags)
    {
        return syscall(__NR_perf_event_open, hw_event, pid, cpu,
                       group_fd, flags);
    }

    ~TimeProfilingSpinner();

    struct read_format {
         uint64_t nr;
         struct {
             uint64_t value;
             uint64_t id;
         } values[NUM_PERF_EVENTS];
     };


private:
    //OperationMode op_mode_;
    std::shared_ptr<rclcpp::Node> cb_prof_nh;

    long perfGroupFd;
    std::vector<long> perfFileDescriptors;
    std::map<uint64_t, std::string> perfEventIDsMap;
    int period_id;
};

#endif //SCHED_SERVICE_H

