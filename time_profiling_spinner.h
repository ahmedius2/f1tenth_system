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
#include <sched_server/CallbackProfile.h>
#include <rclcpp/rclcpp.hpp>

#define NUM_PERF_EVENTS 7

#define USE_DEFAULT_CALLBACK_FREQ 0

class TimeProfilingSpinner : public rclcpp::Node
{
public:
    enum class OperationMode {
        CHAIN_HEAD,
        RUN_CB_ON_ARRIVAL,
        PERIODIC
    };

    TimeProfilingSpinner(OperationMode op_mode,
        double callbackCheckFrequency = USE_DEFAULT_CALLBACK_FREQ,
        bool useCompanionThread = false,
        std::function<void()> funcToCall = std::function<void()>(),
        std::string fname_post = "");

    void perfProfileInit();

    void perfProfileStart();

    void perfProfileEnd(sched_server::CallbackProfile& cb_prof_info);

    std::chrono::system_clock::time_point getInitTargetTime();

    void spinAndProfileUntilShutdown();

    int callAvailableOneByOne(
            std::chrono::system_clock::time_point timeout);

    void saveProfilingData() {return;}

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
    OperationMode op_mode_;
    std::function<void()> funcToCall_;
    double callbackCheckFrequency_;
    std::string fname_post_;

    long perfGroupFd;
    std::vector<long> perfFileDescriptors;
    std::map<uint64_t, std::string> perfEventIDsMap;
    rclcpp::Publisher<sched_server::CallbackProfile>::SharedPtr cbProfPublisher;
    uint64_t period_id;

    bool synchronizedStart_;

    sched_param spinner_sched_param_;
};

#endif //SCHED_SERVICE_H

