/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: see the LICENSE file
 * /__/  \__\
 * \endverbatim
 *
 * @file
 * @brief Fixposition SDK: Thread helpers
 */

/* LIBC/STL */
#include <algorithm>
#include <chrono>
#include <cstring>
#include <functional>

/* EXTERNAL */
#include <sys/prctl.h>

/* PACKAGE */
#include "fpsdk_common/logging.hpp"
#include "fpsdk_common/thread.hpp"

namespace fpsdk {
namespace common {
namespace thread {
/* ****************************************************************************************************************** */

Thread::Thread(
    const std::string& name, Thread::ThreadFunc func, void* arg, Thread::PrepFunc prep, Thread::CleanFunc clean)
    : name_{ name }, func_{ func }, arg_{ arg }, prep_{ prep }, clean_{ clean }
{
}

Thread::~Thread()
{
    if (started_) {
        Stop();
    }
}

// ---------------------------------------------------------------------------------------------------------------------

bool Thread::Start(const bool try_catch)
{
    if (started_) {
        WARNING("%s thread already started", name_.c_str());
        return false;
    }
    bool res = true;
    try {
        abort_ = false;
        thread_ = std::make_unique<std::thread>(&Thread::_Thread, this, try_catch);
        res = true;
        started_ = true;
        status_ = Status::RUNNING;
    } catch (std::exception& e) {
        WARNING("%s thread fail: %s", name_.c_str(), e.what());
        res = false;
    }
    return res;
}

// ---------------------------------------------------------------------------------------------------------------------

bool Thread::Stop()
{
    if (!started_) {
        WARNING("%s thread not started", name_.c_str());
        return false;
    }

    // Thread cannot stop itself (using this method)
    if (std::this_thread::get_id() == thread_->get_id()) {
        WARNING("Thread::Stop() called from thread");
        return false;
    }

    // Signal that we want to stop
    abort_ = true;
    Wakeup();
    thread_->join();
    thread_.reset();

    // Crashed
    if (status_ == Status::RUNNING) {
        status_ = Status::FAILED;
    }

    started_ = false;

    return true;
}

// ---------------------------------------------------------------------------------------------------------------------

Thread::Status Thread::GetStatus() const
{
    return status_;
}

// ---------------------------------------------------------------------------------------------------------------------

/*static*/ const char* Thread::StatusStr(const Status status)
{
    switch (status) {  // clang-format off
        case STOPPED: return "STOPPED";
        case RUNNING: return "RUNNING";
        case FAILED:  return "FAILED";
    }  // clang-format on
    return "?";
}

// ---------------------------------------------------------------------------------------------------------------------

void Thread::Wakeup()
{
    sem_.Notify();
}

// ---------------------------------------------------------------------------------------------------------------------

WaitRes Thread::Sleep(const uint32_t millis)
{
    // Sleep, but wake up early in case we're notified
    return sem_.WaitFor(millis);
}

// ---------------------------------------------------------------------------------------------------------------------

WaitRes Thread::SleepUntil(const uint32_t period, const uint32_t min_sleep)
{
    return sem_.WaitUntil(period, min_sleep);
}

// ---------------------------------------------------------------------------------------------------------------------

bool Thread::ShouldAbort()
{
    return abort_;
}

// ---------------------------------------------------------------------------------------------------------------------

const std::string& Thread::GetName()
{
    return name_;
}

// ---------------------------------------------------------------------------------------------------------------------

void Thread::_Thread(const bool try_catch)
{
    SetThreadName(name_);

    // Run optional user prepare function
    if (prep_) {
        prep_(arg_);
    }

    // Run user thread function
    if (try_catch) {
        try {
            if (func_(*this, arg_)) {
                status_ = Status::STOPPED;
            } else {
                status_ = Status::FAILED;
            }
        } catch (const std::exception& e) {
            WARNING("%s thread unhandled exception: %s", name_.c_str(), e.what());
            status_ = Status::FAILED;
        }
    } else {
        if (func_(*this, arg_)) {
            status_ = Status::STOPPED;
        } else {
            status_ = Status::FAILED;
        }
    }

    // Run optional user cleanup function
    if (clean_) {
        clean_(arg_);
    }
}

/* ****************************************************************************************************************** */

void SetThreadName(const std::string& name, const std::size_t curr)
{
    if (name.empty()) {
        return;
    }
    char curr_name[16];
    char thread_name[32];  // max 16 cf. prctl(2)
    curr_name[0] = '\0';
    if (prctl(PR_GET_NAME, curr_name, 0, 0, 0) == 0) {
        const std::size_t clip = std::clamp<std::size_t>(curr, 0, 10);
        if (clip > 0) {
            curr_name[clip] = '\0';  // limit original (main process) name to 6 chars
            std::snprintf(thread_name, sizeof(thread_name), "%s:%s", curr_name, name.c_str());
        } else {
            std::snprintf(thread_name, sizeof(thread_name), "%s", name.c_str());
        }
        prctl(PR_SET_NAME, thread_name, 0, 0, 0);  // This clips thread_name at the max size (typically, 12 chars)
    }
}

// ---------------------------------------------------------------------------------------------------------------------

BinarySemaphore::BinarySemaphore()
{
}

void BinarySemaphore::Notify()
{
    std::unique_lock<std::mutex> lock(mutex_);
    pend_ = true;
    cond_.notify_all();
}

WaitRes BinarySemaphore::WaitFor(const uint32_t millis)
{
    std::unique_lock<std::mutex> lock(mutex_);
    if (pend_) {
        pend_ = false;
        return WaitRes::WOKEN;
    }
    if (cond_.wait_for(lock, std::chrono::milliseconds(millis)) == std::cv_status::timeout) {
        return WaitRes::TIMEOUT;
    } else {
        pend_ = false;
        return WaitRes::WOKEN;
    }
}

WaitRes BinarySemaphore::WaitUntil(const uint32_t period, const uint32_t min_sleep)
{
    using namespace std::chrono;

    if (period == 0) {
        return WaitRes::TIMEOUT;
    }

    // Now, in [ms]
    const auto now = system_clock::now();
    const auto now_ms = duration_cast<milliseconds>(now.time_since_epoch()).count();

    // Calculate sleep duration [ms] to target the start of the next millis period
    uint32_t sleep_ms = period - (now_ms % period);

    // Skip one period in case minimal sleep time cannot be reached
    if ((sleep_ms < min_sleep) && (min_sleep < period)) {
        sleep_ms += period;
    }

    // Calculate wakup time
    const auto wakeup_time = time_point_cast<milliseconds>(now + milliseconds(sleep_ms));

    // printf("WaitUntil(%u, %u) now=%lu sleep_ms=%u wakeup_time=%lu\n", period, min_sleep,
    //     now_ms, sleep_ms, duration_cast<milliseconds>(wakeup_time.time_since_epoch()).count());

    // Sleep, but wake up early in case we're notified
    std::unique_lock<std::mutex> lock(mutex_);
    return cond_.wait_until(lock, wakeup_time) == std::cv_status::timeout ? WaitRes::TIMEOUT : WaitRes::WOKEN;
}

/* ****************************************************************************************************************** */

std::size_t ThisThreadId()
{
    return std::hash<std::thread::id>{}(std::this_thread::get_id());
}

/* ****************************************************************************************************************** */
}  // namespace thread
}  // namespace common
}  // namespace fpsdk
