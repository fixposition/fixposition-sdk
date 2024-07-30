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
#include <chrono>
#include <cstring>
#include <functional>

/* EXTERNAL */
#include <sys/prctl.h>

/* PACKAGE */
#include "fpcommon/logging.hpp"
#include "fpcommon/thread.hpp"

namespace fp {
namespace common {
namespace thread {
/* ****************************************************************************************************************** */

Thread::Thread(
    const std::string& name, Thread::ThreadFunc func, void* arg, Thread::PrepFunc prep, Thread::CleanFunc clean)
    : name_{name}, func_{func}, arg_{arg}, prep_{prep}, clean_{clean}, abort_{false}, running_{false}
{
}

Thread::~Thread()
{
    Stop();
}

// ---------------------------------------------------------------------------------------------------------------------

bool Thread::Start()
{
    bool res = true;
    Stop();
    try {
        abort_ = false;
        thread_ = std::make_unique<std::thread>(&Thread::_Thread, this);
        res = true;
        running_ = true;
    } catch (std::exception& e) {
        WARNING("%s thread fail: %s", name_.c_str(), e.what());
    }
    return res;
}

// ---------------------------------------------------------------------------------------------------------------------

void Thread::Stop()
{
    if (thread_ && (std::this_thread::get_id() != thread_->get_id())) {
        abort_ = true;
        Wakeup();
        thread_->join();
        thread_ = nullptr;
        running_ = false;
    }
}

// ---------------------------------------------------------------------------------------------------------------------

bool Thread::IsRunning()
{
    return running_;
}

// ---------------------------------------------------------------------------------------------------------------------

void Thread::Wakeup()
{
    sem_.Notify();
}

// ---------------------------------------------------------------------------------------------------------------------

bool Thread::Sleep(const uint32_t millis)
{
    // Sleep, but wake up early in case we're notified
    return sem_.WaitFor(millis);
}

// ---------------------------------------------------------------------------------------------------------------------

bool Thread::SleepUntil(const uint32_t period, const uint32_t min_sleep)
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

void Thread::_Thread()
{
    SetThreadName(name_);

    // Run optional user prepare function
    if (prep_) {
        prep_(arg_);
    }

    // Run user thread function
    try {
        func_(this, arg_);
    } catch (const std::exception& e) {
        WARNING("%s thread crash: %s", name_.c_str(), e.what());
    }

    // Run optional user cleanup function
    if (clean_) {
        clean_(arg_);
    }

    abort_ = true;
    running_ = false;
}

/* ****************************************************************************************************************** */

void SetThreadName(const std::string& name)
{
    if (name.empty()) {
        return;
    }
    char curr_name[16];
    char thread_name[32];  // max 16 cf. prctl(2)
    curr_name[0] = '\0';
    if (prctl(PR_GET_NAME, curr_name, 0, 0, 0) == 0) {
        curr_name[7] = '\0';  // limit original (main process) name to 6 chars
        std::snprintf(thread_name, sizeof(thread_name), "%s:%s", curr_name, name.c_str());
        prctl(PR_SET_NAME, thread_name, 0, 0, 0);  // This clips thread_name at the max size (typically, 12 chars)
    }
}

// ---------------------------------------------------------------------------------------------------------------------

BinarySemaphore::BinarySemaphore()
{
}

void BinarySemaphore::Notify()
{
    cond_.notify_all();
}

bool BinarySemaphore::WaitFor(const uint32_t millis)
{
    std::unique_lock<std::mutex> lock(mutex_);
    return cond_.wait_for(lock, std::chrono::milliseconds(millis)) != std::cv_status::timeout;
}

bool BinarySemaphore::WaitUntil(const uint32_t period, const uint32_t min_sleep)
{
    using namespace std::chrono;

    if (period == 0) {
        return false;
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
    return cond_.wait_until(lock, wakeup_time) != std::cv_status::timeout;
}

/* ****************************************************************************************************************** */

std::size_t ThisThreadId()
{
    return std::hash<std::thread::id>{}(std::this_thread::get_id());
}

/* ****************************************************************************************************************** */
}  // namespace thread
}  // namespace common
}  // namespace fp
