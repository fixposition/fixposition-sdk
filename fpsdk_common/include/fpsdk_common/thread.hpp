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
 *
 * @page FPSDK_COMMON_THREAD Thread helpers
 *
 * **API**: fpsdk_common/thread.hpp and fpsdk::common::thread
 *
 */
#ifndef __FPSDK_COMMON_THREAD_HPP__
#define __FPSDK_COMMON_THREAD_HPP__

/* LIBC/STL */
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

/* EXTERNAL */

/* PACKAGE */

namespace fpsdk {
namespace common {
/**
 * @brief Thread helpers
 */
namespace thread {
/* ****************************************************************************************************************** */

/**
 * @brief Thread sleep result
 */
enum class WaitRes
{
    WOKEN,    //!< Thread was woken up (sleep interrupted), or semaphore was taken (wait interrupted)
    TIMEOUT,  //!< Sleep or wait timeout has expired (no wakeup, no interrupt)
};

/**
 * @brief A binary semaphore, useful for thread synchronisation
 *
 * The default state is "taken", that is, WaitFor() and WaitUntil() block (sleep) until the semaphore is "given" using
 * Notify(). See the example in the Thread class documentation below.
 */
class BinarySemaphore
{
   public:
    BinarySemaphore();

    /**
     * @brief Notify ("signal", "give")
     */
    void Notify();

    /**
     * @brief Wait (take), with timeout
     *
     * @param[in]  millis  Number of [ms] to sleep, must be > 0
     *
     * @returns WaitRes::WOKEN if taken (within time limit), WaitRes::TIMEOUT if the timeout has expired
     */
    WaitRes WaitFor(const uint32_t millis);

    /**
     * @brief Wait (take), with timout aligned to a period
     *
     * This blocks (sleeps) until the start of the next millis period is reached (or the semaphore has been taken). The
     * period is aligned to the system clock. For example, with a period of 500ms it would timeout at t+0.0s, t+0.5s,
     * t+1.0s, t+1.5s, etc. If the calculated wait duration is less then the given minimal wait duration, it waits
     * longer than one period in order to achieve the minimal wait duration and still timeout on the start of a period.
     *
     * Note that this works on the actual system clock. In ROS replay scenario this may not behave as expected as the
     * fake ROS system clock may be faster or slower than the actual system clock.
     *
     * @param[in]  period     Period duration [ms], must be > 0
     * @param[in]  min_sleep  Minimal sleep duration [ms], must be < period
     *
     * @returns WaitRes::WOKEN if taken (within time limit), WaitRes::TIMEOUT if the timeout has expired or period was 0
     */
    WaitRes WaitUntil(const uint32_t period, const uint32_t min_sleep = 0);

   private:
    std::mutex mutex_;              //!< Mutex
    std::condition_variable cond_;  //!< Condition
};

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief Helper class for handling threads
 *
 * Example:
 *
 * @code{.cpp}
 * class Example {
 *    public:
 *
 *        Example() :
 *            thread_ { "worker", std::bind(&Example::Worker, this, std::placeholders::_1) }
 *        { }
 *
 *    void Run() {
 *
 *         if (!thread_.Start()) {
 *             throw "...";
 *         }
 *
 *         int n = 0;
 *         while (thread_.IsRunning()) {
 *
 *             sleep(500);
 *             thread_.Wakeup()  // = BinarySemaphore::Notify()
 *
 *             sleep(2000);
 *             thread_.Wakeup()  // = BinarySemaphore::Notify()
 *
 *             n++;
 *             if (n >= 10) {
 *                 thread_.Stop();
 *             }
 *         }
 *     }
 *
 *    private:
 *
 *     void Worker(void *) {
 *          while (!thread_.ShouldAbort()) {
 *              if (thread_.Sleep(1000) == WaitRes::WOKEN) {   // = BinarySemaphore::SleepFor(1000)
 *                  // We have been woken up
 *              } else {
 *                  // Timeout expired
 *              }
 *          }
 *     }
 *
 *    Thread thread_;
 * };
 * @endcode
 */
class Thread
{
   public:
    using ThreadFunc = std::function<void(Thread*, void*)>;  //!< Thread main function
    using PrepFunc = std::function<void(void*)>;             //!< Thread prepare function
    using CleanFunc = std::function<void(void*)>;            //!< Thread cleanup function

    /**
     * @brief Constructor
     *
     * @param[in]  name   Name of the thread, for debugging
     * @param[in]  func   The thread function
     * @param[in]  arg    Optional thread function user argument
     * @param[in]  prep   Optional prepare function, called before the thread function
     * @param[in]  clean  Optional cleanup function, called after the threadd stopped (or crashed)
     *
     * @note Constructing an instance delibarately does not automatically start the thread!
     */
    Thread(const std::string& name, ThreadFunc func, void* arg = nullptr, PrepFunc prep = nullptr,
        CleanFunc clean = nullptr);

    /**
     * @brief Destructor, blocks until thread has stopped
     */
    ~Thread();

    // ----- Common methods -----

    /**
     * @brief Get thread name
     * @returns the thread name
     */
    const std::string& GetName();

    // -----------------------------------------------------------------------------------------------------------------

    //! \name Main (controlling) thread methods
    //@{

    /**
     * @brief Start the thread
     *
     * @note This method is used by the main, controlling thread
     */
    bool Start();

    /**
     * @brief Stop the thread
     *
     * @note This method is used by the main, controlling thread
     */
    void Stop();

    /**
     * @brief Wakup a sleeping thread
     *
     * @note This method is used by the main, controlling thread
     */
    void Wakeup();

    /**
     * @brief Check if thread is running
     * @returns true if thread is running, false if it has stopped
     * @note This method is used by the main, controlling thread
     */
    bool IsRunning();

    //@}

    // -----------------------------------------------------------------------------------------------------------------

    //! \name Worker thread methods
    //@{

    /**
     * @brief Sleep until timeout or woken up
     *
     * @param[in]  millis  Number of [ms] to sleep
     *
     * @returns WaitRes::WOKEN if the thread has been woken up, WaitRes::TIMEOUT if the timeout has expired
     *
     * @note This method is used by the worker thread
     */
    WaitRes Sleep(const uint32_t millis);

    /**
     * @brief Sleep until next period start or woken up
     *
     * See BinarySemaphore::WaitUntil() for a detailed explanation.
     *
     * @param[in]  period     Period duration [ms], must be > 0
     * @param[in]  min_sleep  Minimal sleep duration [ms], must be < period
     *
     * @returns WaitRes::WOKEN if the thread has been woken up, WaitRes::TIMEOUT if the timeout has expired or period
     *          was 0
     *
     * @note This method is used by the worker thread
     */
    WaitRes SleepUntil(const uint32_t period, const uint32_t min_sleep = 0);

    /**
     * @brief Check if we should abort
     *
     * @note This method is used by the worker thread, typically as the predicate in the main loop, e.g.
     *       `while (!thread_.ShouldAbort()) { do_stuff(); }`
     */
    bool ShouldAbort();

    //@}

    // -----------------------------------------------------------------------------------------------------------------

   private:
    // clang-format off
    std::string                  name_;    //!< Thread name
    std::unique_ptr<std::thread> thread_;  //!< Thread handle
    ThreadFunc                   func_;    //!< Thread function
    void                        *arg_;     //!< Thread function argument
    PrepFunc                     prep_;    //!< Thread prepare function
    CleanFunc                    clean_;   //!< Thread cleanup function
    std::atomic<bool>            abort_;   //!< Abort signal
    bool                         running_; //!< Running flag
    BinarySemaphore              sem_;     //!< Semaphore
    // clang-format on
    void _Thread();  //!< Wrapper function to call the user thread function
};

// ---------------------------------------------------------------------------------------------------------------------

/**
 * @brief Set thread name
 *
 * Sets the thread name, which shows in htop etc.
 *
 * @param[in]  name  The thread name (1-6 characters, longer \c name is clipped at 6 chars)
 */
void SetThreadName(const std::string& name);

/**
 * @brief Get numeric thread ID
 *
 * Like std::this_thread::get_id(), but numeric (and therefore printf()-able, etc.).
 */
std::size_t ThisThreadId();

/* ****************************************************************************************************************** */
}  // namespace thread
}  // namespace common
}  // namespace fpsdk
#endif  // __FPSDK_COMMON_THREAD_HPP__
