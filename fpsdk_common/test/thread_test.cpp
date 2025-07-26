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
 * @brief Fixposition SDK: tests for fpsdk::common::thread
 */

/* LIBC/STL */
#include <exception>
#include <thread>

/* EXTERNAL */
#include <gtest/gtest.h>

/* PACKAGE */
#include <fpsdk_common/logging.hpp>
#include <fpsdk_common/thread.hpp>
#include <fpsdk_common/time.hpp>

namespace {
/* ****************************************************************************************************************** */
using namespace fpsdk::common::thread;

TEST(ThreadTest, HappyThread)
{
    auto s = fpsdk::common::time::Duration::FromSec(0.2);

    Thread thread(std::string("worker"), [](Thread& t, void*) -> bool {
        DEBUG("thread begin");
        while (!t.ShouldAbort()) {
            DEBUG("thread...");
            t.Sleep(50);
        }
        DEBUG("thread end");
        return true;
    });

    EXPECT_EQ(thread.GetStatus(), Thread::Status::STOPPED);
    EXPECT_TRUE(thread.Start());
    EXPECT_FALSE(thread.Start());
    EXPECT_EQ(thread.GetStatus(), Thread::Status::RUNNING);
    s.Sleep();
    EXPECT_TRUE(thread.Stop());
    EXPECT_EQ(thread.GetStatus(), Thread::Status::STOPPED);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ThreadTest, ShortThread)
{
    auto s = fpsdk::common::time::Duration::FromSec(0.2);

    Thread thread(std::string("worker"), [](Thread&, void*) -> bool {
        DEBUG("thread begin");
        DEBUG("thread end");
        return true;
    });

    EXPECT_EQ(thread.GetStatus(), Thread::Status::STOPPED);
    EXPECT_TRUE(thread.Start());
    EXPECT_EQ(thread.GetStatus(), Thread::Status::RUNNING);
    s.Sleep();
    EXPECT_TRUE(thread.Stop());
    EXPECT_EQ(thread.GetStatus(), Thread::Status::STOPPED);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ThreadTest, SadThread)
{
    auto s = fpsdk::common::time::Duration::FromSec(0.25);

    Thread thread(std::string("worker"), [](Thread& t, void*) -> bool {
        DEBUG("thread begin");
        int n = 0;
        while (!t.ShouldAbort()) {
            DEBUG("thread...");
            t.Sleep(50);
            if (n >= 2) {
                DEBUG("thread return false");
                return false;
            }
            n++;
        }
        DEBUG("thread end");
        return true;
    });

    EXPECT_EQ(thread.GetStatus(), Thread::Status::STOPPED);
    EXPECT_TRUE(thread.Start());
    EXPECT_EQ(thread.GetStatus(), Thread::Status::RUNNING);
    s.Sleep();
    EXPECT_EQ(thread.GetStatus(), Thread::Status::FAILED);
    EXPECT_TRUE(thread.Stop());
    EXPECT_EQ(thread.GetStatus(), Thread::Status::FAILED);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ThreadTest, CrashedThread)
{
    auto s = fpsdk::common::time::Duration::FromSec(0.25);

    Thread thread(std::string("worker"), [](Thread& t, void*) -> bool {
        DEBUG("thread begin");
        int n = 0;
        while (!t.ShouldAbort()) {
            DEBUG("thread...");
            t.Sleep(50);
            if (n >= 2) {
                throw std::runtime_error("deliberate crash");
                return false;
            }
            n++;
        }
        DEBUG("thread end");
        return true;
    });

    EXPECT_EQ(thread.GetStatus(), Thread::Status::STOPPED);
    EXPECT_TRUE(thread.Start());
    EXPECT_EQ(thread.GetStatus(), Thread::Status::RUNNING);
    s.Sleep();
    EXPECT_EQ(thread.GetStatus(), Thread::Status::FAILED);
    EXPECT_TRUE(thread.Stop());
    EXPECT_EQ(thread.GetStatus(), Thread::Status::FAILED);
}

// ---------------------------------------------------------------------------------------------------------------------

TEST(ThreadTest, BinarySemaphore)
{
    BinarySemaphore sem;

    EXPECT_EQ(sem.WaitFor(10), WaitRes::TIMEOUT);
    EXPECT_NE(sem.WaitFor(10), WaitRes::WOKEN);

    sem.Notify();
    EXPECT_EQ(sem.WaitFor(10), WaitRes::WOKEN);
    EXPECT_EQ(sem.WaitFor(10), WaitRes::TIMEOUT);

    std::thread thread([&sem] {
        fpsdk::common::time::Duration::FromSec(0.05).Sleep();
        sem.Notify();
    });
    EXPECT_EQ(sem.WaitFor(100), WaitRes::WOKEN);
    thread.join();
}

/* ****************************************************************************************************************** */
}  // namespace

int main(int argc, char** argv)
{
    testing::InitGoogleTest(&argc, argv);
    auto level = fpsdk::common::logging::LoggingLevel::WARNING;
    for (int ix = 0; ix < argc; ix++) {
        if ((argv[ix][0] == '-') && argv[ix][1] == 'v') {
            level++;
        }
    }
    fpsdk::common::logging::LoggingSetParams(level);
    return RUN_ALL_TESTS();
}
