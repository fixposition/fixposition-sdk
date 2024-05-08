/**
 * \verbatim
 * ___    ___
 * \  \  /  /
 *  \  \/  /   Copyright (c) Fixposition AG (www.fixposition.com) and contributors
 *  /  /\  \   License: MIT (see the LICENSE file)
 * /__/  \__\
 * \endverbatim
 *
 * @file
 * @brief fpltool: meta
 */

/* LIBC/STL */
#include <cstdio>
#include <memory>

/* EXTERNAL */

/* Fixposition SDK */
#include <fpcommon/app.hpp>
#include <fpcommon/fpl.hpp>
#include <fpcommon/logging.hpp>

/* PACKAGE */
#include "fpltool_meta.hpp"

namespace fp {
namespace fpltool {
/* ****************************************************************************************************************** */

using namespace fp::common::app;
using namespace fp::common::fpl;

bool DoMeta(const FpltoolArgs& args)
{
    if (args.inputs_.size() != 1) {
        WARNING("Need exactly one input file");
        return false;
    }
    const std::string input_fpl = args.inputs_[0];

    NOTICE("Getting metadata from %s", input_fpl.c_str());

    FplFileReader reader;
    if (!reader.Open(input_fpl)) {
        return false;
    }

    // Handle SIGINT (C-c) to abort nicely
    SigIntHelper sig_int;

    // Process all messages, until we find the meta data. It *should* be the first message.
    FplMessage log_msg;
    double progress = 0.0;
    double rate = 0.0;
    std::unique_ptr<LogMeta> logmeta;
    bool logmeta_first = false;

    while (!logmeta && !sig_int.ShouldAbort() && reader.Next(log_msg)) {
        // Report progress
        if (args.progress_ > 0) {
            if (reader.GetProgress(progress, rate)) {
                INFO("Scanning... %.1f%% (%.0f MiB/s)\r", progress, rate);
            }
        }

        switch (log_msg.PayloadType()) {
            case FplType::LOGMETA: {
                const LogMeta cand(log_msg);
                if (cand.valid_) {
                    logmeta = std::make_unique<LogMeta>(std::move(cand));
                    logmeta_first = (log_msg.file_seq_ == 1);
                }
                break;
            }
            case FplType::ROSMSGDEF:
            case FplType::ROSMSGBIN:
            case FplType::STREAMMSG:
            case FplType::LOGSTATUS:
            case FplType::BLOB:
            case FplType::UNSPECIFIED:
            case FplType::INT_D:
            case FplType::INT_F:
            case FplType::INT_X:
                break;
        }
    }

    // Are we happy?
    bool ok = false;
    if (logmeta) {
        if (!logmeta_first) {
            WARNING("LOGMETA message found at unexpected offset");
        }
        std::fputs(logmeta->yaml_.c_str(), stdout);
        ok = true;
    } else {
        WARNING("No LOGMETA message found");
    }

    return ok;
}

/* ****************************************************************************************************************** */
}  // namespace fpltool
}  // namespace fp
