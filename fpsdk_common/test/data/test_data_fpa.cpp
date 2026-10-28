// clang-format off
static const std::vector<uint8_t> TEST_DATA_FPA_BIN = []() {
    // Separate string literals, as compilers may not support long strings (-Woverlength-strings)
    static const char* const b64[] = {
        "JEZQLE9ET01FVFJZLDIsMjI1MywzMjMyOTkuMTAwMDAwLCwsLCwsLCwsLCwsLCwsLCwwLDAsOCw4LC0xLCwsLCwsLCwsLCwsLCwsLCwsLGZwX3JlbGVhc2VfdnIyXzIu",
        "NjMuMV8yMDQqN0INCiRGUCxURiwyLDIyNTMsMzIzMjk5LjAwMDAwMCxWUlRLLENBTSwtMC4wMzQwMCwwLjAwMjAwLC0wLjAxNjAwLC0wLjAwMDAwMCwwLjAwMDAwMCww",
        "LjAwMDAwMCwxLjAwMDAwMCo2Ng0K",
    };
    std::string str;
    for (const char* s : b64) { str += s; }
    return fpsdk::common::string::Base64Dec(str);
}();
