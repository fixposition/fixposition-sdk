// clang-format off
static const std::vector<uint8_t> TEST_DATA_FPB_BIN = []() {
    // Separate string literals, as compilers may not support long strings (-Woverlength-strings)
    static const char* const b64[] = {
        "ZiHRB1wAAAABAwAAAAAAAAoAAAAUAAAA4v///wEBAQEBAAAAAAEKAE5hvAALAAAAFQAAAOP///8BAQEBAgAAAAABCgBOYbwADAAAABYAAADk////AQEBAQMAAAAAAQoA",
        "TmG8ACZUqjQ=",
    };
    std::string str;
    for (const char* s : b64) { str += s; }
    return fpsdk::common::string::Base64Dec(str);
}();
