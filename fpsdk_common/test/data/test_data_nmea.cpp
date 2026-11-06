// clang-format off
static const std::vector<uint8_t> TEST_DATA_NMEA_BIN = []() {
    // Separate string literals, as compilers may not support long strings (-Woverlength-strings)
    static const char* const b64[] = {
        "JEdOUk1DLDE3NDY0NC4yMCxBLDQ3MjQuMDE4MTgsTiwwMDgyNy4wMjI0NCxFLDAuMDEzLCwxNTAzMjMsLCxSLFYqMEQNCiRHTkdHQSwxNzQ2NDQuMjAsNDcyNC4wMTgx",
        "OCxOLDAwODI3LjAyMjQ0LEUsNCwxMiwwLjU4LDQxMi4xLE0sNDcuMyxNLDEuMiwwMDAwKjZDDQo=",
    };
    std::string str;
    for (const char* s : b64) { str += s; }
    return fpsdk::common::string::Base64Dec(str);
}();
