// clang-format off
static const std::vector<uint8_t> TEST_DATA_SBF_BIN = []() {
    // Separate string literals, as compilers may not support long strings (-Woverlength-strings)
    static const char* const b64[] = {
        "JECJCqdPYAAIzqgNWAkBAPqVcKQreeo/3XFhDdngwj/OjNGAYbV8QEr1P0JYc6I5+9LhObifhTn5ApXQvuNh5viPxL9A1yU+AAApAP////8FSSJQAQAAADIAxQAuAVAA",
        "JEAGNxIXOAAIzqgNWAkBAEbOEj8gisw+vSoSQP51cz9cdoy7ykjGPV+Yhz0Giok+vBI1PnyEqD8kQESFFBc4AAjOqA1YCQEAdwjdN4XXmDesu0k4zygcONIAObVeDgS1",
        "ztV+Nc0dxjXgL80100YBOA==",
    };
    std::string str;
    for (const char* s : b64) { str += s; }
    return fpsdk::common::string::Base64Dec(str);
}();
