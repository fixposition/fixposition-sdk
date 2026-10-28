// clang-format off
static const std::vector<uint8_t> TEST_DATA_NOVB_BIN = []() {
    // Separate string literals, as compilers may not support long strings (-Woverlength-strings)
    static const char* const b64[] = {
        "qkQSHCoAAKBIAAAAI7SDCGTbaAwACAADus0ZQAAAAAA4AAAAz2pIFUyzR0Dm6xlruOMgQAAAWDHShHhAZmY+Qj0AAACFLRE989YEPcZXUj0wAAAAAACAPwAAAAAWFRUQ",
        "ACEVMz9svP6qRBIc8QAAoHAAAAAjtIMIZNtoDAAIAAPPRBlAAAAAADgAAAAfPfjLH1JQQXjDTDp2YiNBb4WyF9rSUUEWnEU9M1UAPcesJT0AAAAAOAAAAETlXYMZk2A/",
        "8nCwa8/NWL+g7lSC/eEMvyh84TrKZek6+K7fOjAAAAAAAAAAAACAPwAAAAAWFRUQACEVMw3bDM2qRBMotgWDCFvbaAwEPYMIMqzi7TNqCUEA/if4BBHVCbxgYABU/mEA",
        "lA0BAOBjCACc9QIA6vGTjA==",
    };
    std::string str;
    for (const char* s : b64) { str += s; }
    return fpsdk::common::string::Base64Dec(str);
}();
