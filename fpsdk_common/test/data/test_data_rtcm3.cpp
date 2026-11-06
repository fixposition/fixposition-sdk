// clang-format off
static const std::vector<uint8_t> TEST_DATA_RTCM3_BIN = []() {
    // Separate string literals, as compilers may not support long strings (-Woverlength-strings)
    static const char* const b64[] = {
        "0wAWRTAATRCTogAAAAAAAAAAAAAAAAAAAB+bL9MBIUNQAE0Qk6IAAEChpTKAAAAAIACAAH/d+6iiIqmlqSUkqiGqKQAAAAAAAG05qq1/SihxH9ktAI8XBn1uAagt4XwH",
        "jB1f9UGLB/4PUDXfhQoUuK3malp+oEHi3nYcoGJwZhaIG5/O3hTtbL9glPgO329E/hx3sTt8J6ey6G6MLjQZY2lP//egnizP/UwHj6ZXYXV3hCDgCVw//+tgTgVf+JOv",
        "pOUoQKCnpVKgYQd4AA9///dQcqFQGL5n//g3n/x/WxXYAODKMtzPMcSABO1BULCk1zIAAATTOQAQM1gABEfCgLwvC4KAlB8LQkCYLgqCUFAcC0KgRCgJQ4LnDICUQTg5",
        "gHPBEWovNH2HpZ7mH4g2r2qIBRBj9IwhV0HFHI02vYCRsnnTAC5AkAALTlVMTEFOVEVOTkEAABVSVEtCYXNlIFVibG94X1pFRC1GOVAFMi4zLjAAfJL7",
    };
    std::string str;
    for (const char* s : b64) { str += s; }
    return fpsdk::common::string::Base64Dec(str);
}();
