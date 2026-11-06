// clang-format off
static const std::vector<uint8_t> TEST_DATA_SPARTN_BIN = []() {
    // Separate string literals, as compilers may not support long strings (-Woverlength-strings)
    static const char* const b64[] = {
        "cwAQ4xi5oY1oWx+IoLrCN0PrP74I+FF/Opp4MoXG4tB+7jfLPWqP8UwnQw0qcPIBcwB6agi5oD3gWx1IkiDjV6GJJpv+TBj0FC4swWwjpwtm/MDJy6fAGILVWUrabqNr",
        "BtWnANXN07iYuzc8NmsTcPfVRhEAHrENRx7N3YuwQLq2hlxMJWLd/9vubJjl1K72uF7zPp+gbBu97LvqiDTzVzOaSilwnxSZHU+e3ZJIcCQoXBq0ZRx9L4Yx0z+m9qBD",
        "sAtfzK20oOjZ4WAZRue9ZC8JGoK2Hzcged5kYEqfLlJtoCCXq7bSV0THpNEI8NA85MiedZpm6th4JBN60NKrKRyvLjeUO1gxWFbhBEWZesZ/heGRmneFW2s9L1WMWecG",
        "fzKkJkS3RBAdpbv0WQD2GyoGx3MEB+QBTNBbHkj+krLW0eZvBzBoGI5Is+EmIEpzAljqGLmhj8BbGYjactJHtPYLWJpXniizKRPdrQMYgL7fzWf5rhe+nFM47UHwwtfK",
        "dzE9raK1GFNkpMHwFlOlHnapN5dSVEftQXPhcGOJMCl3GjmdAbICZW9LdW8BAfAhWk7GLSqjnzoIA5MtHoKU6Wez29d+vUlaRuUD6J567rp4bBJxsffH+8Vlp6iD+Lmb",
        "gwgp5oT1b42yAXTtJ8dXyUFjjrV8b/of+u78F/mbYKQWbKE+t+IWxtck0gbGmrI=",
    };
    std::string str;
    for (const char* s : b64) { str += s; }
    return fpsdk::common::string::Base64Dec(str);
}();
