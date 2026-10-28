INPUTS := $(wildcard *.bin)
OUTPUTS := $(patsubst %.bin, %.cpp, $(INPUTS))

# $ (info INPUTS=$(INPUTS))
# $ (info OUTPUTS=$(OUTPUTS))

all: $(OUTPUTS)

.SUFFIXES:

%.cpp: %.bin Makefile
	@echo "$< -> $@"
	@echo -n > $@.tmp
	@echo "// clang-format off" >> $@.tmp
	@echo "static const std::vector<uint8_t> $(shell echo $(subst .,_,$<) | tr a-z A-Z) = []() {" >> $@.tmp
	@echo "    // Separate string literals, as compilers may not support long strings (-Woverlength-strings)" >> $@.tmp
	@echo "    static const char* const b64[] = {" >> $@.tmp
	@base64 -w 128 $< | sed -e 's/^/        "/' -e 's/$$/",/' >> $@.tmp
	@echo "    };" >> $@.tmp
	@echo "    std::string str;" >> $@.tmp
	@echo "    for (const char* s : b64) { str += s; }" >> $@.tmp
	@echo "    return fpsdk::common::string::Base64Dec(str);" >> $@.tmp
	@echo "}();" >> $@.tmp
	@mv $@.tmp $@




