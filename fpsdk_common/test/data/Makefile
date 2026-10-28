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
	@echo "static const std::vector<uint8_t> $(shell echo $(subst .,_,$<) | tr a-z A-Z) = fpsdk::common::string::Base64Dec(" >> $@.tmp
	@base64 -w 128 $< | sed -e 's/^/    "/' -e 's/$$/"/' >> $@.tmp
	@echo ");" >> $@.tmp
	@mv $@.tmp $@




