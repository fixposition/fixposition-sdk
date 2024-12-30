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
	@xxd -i -C -c 32 $< \
		| sed -e 's/unsigned char/static const std::vector<uint8_t>/' -e 's/\[\]//' -e '/unsigned int/d' \
		>> $@.tmp
	@mv $@.tmp $@




