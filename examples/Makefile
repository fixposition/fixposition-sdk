.SUFFIXES:

.PHONY: build
build: build-fpb_measurements build-fusion_epoch build-parser_intro

.PHONY: clean
clean: clean-fpb_measurements clean-fusion_epoch clean-parser_intro



.PHONY: build-fpb_measurements
build-fpb_measurements:
	make -C fpb_measurements

.PHONY: clean-fpb_measurements
clean-fpb_measurements:
	make -C fpb_measurements clean



.PHONY: build-fusion_epoch
build-fusion_epoch:
	make -C fusion_epoch

.PHONY: clean-fusion_epoch
clean-fusion_epoch:
	make -C fusion_epoch clean



.PHONY: build-parser_intro
build-parser_intro:
	make -C parser_intro

.PHONY: clean-parser_intro
clean-parser_intro:
	make -C parser_intro clean
