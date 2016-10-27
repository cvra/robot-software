PROTOC = protoc
PROTOC_OPTS = --plugin=protoc-gen-nanopb=$(NANOPB_DIR)/generator/protoc-gen-nanopb \
			  --proto_path=$(MESSAGES)

NANOPB_DIR := $(MESSAGES)/../lib/nanopb

# Files for the nanopb core
NANOPB_CORE = $(NANOPB_DIR)/pb_encode.c \
			  $(NANOPB_DIR)/pb_decode.c \
			  $(NANOPB_DIR)/pb_common.c

PROTO = $(wildcard $(MESSAGES)/*.proto)

PROTOSRC = $(notdir $(PROTO:.proto=.pb.c)) \
		   $(NANOPB_CORE)
PROTOINC = $(NANOPB_DIR) $(BUILDDIR)/pb/


# Rule for building .pb.c and .pb.h
%.pb.c : $(MESSAGES)/%.proto
	mkdir -p $(BUILDDIR)/pb
	@$(COLOR_PRINTF) "Compiling $(notdir $<)"
	echo $(addprefix $(BUILDDIR)/, $(notdir $@))
	$(PROTOC) $(PROTOC_OPTS) --nanopb_out=. $<
