# Generates a ctags file containing the correct definition for the build
.PHONY: ctags
ctags:
	@echo "Generating ctags file..."
	@cat .dep/*.d | grep ":$$" | sed "s/://" | sort | uniq | xargs ctags --file-scope=no --extra=+q $(CSRC) $(CPPSRC)
