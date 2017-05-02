all:
	@$(MAKE) -f Makefile.lcmgen all || exit 2;
	@make -C lcmtypes
	@make -C src

clean:
	@$(MAKE) -f Makefile.lcmgen clean
	@make -C src -s clean

