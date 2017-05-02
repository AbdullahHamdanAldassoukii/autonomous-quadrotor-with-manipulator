all:
	@make -C src --no-print-directory
	@make -C examples --no-print-directory
	@make -C template --no-print-directory

clean:
	#find . -type f | xargs -n 5 touch
	rm -f lib/*.so
	@make -C src -s clean
	@make -C examples -s clean
	@make -C template -s clean
	@echo "All Directories Cleaned"

