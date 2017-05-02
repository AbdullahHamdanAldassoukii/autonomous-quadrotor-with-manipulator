# makefile to make all example programs

CWD = $(PWD)
SUBDIRS = $(shell find * -type d -print)
PREFIX := ../


#echo "Making Example: $$dir" ; \

all:
	@mkdir -p ../bin
	@for dir in $(SUBDIRS) ; do \
		$(MAKE) -C $$dir ; \
	done
	
clean:
	@for dir in $(SUBDIRS) ; do \
		echo "Cleaning Example: $$dir" ; \
		$(MAKE) -C $$dir -s clean; \
	done


