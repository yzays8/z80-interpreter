SRCDIR = src

.PHONY: all
all:
	make -C $(SRCDIR)

.PHONY: clean
clean:
	make clean -C $(SRCDIR)

.PHONY: run
run:
	make run -C $(SRCDIR)