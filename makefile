a.out: main.ino
	particle compile --saveTo $@ argon $<

clean:
	rm -f *.bin
