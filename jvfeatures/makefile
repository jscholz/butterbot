CFLAGS=-g
cc=g++
PREFIX := /usr/local

#CV_INCLUDE=`pkg-config --cflags opencv`
#CV_LIB=`pkg-config --libs opencv`
CV_INCLUDE=-I/usr/include/opencv
CV_LIB=-lcxcore -lcv -lhighgui -lcvaux -lml

.PHONY: all clean install uninstall

all: jvtest

## use this if libjvfeatures is already installed
#jvtest: jvtest.cpp
## pick one:
#	$(cc) $(CFLAGS) $^ -o $@ -ljvfeatures `pkg-config --cflags --libs opencv`
#	$(cc) -static $(CFLAGS) $^ -o $@ -ljvfeatures `pkg-config --cflags --libs opencv`

chessSeg: chessSeg.cpp jvfeatures.o jvtypes.o
	$(cc) $(CFLAGS) $^ -o $@ $(CV_LIB)

jvtest: jvtest.cpp jvfeatures.o jvtypes.o
	$(cc) $(CFLAGS) $^ -o $@ $(CV_LIB)
	
libjvfeatures.so: jvfeatures.cpp jvtypes.o
	$(cc) $(CFLAGS) -shared  -Wl,-soname,$@ -o $@ $^

jvfeatures.o: jvfeatures.cpp
	$(cc) $(CFLAGS) -c $<
	
jvtypes.o: jvtypes.cpp
	$(cc) $(CFLAGS) -c $<

libjvfeatures.a: jvfeatures.o jvtypes.o
	ar r $@ $^
	
clean:
	rm -vf *~ libjvfeatures.so libjvfeatures.a jvfeatures.o jvtest 

install: jvtest libjvfeatures.a libjvfeatures.so
	mkdir -p $(PREFIX)/lib
	mkdir -p $(PREFIX)/include
	mkdir -p $(PREFIX)/bin
	install --mode=755 libjvfeatures.so $(PREFIX)/lib
	install --mode=755 libjvfeatures.a $(PREFIX)/lib
	install --mode=644 jvfeatures.h $(PREFIX)/include
	install --mode=755 jvtest $(PREFIX)/bin
	
uninstall:
	rm -f $(PREFIX)/bin/jvtest
	rm -f $(PREFIX)/lib/libjvfeatures.so
	rm -f $(PREFIX)/lib/libjvfeatures.a
	rm -f $(PREFIX)/include/jvfeatures.h