TESTCFLAGS=-g -Wall -Werror
TESTLINKER=-lcunit
TESTSOURCES=test.c sha1.c
TESTBINARY=sha1test

.FORCE: test
all:
	make test

clean:
	rm -rf *.o $(TESTBINARY)

test:
	$(CC) $(TESTCFLAGS) -o $(TESTBINARY) $(TESTSOURCES) -L ../sha1 liblmbapi.a liblmbio.a -lpthread -lm
