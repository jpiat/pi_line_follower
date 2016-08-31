
CFLAGS=-Wall -O3
LDFLAGS=-pthread -lpigpio -lrt

%o: %c
	gcc -c $(CFLAGS) $< -o $@

servo_control : servo_control.o
	gcc $(CFLAGS) -o $@ $< $(LDFLAGS)

clean:
	rm *.o servo_control
