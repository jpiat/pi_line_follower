OBJS = objs

CFLAGS += -I/usr/local/include/opencv -I/usr/local/include
LDFLAGS += -L/usr/local/lib -lopencv_highgui -lopencv_core -lopencv_video -lopencv_features2d -lopencv_calib3d -lopencv_imgproc -lpthread -lm
CFLAGS += -O3 -mfpu=vfp
LDFLAGS += -lraspicamcv -lm -lrt -lwiringPi -ljpeg


all: capture_calibrate

%o: %c
	gcc -c $(CFLAGS) $< -o $@

capture_calibrate : capture_calibrate.o
	gcc $(LDFLAGS) -o $@ capture_calibrate.o

clean:
	rm -f *.o
	rm capture_calibrate
