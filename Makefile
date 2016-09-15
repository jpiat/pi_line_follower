LDFLAGS=-L/usr/local/lib -lm -lrt -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_imgcodecs -lpigpio
CFLAGS=-O3 -Wall -DDEBUG -Iinc/ -Iinc/Eigen -mfpu=vfp

VPATH=src:src/fast:tests

PI_CFLAGS=${CFLAGS} -mfpu=vfp
PI_LDFLAGS=${LDFLAGS} -lpigpio

EXEC_NAME=detect_line 

SRC_DIR=src/

EXAMPLES_DIR=tests/
OBJS_DIR=build/

C_SRC_FILES=$(shell find ${SRC_DIR} -name '*.c' -exec basename {} \;)
CPP_SRC_FILES=$(shell find ${SRC_DIR} -name '*.cpp' -exec basename {} \;)
EXAMPLES_FILES=$(shell ls ${EXAMPLES_DIR})

OBJ_FILES=$(C_SRC_FILES:.c=.o)
OBJ_FILES+=$(CPP_SRC_FILES:.cpp=.o)


OBJS=$(addprefix ${OBJS_DIR},${OBJ_FILES})

all : test_detect_line test_visual_odometry

clean :
	rm -Rf ${OBJS_DIR} test_detect_line
	
test_detect_line : ${OBJS_DIR}/test_detect_line.o ${OBJS}
	g++ -o $@ ${OBJS_DIR}/test_detect_line.o ${OBJS} ${LDFLAGS}

test_visual_odometry : ${OBJS_DIR}/test_visual_odometry.o ${OBJS}
	g++ -o $@ ${OBJS_DIR}/test_visual_odometry.o ${OBJS} ${LDFLAGS}
	
${OBJS_DIR}%.o : %.c
	mkdir -p ${OBJS_DIR}
	gcc ${CFLAGS} -c $< -o $@
	
${OBJS_DIR}%.o : ${EXAMPLES_DIR}%.c
	mkdir -p ${OBJS_DIR}
	gcc ${CFLAGS} -c $< -o $@

${OBJS_DIR}%.o : ${EXAMPLES_DIR}%.cpp
	mkdir -p ${OBJS_DIR}
	g++ ${CFLAGS} -c $< -o $@
	
${OBJS_DIR}%.o : %.cpp
	mkdir -p ${OBJS_DIR}
	g++ ${CFLAGS} -c $< -o $@
