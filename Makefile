LDFLAGS=-L/usr/local/lib -lm -lrt -lopencv_core -lopencv_imgproc -lopencv_highgui -lopencv_imgcodecs
CFLAGS=-O3 -DDEBUG -Iinc/

PI_CFLAGS=${CFLAGS} -mfpu=vfp
PI_LDFLAGS=${LDFLAGS} -lpigpio

EXEC_NAME=detect_line

SRC_DIR=src/
EXAMPLES_DIR=tests/
OBJS_DIR=build/

SRC_FILES=$(shell ls ${SRC_DIR})
EXAMPLES_FILES=$(shell ls ${EXAMPLES_DIR})

OBJ_FILES=$(SRC_FILES:.c=.o) \
	$(LIB_FILES:.c=.o)

OBJS=$(addprefix ${OBJS_DIR},${OBJ_FILES})

all : test_detect_line

clean :
	rm -Rf ${OBJS_DIR} test_detect_line
	
test_detect_line : ${OBJS_DIR}/test_detect_line.o ${OBJS}
	gcc -o $@ ${OBJS_DIR}/test_detect_line.o ${OBJS} ${LDFLAGS}

${OBJS_DIR}%.o : ${EXAMPLES_DIR}%.c
	mkdir -p ${OBJS_DIR}
	gcc ${CFLAGS} -c $< -o $@

${OBJS_DIR}%.o : ${SRC_DIR}%.c
	mkdir -p ${OBJS_DIR}
	gcc ${CFLAGS} -c $< -o $@

${OBJS_DIR}%.o : ${LIB_DIR}%.c
	mkdir -p ${OBJS_DIR}
	gcc ${CFLAGS} -c $< -o $@

