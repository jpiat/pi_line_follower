LDFLAGS=-lm
CFLAGS=-O3 -mfpu=neon -Iinc/ 

EXEC_NAME=detect_line

SRC_DIR=src/

OBJS_DIR=build/

SRC_FILES=$(shell ls ${SRC_DIR})

OBJ_FILES=$(SRC_FILES:.c=.o) \
	$(LIB_FILES:.c=.o) 

OBJS=$(addprefix ${OBJS_DIR},${OBJ_FILES})

all : ${EXEC_NAME}

clean :
	rm -Rf ${OBJS_DIR} radar
	
${EXEC_NAME} : ${OBJS}
	gcc -o $@ ${OBJS} ${LDFLAGS}

${OBJS_DIR}%.o : ${SRC_DIR}%.c
	mkdir -p ${OBJS_DIR}
	gcc ${CFLAGS} -c $< -o $@

${OBJS_DIR}%.o : ${LIB_DIR}%.c
	mkdir -p ${OBJS_DIR}
	gcc ${CFLAGS} -c $< -o $@

