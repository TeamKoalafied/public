DEPS_CFLAGS?=$(shell env PKG_CONFIG_PATH=/usr/local/frc/lib/pkgconfig pkg-config --cflags wpilibc)
DEPS_LIBS?=$(shell env PKG_CONFIG_PATH=/usr/local/frc/lib/pkgconfig pkg-config --libs wpilibc)
EXE=multiCameraServerExample
DESTDIR?=/home/pi/

.PHONY: clean build install

build: ${EXE}

install: build
	rm -f ${DESTDIR}/${EXE}
	cp ${EXE} runCamera cameras.json ${DESTDIR}
	sudo cp frc-lines.json /boot/frc.json
	sudo svc -t /service/camera

clean:
	rm ${EXE} *.o

OBJS=main-2020.o GripPipeline.o

${EXE}: ${OBJS}
	${CXX} -pthread -g -o $@ $^ ${DEPS_LIBS} -Wl,--unresolved-symbols=ignore-in-shared-libs

.cpp.o:
	${CXX} -pthread -g -Og -c -o $@ ${CXXFLAGS} ${DEPS_CFLAGS} $<
