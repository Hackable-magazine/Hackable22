TARGET = premier

all: ${TARGET}

${TARGET}: ${TARGET}.bin
	xxd -c 16 -i ${TARGET}.bin

${TARGET}.bin : ${TARGET}.asm
	z80asm -o ${TARGET}.bin ${TARGET}.asm --list=${TARGET}.lst

clean:
	rm -rf ${TARGET}.bin ${TARGET}.lst
