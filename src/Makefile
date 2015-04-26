PREFIX=build/
CROSS_COMPILE=x86_64-w64-mingw32-
CC=gcc
LD=gcc
CFLAGS=-std=c99
LDFLAGS=-L./ -linpoutx64
SRC=applesmc-win32.c ui.c
OBJ=applesmc-win32.o ui.o
EXE=applesmc-win32.exe

IOPOUTX64=inpoutx64.dll inpout32.h

.PHONY:				clean all install

all:				inpoutx $(OBJ)
					$(CROSS_COMPILE)$(LD) $(LDFLAGS) -o $(EXE) $(OBJ)
					
%.o:				%.c
					$(CROSS_COMPILE)$(CC) $(CFLAGS) -c -o $@ $<
					
inpoutx:			
					cp ../3rd-party/Inpoutx64/* .
					
install:			
					mkdir -p $(PREFIX)/
					cp *.exe *.dll $(PREFIX)/
					
uninstall:			
					rm -rf $(PREFIX)/
					
clean:				
					rm -f $(EXE) $(OBJ) $(IOPOUTX64)
