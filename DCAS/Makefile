all: map.o drone.o dcas.o DCAS_Output

map.o: map.c map.h drone.h attribute.h
	gcc -I DCAS/ -o map.o -c map.c -lpthread

drone.o: drone.c drone.h map.h attribute.h
	gcc -I DCAS/ -o drone.o -c drone.c -lpthread

dcas.o: dcas.c map.h drone.h attribute.h
	gcc -I DCAS/ -o dcas.o -c dcas.c -lpthread

DCAS_Output: dcas.o map.o drone.o
	gcc -o DCAS_lab1 map.o drone.o dcas.o -lpthread







