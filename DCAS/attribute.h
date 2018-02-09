//
// Created by sonam Wangchuk on 10/11/17.
//
#ifndef DCAS_ATTRIBUTE_H_
#define DCAS_ATTRIBUTE_H_

#include <pthread.h>

#define SIZE 41

typedef struct Coord{
    int point[2];   //x == [0] &&
                    //y == [1]

} Coord;

typedef struct Box{
    Coord coord;    						//Box's coordinate
    int drone;      						//Number of Drones,
    int id;         						//Drone id
    int dest;								//Cood is a destination

    char item;   							// if map grid then '.',
                                            // if runway then 'x',
                                            // if destination then 'P' for package,
                                            // 'D' if drone

    int locked;     						//Box currently being used.
    pthread_mutex_t mutexLock;				//lock box for drone;
} Box;

typedef struct Drone{
    int id;         						//Drones id
    Coord currLocation,destLocation;        //Current and destination location
    int state;      						//State drone is in
    int speed;      						//Speed the drone moves
    int delivered;  						//Is package delivered?
    int package;    						//Does drone have a package?
    int avoid;								//Direction it moved from
    int avoid2;								//Direction it moved two steps ago
    int move;								//Avoid repeating
} Drone;

typedef struct Map{
    Box grid[SIZE][SIZE];   				//Original grid is 41 by 41 with Base in middle
    Box* gridRow[SIZE];     				//Array of pointers to each row
    Box** map;              				//Pointer of pointers to original grid
    Coord base[5];          				//Center Base and runways, Center (B) is base where drone takes off from
    int runway[4];          				//Are there any drones in runways
    Drone* drones[250];       				//Stores drones
} Map;

typedef struct Tracker{
    Map *map;								//Map for thread
    Drone *drone;							//Drone for thread
    int count;							    //Amount of drones
} Tracker;

#endif
