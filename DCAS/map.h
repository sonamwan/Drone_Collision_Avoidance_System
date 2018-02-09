//
// Created by sonam Wangchuk on 10/11/17.
//
#ifndef DCAS_MAP_H_
#define DCAS_MAP_H_

#include "attribute.h"

//Generate default map with x,y coord as base
Map createMap(int x, int y);

//Printes out map
void printMap(Map *m);

//Generate new drone passing in an id
Drone* orderDrone(Map *m,int id);

//Assign job to drone
void assignJob(Drone *d, Map *m);
//void assignJob(Drone *d, Map *m, int x, int y);

//Get a package
void getPackage(Drone *d);

//Assign destenation to base
void returnHome(Drone *d,Map *m);

//Check runway for takeoff
int requestTakeoff(Drone *d,Map *m);

//Check runway to land

int requestLand(Drone *d,Map *m);

//Controls drone state and movement
void controlTower(Drone *d,Map * m);

#endif
