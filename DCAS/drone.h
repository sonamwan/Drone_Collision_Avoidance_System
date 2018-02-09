//
// Created by sonam Wangchuk on 10/11/17.
//

#ifndef DCAS_DRONE_H
#define DCAS_DRONE_H

#include "attribute.h"

//Sets drone state to standby
void standby(Drone *d);

//Allows drone to takeoff
void takeoff(Drone *d);

//Moves drone around map
int move(Drone *d,Map *m,int state);

////Drone has crashed
//void collision(Drone *d);

//Drone reached destination and delivers
void deliver(Drone *d);

//Drone can land
void land(Drone *d,Map *m);

#endif // DRONE_H_
