//
// Created by sonam Wangchuk on 10/11/17.
//

#include "map.h"
#include "drone.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

void* action(void* drone){
    Tracker *t = (Tracker*)drone;
    Drone *d = t->drone;
    Map *m = t->map;

    controlTower(d,m);

    pthread_exit(NULL);
}

void* printer(void* map){
    struct timespec tim, tim2;
    tim.tv_sec = 0;
    tim.tv_nsec = 100000000L;

    Tracker *t = (Tracker*)map;
    Map *m = t->map;

    sleep(1);
    while(m->map[0][0].drone != t->count){
        printMap(m);
        nanosleep(&tim , &tim2);
    }

    pthread_exit(NULL);
}

int main(int argc,char *argv[]){
    //can adjust droneCount to make it higher
    int droneCount = 8;

    srand(time(NULL));
    struct timespec tim, tim2;
    tim.tv_sec = 0;
    tim.tv_nsec = 100000000L;

    pthread_t threads[droneCount+1];
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
    void *status;

    Map *m;
    Map map = createMap(0,0);
    m = &map;

    int i;

    Tracker t;
    t.map = m;
    t.count = droneCount;
    pthread_create(&threads[0], &attr, printer,(void*)&t);

    //Create Drones
    for(i = 1;i <= droneCount;i++){
        Drone *d = orderDrone(m,i);
        m->drones[i] = d;
        t.drone = m->drones[i];
        pthread_create(&threads[i], &attr, action,(void*)&t);
        nanosleep(&tim , &tim2);
        // sleep(1);
    }

    pthread_attr_destroy(&attr);


    //Wait for all drones to complete
    for(i = 1; i <=droneCount;i++){
        pthread_join(threads[i],&status);
    }

    pthread_join(threads[0],status);
    pthread_exit(NULL);
//  printf("Execute!");


}
