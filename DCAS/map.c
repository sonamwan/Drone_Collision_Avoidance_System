//
// Created by Sonam Wangchuk on 10/11/17.
//

#include "map.h"
#include "drone.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

Map createMap(int x, int y){
    Map m = {0};

    //	Base
    m.base[0].point[0] = x;
    m.base[0].point[1] = y;

    //	Up Runway
    m.base[1].point[0] = x-1;
    m.base[1].point[1] = y;
    m.runway[0] = 1;

    //	Down Runway
    m.base[2].point[0] = x+1;
    m.base[2].point[1] = y;
    m.runway[1] = 1;

    //	Left Runway
    m.base[3].point[0] = x;
    m.base[3].point[1] = y-1;
    m.runway[2] = 1;

    //	Right Runway
    m.base[4].point[0] = x;
    m.base[4].point[1] = y+1;
    m.runway[3] = 1;

    int i,j,k;
    int mid = SIZE/2;

    //Create get middle of rows in map
    for(i = 0;i < SIZE;i++){
        m.gridRow[i] = &(m.grid[i][0]) + mid;
    }

    m.map = m.gridRow + mid;

    //Defaults for each grid box
    Box grid;
    grid.drone = 0;
    grid.id = 0;
    grid.item = '.';
    grid.locked = 0;
    grid.dest = 0;
    pthread_mutex_init(&(grid.mutexLock), NULL);

    // Initiate grid to each box on the map
    for(i = -20;i <= 20;i++){
        for(j = -20;j <= 20;j++){
            grid.coord.point[0] = i;
            grid.coord.point[1] = j;
            m.map[i][j] = grid;
            //Sets base and runway
            for(k = 0;k<5;k++){
                if(m.map[i][j].coord.point[0] == m.base[k].point[0] && m.map[i][j].coord.point[1] == m.base[k].point[1]){
                    m.map[i][j].item = 'x';
                }
            }
        }
    }

    return m;
}

void printMap(Map *m){
    int i,j;
    printf("-------------------------------------------\n");

    for (i = -20;i <= 20;i++){
        printf("|");
        for (j = -20;j <= 20;j++){
            printf("%c",m->map[i][j].item);
        }
        printf("|");
        printf("\n");
    }
    printf("-------------------------------------------\n");
    printf("Drones in base:%d\n",m->map[0][0].drone);
}

Drone* orderDrone(Map *m,int id){
    Drone *d = malloc(sizeof(Drone));
    d->id = id;
    d->currLocation.point[0] = m->base[0].point[0];
    d->currLocation.point[1] = m->base[0].point[1];
    pthread_mutex_lock (&(m->map[m->base[0].point[0]][m->base[0].point[0]].mutexLock));
    m->map[m->base[0].point[0]][m->base[0].point[0]].drone += 1;
    pthread_mutex_unlock (&(m->map[m->base[0].point[0]][m->base[0].point[0]].mutexLock));
    d->state =  0;
    d->speed = 0;
    d->delivered = 0;
    d->package = 0;
    d->avoid = 0;
    d->avoid2 = 0;
    d->move = 0;
    return d;
}

void assignJob(Drone *d,Map *m){
    d->state = 1;
    int x,y;
    // Get random points
    do {
        x = (rand()%41) - 20;
        y = (rand()%41) - 20;
    }while((x == 0 && y == 0) || (x == 0 && y == 1) || (x == 0 && y == -1) || (x == 1 && y == 0) || (x == -1 && y == 0) || (m->map[x][y].locked == 1));
    d->destLocation.point[0] = x;
    d->destLocation.point[1] = y;
    pthread_mutex_lock (&(m->map[x][y].mutexLock));
    m->map[x][y].dest = 1;
    m->map[x][y].item = 'P';
    m->map[x][y].locked = 0;
    pthread_mutex_unlock (&(m->map[x][y].mutexLock));
    d->state += 1;
}

void getPackage(Drone *d){
    d->state = 2;
    d->package = 1;
    d->delivered = 0;
    d->state += 1;
}

void returnHome(Drone* d,Map *m){
    d->state = 6;
    pthread_mutex_lock (&(m->map[d->destLocation.point[0]][d->destLocation.point[1]].mutexLock));
    m->map[d->destLocation.point[0]][d->destLocation.point[1]].dest = 0;
    pthread_mutex_unlock (&(m->map[d->destLocation.point[0]][d->destLocation.point[1]].mutexLock));
    d->destLocation.point[0] = m->base[0].point[0];
    d->destLocation.point[1] = m->base[0].point[1];
    d->state += 1;
}

int requestTakeoff(Drone *d,Map *m){
    int i;
    d->state = 3;
    // Loop till a runway is free and set drone to runway
    while (1==1){
        for(i = 0;i < 4;i++){
            if(m->runway[i] == 1){
                pthread_mutex_lock (&(m->map[m->base[i+1].point[0]][m->base[i+1].point[0]].mutexLock));
                m->map[m->base[0].point[0]][m->base[0].point[0]].drone -= 1;
                m->runway[i] = 0;
                pthread_mutex_unlock (&(m->map[m->base[i+1].point[0]][m->base[i+1].point[0]].mutexLock));
                d->currLocation = m->base[i+1];
                d->state += 1;
                return 1;
            }
        }
    }
    return 0;
}

int requestLand(Drone *d,Map *m){
    int i;
    d->state = 8;
    // Loop till runway is free and return to base
    while(1==1){
        for(i = 0;i < 4;i++){
            if(m->runway[i] == 1){
                m->runway[i] = 0;
                d->destLocation = m->base[i+1];
                d->state += 1;
                return 1;
            }
        }
    }
    return 0;
}


void controlTower(Drone *d,Map *m){
    struct timespec tim, tim2;
    tim.tv_sec = 0;
    tim.tv_nsec = 100000000L;
    int run = 1;
    while(run == 1){
        switch(d->state){
            case 0:
                standby(d);
                break;
            case 1:
                assignJob(d,m);
                break;
            case 2:
                getPackage(d);
                break;
            case 3:
                requestTakeoff(d,m);
                break;
            case 4:
                move(d,m,1);
                nanosleep(&tim,&tim2);
                break;
            case 5:
                deliver(d);
                break;
            case 6:
                returnHome(d,m);
                break;
            case 7:
                move(d,m,0);
                nanosleep(&tim,&tim2);
                break;
            case 8:
                land(d,m);
                run = 0;
                break;
            default:
                printf("ERROR/n");
                run = 0;
                break;
        }
    }
}
