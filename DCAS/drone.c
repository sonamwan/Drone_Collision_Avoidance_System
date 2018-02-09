//
// Created by sonam Wangchuk on 10/11/17.
//

#include "map.h"
#include "drone.h"

#include <stdio.h>
#include <unistd.h>

void standby(Drone *d){
    d->state = 0;
    d->state += 1;
}

void takeoff(Drone *d){
    d->state = 3;
    d->state += 1;
}

void setMap(Drone *d,Map *m){
    //	If item is a base or runway don't lock
    if((d->currLocation.point[0] == m->base[0].point[0]) && (d->currLocation.point[1] == m->base[0].point[1])){
        m->map[d->currLocation.point[0]][d->currLocation.point[1]].id = 0;
        m->map[d->currLocation.point[0]][d->currLocation.point[1]].locked = 0;
        m->map[d->currLocation.point[0]][d->currLocation.point[1]].item = 'B';
    }
    else{
        m->map[d->currLocation.point[0]][d->currLocation.point[1]].drone += 1;
        m->map[d->currLocation.point[0]][d->currLocation.point[1]].id = d->id;
        m->map[d->currLocation.point[0]][d->currLocation.point[1]].locked = 1;
        if(m->map[d->currLocation.point[0]][d->currLocation.point[1]].item == 'x'){
            m->map[d->currLocation.point[0]][d->currLocation.point[1]].item = 'x';
        }
        else{
            m->map[d->currLocation.point[0]][d->currLocation.point[1]].item = 'D';
        }
    }
}

void removeOld(Map *m,Coord c){
    int k;
    if(c.point[0] != m->base[0].point[0] && c.point[1] != m->base[0].point[1]){
        m->map[c.point[0]][c.point[1]].drone -= 1;
    }
    m->map[c.point[0]][c.point[1]].id = 0;
    m->map[c.point[0]][c.point[1]].locked = 0;
    //	Sets print out item
    if (m->map[c.point[0]][c.point[1]].item == 'B'){
        m->map[c.point[0]][c.point[1]].item = 'B';
    }
    else if (m->map[c.point[0]][c.point[1]].item == 'x') {
        m->map[c.point[0]][c.point[1]].item = 'x';
    }
    else if(m->map[c.point[0]][c.point[1]].dest == 1){
        m->map[c.point[0]][c.point[1]].item = 'P';
    }
    else{
        m->map[c.point[0]][c.point[1]].item = '.';
    }
}

//	dir movement is either 0 = negative or 1 = positive
void setAvoid(Map *m,Drone *d,Coord c,int dir,int axis){
    if(axis == 0){
        if(dir == 0){
            pthread_mutex_lock (&(m->map[d->currLocation.point[0]-1][d->currLocation.point[1]].mutexLock));
            pthread_mutex_lock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]].mutexLock));
            d->currLocation.point[0] -= 1;
            setMap(d,m);
            removeOld(m,c);
            pthread_mutex_unlock (&(m->map[d->currLocation.point[0]+1][d->currLocation.point[1]].mutexLock));
            pthread_mutex_unlock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]].mutexLock));
        }
        else if(dir == 1){
            pthread_mutex_lock (&(m->map[d->currLocation.point[0]+1][d->currLocation.point[1]].mutexLock));
            pthread_mutex_lock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]].mutexLock));
            d->currLocation.point[0] += 1;
            setMap(d,m);
            removeOld(m,c);
            pthread_mutex_unlock (&(m->map[d->currLocation.point[0]-1][d->currLocation.point[1]].mutexLock));
            pthread_mutex_unlock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]].mutexLock));
        }
    }
    else if(axis == 1){
        if(dir == 0){
            pthread_mutex_lock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]-1].mutexLock));
            pthread_mutex_lock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]].mutexLock));
            d->currLocation.point[1] -= 1;
            setMap(d,m);
            removeOld(m,c);
            pthread_mutex_unlock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]+1].mutexLock));
            pthread_mutex_unlock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]].mutexLock));
        }
        else if(dir == 1){
            pthread_mutex_lock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]+1].mutexLock));
            pthread_mutex_lock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]].mutexLock));
            d->currLocation.point[1] += 1;
            setMap(d,m);
            removeOld(m,c);
            pthread_mutex_unlock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]-1].mutexLock));
            pthread_mutex_unlock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]].mutexLock));
        }
    }
}

//	Navigates around items and prevents drone from looping in a circle
int isLocked(Drone *d,Map *m,Coord c,Coord locked,int lock){
    if(m->map[locked.point[0]][locked.point[1]].locked == 1){
        switch(lock){
            case 1:		//Up is blocked
                if(c.point[1] == 20){			//Check if right side is edge of map
                    if(c.point[0] == 20){		//Checks if corner
                        setAvoid(m,d,c,0,1);
                        d->avoid2 = d->avoid;
                        d->avoid = 3;
                    }
                    else if(m->map[c.point[0]][c.point[1]-1].locked == 1){
                        setAvoid(m,d,c,1,0);
                        d->avoid2 = d->avoid;
                        d->avoid = 2;
                    }
                    else{
                        setAvoid(m,d,c,0,1);
                        d->avoid2 = d->avoid;
                        d->avoid = 3;
                    }
                }
                else if(c.point[1] == -20){		//Check if left side is edge of map
                    if(c.point[0] == 20){		//checks if corner
                        setAvoid(m,d,c,1,1);
                        d->avoid2 = d->avoid;
                        d->avoid = 4;
                    }
                    else if(m->map[c.point[0]][c.point[1]+1].locked == 1){
                        setAvoid(m,d,c,1,0);
                        d->avoid2 = d->avoid;
                        d->avoid = 2;
                    }
                    else{
                        setAvoid(m,d,c,1,1);
                        d->avoid2 = d->avoid;
                        d->avoid = 4;
                    }
                }
                else if(c.point[0] == 20){		//Checks if bottom of map
                    if(d->destLocation.point[1] > d->currLocation.point[1]){
                        if(m->map[c.point[0]][c.point[1]+1].locked == 1){
                            setAvoid(m,d,c,0,1);
                            d->avoid2 = d->avoid;
                            d->avoid = 3;
                        }
                        else if(d->avoid != 3 && d->avoid2 != 3){
                            setAvoid(m,d,c,1,1);
                            d->avoid2 = d->avoid;
                            d->avoid = 4;
                        }
                        else{
                            setAvoid(m,d,c,0,1);
                            d->avoid2 = d->avoid;
                            d->avoid = 3;
                        }
                    }
                    else if(m->map[c.point[0]][c.point[1]-1].locked == 1){
                        setAvoid(m,d,c,1,1);
                        d->avoid2 = d->avoid;
                        d->avoid = 4;
                    }
                    else if(d->avoid != 4 && d->avoid2 != 4){
                        setAvoid(m,d,c,0,1);
                        d->avoid2 = d->avoid;
                        d->avoid = 3;
                    }
                    else{
                        setAvoid(m,d,c,1,1);
                        d->avoid2 = d->avoid;
                        d->avoid = 4;
                    }
                }
                else if(d->destLocation.point[1] >= d->currLocation.point[1]){
                    if(m->map[c.point[0]][c.point[1]+1].locked == 1){
                        if(m->map[c.point[0]][c.point[1]-1].locked == 1){
                            setAvoid(m,d,c,1,0);
                            d->avoid = 2;
                        }
                        else if(d->avoid != 4 && d->avoid2 != 4){
                            setAvoid(m,d,c,0,1);
                            d->avoid2 = d->avoid;
                            d->avoid = 3;
                        }
                        else{
                            setAvoid(m,d,c,1,0);
                            d->avoid2 = d->avoid;
                            d->avoid = 2;
                        }
                    }
                    else if(d->avoid != 3 && d->avoid2 != 3){
                        setAvoid(m,d,c,1,1);
                        d->avoid2 = d->avoid;
                        d->avoid = 4;
                    }
                    else if(m->map[c.point[0]][c.point[1]-1].locked == 1){
                        setAvoid(m,d,c,1,0);
                        d->avoid2 = d->avoid;
                        d->avoid = 2;
                    }
                    else{
                        setAvoid(m,d,c,0,1);
                        d->avoid2 = d->avoid;
                        d->avoid = 3;
                    }
                }
                else{
                    if(m->map[c.point[0]][c.point[1]-1].locked == 1){
                        if(m->map[c.point[0]][c.point[1]+1].locked == 1){
                            setAvoid(m,d,c,1,0);
                            d->avoid = 2;
                        }
                        else if(d->avoid != 3 && d->avoid2 != 3){
                            setAvoid(m,d,c,1,1);
                            d->avoid2 = d->avoid;
                            d->avoid = 4;
                        }
                        else{
                            setAvoid(m,d,c,1,0);
                            d->avoid2 = d->avoid;
                            d->avoid = 2;
                        }
                    }
                    else if(d->avoid != 4 && d->avoid2 != 4){
                        setAvoid(m,d,c,0,1);
                        d->avoid2 = d->avoid;
                        d->avoid = 3;
                    }
                    else if(m->map[c.point[0]][c.point[1]+1].locked == 1){
                        setAvoid(m,d,c,1,0);
                        d->avoid2 = d->avoid;
                        d->avoid = 2;
                    }
                    else{
                        setAvoid(m,d,c,1,1);
                        d->avoid2 = d->avoid;
                        d->avoid = 4;
                    }
                }
                break;
            case 2:		//Down is blocked
                if (c.point[1] == 20){
                    if(c.point[0] == -20){
                        setAvoid(m,d,c,0,1);
                        d->avoid2 = d->avoid;
                        d->avoid = 3;
                    }
                    else if(m->map[c.point[0]][c.point[1]-1].locked == 1){
                        setAvoid(m,d,c,0,0);
                        d->avoid2 = d->avoid;
                        d->avoid = 1;
                    }
                    else{
                        setAvoid(m,d,c,0,1);
                        d->avoid2 = d->avoid;
                        d->avoid = 3;
                    }
                }
                else if(c.point[1] == -20){
                    if(c.point[0] == -20){
                        setAvoid(m,d,c,1,1);
                        d->avoid2 = d->avoid;
                        d->avoid = 4;
                    }
                    else if(m->map[c.point[0]][c.point[1]+1].locked == 1){
                        setAvoid(m,d,c,0,0);
                        d->avoid2 = d->avoid;
                        d->avoid = 1;
                    }
                    else{
                        setAvoid(m,d,c,1,1);
                        d->avoid2 = d->avoid;
                        d->avoid = 4;
                    }
                }
                else if(c.point[0] == -20){
                    if(d->destLocation.point[1] > d->currLocation.point[1]){
                        if(m->map[c.point[0]][c.point[1]+1].locked == 1){
                            setAvoid(m,d,c,0,1);
                            d->avoid2 = d->avoid;
                            d->avoid = 3;
                        }
                        else if(d->avoid != 3 && d->avoid2 != 3){
                            setAvoid(m,d,c,1,1);
                            d->avoid2 = d->avoid;
                            d->avoid = 4;
                        }
                        else{
                            setAvoid(m,d,c,0,1);
                            d->avoid2 = d->avoid;
                            d->avoid = 3;
                        }
                    }
                    else if(m->map[c.point[0]][c.point[1]-1].locked == 1){
                        setAvoid(m,d,c,1,1);
                        d->avoid2 = d->avoid;
                        d->avoid = 4;
                    }
                    else if(d->avoid != 4 && d->avoid2 != 4){
                        setAvoid(m,d,c,0,1);
                        d->avoid2 = d->avoid;
                        d->avoid = 3;
                    }
                    else{
                        setAvoid(m,d,c,1,1);
                        d->avoid2 = d->avoid;
                        d->avoid = 4;
                    }
                }
                else if(d->destLocation.point[1] >= d->currLocation.point[1]){
                    if(m->map[c.point[0]][c.point[1]+1].locked == 1){
                        if(m->map[c.point[0]][c.point[1]-1].locked == 1){
                            setAvoid(m,d,c,0,0);
                            d->avoid = 1;
                        }
                        else if(d->avoid != 4 && d->avoid2 != 4){
                            setAvoid(m,d,c,0,1);
                            d->avoid2 = d->avoid;
                            d->avoid = 3;
                        }
                        else{
                            setAvoid(m,d,c,0,0);
                            d->avoid2 = d->avoid;
                            d->avoid = 1;
                        }
                    }
                    else if(d->avoid != 3 && d->avoid2 != 3){
                        setAvoid(m,d,c,1,1);
                        d->avoid2 = d->avoid;
                        d->avoid = 4;
                    }
                    else if(m->map[c.point[0]][c.point[1]-1].locked == 1){
                        setAvoid(m,d,c,0,0);
                        d->avoid2 = d->avoid;
                        d->avoid = 1;
                    }
                    else{
                        setAvoid(m,d,c,0,1);
                        d->avoid2 = d->avoid;
                        d->avoid = 3;
                    }
                }
                else{
                    if(m->map[c.point[0]][c.point[1]-1].locked == 1){
                        if(m->map[c.point[0]][c.point[1]+1].locked == 1){
                            setAvoid(m,d,c,0,0);
                            d->avoid = 1;
                        }
                        else if(d->avoid != 3 && d->avoid2 != 3){
                            setAvoid(m,d,c,1,1);
                            d->avoid2 = d->avoid;
                            d->avoid = 4;
                        }
                        else{
                            setAvoid(m,d,c,0,0);
                            d->avoid2 = d->avoid;
                            d->avoid = 1;
                        }
                    }
                    else if(d->avoid != 4 && d->avoid2 != 4){
                        setAvoid(m,d,c,0,1);
                        d->avoid2 = d->avoid;
                        d->avoid = 3;
                    }
                    else if(m->map[c.point[0]][c.point[1]+1].locked == 1){
                        setAvoid(m,d,c,0,0);
                        d->avoid2 = d->avoid;
                        d->avoid = 1;
                    }
                    else{
                        setAvoid(m,d,c,1,1);
                        d->avoid2 = d->avoid;
                        d->avoid = 4;
                    }
                }
                break;
            case 3:		//Left is blocked
                if(c.point[0] == 20){
                    if(c.point[1] == 20){
                        setAvoid(m,d,c,0,0);
                        d->avoid2 = d->avoid;
                        d->avoid = 1;
                    }
                    else if(m->map[c.point[0]-1][c.point[1]].locked == 1){
                        setAvoid(m,d,c,1,1);
                        d->avoid2 = d->avoid;
                        d->avoid = 4;
                    }
                    else{
                        setAvoid(m,d,c,0,0);
                        d->avoid2 = d->avoid;
                        d->avoid = 1;
                    }
                }
                else if(c.point[0] == -20){
                    if(c.point[1] == 20){
                        setAvoid(m,d,c,1,0);
                        d->avoid2 = d->avoid;
                        d->avoid = 2;
                    }
                    else if(m->map[c.point[0]+1][c.point[1]].locked == 1){
                        setAvoid(m,d,c,1,1);
                        d->avoid2 = d->avoid;
                        d->avoid = 4;
                    }
                    else{
                        setAvoid(m,d,c,1,0);
                        d->avoid2 = d->avoid;
                        d->avoid = 2;
                    }
                }
                else if(c.point[1] == 20){
                    if(d->destLocation.point[0] > d->currLocation.point[0]){
                        if(m->map[c.point[0]+1][c.point[1]].locked == 1){
                            setAvoid(m,d,c,0,0);
                            d->avoid2 = d->avoid;
                            d->avoid = 1;
                        }
                        else if(d->avoid != 1 && d->avoid2 != 1){
                            setAvoid(m,d,c,1,0);
                            d->avoid2 = d->avoid;
                            d->avoid = 2;
                        }
                        else{
                            setAvoid(m,d,c,0,0);
                            d->avoid2 = d->avoid;
                            d->avoid = 1;
                        }
                    }
                    else if(m->map[c.point[0]-1][c.point[1]].locked == 1){
                        setAvoid(m,d,c,1,0);
                        d->avoid2 = d->avoid;
                        d->avoid = 2;
                    }
                    else if(d->avoid != 2 && d->avoid2 != 2){
                        setAvoid(m,d,c,0,0);
                        d->avoid2 = d->avoid;
                        d->avoid = 1;
                    }
                    else{
                        setAvoid(m,d,c,1,0);
                        d->avoid2 = d->avoid;
                        d->avoid = 2;
                    }
                }
                else if(d->destLocation.point[0] >= d->currLocation.point[0]){
                    if(m->map[c.point[0]+1][c.point[1]].locked == 1){
                        if(m->map[c.point[0]-1][c.point[1]].locked == 1){
                            setAvoid(m,d,c,1,1);
                            d->avoid = 4;
                        }
                        else if(d->avoid != 2 && d->avoid2 != 2){
                            setAvoid(m,d,c,0,0);
                            d->avoid2 = d->avoid;
                            d->avoid = 1;
                        }
                        else{
                            setAvoid(m,d,c,1,1);
                            d->avoid2 = d->avoid;
                            d->avoid = 4;
                        }
                    }
                    else if(d->avoid != 1 && d->avoid2 != 1){
                        setAvoid(m,d,c,1,0);
                        d->avoid2 = d->avoid;
                        d->avoid = 2;
                    }
                    else if(m->map[c.point[0]-1][c.point[1]].locked == 1){
                        setAvoid(m,d,c,1,1);
                        d->avoid2 = d->avoid;
                        d->avoid = 4;
                    }
                    else{
                        setAvoid(m,d,c,0,0);
                        d->avoid2 = d->avoid;
                        d->avoid = 1;
                    }
                }
                else{
                    if(m->map[c.point[0]-1][c.point[1]].locked == 1){
                        if(m->map[c.point[0]+1][c.point[1]].locked == 1){
                            setAvoid(m,d,c,1,1);
                            d->avoid = 4;
                        }
                        else if(d->avoid != 1 && d->avoid2 != 1){
                            setAvoid(m,d,c,1,0);
                            d->avoid2 = d->avoid;
                            d->avoid = 2;
                        }
                        else{
                            setAvoid(m,d,c,1,1);
                            d->avoid2 = d->avoid;
                            d->avoid = 4;
                        }
                    }
                    else if(d->avoid != 2 && d->avoid2 != 2){
                        setAvoid(m,d,c,0,0);
                        d->avoid2 = d->avoid;
                        d->avoid = 1;
                    }
                    else if(m->map[c.point[0]+1][c.point[1]].locked == 1){
                        setAvoid(m,d,c,1,1);
                        d->avoid2 = d->avoid;
                        d->avoid = 4;
                    }
                    else{
                        setAvoid(m,d,c,1,0);
                        d->avoid2 = d->avoid;
                        d->avoid = 2;
                    }
                }
                break;
            case 4:		//Right is blocked
                if(c.point[0] == 20){
                    if(c.point[1] == 20){
                        setAvoid(m,d,c,0,0);
                        d->avoid2 = d->avoid;
                        d->avoid = 1;
                    }
                    else if(m->map[c.point[0]-1][c.point[1]].locked == 1){
                        setAvoid(m,d,c,0,1);
                        d->avoid2 = d->avoid;
                        d->avoid = 3;
                    }
                    else{
                        setAvoid(m,d,c,0,0);
                        d->avoid2 = d->avoid;
                        d->avoid = 1;
                    }
                }
                else if(c.point[0] == -20){
                    if(c.point[1] == 20){
                        setAvoid(m,d,c,1,0);
                        d->avoid2 = d->avoid;
                        d->avoid = 2;
                    }
                    else if(m->map[c.point[0]+1][c.point[1]].locked == 1){
                        setAvoid(m,d,c,0,1);
                        d->avoid2 = d->avoid;
                        d->avoid = 3;
                    }
                    else{
                        setAvoid(m,d,c,1,0);
                        d->avoid2 = d->avoid;
                        d->avoid = 2;
                    }
                }
                else if(c.point[1] == -20){
                    if(d->destLocation.point[0] > d->currLocation.point[0]){
                        if(m->map[c.point[0]+1][c.point[1]].locked == 1){
                            setAvoid(m,d,c,0,0);
                            d->avoid2 = d->avoid;
                            d->avoid = 1;
                        }
                        else if(d->avoid != 1 && d->avoid2 != 1){
                            setAvoid(m,d,c,1,0);
                            d->avoid2 = d->avoid;
                            d->avoid = 2;
                        }
                        else{
                            setAvoid(m,d,c,0,0);
                            d->avoid2 = d->avoid;
                            d->avoid = 1;
                        }
                    }
                    else if(m->map[c.point[0]-1][c.point[1]].locked == 1){
                        setAvoid(m,d,c,1,0);
                        d->avoid2 = d->avoid;
                        d->avoid = 2;
                    }
                    else if(d->avoid != 2 && d->avoid2 != 2){
                        setAvoid(m,d,c,0,0);
                        d->avoid2 = d->avoid;
                        d->avoid = 1;
                    }
                    else{
                        setAvoid(m,d,c,1,0);
                        d->avoid2 = d->avoid;
                        d->avoid = 2;
                    }
                }
                else if(d->destLocation.point[0] >= d->currLocation.point[0]){
                    if(m->map[c.point[0]+1][c.point[1]].locked == 1){
                        if(m->map[c.point[0]-1][c.point[1]].locked == 1){
                            setAvoid(m,d,c,0,1);
                            d->avoid = 3;
                        }
                        else if(d->avoid != 2 && d->avoid2 != 2){
                            setAvoid(m,d,c,0,0);
                            d->avoid2 = d->avoid;
                            d->avoid = 1;
                        }
                        else{
                            setAvoid(m,d,c,0,1);
                            d->avoid2 = d->avoid;
                            d->avoid = 3;
                        }
                    }
                    else if(d->avoid != 1 && d->avoid2 != 1){
                        setAvoid(m,d,c,1,0);
                        d->avoid2 = d->avoid;
                        d->avoid = 2;
                    }
                    else if(m->map[c.point[0]-1][c.point[1]].locked == 1){
                        setAvoid(m,d,c,0,1);
                        d->avoid2 = d->avoid;
                        d->avoid = 3;
                    }
                    else{
                        setAvoid(m,d,c,0,0);
                        d->avoid2 = d->avoid;
                        d->avoid = 1;
                    }
                }
                else{
                    if(m->map[c.point[0]-1][c.point[1]].locked == 1){
                        if(m->map[c.point[0]+1][c.point[1]].locked == 1){
                            setAvoid(m,d,c,0,1);
                            d->avoid = 3;
                        }
                        else if(d->avoid != 1 && d->avoid2 != 1){
                            setAvoid(m,d,c,1,0);
                            d->avoid2 = d->avoid;
                            d->avoid = 2;
                        }
                        else{
                            setAvoid(m,d,c,0,1);
                            d->avoid2 = d->avoid;
                            d->avoid = 3;
                        }
                    }
                    else if(d->avoid != 2 && d->avoid2 != 2){
                        setAvoid(m,d,c,0,0);
                        d->avoid2 = d->avoid;
                        d->avoid = 1;
                    }
                    else if(m->map[c.point[0]+1][c.point[1]].locked == 1){
                        setAvoid(m,d,c,0,1);
                        d->avoid2 = d->avoid;
                        d->avoid = 3;
                    }
                    else{
                        setAvoid(m,d,c,1,0);
                        d->avoid2 = d->avoid;
                        d->avoid = 2;
                    }
                }
                break;
        }
        return 1;
    }
    return 0;
}

int move(Drone *d,Map *m,int state){
    int i;
    //Checks if deliver or return
    if(state == 1){
        d->state = 4;
    }
    else if(state == 0){
        d->state = 7;
    }
    //If drone is at a runway then move it
    if(m->map[d->currLocation.point[0]][d->currLocation.point[1]].item == 'x'){
        for(i = 0;i < 4;i++){
            if(d->currLocation.point[0] == m->base[i+1].point[0] && d->currLocation.point[1] == m->base[i+1].point[1]){
                break;
            }
        }
        m->runway[i] = 1;
    }

    Coord c = d->currLocation;	//old location

    if(c.point[1] == d->destLocation.point[1] || c.point[0] == d->destLocation.point[0] || (d->destLocation.point[1] == 0 && d->destLocation.point[0] == 0)){
        d->move = 0;
    }

    //At Destination
    if ((d->destLocation.point[0] == d->currLocation.point[0]) && (d->destLocation.point[1] == d->currLocation.point[1])){
        d->state += 1;
        return 1;
    }
        //Move up
    else if(d->destLocation.point[0] < d->currLocation.point[0] && d->avoid != 2 && d->avoid2 != 2 && d->move == 0){
        Coord locked = c;
        locked.point[0] -= 1;
        if(isLocked(d,m,c,locked,1) == 0){
            pthread_mutex_lock (&(m->map[d->currLocation.point[0]-1][d->currLocation.point[1]].mutexLock));
            pthread_mutex_lock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]].mutexLock));
            d->currLocation.point[0] -= 1;
            d->avoid2 = d->avoid;
            d->avoid = 1;
            setMap(d,m);
            removeOld(m,c);
            pthread_mutex_unlock(&(m->map[d->currLocation.point[0]+1][d->currLocation.point[1]].mutexLock));
            pthread_mutex_unlock(&(m->map[d->currLocation.point[0]][d->currLocation.point[1]].mutexLock));
        }
    }
        //Move down
    else if(d->destLocation.point[0] > d->currLocation.point[0] && d->avoid != 1 && d->avoid2 != 1 && d->move == 0){
        Coord locked = c;
        locked.point[0] += 1;
        if(isLocked(d,m,c,locked,2) == 0){
            pthread_mutex_lock (&(m->map[d->currLocation.point[0]+1][d->currLocation.point[1]].mutexLock));
            pthread_mutex_lock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]].mutexLock));
            d->currLocation.point[0] += 1;
            d->avoid2 = d->avoid;
            d->avoid = 2;
            setMap(d,m);
            removeOld(m,c);
            pthread_mutex_unlock (&(m->map[d->currLocation.point[0]-1][d->currLocation.point[1]].mutexLock));
            pthread_mutex_unlock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]].mutexLock));
        }
    }
        //Move left
    else if(d->destLocation.point[1] < d->currLocation.point[1] && d->avoid != 4 && d->avoid2 != 4){
        Coord locked = c;
        locked.point[1] -= 1;
        if(isLocked(d,m,c,locked,3) == 0){
            pthread_mutex_lock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]-1].mutexLock));
            pthread_mutex_lock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]].mutexLock));
            d->currLocation.point[1] -= 1;
            d->avoid2 = d->avoid;
            d->avoid = 3;
            setMap(d,m);
            removeOld(m,c);
            pthread_mutex_unlock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]+1].mutexLock));
            pthread_mutex_unlock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]].mutexLock));
        }
        d->move = 1;
    }
        //Move right
    else if(d->destLocation.point[1] > d->currLocation.point[1] && d->avoid != 3 && d->avoid2 != 3){
        Coord locked = c;
        locked.point[1] += 1;
        if(isLocked(d,m,c,locked,4) == 0){
            pthread_mutex_lock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]+1].mutexLock));
            pthread_mutex_lock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]].mutexLock));
            d->currLocation.point[1] += 1;
            d->avoid2 = d->avoid;
            d->avoid = 4;
            setMap(d,m);
            removeOld(m,c);
            pthread_mutex_unlock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]-1].mutexLock));
            pthread_mutex_unlock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]].mutexLock));
        }
        d->move = 1;
    }
        //Move around object in left right
    else if(d->destLocation.point[0] == d->currLocation.point[0]){
        if (d->currLocation.point[0] == 20){
            Coord locked = c;
            locked.point[0] -= 1;
            if(isLocked(d,m,c,locked,1) == 0){
                pthread_mutex_lock (&(m->map[d->currLocation.point[0]-1][d->currLocation.point[1]].mutexLock));
                pthread_mutex_lock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]].mutexLock));
                d->currLocation.point[0] -= 1;
                d->avoid2 = d->avoid;
                d->avoid = 1;
                setMap(d,m);
                removeOld(m,c);
                pthread_mutex_unlock(&(m->map[d->currLocation.point[0]+1][d->currLocation.point[1]].mutexLock));
                pthread_mutex_unlock(&(m->map[d->currLocation.point[0]][d->currLocation.point[1]].mutexLock));
            }
        }
        else if(d->currLocation.point[0] == -20){
            Coord locked = c;
            locked.point[0] += 1;
            if(isLocked(d,m,c,locked,2) == 0){
                pthread_mutex_lock (&(m->map[d->currLocation.point[0]+1][d->currLocation.point[1]].mutexLock));
                pthread_mutex_lock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]].mutexLock));
                d->currLocation.point[0] += 1;
                d->avoid2 = d->avoid;
                d->avoid = 2;
                setMap(d,m);
                removeOld(m,c);
                pthread_mutex_unlock(&(m->map[d->currLocation.point[0]-1][d->currLocation.point[1]].mutexLock));
                pthread_mutex_unlock(&(m->map[d->currLocation.point[0]][d->currLocation.point[1]].mutexLock));
            }
        }
        else if(d->avoid != 1 && d->avoid2 != 1){
            Coord locked = c;
            locked.point[0] += 1;
            if(isLocked(d,m,c,locked,2) == 0){
                pthread_mutex_lock (&(m->map[d->currLocation.point[0]+1][d->currLocation.point[1]].mutexLock));
                pthread_mutex_lock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]].mutexLock));
                d->currLocation.point[0] += 1;
                d->avoid2 = d->avoid;
                d->avoid = 2;
                setMap(d,m);
                removeOld(m,c);
                pthread_mutex_unlock(&(m->map[d->currLocation.point[0]-1][d->currLocation.point[1]].mutexLock));
                pthread_mutex_unlock(&(m->map[d->currLocation.point[0]][d->currLocation.point[1]].mutexLock));
            }
        }
        else if(d->avoid != 2 && d->avoid2 != 2){
            Coord locked = c;
            locked.point[0] -= 1;
            if(isLocked(d,m,c,locked,1) == 0){
                pthread_mutex_lock (&(m->map[d->currLocation.point[0]-1][d->currLocation.point[1]].mutexLock));
                pthread_mutex_lock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]].mutexLock));
                d->currLocation.point[0] -= 1;
                d->avoid2 = d->avoid;
                d->avoid = 1;
                setMap(d,m);
                removeOld(m,c);
                pthread_mutex_unlock(&(m->map[d->currLocation.point[0]+1][d->currLocation.point[1]].mutexLock));
                pthread_mutex_unlock(&(m->map[d->currLocation.point[0]][d->currLocation.point[1]].mutexLock));
            }
        }
    }
        //Move around object in up down
    else if(d->destLocation.point[1] == d->currLocation.point[1]){
        if(d->currLocation.point[1] == 20){
            Coord locked = c;
            locked.point[1] -= 1;
            if(isLocked(d,m,c,locked,3) == 0){
                pthread_mutex_lock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]-1].mutexLock));
                pthread_mutex_lock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]].mutexLock));
                d->currLocation.point[1] -= 1;
                d->avoid2 = d->avoid;
                d->avoid = 3;
                setMap(d,m);
                removeOld(m,c);
                pthread_mutex_unlock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]+1].mutexLock));
                pthread_mutex_unlock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]].mutexLock));
            }
        }
        else if(d->currLocation.point[1] == -20){
            Coord locked = c;
            locked.point[1] += 1;
            if(isLocked(d,m,c,locked,4) == 0){
                pthread_mutex_lock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]+1].mutexLock));
                pthread_mutex_lock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]].mutexLock));
                d->currLocation.point[1] += 1;
                d->avoid2 = d->avoid;
                d->avoid = 4;
                setMap(d,m);
                removeOld(m,c);
                pthread_mutex_unlock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]-1].mutexLock));
                pthread_mutex_unlock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]].mutexLock));
            }
        }
        else if (d->avoid != 3 && d->avoid2 != 3){
            Coord locked = c;
            locked.point[1] += 1;
            if(isLocked(d,m,c,locked,4) == 0){
                pthread_mutex_lock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]+1].mutexLock));
                pthread_mutex_lock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]].mutexLock));
                d->currLocation.point[1] += 1;
                d->avoid2 = d->avoid;
                d->avoid = 4;
                setMap(d,m);
                removeOld(m,c);
                pthread_mutex_unlock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]-1].mutexLock));
                pthread_mutex_unlock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]].mutexLock));
            }
        }
        else if (d->avoid != 4 && d->avoid2 != 4){
            Coord locked = c;
            locked.point[1] -= 1;
            if(isLocked(d,m,c,locked,3) == 0){
                pthread_mutex_lock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]-1].mutexLock));
                pthread_mutex_lock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]].mutexLock));
                d->currLocation.point[1] -= 1;
                d->avoid2 = d->avoid;
                d->avoid = 3;
                setMap(d,m);
                removeOld(m,c);
                pthread_mutex_unlock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]+1].mutexLock));
                pthread_mutex_unlock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]].mutexLock));
            }
        }
    }
        //Last resort moves
    else if(d->avoid == 1){
        Coord locked = c;
        locked.point[0] -= 1;
        if(isLocked(d,m,c,locked,1) == 0){
            pthread_mutex_lock (&(m->map[d->currLocation.point[0]-1][d->currLocation.point[1]].mutexLock));
            pthread_mutex_lock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]].mutexLock));
            d->currLocation.point[0] -= 1;
            d->avoid2 = d->avoid;
            d->avoid = 1;
            setMap(d,m);
            removeOld(m,c);
            pthread_mutex_unlock (&(m->map[d->currLocation.point[0]+1][d->currLocation.point[1]].mutexLock));
            pthread_mutex_unlock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]].mutexLock));
        }
    }
    else if(d->avoid == 2){
        Coord locked = c;
        locked.point[0] += 1;
        if(isLocked(d,m,c,locked,2) == 0){
            pthread_mutex_lock (&(m->map[d->currLocation.point[0]+1][d->currLocation.point[1]].mutexLock));
            pthread_mutex_lock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]].mutexLock));
            d->currLocation.point[0] += 1;
            d->avoid2 = d->avoid;
            d->avoid = 2;
            setMap(d,m);
            removeOld(m,c);
            pthread_mutex_unlock(&(m->map[d->currLocation.point[0]-1][d->currLocation.point[1]].mutexLock));
            pthread_mutex_unlock(&(m->map[d->currLocation.point[0]][d->currLocation.point[1]].mutexLock));
        }
    }
    else if(d->avoid == 3){
        Coord locked = c;
        locked.point[1] -= 1;
        if(isLocked(d,m,c,locked,3) == 0){
            pthread_mutex_lock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]-1].mutexLock));
            pthread_mutex_lock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]].mutexLock));
            d->currLocation.point[1] -= 1;
            d->avoid2 = d->avoid;
            d->avoid = 3;
            setMap(d,m);
            removeOld(m,c);
            pthread_mutex_unlock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]+1].mutexLock));
            pthread_mutex_unlock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]].mutexLock));
        }
    }
    else if(d->avoid == 4){
        Coord locked = c;
        locked.point[1] += 1;
        if(isLocked(d,m,c,locked,4) == 0){
            pthread_mutex_lock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]+1].mutexLock));
            pthread_mutex_lock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]].mutexLock));
            d->currLocation.point[1] += 1;
            d->avoid2 = d->avoid;
            d->avoid = 4;
            setMap(d,m);
            removeOld(m,c);
            pthread_mutex_unlock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]-1].mutexLock));
            pthread_mutex_unlock (&(m->map[d->currLocation.point[0]][d->currLocation.point[1]].mutexLock));
        }
    }
    return 0;
}

void collision(Drone *d){
    d->state = 10;
}

void deliver(Drone *d){
    d->state = 5;
    d->delivered = 1;
    d->package = 0;
    d->state += 1;
    d->avoid = 0;
}

void land(Drone *d,Map *m){
    int i;
    d->state = 8;
    //Get runway number
    for(i = 0;i < 4;i++){
        if(d->currLocation.point[0] == m->base[i+1].point[0] && d->currLocation.point[1] == m->base[i+1].point[1]){
            break;
        }
    }
    d->currLocation = m->base[0];
    pthread_mutex_lock (&(m->map[m->base[0].point[0]][m->base[0].point[0]].mutexLock));
    m->map[m->base[0].point[0]][m->base[0].point[0]].drone += 1;
    m->runway[i] = 1;
    pthread_mutex_unlock (&(m->map[m->base[0].point[0]][m->base[0].point[0]].mutexLock));
    d->state = 0;
}
