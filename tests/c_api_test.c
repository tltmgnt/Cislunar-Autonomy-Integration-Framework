#include "cislunar/c_api.h"
#include <stdio.h>
int main(void){
 ca_controller* c=ca_create();if(!c)return 1;
 ca_observation o={{0.82,0,0.01,0,0.12,0},0,1,290,1,1};
 double goal[6]={0.82,0,0.01,0,0.12,0};ca_decision d;
 if(ca_step(c,&o,goal,0.001,&d)!=0||d.mode!=0)return 2;
 o.sensor_valid=0;if(ca_step(c,&o,goal,0.001,&d)!=0||d.mode!=2)return 3;
 if(ca_step(NULL,&o,goal,0.001,&d)!=-1||d.mode!=2)return 4;
 ca_destroy(c);ca_destroy(NULL);puts("C ABI checks passed");return 0;
}
