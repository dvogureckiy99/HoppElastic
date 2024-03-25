#include <stdio.h> // define the header file  
#include <math.h>
#include <stdint.h>

#define ACCEL_cmd 0.006528 // cmd/msec^2
const uint16_t halfT = sqrt(2*(60*3.4099999999999997)/ACCEL_cmd);
uint32_t real_time_counter_4CPUticks = 0; //real time in cmd with dt=0.1220703125 msec
int a = 0;
uint16_t b = 0;

int main()
{
    printf( "%u\n", halfT );
    real_time_counter_4CPUticks = 145824;
    b =   fmod(real_time_counter_4CPUticks*0.1220703125, 2*halfT);
    printf( "%u\n", b );
    a +=  ACCEL_cmd*pow( ((float) real_time_counter_4CPUticks*0.1220703125),2)/2;
    a *= 0.293255131964809;
    printf( "%u", a );

    return 0;
}