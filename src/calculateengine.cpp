#include "calculateengine.h"

void calculate::offset(int fromservo1, int fromservo2, int fromservo3, int fromservo4){
    servo1offset1 = fromservo1;
    servo1offset2 = fromservo2;
    servo1offset3 = fromservo3;
    servo1offset4 = fromservo4;
}

void calculate::run(int *servo1, int *servo2, int *servo3, int *servo4, pidvar xpid, pidvar ypid, pidvar pidcomass){
    if(xpid.direction){
        servo1mpu =   xpid.value ;
        servo3mpu = -(xpid.value);
    }
    else{
        servo3mpu =   xpid.value ;
        servo1mpu = -(xpid.value);
    }

    if(ypid.direction){
        servo2mpu =   ypid.value ;
        servo4mpu = -(ypid.value);
    }
    else{
        servo4mpu =   ypid.value ;
        servo2mpu = -(ypid.value);
    }

    if(pidcomass.direction){
        servo1com =  pidcomass.value;
        servo2com = -(pidcomass.value);
        servo3com = -(pidcomass.value);
        servo4com =  pidcomass.value;
    }
    else{
        servo1com = -(pidcomass.value);
        servo2com =  pidcomass.value;
        servo3com =  pidcomass.value;
        servo4com = -(pidcomass.value);
    }
    
    *servo1 = servo1mpu + servo1com / 2;
    *servo2 = servo2mpu + servo2com / 2;
    *servo3 = servo3mpu + servo3com / 2;
    *servo4 = servo4mpu + servo4com / 2;
}
