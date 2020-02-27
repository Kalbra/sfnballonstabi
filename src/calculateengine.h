#ifndef CALCULATE_H
#define CALCULATE_H

struct pidvar{
    int value;
    bool direction;
};

class calculate{
public:
    void run(int* servo1, int* servo2, int* servo3, int* servo4, pidvar  xpid, pidvar ypid);
    void offset(int fromservo1, int fromservo2, int fromservo3, int fromservo4);

private:
    int servo1offset1 = 0;
    int servo1offset2 = 0;
    int servo1offset3 = 0;
    int servo1offset4 = 0;
};

#endif // CALCULATE_H
