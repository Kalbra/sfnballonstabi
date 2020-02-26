#ifndef CALCULATE_H
#define CALCULATE_H

class pidvalue{
public:
    int value;
    bool direction;
};

class calculate{
public:
    void firstdefine(int* servo1, int* servo2, int* servo3, int* servo4, pidvalue xpid, pidvalue ypid);
};

#endif // CALCULATE_H
