#ifndef CALCULATE_H
#define CALCULATE_H



class pidvar{
public:
    int value;
    bool direction;
};

class calculate{
public:
    void run(int*, int*, int*, int*, pidvar, pidvar, pidvar);
    void offset(int, int, int, int);
    void debug();

private:
    int servo1offset1 = 0;
    int servo1offset2 = 0;
    int servo1offset3 = 0;
    int servo1offset4 = 0;

    int servo1mpu;
    int servo2mpu;
    int servo3mpu;
    int servo4mpu;

    int servo1com;
    int servo2com;
    int servo3com;
    int servo4com;
};

#endif // CALCULATE_H
