#ifndef _STATISTICS
#define _STATISTICS

double randNormal(double std){
    // emulate a normal distribution
    double n = 0;
    int iterations = 12;
    for(int i = 0; i < iterations; i++){
        n += (rand()%1000000)/1000000.0f;
    }
    return (n-double(iterations)/2.0f)*std;
}

#endif


