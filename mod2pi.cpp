#include    "LinearMPC_header.h"

#define fix(x) ((x)>0? floor(x) : ceil(x))

double LinearMPC_app::mod2pi(
    double input
){
    while (input >= PI ||input <= -PI) {
        if(input >= PI){
            input -= 2*PI;
        }
        else {
            input += 2*PI;
        }
    }

    return input;
}
