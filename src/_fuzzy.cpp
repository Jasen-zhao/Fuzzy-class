#include "_fuzzy.h"
#include "fuzzy_controller.h"

//返回实例方法里面更新数据后的值
float getData(float a,float b) {
    Fuzzy_controller fuzzy(50, 1, 1);//emax,demax,umax
    fuzzy.default_set();
    float u = fuzzy.realize(a,b);
    return u;
}
