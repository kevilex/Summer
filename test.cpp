#include <iostream>
#include <math.h>

int main(){

for(int i = -20; i <= 20; i++){
    double x = (1.0 * i) / 100;
    double y = 0.2;
    //double z = (3.0 - sqrt(abs(pow(0.2, 2.0) - pow(x, 2.0))));
    double r = 0.2;
    double z = r * (cos(((3.14/2) / 40) * i));


    std::cout << x << "," << y << ","  << z << "\n";
}


}