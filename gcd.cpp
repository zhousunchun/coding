#include <iostream>
#include <vector>
using namespace std;

int euclid_gcd(int a, int b) // greatest common divisor
{
    int temp;
    if(a <b ) {
        temp = a;
        a = b;
        b =temp;
    }
    while(b!= 0) {
        temp = a%b;
        a = b; 
        b = temp;
    }
    return a;
}


int lcm(int gcd, int a, int b) // least common multiple
{
    if(gcd != 0)
        return a * b / gcd;
    return 0;
}

int main()
{
    int gcd = euclid_gcd(4,9);
    cout<< gcd << endl;
    cout << lcm(gcd, 4,9) << std::endl;
    return 0;
}
