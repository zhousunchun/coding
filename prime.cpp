#include <iostream>
#include <vector>
#include <random>
#include <chrono>

bool isPrimeSqrt(int n)
{
    for(int i = 2; i< sqrt(n); i++)
    {
        if(n%i == 0)
        {
            return false;
        }
    }
    return true;
}

int main() {
    int num = 49999;
    auto start = std::chrono::high_resolution_clock::now();
    if (isPrimeSqrt(num))
        std::printf("%d is prime\n", num);
    else
            std::printf("%d is not prime\n", num);
    auto end = std::chrono::high_resolution_clock::now();
    auto timeCost = std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count();
    std::printf("sqrt check  %d  is prime cost  %d ns \n", num, timeCost);
    return 0;
}
