#include <iostream>
#include <vector>
#include <random>
#include <chrono>

void generate_vec(std::vector<int> & nums , const int& n)
{
    nums.clear();
    std::default_random_engine re(time(nullptr));
    std::uniform_int_distribution<int> distr(0,1000);
    nums.reserve(n);
    for(int i=0; i < n; i++)
        nums.emplace_back(distr(re));
}


void print_vector(const std::vector<int> & nums)
{
    for(int i= 0; i <  nums.size(); i++)
        std::cout << nums[i] << " ";
    std::cout << std::endl;
}
/**
 * (a + b) % p = (a%p + b%p) %p
 * (a - b) % p = (a%p - b%p) %p
 * (a*b)%p = (a%p *b%p) %p
 * (a * b *c)%p = (a%p * b%p * c%p) %p
 *
 */


long long forceNormalPower(long long base, long long power, int mod) {
    long long result = 1;
    for(int i =0;  i< power; i++)
    {
        result = result * base;
        result = result %mod;
    }
    return result %mod;
}

long long fastPower(long long base, long long power, int mod){
    long long result = 1;
    while(power > 0) {
        if(power %2 == 0) {
            power = power /2;
            base = base * base %mod;
        }
        else {
            power = power -1;
            result = result* base %mod;
            power = power/2;
            base = base * base %mod;
        }
    }
    return result;
}

long long fineFastPower(long long base, long long power, int mod){
    long long result = 1;
    while(power>0)
    {
        if(power&1)
        {
            result = result * base %mod;
        }
        power = power>>1;
        base = base * base %mod;
    }
    return result;
}
int main() {
    std::vector<int> nums;
    generate_vec(nums, 1);
    int base = 2;
    int power = 100000000;
    int mod = 1000;
    std::printf("(%d ^ %d) mod %d = %d", base, power, mod, fineFastPower(base, power, mod));
    std::cout << std::endl;
    {
        std::cout << "Force noraml power cost: ";
        auto start = std::chrono::high_resolution_clock::now();
        forceNormalPower(base, power, mod);
        auto end = std::chrono::high_resolution_clock::now();
        std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() << " ns" <<std::endl;
    }

    {
        std::cout << "Force noraml power cost: ";
        auto start = std::chrono::high_resolution_clock::now();
        fastPower(base, power, mod);
        auto end = std::chrono::high_resolution_clock::now();
        std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() << " ns" << std::endl;
    }

    {
        std::cout << "Force noraml power cost: ";
        auto start = std::chrono::high_resolution_clock::now();
        fineFastPower(base, power, mod);
        auto end = std::chrono::high_resolution_clock::now();
        std::cout << std::chrono::duration_cast<std::chrono::nanoseconds>(end - start).count() << " ns" <<std::endl;
    }
    return 0;
}
