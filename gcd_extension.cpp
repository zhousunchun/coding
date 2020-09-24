#include <iostream>
#include <vector>
#include <random>

void generate_vec(std::vector<int> & nums , const int& n)
{
    std::default_random_engine re(time(nullptr));
    std::uniform_int_distribution<int> distr(0,10);
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

int gcd(int a, int b)
{
    return b == 0 ? a : gcd(b, a%b);
}

/**
 * a * x + b * y = gcd(a,b)
 * gcd(a,b) = gcd(b, a mod b)
 * a * x + b * y = gcd(a,b)
 *  = gcd(b, a mod b)
 *  = b * x1 + (a mod b) * y1
 *  = b * x1 + (a - a/b *b ) * y1
 *  ==>
 *  a * x + b * y = b * x1 + (a - a/b *b) * y1
 *   = b * x1 - b * a/ b * y1 + a * y1
 *   = a * y1 + b(x1 - a/b * y1)
 */
void extend_gcd(int a, int b, int & d, int &x, int & y)
{
    if(b == 0)
    {
        x = 1;  y= 0;d = a;
        return;
    }

    int x1, y1;
    extend_gcd(b, a%b, d, x1, y1);
    x = y1;
    y = x1 - (a/b) * y1;
    return;
}

int main() {
    std::vector<int> nums;
    generate_vec(nums, 2);
    print_vector(nums);
    int d, x, y;
    extend_gcd(nums[0],nums[1],d,x,y);
    std::cout << nums[0] << "*" << x << " + " << nums[1] << " * " << y << " = " << gcd(nums[0], nums[1]) << std::endl;
    return 0;
}
