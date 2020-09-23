#include <iostream>
#include <vector>
#include <random>

void generate_vec(std::vector<int> & nums , const int& n)
{
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

void swap(int &a, int &b)
{
    int temp = a;
    a = b;
    b = a;
}

void buble_sort(std::vector<int> & nums)
{
    if(nums.empty())
        return;

    int len = nums.size();
    for(int i = 0; i < len; i++){
        for(int j= 0; j <len-i-1; j++){
            if(nums[j] > nums[j+1]){
                int temp = nums[j];
                nums[j] = nums[j+1];
                nums[j+1]= temp;
            }
        }
    }
}

int main() {
    std::vector<int> nums;
    generate_vec(nums, 10);
    print_vector(nums);
    buble_sort(nums);
    print_vector(nums);
    return 0;
}
