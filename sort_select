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

void select_sort(std::vector<int> & nums)
{
    int len = nums.size();
    for (int i = 0; i < len; i++) {
        int minIndex = i;
        for (int j = i + 1; j < len; j++) {
            if (nums[j] < nums[minIndex]) {
                minIndex = j;
            }
        }
        int temp = nums[i];
        nums[i] = nums[minIndex];
        nums[minIndex] = temp;
    }
}

int main() {
    std::vector<int> nums;
    generate_vec(nums, 10);
    print_vector(nums);
    select_sort(nums);
    print_vector(nums);
    return 0;
}
