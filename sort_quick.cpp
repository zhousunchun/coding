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

int adjust(std::vector<int> & nums, int left, int right) {
    int temp = nums[left];
    while(left < right){
        while(left < right && nums[right] >= temp)
            right--;
        if(left<right)
        {
            nums[left] = nums[right];
            left++;
        }

        while(left < right && nums[left] < temp)
            left++;
        if(left<right)
        {
            nums[right] = nums[left];
            right--;
        }
    }
    nums[left] = temp;
    return left;
}
void quick_sort(std::vector<int> & nums, int left, int right)
{
    if (left < right)
    {
        int mid = adjust(nums, left, right);
        quick_sort(nums, left, mid-1);
        quick_sort(nums, mid+1, right);
    }
}

int main() {
    std::vector<int> nums;
    generate_vec(nums, 10);
    print_vector(nums);
    quick_sort(nums, 0, nums.size()-1);
    print_vector(nums);
    return 0;
}
