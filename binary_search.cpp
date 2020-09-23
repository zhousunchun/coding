/**
* This programe is about binary search 
*/

#include <iostream>
#include <vector>
using namespace std;

int bindary_search(const std::vector<int> & nums, int target)
{
    int left = 0;
    int right = nums.size() - 1;
    while(left <= right)
    {
        int mid = left +(right-left)/2;
        if(target == nums[mid])
            return mid;
        else if (target < nums[mid])
            right = mid -1;
        else if (target > nums[mid])
            left = mid +1;
    }
    return -1;
}

int left_bound(const std::vector<int> & nums, int target)
{
    int left = 0;
    int right = nums.size();
    while(left < right)
    {
        int mid = left + (right-left)/2;
        if(nums[mid] == target)
            right =mid;
        else if (nums[mid] < target)
           left = mid +1;
        else if (nums[mid]> target)
            right = mid;
    }
    if(nums[left] != target) return -1;
    return left;
}

int right_bound(const std::vector<int> & nums, int target)
{
    int left = 0;
    int right = nums.size();
    while(left < right)
    {
        int mid = left + (right-left)/2;
        if(nums[mid] == target)
            left =mid+1;
        else if (nums[mid] < target)
           left = mid +1;
        else if (nums[mid]> target)
            right = mid;
    }
    if(nums[left -1] != target) return -1;
    return left -1;
}

int main()
{
    std::vector<int> nums;
    nums.push_back(1);
    nums.push_back(2);
    nums.push_back(2);
    nums.push_back(2);
    nums.push_back(5);
    
    cout << bindary_search(nums,5) << endl;
    cout <<bindary_search(nums,1) << endl;
    cout << bindary_search(nums, 2) << endl;
    cout << bindary_search(nums,0) << endl;
    cout << bindary_search(nums, 7) << endl;
    cout<< "==============================" <<endl;
    cout << left_bound(nums, 5) << std::endl;
    cout <<left_bound(nums,1) << endl;
    cout << left_bound(nums, 2) << endl;
    cout << left_bound(nums,0) << endl;
    cout << left_bound(nums, 7) << endl;
    cout<< "============================" << endl;
    cout << right_bound(nums, 5) << std::endl;
    cout << right_bound(nums,1) << endl;
    cout << right_bound(nums, 2) << endl;
    cout << right_bound(nums,0) << endl;
    cout << right_bound(nums, 7) << endl;
    
    return 0;
}
