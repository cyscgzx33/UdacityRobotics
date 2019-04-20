#include <vector>
#include <iostream>
#include <random>

double frand() 
{
	std::random_device dev;
    std::mt19937 rng(dev());
    std::uniform_int_distribution<std::mt19937::result_type> dist6(1,100); // distribution in range [1, 6]
	double result = dist6(rng) / 100.0;
	return result;
}


int binarySearch(std::vector<double>& accum, double target) 
{
	int start = 0, end = accum.size() - 1;

	while (start + 1 < end )
	{
		int mid = start + (end - start) / 2;
		if (accum[mid] == target) return mid;
		else if (accum[mid] < target) start = mid;
		else end = mid;
	}

	if (target <= accum[start]) return start;
	else return end;
}

int main()
{
	std::vector<int> A = {1, 5, 3, 2, 4};
	std::vector<double> B = {0.11, 0.22, 0.33, 0.16, 0.18};

	std::vector<double> accum_B;
	double sum = 0;
	for (auto ele : B)
	{
		sum += ele;
		accum_B.push_back(sum);
	}

	std::vector<int> distribution(5, 0);
	for (int i = 0; i < 1000000; i++)
	{	
		int resample = binarySearch(accum_B, frand());
		distribution[resample]++;
	}

	for (auto res : distribution)
	{
		std::cout << res << std::endl;
	}

}
