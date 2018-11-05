
#include <map>
#include <iostream>
#include <list>
#include <vector>
#include <queue>

struct fullname
{
	std::string first_name;
	std::string last_name;
	fullname(std::string newFirst, std::string newLast)
		: first_name(newFirst), last_name(newLast)
	{
	};
};



int main()
{
	

	std::vector<int> v{1, 2, 3, 4, 5};
	std::map<int,int> m{{1, 50},{2, 30},{3, 40}, {4, 1}, {5,100}};
	auto my_comp = [m](const int& lhs, const int& rhs){return m.at(lhs) < m.at(rhs);};

	std::priority_queue <int, std::vector<int>, decltype(my_comp)> queue(my_comp);

	queue.push(1);
	queue.push(2);
	queue.push(3);
	queue.push(4);
	queue.push(5);

	while (!queue.empty())
	{
		int p = queue.top();
		std::cout << p << std::endl;
		queue.pop();
	
	}

	/*

	std::vector<int> v{1, 2, 3, 4, 5};
	std::vector<int>::const_iterator it;

	it = std::find(v.begin(), v.end(), 10);


	if (it == v.end())
	{
		std::cout << "IT IS UNVISITED" << std::endl;
	}
	
	for (auto i : v)
	{
		std::cout << i <<  " ";
	}
	std::cout << std::endl;
	*/




    return 0;
}

