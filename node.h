#pragma once
#include <list>


using namespace std;

struct node
{
	int x, y;
	bool obstacle=false;
	bool visited=false;
	bool marked = false;
	float fGlobalGoal;
	float fLocalGoal;
	std::list<node*> vecNeighbors;
	node* parent=nullptr;
public:
	node(int x,int y);
	~node();
};

