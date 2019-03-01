#include <SDL.h>
#include <iostream>
#include "node.h"
#include <list>
#undef main


int sourceX =-1, sourceY=-1;
int destX=-1, destY=-1;
bool ctrlPressed = false;
int tmpObstacleX = -1, tmpObstacleY = -1;
node *PathMap[10][10];
SDL_Rect *rects[10][10];
node *NodeEnd,* NodeStart;
bool drawPath = false;

void init() {
	for (size_t x = 0; x < 10; x++) {
		for (size_t y = 0; y < 10; y++)
		{
			PathMap[x][y] = new node(x, y);
			rects[x][y] = new SDL_Rect();
		}
	}

}



void findNeighbors(int x, int y) {
	node* aFromTile = PathMap[x][y];

	if (y > 0) {
		if (PathMap[x][y - 1]->visited == false && PathMap[x][y - 1]->obstacle == false) {
			aFromTile->vecNeighbors.push_front(PathMap[x][y - 1]);
		}
	}
	if (y < 9) {
		if (PathMap[x][y + 1]->visited == false && PathMap[x][y + 1]->obstacle == false) {
			aFromTile->vecNeighbors.push_front(PathMap[x][y + 1]);
		}
	}

	if (x > 0) {
		if (PathMap[x - 1][y]->visited == false && PathMap[x - 1][y]->obstacle == false) {
			aFromTile->vecNeighbors.push_front(PathMap[x - 1][y]);
		}
	}
	if (x < 9) {
		if (PathMap[x + 1][y]->visited == false && PathMap[x + 1][y]->obstacle == false) {
			aFromTile->vecNeighbors.push_front(PathMap[x + 1][y]);
		}
	}
	return;
}
void findallNeighbors() {
	for (size_t x = 0; x < 10; x++) {
		for (size_t y = 0; y < 10; y++)
		{
			findNeighbors(x, y);
		}
	}
	return;
}

void render(SDL_Renderer* renderer) {
	if (drawPath == true) {
		node* first= PathMap[NodeEnd->x][NodeEnd->y];

		node* p = PathMap[NodeEnd->x][NodeEnd->y];

		while (p->parent != nullptr) {
			if (p != first) {
				p->marked = true;
			}
			p = p->parent;
		}

		drawPath = false;
	}


	for (int i = 0; i < 10; i++) {
		for (int j = 0; j < 10; j++) {
			SDL_Rect *rect = rects[i][j];
			rect->h = 50;
			rect->w = 50;
			rect->x = 60 * i;
			rect->y = 60 * j;
			if (PathMap[i][j]->marked == true) {
				SDL_SetRenderDrawColor(renderer, 211, 211, 211, 255);
				SDL_RenderDrawRect(renderer, rect);
				SDL_RenderFillRect(renderer, rect);
			}else if (i == sourceX && j == sourceY || i == destX && j == destY) {
				SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
				SDL_RenderDrawRect(renderer, rect);
				SDL_RenderFillRect(renderer, rect);
			}else if(PathMap[i][j]->obstacle==true){
				SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
				SDL_RenderDrawRect(renderer, rect);
				SDL_RenderFillRect(renderer, rect);
			}
			else {
				SDL_SetRenderDrawColor(renderer, 0, 255, 0, 200);
				SDL_RenderDrawRect(renderer, rect);
				SDL_RenderFillRect(renderer, rect);
			}
		}
	}
	
	SDL_RenderPresent(renderer);

}



void SolveAstar(node* NodeStart, node* NodeEnd)
{
	for (int x = 0; x < 10; x++) {
		for (int y = 0; y < 10; y++) {
			PathMap[x][y]->visited = false;
			PathMap[x][y]->fGlobalGoal = FLT_MAX;
			PathMap[x][y]->fLocalGoal = FLT_MAX;
			PathMap[x][y]->parent = nullptr;
			PathMap[x][y]->marked = false;

		}
	}
	auto distance = [](node * a, node *b) {
		return sqrtf(static_cast<float>((((a->x) - (b->x))*((a->x) - (b->x)) + ((a->y) - (b->y))*((a->y) - (b->y)))));
	};
	auto heuristic = [distance](node* a, node*b) {
		return distance(a, b);
	};
	node* nodeCurrent = NodeStart;
	NodeStart->fLocalGoal = 0.0f;
	NodeStart->fGlobalGoal = heuristic(NodeStart, NodeEnd);


	list<node *> listOpenNodes;
	listOpenNodes.push_back(NodeStart);

	//stop searching when finding a path
	while (!listOpenNodes.empty()/* && nodeCurrent != NodeEnd*/) {
		//sort list into ascending order of global goals
		listOpenNodes.sort([](const node* lhs, const node* rhs) {return lhs->fGlobalGoal < rhs->fGlobalGoal; });
		//if the list is already visited pop
		while (!listOpenNodes.empty() && listOpenNodes.front()->visited)
			listOpenNodes.pop_front();
		//but this way we can end up with an empty list 
		if (listOpenNodes.empty())
			break;

		//so the first one is the possibly the best candidate to the shortest path
		nodeCurrent = listOpenNodes.front();
		nodeCurrent->visited = true;
		//check neighbors of current node
		for (auto nodeNeigbor : nodeCurrent->vecNeighbors) {
			//and push_back them unless they're visited or an wall
			if (!nodeNeigbor->visited && nodeNeigbor->obstacle == 0)
				listOpenNodes.push_back(nodeNeigbor);
			//calculate neighbors local goals
			float fPossiblyLowerGoal = nodeCurrent->fLocalGoal + distance(nodeCurrent, nodeNeigbor);
			if (fPossiblyLowerGoal < nodeNeigbor->fLocalGoal) {
				nodeNeigbor->parent = nodeCurrent;
				nodeNeigbor->fLocalGoal = fPossiblyLowerGoal;
				//and calculate global goal
				nodeNeigbor->fGlobalGoal = nodeNeigbor->fLocalGoal + heuristic(nodeNeigbor, NodeEnd);

			}

		}
	}

}




void mousePress(SDL_MouseButtonEvent& b) {
	if (b.button == SDL_BUTTON_LEFT && ctrlPressed == true) {
		SDL_GetMouseState(&tmpObstacleX, &tmpObstacleY);
		if (tmpObstacleX != -1 || tmpObstacleY != -1) {
			tmpObstacleX = tmpObstacleX / 60;
			tmpObstacleY = tmpObstacleY / 60;
			PathMap[tmpObstacleX][tmpObstacleY]->obstacle = true;
			if (NodeStart != nullptr && NodeEnd != nullptr) {
				SolveAstar(NodeStart, NodeEnd);
				drawPath = true;
			}
		}
	}
	else if (b.button == SDL_BUTTON_LEFT) {
		SDL_GetMouseState(&sourceX, &sourceY);
		if (sourceX != 0 || sourceY != 0) {
			sourceX = sourceX / 60;
			sourceY = sourceY / 60;
			NodeStart = PathMap[sourceX][sourceY];
			if (NodeStart != nullptr && NodeEnd != nullptr) {
				SolveAstar(NodeStart, NodeEnd);
				drawPath = true;
			}
		}
	}else if(b.button == SDL_BUTTON_RIGHT) {
		SDL_GetMouseState(&destX, &destY);
		if (destX != 0 || destY != 0) {
			destX = destX / 60;
			destY = destY / 60;
			NodeEnd = PathMap[destX][destY];
			if (NodeStart != nullptr && NodeEnd != nullptr) {
				SolveAstar(NodeStart, NodeEnd);
				drawPath = true;
			}
		}
	}
}

void handleEvents() {
	SDL_Event event;
	SDL_PollEvent(&event);
	switch (event.type) {
	case SDL_MOUSEBUTTONDOWN:
		mousePress(event.button);
		break;
	case SDL_KEYDOWN:
		if (event.key.keysym.sym == SDLK_LCTRL)ctrlPressed = true;
		break;
	case SDL_KEYUP:
		if (event.key.keysym.sym == SDLK_LCTRL)ctrlPressed = false;
		break;
	}
}



int main() {
	bool isRunning = true;
	SDL_Init(SDL_INIT_EVERYTHING);
	SDL_Window * window = SDL_CreateWindow("A*", SDL_WINDOWPOS_CENTERED,  SDL_WINDOWPOS_CENTERED, 600, 600,SDL_WINDOW_SHOWN);
	SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, 0);

	init();
	NodeStart = nullptr;
	NodeEnd = nullptr;
	
	findallNeighbors();
	while (isRunning) {
		SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
		SDL_RenderClear(renderer);
		handleEvents();
		render(renderer);
		
	}
	return 0;
}