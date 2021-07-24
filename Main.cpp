#include <SDL.h>
#include <iostream>
#include "node.h"
#include <queue>          // std::priority_queue
#undef main

const int numOfTiles = 15;
const int sizeOfTiles = 40;
int sourceX = -1, sourceY = -1;
int destX = -1, destY = -1;
bool ctrlPressed = false;
int tmpObstacleX = -1, tmpObstacleY = -1;
node* PathMap[15][15];
SDL_Rect* rects[15][15];
node* NodeEnd, * NodeStart;
bool drawPath = false;

void init() {
	for (size_t x = 0; x < numOfTiles; x++) {
		for (size_t y = 0; y < numOfTiles; y++)
		{
			PathMap[x][y] = new node(x, y);
			rects[x][y] = new SDL_Rect();
		}
	}

}


void findNeighbors(int x, int y) {
	node* aFromTile = PathMap[x][y];
	std::vector<std::pair<int, int>> validMoves = { {1,0}, {0,1}, {-1,0}, {0, -1} };
	for (auto i=0; i<validMoves.size();i++)
	{
		int offSetX = x + validMoves[i].first;
		int offSetY = y + validMoves[i].second;
		if (offSetX > 0 && offSetX < 14 && offSetY>0 && offSetY < 14)
		{
			if (PathMap[offSetX][offSetY]->visited == false && PathMap[offSetX][offSetY]->obstacle == false) {
				aFromTile->vecNeighbors.push_front(PathMap[offSetX][offSetY]);
			}
		}
	}
	return;
}

void findallNeighbors() {
	for (size_t x = 0; x < numOfTiles; x++) {
		for (size_t y = 0; y < numOfTiles; y++)
		{
			findNeighbors(x, y);
		}
	}
	return;
}

void render(SDL_Renderer* renderer) {
	if (drawPath == true) {
		node* first = PathMap[NodeEnd->x][NodeEnd->y];

		node* p = PathMap[NodeEnd->x][NodeEnd->y];

		while (p->parent != nullptr) {
			if (p != first) {
				p->marked = true;
			}
			p = p->parent;
		}

		drawPath = false;
	}

	for (int i = 0; i < numOfTiles; i++) {
		for (int j = 0; j < numOfTiles; j++) {
			SDL_Rect* rect = rects[i][j];
			rect->h = 30;
			rect->w = 30;
			rect->x = 40 * i;
			rect->y = 40 * j;
			if (PathMap[i][j]->marked == true) {
				SDL_SetRenderDrawColor(renderer, 211, 211, 211, 255);
				SDL_RenderDrawRect(renderer, rect);
				SDL_RenderFillRect(renderer, rect);
			}
			else if (i == sourceX && j == sourceY || i == destX && j == destY) {
				SDL_SetRenderDrawColor(renderer, 0, 0, 255, 255);
				SDL_RenderDrawRect(renderer, rect);
				SDL_RenderFillRect(renderer, rect);
			}
			else if (PathMap[i][j]->obstacle == true) {
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
	for (int x = 0; x < numOfTiles; x++) {
		for (int y = 0; y < numOfTiles; y++) {
			PathMap[x][y]->visited = false;
			PathMap[x][y]->fGlobalGoal = FLT_MAX;
			PathMap[x][y]->fLocalGoal = FLT_MAX;
			PathMap[x][y]->parent = nullptr;
			PathMap[x][y]->marked = false;

		}
	}
	auto distance = [](node* a, node* b) {
		return sqrtf(static_cast<float>((((a->x) - (b->x)) * ((a->x) - (b->x)) + ((a->y) - (b->y)) * ((a->y) - (b->y)))));
	};
	node* nodeCurrent = NodeStart;
	NodeStart->fLocalGoal = 0.0f;
	NodeStart->fGlobalGoal = distance(NodeStart, NodeEnd);

	struct CustomCompare
	{
		bool operator()(const node* lhs, const node* rhs)
		{
			return lhs->fGlobalGoal < rhs->fGlobalGoal;;
		}
	};
	std::priority_queue<node* , std::vector<node*>, CustomCompare> queueOpenNodes;

	queueOpenNodes.push(NodeStart);

	//stop searching when finding a path
	while (!queueOpenNodes.empty()/* && nodeCurrent != NodeEnd*/) {
		//sort list into ascending order of global goals
		//if the list is already visited pop
		while (!queueOpenNodes.empty() && queueOpenNodes.top()->visited)
			queueOpenNodes.pop();
		//but this way we can end up with an empty list 
		if (queueOpenNodes.empty())
			break;

		//so the first one is the possibly the best candidate to the shortest path
		nodeCurrent = queueOpenNodes.top();
		nodeCurrent->visited = true;
		//check neighbors of current node
		for (auto nodeNeigbor : nodeCurrent->vecNeighbors) {
			//and push_back them unless they're visited or an wall
			if (!nodeNeigbor->visited && nodeNeigbor->obstacle == 0)
				queueOpenNodes.push(nodeNeigbor);
			//calculate neighbors local goals
			float fPossiblyLowerGoal = nodeCurrent->fLocalGoal + distance(nodeCurrent, nodeNeigbor);
			if (fPossiblyLowerGoal < nodeNeigbor->fLocalGoal) {
				nodeNeigbor->parent = nodeCurrent;
				nodeNeigbor->fLocalGoal = fPossiblyLowerGoal;
				//and calculate global goal
				nodeNeigbor->fGlobalGoal = nodeNeigbor->fLocalGoal + distance(nodeNeigbor, NodeEnd);

			}

		}
	}

}




void mousePress(SDL_MouseButtonEvent& b) {
	if (b.button == SDL_BUTTON_LEFT && ctrlPressed == true) {
		SDL_GetMouseState(&tmpObstacleX, &tmpObstacleY);
		if (tmpObstacleX != -1 || tmpObstacleY != -1) {
			tmpObstacleX = tmpObstacleX / sizeOfTiles;
			tmpObstacleY = tmpObstacleY / sizeOfTiles;
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
			sourceX = sourceX / sizeOfTiles;
			sourceY = sourceY / sizeOfTiles;
			NodeStart = PathMap[sourceX][sourceY];
			if (NodeStart != nullptr && NodeEnd != nullptr) {
				SolveAstar(NodeStart, NodeEnd);
				drawPath = true;
			}
		}
	}
	else if (b.button == SDL_BUTTON_RIGHT) {
		SDL_GetMouseState(&destX, &destY);
		if (destX != 0 || destY != 0) {
			destX = destX / sizeOfTiles;
			destY = destY / sizeOfTiles;
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
	SDL_Window* window = SDL_CreateWindow("A*", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 600, 600, SDL_WINDOW_SHOWN);
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