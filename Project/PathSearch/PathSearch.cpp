#include "PathSearch.h"

namespace fullsail_ai {
	namespace algorithms {

		bool isGreater(PathSearch::PlannerNode* const &lhs, PathSearch::PlannerNode* const &rhs)
		{
			return (lhs->finalCost > rhs->finalCost);
		}
		PathSearch::PathSearch() : open(isGreater)
		{
		}

		PathSearch::~PathSearch()
		{
		}

		void PathSearch::initialize(TileMap* _tileMap)
		{
			int rowCount = _tileMap->getRowCount();
			int colCount = _tileMap->getColumnCount();
			tileMap = _tileMap;

			for (int i = 0; i < rowCount; i++)
			{
				for (int j = 0; j < colCount; j++)
				{
					if (_tileMap->getTile(i, j)->getWeight() != 0)
					{
						Tile* tile = _tileMap->getTile(i, j);
						SearchNode* node = new SearchNode();
						node->tileLoc = tile;
						map[node->tileLoc] = node;
					}
				}
			}

			for (int i = 0; i < rowCount; i++)
			{
				for (int j = 0; j < colCount; j++)
				{
					if (_tileMap->getTile(i, j)->getWeight() != 0)
					{
						Tile* curTile = _tileMap->getTile(i, j);
						SearchNode* myNode = map[curTile];
						if ((i + 1 < rowCount) && areAdjacent(curTile, _tileMap->getTile(i + 1, j)))
						{
							GenerateEdgesFromTile(map[_tileMap->getTile(i + 1, j)], myNode);
						}
						if ((i - 1 >= 0) && (j - 1 >= 0) && areAdjacent(curTile, _tileMap->getTile(i - 1, j - 1)))
						{
							GenerateEdgesFromTile(map[_tileMap->getTile(i - 1, j - 1)], myNode);
						}
						if ((i + 1 < rowCount) && (j + 1 < colCount) && areAdjacent(curTile, _tileMap->getTile(i + 1, j + 1)))
						{
							GenerateEdgesFromTile(map[_tileMap->getTile(i + 1, j + 1)], myNode);
						}
						if ((i - 1 >= 0) && (j + 1 < colCount) && areAdjacent(curTile, _tileMap->getTile(i - 1, j + 1)))
						{
							GenerateEdgesFromTile(map[_tileMap->getTile(i - 1, j + 1)], myNode);
						}
						if ((j - 1 >= 0) && areAdjacent(curTile, _tileMap->getTile(i, j - 1)))
						{
							GenerateEdgesFromTile(map[_tileMap->getTile(i, j - 1)], myNode);
						}
						if ((j + 1 < colCount) && areAdjacent(curTile, _tileMap->getTile(i, j + 1)))
						{
							GenerateEdgesFromTile(map[_tileMap->getTile(i, j + 1)], myNode);
						}
					}
				}
			}
		}

		void PathSearch::enter(int startRow, int startColumn, int goalRow, int goalColumn)
		{
			startNode = map[tileMap->getTile(startRow, startColumn)];
			endNode = map[tileMap->getTile(goalRow, goalColumn)];
			PlannerNode* node = new PlannerNode(startNode, nullptr);
			open.push(node);
			open.front()->heuristicCost = estimate(startNode, endNode);
			open.front()->finalCost = open.front()->heuristicCost * hWEIGHT;
			visited[startNode] = open.front();
			solutionVec.clear();
		}

		void PathSearch::update(long timeslice)
		{
			DWORD time = GetTickCount();
			while (!open.empty())
			{
				current = open.front();
				open.pop();
				if (current->vertex == endNode)
				{
					while (current)
					{
						solutionVec.push_back(current->vertex->tileLoc);
						current = current->parent;
						if (!current)
							return;
					}
				}

				for (size_t i = 0; i < current->vertex->edges.size(); i++)
				{
					successor = current->vertex->edges[i]->endpoint;
					float tempGivenCost = current->givenCost + estimate(current->vertex, current->vertex->edges[i]->endpoint)*hWEIGHT;
					successor->tileLoc->setFill(COLOR_BLUE);
					//////
					PlannerNode* visSuc = visited[successor];
					///////
					if (visSuc != nullptr)
					{
						if (tempGivenCost < visSuc->givenCost)
						{
							successorNode = visSuc;
							open.remove(successorNode);
							successorNode->givenCost = tempGivenCost;
							successorNode->finalCost = successorNode->givenCost + successorNode->heuristicCost * hWEIGHT;
							successorNode->parent = current;
							open.push(successorNode);
						}
					}
					else
					{
						successorNode = new PlannerNode(successor, current);
						successorNode->givenCost = tempGivenCost;
						successorNode->heuristicCost = estimate(successor, endNode);
						successorNode->finalCost = successorNode->givenCost + successorNode->heuristicCost * hWEIGHT;
						visited[successor] = successorNode;
						open.push(successorNode);
						successorNode->vertex->tileLoc->setFill(COLOR_RED);
					}
				}
				currTime = GetTickCount();
				if (currTime - time >= (DWORD)timeslice)
					break;
			}
		}

		void PathSearch::exit()
		{
			open.clear();
			visited.clear();
		}

		void PathSearch::shutdown()
		{

		}

		bool PathSearch::isDone() const
		{
			if (solutionVec.size() > 0)
				return true;
			else
				return false;
		}

		void PathSearch::GenerateEdgesFromTile(SearchNode* inNode, SearchNode* passNode)
		{
			SearchNode* node = inNode;
			Edge* edge = new Edge;
			edge->endpoint = node;
			passNode->edges.push_back(edge);
		}

		std::vector<Tile const*> const PathSearch::getSolution() const
		{
			return solutionVec;
		}
	}
}  // namespace fullsail_ai::algorithms

