//! \file PathSearch.h
//! \brief Defines the <code>fullsail_ai::algorithms::PathSearch</code> class interface.
//! \author Cromwell D. Enage, 2009; Jeremiah Blanchard, 2012
#ifndef _FULLSAIL_AI_PATH_PLANNER_PATH_SEARCH_H_
#define _FULLSAIL_AI_PATH_PLANNER_PATH_SEARCH_H_
#include <unordered_map>
#include <vector>
#include <list>
#include "../PriorityQueue.h"
using namespace std;

// change this to start the program on whatever default map as you like from the table below
#define USEDEFAULTMAP hex035x035

#define hWEIGHT 1.2f

#define hex006x006 "./Data/hex006x006.txt"
#define hex014x006 "./Data/hex014x006.txt"
#define hex035x035 "./Data/hex035x035.txt"
#define hex054x045 "./Data/hex054x045.txt"
#define hex098x098 "./Data/hex098x098.txt"
#define hex113x083 "./Data/hex113x083.txt"

#define COLOR_RED 0xFFFF0000
#define COLOR_BLUE 0xFF0000FF
#define COLOR_GREEN 0xFF00FF00

// change this to 1(true), and change the data below when you want to test specific starting and goal locations on startup
#define OVERRIDE_DEFAULT_STARTING_DATA 0

// Make sure your start and goal are valid locations!
#define DEFAULT_START_ROW 0
#define DEFAULT_START_COL 0
#define DEFAULT_GOAL_ROW ?
#define DEFAULT_GOAL_COL ?

#include "../TileSystem/Tile.h"
#include "../TileSystem/TileMap.h"
#include "../platform.h"
#include <vector>

namespace fullsail_ai {
	namespace algorithms {


		class PathSearch
		{
		public:
			class Edge;
			class SearchNode;

			class PlannerNode
			{
			public:
				PlannerNode(SearchNode* myvert, PlannerNode* myRent)
				{
					vertex = myvert;
					parent = myRent;
					givenCost = 0;
				}
				PlannerNode* parent;
				SearchNode* vertex;
				float heuristicCost;
				float givenCost;
				float finalCost;
			};
			class SearchNode
			{
			public:
				vector<Edge*> edges;
				Tile* tileLoc;
			};
			class Edge
			{
			public:
				SearchNode* endpoint;
				float cost;
			};
			unordered_map<Tile*, SearchNode*> map;
			vector<const Tile*> solutionVec;
			SearchNode* startNode;
			SearchNode* endNode;
			unordered_map<SearchNode*, PlannerNode*> visited;
			TileMap* tileMap;
			PriorityQueue<PlannerNode*> open;
			DWORD currTime;
			PlannerNode* successorNode, *current;
			SearchNode* successor;



			bool areAdjacent(Tile const* left, Tile const* right)
			{
				if (right->getWeight() == 0)
					return false;
				if (left->getRow() == right->getRow() && (left->getColumn() - 1 == right->getColumn() || left->getColumn() + 1 == right->getColumn()))
					return true;
				if (left->getColumn() == right->getColumn() && (left->getRow() - 1 == right->getRow() || left->getRow() + 1 == right->getRow()))
					return true;
				if (left->getRow() % 2 == 0 && (left->getColumn() - 1 == right->getColumn() && (left->getRow() - 1 == right->getRow() || left->getRow() + 1 == right->getRow())))
					return true;
				if (left->getRow() % 2 == 1 && (left->getColumn() + 1 == right->getColumn() && (left->getRow() - 1 == right->getRow() || left->getRow() + 1 == right->getRow())))
					return true;
				return false;
			}
			
			float estimate(SearchNode* lhs, SearchNode* rhs)
			{
				return (float)sqrt(
					(rhs->tileLoc->getRow() - lhs->tileLoc->getRow()) *
					(rhs->tileLoc->getRow() - lhs->tileLoc->getRow())
					+
					(rhs->tileLoc->getColumn() - lhs->tileLoc->getColumn()) *
					(rhs->tileLoc->getColumn() - lhs->tileLoc->getColumn())
					);
			}

			//////////////////////////////////////////////////////
			//! \brief Default constructor.
			DLLEXPORT PathSearch();

			//! \brief Destructor.
			DLLEXPORT ~PathSearch();

			//! \brief Sets the tile map.
			//!
			//! Invoked when the user opens a tile map file.
			//!
			//! \param   _tileMap  the data structure that this algorithm will use
			//!                    to access each tile's location and weight data.
			DLLEXPORT void initialize(TileMap* _tileMap);

			//! \brief Enters and performs the first part of the algorithm.
			//!
			//! Invoked when the user presses one of the play buttons.
			//!
			//! \param   startRow         the row where the start tile is located.
			//! \param   startColumn      the column where the start tile is located.
			//! \param   goalRow          the row where the goal tile is located.
			//! \param   goalColumn       the column where the goal tile is located.
			DLLEXPORT void enter(int startRow, int startColumn, int goalRow, int goalColumn);

			//! \brief Returns <code>true</code> if and only if no nodes are left open.
			//!
			//! \return  <code>true</code> if no nodes are left open, <code>false</code> otherwise.
			DLLEXPORT bool isDone() const;

			//! \brief Performs the main part of the algorithm until the specified time has elapsed or
			//! no nodes are left open.
			DLLEXPORT void update(long timeslice);

			//! \brief Returns an unmodifiable view of the solution path found by this algorithm.
			DLLEXPORT std::vector<Tile const*> const getSolution() const;

			//! \brief Resets the algorithm.
			DLLEXPORT void exit();

			//! \brief Uninitializes the algorithm before the tile map is unloaded.
			DLLEXPORT void shutdown();

			void GenerateEdgesFromTile(SearchNode* inNode, SearchNode* passNode);

		};
	}
}  // namespace fullsail_ai::algorithms

#endif  // _FULLSAIL_AI_PATH_PLANNER_PATH_SEARCH_H_

