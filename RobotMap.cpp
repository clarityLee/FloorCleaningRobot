#include <iostream>
#include <set>
#include <vector>
#include <map>
#include <string>
#include <random>
#include <chrono>
#include "RobotMap.hpp"
using namespace std;

bool CellCompareForInit::operator() (Cell* const & lhs, Cell* const & rhs) const {
    return lhs->distance < rhs->distance;
};
bool CellCompare::operator() (Cell* const & lhs, Cell* const & rhs) const {
    return lhs->_tempDistance < rhs->_tempDistance;
};
bool PathCompare::operator() (_tmpPathWrapper* const & lhs, _tmpPathWrapper* const & rhs) const {
    return lhs->visitedSum > rhs->visitedSum;
};

string Cell::info() {
    string s = "cell[" + to_string(index) + "]:(" + to_string(i) + ", " + to_string(j) +"), distance to R: " + to_string(distance) + " .";
    return s;
}

Cells::Cells(int reserveSize) {
    cells.reserve(reserveSize);
};
Cells::~Cells() {
    for (int i = 0 ; i < cells.size() ; ++i) {
        delete cells[i];
    }
};
int Cells::size() {return cells.size();};
void Cells::push_back(Cell* cell) {
    cells.push_back(cell);
};
void Cells::resetTempDistance() {
    for (int i = 0 ; i < cells.size() ; ++i)
        cells[i]->_tempDistance = MAXINT;
};
Cell* Cells::back() {return cells.back();};
Cell* Cells::get(short i, short j) {

    // using binary search
    int left = 0, right = cells.size() - 1;
    int mid;
    while(left <= right) {
        mid = left + (right-left)/2;
        Cell* cm = cells[mid];
        if (i == cm->i && j == cm->j) return cells[mid];

        if (i < cm->i || (i == cm->i && j < cm->j)) {
            right = mid - 1;
        } else if (i > cm->i || (i == cm->i && j > cm->j)) {
            left = mid + 1;
        }
    }
    return nullptr;
};
Cell* Cells::get(int i) {
    return cells[i];
}

RobotMap::RobotMap(short r, short c) : rows(r), columns(c) {
    rawData = new char*[rows];
    // visited = new bool*[rows];
    cells = new Cells(r*c);
    for (short i = 0 ; i < rows ; ++i) {
        rawData[i] = new char[columns];
        // visited[i] = new bool[columns];
    }
};

RobotMap::~RobotMap() {
    for (short i = 0 ; i < rows ; ++i) {
        delete [] rawData[i];
        // delete [] visited[i];
    }
    delete [] rawData;
    // delete [] visited;
    delete cells;
};

void RobotMap::readFloorData(std::ifstream &readInFile) {
    int cellIndex = 0;
    for (short i = 0 ; i < rows ; ++i) {
        for (short j = 0 ; j < columns ; ++j) {
            readInFile >> rawData[i][j];
            if (rawData[i][j] == '0') {
                cells->push_back(new Cell(i, j, cellIndex++));
            } else if (rawData[i][j] == 'R') {
                cells->push_back(new Cell(i, j, cellIndex++));
                recharger = cells->back();
            }
        }
    }
    totalCells = cells->size();
};

void RobotMap::processData() {
    constexpr auto &&now = std::chrono::high_resolution_clock::now;

    // construct adjacency list
    auto phaseStartTime = now();
    adjCells.resize(totalCells);
    for (short i = 0 ; i < rows ; ++i) {
        for (short j = 0 ; j < rows ; ++j) {
            if (rawData[i][j] == '1') continue;
            Cell* cell = cells->get(i,j);
            if (i>0 && rawData[i-1][j] != '1') {
                adjCells[cell->index].push_back(cells->get(i-1, j));
            }
            if (j<columns-1 && rawData[i][j+1] != '1') {
                adjCells[cell->index].push_back(cells->get(i, j+1));
            }
            if (i<rows-1 && rawData[i+1][j] != '1') {
                adjCells[cell->index].push_back(cells->get(i+1, j));
            }
            if (j>0 && rawData[i][j-1] != '1') {
                adjCells[cell->index].push_back(cells->get(i, j-1));
            }
        }
    }
    chrono::duration<double> elapsed = now() - phaseStartTime;
    cout << "construct adjacency list completed in : " << elapsed.count() * 1000 << " ms." << endl;

    // construct distance array
    // distance = new int[totalCells];
    // for (int i = 0 ; i < totalCells ; ++i) {
    //     distance[i] = MAXINT;
    // }

    // using dijkstra to calculate distance
    phaseStartTime = now();
    calculateDistanceToR();
    elapsed = now() - phaseStartTime;
    cout << "calculateDistanceToR() completed in : " << elapsed.count() * 1000 << " ms." << endl;
};

void RobotMap::findClosestUnvisited(vector<Cell*> &path, Cell* source) {
    findClosestUnvisited(path, source, nullptr);
};

void RobotMap::findClosestUnvisited(vector<Cell*> &path, Cell* source, Cell* lastIncToR) {
    cells->resetTempDistance();
    multiset<Cell*, CellCompare> closedSet;
    multiset<Cell*, CellCompare> openSet;
    map<Cell*, Cell*> cameFrom;

    if (source == recharger) {
        recharger->_tempDistance = 0;
        closedSet.insert(recharger);

        lastIncToR->_tempDistance = 1;
        openSet.insert(lastIncToR);
    } else {
        source->_tempDistance = 0;
        openSet.insert(source);
    }

    Cell *current, *neighbor;
    multiset<Cell*, CellCompare>::iterator it;

    // cout << endl << endl;
    while(openSet.size()) {
        it = openSet.begin();
        current = *it;

        openSet.erase(it);
        closedSet.insert(current);
        // cout << "select ("<<current->i<<", "<<current->j<<") from openSet." << endl;
        // cout << "openSet.size: " << openSet.size() << endl;
        
        // for each neighbor of current
        for (int i = 0 ; i < adjCells[current->index].size() ; ++i) {
            neighbor = adjCells[current->index][i];
            // cout << "  neighbor("<<neighbor->i<<", "<<neighbor->j<<") " << (neighbor->visited ? "visited" : "not visited, ") << endl;

            // if neighbor in closed set
            if (closedSet.find(neighbor) != closedSet.end()) {
                // cout << "                 is already in closed set." << endl;
                continue;
            }

            // the distance from R to neighbor via current
            int distance = current->_tempDistance + 1; // distance of neighbor to R
            
            // if neighbor is in open set and 
            if (openSet.find(neighbor) != openSet.end()) {
                // cout << "                 openSet.find(neighbor) : (" << (*openSet.find(neighbor))->i << ", " << (*openSet.find(neighbor))->j << ")" << endl;
                if (distance >= neighbor->_tempDistance) {
                    // cout << "                 is in openSet, but no better path found, do nothing." << endl;
                    // not a better option , do nothing
                } else {
                    // the new path is a better option, do update
                    openSet.erase(neighbor);
                    neighbor->_tempDistance = distance;
                    openSet.insert(neighbor);
                    cameFrom[neighbor] = current;
                    // cout << "                 is in open set, and a better path is found, do update." << endl;
                }

            // if neighbor not in open set yet
            } else {
                if (distance < neighbor->_tempDistance) neighbor->_tempDistance = distance;
                openSet.insert(neighbor);
                cameFrom[neighbor] = current;
                // cout << "                 is not in open set yet, add to openset."<< endl;
            }
        }

        if (!current->visited) {
            // cout << "         current("<<current->i<<", "<<current->j<<") visited: "<< (current->visited ? "TRUE" : "FALSE") << endl;
            break;
        } // an unvisited node is reached!
    }

    // cout << "         destination("<<current->i<<", "<<current->j<<") visited: "<< (current->visited ? "TRUE" : "FALSE") << endl;

    constructPath(path, cameFrom, current);
};

void RobotMap::randomShortestWayHome(vector<Cell*> &path, Cell* current) {
    // cout << "             doing: map->randomShortestWayHome(path, current)" << endl;
    multiset<_tmpPathWrapper*, PathCompare> paths;
    hasMultiplePaths = false;
    for (short i = 0 ; i < randomHomingPaths ; ++i) {
        _tmpPathWrapper *oneRandomPath = new _tmpPathWrapper();;
        findRandomShortestWayHome(oneRandomPath, current);
        paths.insert(oneRandomPath);
        if (!hasMultiplePaths) break;
    }

    // ↓ for test only
    // cout << "There are " << paths.size() << " shortest paths in the multiset." << endl;
    // cout << "    The first one in the multiSet has total visited Sum : " << (*paths.begin())->visitedSum << "." << endl;
    // cout << "    The last one in the multiSet has total visited Sum : " << (*paths.rbegin())->visitedSum << "." << endl;
    // cout << "    The program is selecting first one." << endl;
    // ↑ for test only
    
    // select one path from multiset, and "copy" the path into container in argument
    path = (*paths.begin())->path;
    
    // deallocate all _tmpPathWrapper.
    for (multiset<_tmpPathWrapper*, PathCompare>::iterator it = paths.begin() ; it != paths.end() ; ++it) {
        delete *it;
    }
    // cout << "             completed: map->randomShortestWayHome(path, current)" << endl;
};


Cell* RobotMap::randomStart() {
    int neighbors = adjCells[recharger->index].size();
    int useNeighbor = 0;

    // cout << "           There are " << neighbors << " neighbors from recharger." << endl;

    if (neighbors > 1) { // if there is more than 1 neighbor to choose
        auto seed = chrono::high_resolution_clock::now().time_since_epoch().count();
        std::mt19937 rng(seed);    // random-number engine used (Mersenne-Twister in this case)
        std::uniform_int_distribution<int> uni(0, neighbors-1); // guaranteed unbiased
        useNeighbor = uni(rng);
    }
    // cout << "           useNeighbor index : " << useNeighbor << endl;

    return adjCells[recharger->index][useNeighbor];
};

void RobotMap::recordPath(Cell* cell) {
    path.emplace_back(cell->i, cell->j);
};

Cell* RobotMap::unvisitedAdjacent(Cell* cell) {
    vector<Cell*> cellsMinToR;
    int minDistance = MAXINT;
    for (short i = 0 ; i < adjCells[cell->index].size() ; ++i) {

        if (adjCells[cell->index][i]->visited) continue;

        if (adjCells[cell->index][i]->distance < minDistance) {
            minDistance = adjCells[cell->index][i]->distance;
            cellsMinToR.clear();
            cellsMinToR.push_back(adjCells[cell->index][i]);
        } else if (adjCells[cell->index][i]->distance == minDistance) {
            cellsMinToR.push_back(adjCells[cell->index][i]);
        }
    }
    
    if (cellsMinToR.size() == 0) return nullptr;

    short returnIndex = 0;
    if (cellsMinToR.size() > 1) {
        auto seed = chrono::high_resolution_clock::now().time_since_epoch().count();
        std::mt19937 rng(seed);
        std::uniform_int_distribution<int> uni(0, cellsMinToR.size()-1); // guaranteed unbiased
        returnIndex = uni(rng);
    }
    return cellsMinToR[returnIndex];
};

void RobotMap::calculateDistanceToR() {
    multiset<Cell*, CellCompareForInit> closedSet;
    multiset<Cell*, CellCompareForInit> openSet;
    recharger->distance = 0;
    openSet.insert(recharger);
    Cell *current, // minimum in open seet
        *neighbor;
    multiset<Cell*, CellCompareForInit>::iterator it;

    while(openSet.size()) {
        it = openSet.begin();
        current = *it;

        openSet.erase(it);
        closedSet.insert(current);

        // for each neighbor of current
        for (int i = 0 ; i < adjCells[current->index].size() ; ++i) {
            neighbor = adjCells[current->index][i];

            // if neighbor in closed set
            if (closedSet.find(neighbor) != closedSet.end()) continue;

            // the distance from R to neighbor via current
            int distance = current->distance + 1; // distance of neighbor to R
            
            // if neighbor is in open set and 
            if (openSet.find(neighbor) != openSet.end()) {
                if (distance >= neighbor->distance) {
                    // not a better option , do nothing
                } else {
                    // the new path is a better option, do update
                    openSet.erase(neighbor);
                    neighbor->distance = distance;
                    openSet.insert(neighbor);
                }

            // if neighbor not in open set
            } else {
                if (distance < neighbor->distance) neighbor->distance = distance;
                openSet.insert(neighbor);
            }
        }
    }
};

void RobotMap::a_star(vector<Cell*> &path, Cell* source, Cell* destination) {

    cells->resetTempDistance();
    multiset<Cell*, CellCompare> closedSet;
    multiset<Cell*, CellCompare> openSet;
    map<Cell*, Cell*> cameFrom;

    source->_tempDistance = 0;
    openSet.insert(source);

    Cell *current, *neighbor;
    multiset<Cell*, CellCompare>::iterator it;

    while(openSet.size()) {
        it = openSet.begin();
        current = *it;

        if (current == recharger) break; // recharger arrived

        openSet.erase(it);
        closedSet.insert(current);
        
        // for each neighbor of current
        for (int i = 0 ; i < adjCells[current->index].size() ; ++i) {
            neighbor = adjCells[current->index][i];

            // if neighbor in closed set
            if (closedSet.find(neighbor) != closedSet.end()) continue;

            // the distance from R to neighbor via current
            int distance = current->distance + 1; // distance of neighbor to R
            
            // if neighbor is in open set and 
            if (openSet.find(neighbor) != openSet.end()) {
                if (distance >= neighbor->distance) {
                    // not a better option , do nothing
                } else {
                    // the new path is a better option, do update
                    openSet.erase(neighbor);
                    neighbor->distance = distance;
                    openSet.insert(neighbor);
                    cameFrom[neighbor] = current;
                }

            // if neighbor not in open set yet
            } else {
                if (distance < neighbor->distance) neighbor->distance = distance;
                openSet.insert(neighbor);
                cameFrom[neighbor] = current;
            }
        }
    }

    constructPath(path, cameFrom, current);
};

void RobotMap::findRandomShortestWayHome(_tmpPathWrapper* pathWrapper, Cell* source) {
    // cout << "               doing: RobotMap::findRandomShortestWayHome" << endl;
    Cell *next, *current = source;
    vector<Cell*> cellsMinToR;
    int minDistance;
    int count = 0;
    pathWrapper->path.resize(source->distance);
    int pathStoreIndex = source->distance - 1;

    while(current != recharger) {
        // cout << "               loop count : " << count++ << endl;
        cellsMinToR.clear();
        minDistance = current->distance;

        for (short i = 0 ; i < adjCells[current->index].size(); ++i) {

            if (adjCells[current->index][i]->distance > minDistance) continue;
            if (adjCells[current->index][i]->distance < minDistance) {
                minDistance = adjCells[current->index][i]->distance;
                cellsMinToR.clear();
                cellsMinToR.push_back(adjCells[current->index][i]);

            } else {
                cellsMinToR.push_back(adjCells[current->index][i]);
            }
        }
        

        short index = 0;
        if (cellsMinToR.size() > 1) {
            if (!hasMultiplePaths) hasMultiplePaths = true;
            auto seed = chrono::high_resolution_clock::now().time_since_epoch().count();
            std::mt19937 rng(seed);
            std::uniform_int_distribution<int> uni(0, cellsMinToR.size()-1); // guaranteed unbiased
            index = uni(rng);
        }

        next = cellsMinToR[index];
        // cout << "| push next(" << next->i << ", " << next->j <<") |";
        pathWrapper->path[pathStoreIndex--] = next;
        pathWrapper->visitedSum += next->visited;
        // cout << "               change from current("<<current->i<<", "<<current->j<<") distance: "<<current->distance<<" to next("<<next->i<<", "<<next->j<<") distance:"<<next->distance << endl;
        current = next;
    }
    // cout << endl;
    
    // pathWrapper->path.push_back(recharger);
    // cout << "| push recharger(" << recharger->i << ", " << recharger->j <<") |";
    // cout << "               completed: RobotMap::findRandomShortestWayHome" << endl;
};

void RobotMap::constructPath(vector<Cell*> &path, map<Cell*, Cell*> &cameFrom, Cell* current) {
    path.push_back(current);
    while(true) {

        // if key current doesn't exist
        if (cameFrom.find(current) == cameFrom.end()) break;

        current = cameFrom[current];
        path.push_back(current);
        
    }
};

void RobotMap::printAllDistance() {
    for (int i = 0 ; i < cells->size() ; ++i) {
        Cell* c = cells->get(i);
        cout << c->info() << ", neightbors:" << endl;
        for (int j = 0 ; j < adjCells[c->index].size() ; ++j) {
            cout << "    " << adjCells[c->index][j]->info() << endl;
        }
    }
};

void RobotMap::printPath(vector<Cell*> &path) {
    cout << "           ";
    cout << "There are " << path.size() << " cells in the path : ";
    for (int i = path.size() - 1, j = 0 ; i >= 0 ; --i, ++j) {
        if (j%4 == 0) cout << endl << "           ";
        cout << "(" << path[i]->i << ", " << path[i]->j << ")";
        if (i>0) cout << "  ->  ";
    }
    cout << endl;
};