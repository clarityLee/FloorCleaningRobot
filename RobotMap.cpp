#include <iostream>
#include <set>
#include <vector>
#include <map>
#include <string>
#include <random>
#include <chrono>
#include <algorithm> // for the std::fill function.
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

RobotMap::RobotMap(short r, short c) : rows(r), columns(c),
        randomGenerator(chrono::high_resolution_clock::now().time_since_epoch().count()),
        unidist0to1(0, 1),
        unidist0to2(0, 2),
        unidist0to3(0, 3) {
    rawData = new char*[rows];
    cells = new Cells(r*c);
    for (short i = 0 ; i < rows ; ++i) {
        rawData[i] = new char[columns];
    }
};

RobotMap::~RobotMap() {
    if (opensetFlag) {
        delete [] opensetFlag;
    }
    if (closedSetFlag) {
        delete [] closedSetFlag;
    }
    if (cameFrom) {
        delete [] cameFrom;
    }
    delete cells;
};

void RobotMap::readFloorData(std::ifstream &readInFile) {
    constexpr auto &&now = std::chrono::high_resolution_clock::now;
    auto phaseStartTime = now();
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
    chrono::duration<double> elapsed = now() - phaseStartTime;
    cout << "    - RobotMap::readFloorData(), total used time : " << (int) (elapsed.count() * 1000) << " ms." << endl;
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
    cout << "    - RobotMap::processData : construct adjacency list of all cells. / completed in : "
         << (int) (elapsed.count() * 1000) << " ms." << endl;

    opensetFlag = new bool[totalCells];
    closedSetFlag = new bool[totalCells];
    cameFrom = new int[totalCells];

    // using dijkstra to calculate distance
    phaseStartTime = now();
    calculateDistanceToR();
    elapsed = now() - phaseStartTime;
    cout << "    - RobotMap::processData : calculating all cell distances to recharger. / Completed in : "
         << (int) (elapsed.count() * 1000) << " ms." << endl;

    // deallocate raw floor data
    for (short i = 0 ; i < rows ; ++i) {
        delete rawData[i];
    }
    delete [] rawData;
};

void RobotMap::findClosestUnvisited(vector<Cell*> &path, Cell* source) {
    findClosestUnvisited(path, source, nullptr);
};

void RobotMap::findClosestUnvisited(vector<Cell*> &path, Cell* source, Cell* lastIncToR) {
    cells->resetTempDistance();
    fill(opensetFlag, opensetFlag+totalCells, false);
    fill(closedSetFlag, closedSetFlag+totalCells, false);
    fill(cameFrom, cameFrom+totalCells, -1);
    multiset<Cell*, CellCompare> openSet;

    if (source == recharger) {
        recharger->_tempDistance = 0;
        closedSetFlag[recharger->index] = true;

        lastIncToR->_tempDistance = 1;
        openSet.insert(lastIncToR); opensetFlag[lastIncToR->index] = true;
    } else {
        source->_tempDistance = 0;
        openSet.insert(source); opensetFlag[source->index] = true;
    }

    Cell *current, *neighbor;
    multiset<Cell*, CellCompare>::iterator it;

    while(openSet.size()) {
        it = openSet.begin();
        current = *it;

        openSet.erase(it); opensetFlag[current->index] = false;
        closedSetFlag[current->index] = true;
        
        // for each neighbor of current
        for (int i = 0 ; i < adjCells[current->index].size() ; ++i) {
            neighbor = adjCells[current->index][i];

            // if neighbor in closed set
            if (closedSetFlag[neighbor->index]) continue;

            // the distance from R to neighbor via current
            int distance = current->_tempDistance + 1; // distance of neighbor to R
            
            // if neighbor is in open set and 
            if (opensetFlag[neighbor->index]) {
                if (distance >= neighbor->_tempDistance) {
                    // not a better option , do nothing
                } else {
                    // the new path is a better option, do update
                    openSet.erase(neighbor);
                    neighbor->_tempDistance = distance;
                    openSet.insert(neighbor);
                    // cameFrom[neighbor] = current;
                    cameFrom[neighbor->index] = current->index;
                }

            // if neighbor not in open set yet
            } else {
                if (distance < neighbor->_tempDistance) neighbor->_tempDistance = distance;
                openSet.insert(neighbor); opensetFlag[neighbor->index] = true;
                // cameFrom[neighbor] = current;
                cameFrom[neighbor->index] = current->index;
            }
        }

        if (!current->visited) {
            break;
        } // an unvisited node is reached!
    }

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

    // select one path from multiset, and "copy" the path into container in argument
    path = (*paths.begin())->path;
    
    // deallocate all _tmpPathWrapper.
    for (multiset<_tmpPathWrapper*, PathCompare>::iterator it = paths.begin() ; it != paths.end() ; ++it) {
        delete *it;
    }
};

Cell* RobotMap::randomStart() {
    int neighborNumber = adjCells[recharger->index].size();
    int useNeighborIndex = 0;

    if (neighborNumber > 1) {
        useNeighborIndex = rndFrom0To(neighborNumber-1);
    }

    return adjCells[recharger->index][useNeighborIndex];
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
        returnIndex = rndFrom0To(cellsMinToR.size()-1);
    }
    return cellsMinToR[returnIndex];
};

void RobotMap::calculateDistanceToR() {
    fill(opensetFlag, opensetFlag+totalCells, false);
    fill(closedSetFlag, closedSetFlag+totalCells, false);
    multiset<Cell*, CellCompareForInit> openSet;
    recharger->distance = 0;
    openSet.insert(recharger); opensetFlag[recharger->index] = true;
    Cell *current, // minimum in open seet
        *neighbor;
    multiset<Cell*, CellCompareForInit>::iterator it;

    while(openSet.size()) {
        it = openSet.begin();
        current = *it;

        openSet.erase(it); opensetFlag[current->index] = false;
        closedSetFlag[current->index] = true;

        // for each neighbor of current
        for (int i = 0 ; i < adjCells[current->index].size() ; ++i) {
            neighbor = adjCells[current->index][i];

            // if neighbor in closed set
            if (closedSetFlag[neighbor->index]) continue;

            // the distance from R to neighbor via current
            int distance = current->distance + 1; // distance of neighbor to R
            
            // if neighbor is in open set and 
            if (opensetFlag[neighbor->index]) {
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
                openSet.insert(neighbor); opensetFlag[neighbor->index] = true;
            }
        }
    }
};

void RobotMap::calculateDistanceToR(Cell* adjToR) {
    // TODO:
};

void RobotMap::findRandomShortestWayHome(_tmpPathWrapper* pathWrapper, Cell* source) {
    Cell *next, *current = source;
    vector<Cell*> cellsMinToR;
    int minDistance;
    int count = 0;
    pathWrapper->path.resize(source->distance);
    int pathStoreIndex = source->distance - 1;

    while(current != recharger) {
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
            index = rndFrom0To(cellsMinToR.size()-1);
        }

        next = cellsMinToR[index];
        pathWrapper->path[pathStoreIndex--] = next;
        pathWrapper->visitedSum += next->visited;
        current = next;
    }
};

void RobotMap::constructPath(vector<Cell*> &path, int* cameFrom, Cell* current) {
    int currentIndex = current->index;
    path.clear();
    path.push_back(current);
    
    while(true) {

        // if current does not exist in cameFrom
        if (cameFrom[currentIndex] == -1) break;

        currentIndex = cameFrom[currentIndex];
        path.push_back(cells->cells[currentIndex]);
    }
};

inline short RobotMap::rndFrom0To(short num) {
    if (num > 3) {
        throw std::runtime_error("Error: input num > 3, which is not allowed.");
    }
    if (num == 1) return unidist0to1(randomGenerator);
    if (num == 2) return unidist0to2(randomGenerator);
    return unidist0to3(randomGenerator);
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