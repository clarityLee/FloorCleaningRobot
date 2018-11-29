#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <random>
#include <chrono>
#include <queue>
#include "RobotMap.hpp"
using namespace std;

bool PathCompare::operator() (_tmpPathWrapper* const & lhs, _tmpPathWrapper* const & rhs) const {
    return lhs->visitedSum > rhs->visitedSum;
};
bool DqPathCompare::operator() (DqPathWrapper* const & lhs, DqPathWrapper* const & rhs) const {
    return lhs->visitedSum > rhs->visitedSum;
};

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

bool Qmember::operator< (const Qmember &rhs) const {
    return distance > rhs.distance;
};

template <class T, class S, class C>
void clearPQ(priority_queue<T, S, C>& q) {
    struct HackedQueue : private priority_queue<T, S, C> {
        static S& Container(priority_queue<T, S, C>& q) {
            return q.*&HackedQueue::c;
        }
    };
    HackedQueue::Container(q).clear();
}

template <class T, class S, class C>
void resizePQ(priority_queue<T, S, C>& q, int size) {
    struct HackedQueue : private priority_queue<T, S, C> {
        static S& Container(priority_queue<T, S, C>& q) {
            return q.*&HackedQueue::c;
        }
    };
    HackedQueue::Container(q).reserve(size);
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
    if (tmpCameFrom) {
        delete [] tmpCameFrom;
    }
    if (tmpDist) {
        delete [] tmpDist;
    }
    delete cells;
    for(map<int, DijkData*>::iterator it = allDijkData.begin(); it != allDijkData.end(); ++it) {
        delete it->second;
        it->second = nullptr;
    }
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
    cout << "    - RobotMap::readFloorData(), total Clean Cells: " << totalCells << ", total used time : " << (int) (elapsed.count() * 1000) << " ms." << endl;
};

void RobotMap::processData() {
    constexpr auto &&now = std::chrono::high_resolution_clock::now;

    // construct adjacency list
    cout << "    - RobotMap::processData : construct adjacency list of all cells..." << endl;
    auto phaseStartTime = now();
    adjCells.resize(totalCells);
    for (short i = 0 ; i < rows ; ++i) {
        for (short j = 0 ; j < columns ; ++j) {
            if (rawData[i][j] == '1') continue;
            Cell* cell = cells->get(i,j);
            if (i>0 && rawData[i-1][j] != '1') {
                adjCells[cell->index].push_back(cells->get(i-1, j));
                ++edges;
            }
            if (j<columns-1 && rawData[i][j+1] != '1') {
                adjCells[cell->index].push_back(cells->get(i, j+1));
                ++edges;
            }
            if (i<rows-1 && rawData[i+1][j] != '1') {
                adjCells[cell->index].push_back(cells->get(i+1, j));
                ++edges;
            }
            if (j>0 && rawData[i][j-1] != '1') {
                adjCells[cell->index].push_back(cells->get(i, j-1));
                ++edges;
            }
        }
    }
    edges/=2;
    chrono::duration<double> elapsed = now() - phaseStartTime;
    cout << "    - RobotMap::processData : adjacency list of all cells completed in : "
        << (int) (elapsed.count() * 1000) << " ms." << endl;

    opensetFlag = new bool[totalCells];
    closedSetFlag = new bool[totalCells];
    tmpCameFrom = new int[totalCells];
    tmpDist = new int[totalCells];
    resizePQ(openSet, totalCells);

    // using dijkstra to calculate distance
    phaseStartTime = now();
    vector<Cell*> &adjsToR = adjCells[recharger->index];
    calcDijkRespectTo(-1); // respect to nothing
    distToR = allDijkData[-1]->distanceToR;
    for (short i = 0 ; i < adjsToR.size() ; ++i) {
        calcDijkRespectTo(adjsToR[i]->index);
    }
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

    clearPQ(openSet);

    int* dist = tmpDist;
    int* cameFrom = tmpCameFrom;
    for (int i = 0 ; i < totalCells ; ++i) {
        opensetFlag[i] = false;
        closedSetFlag[i] = false;
        cameFrom[i] = -1;
        dist[i] = MAXINT;
    }

    dist[source->index] = 0;
    openSet.emplace(source, dist[source->index]);
    opensetFlag[source->index] = true;

    Cell *current, *neighbor;
    
    while(!openSet.empty()) {
        const Qmember &q = openSet.top();
        if (!opensetFlag[q.cell->index]) {
            openSet.pop();
            continue;
        }

        current = q.cell;
        if (!current->visited) {  // an unvisited node is reached!
            break;
        }

        openSet.pop();
        opensetFlag[current->index] = false;
        closedSetFlag[current->index] = true;

        // for each neighbor of current
        vector<Cell*> &adjs = adjCells[current->index];
        for (int i = 0 ; i < adjs.size() ; ++i) {
            neighbor = adjs[i];

            // if neighbor in closed set
            if (closedSetFlag[neighbor->index]) continue;

            // the distance from source to neighbor via current
            int distance = dist[current->index] + 1;
            
            // if neighbor is in open set and 
            if (opensetFlag[neighbor->index]) {
                if (distance >= dist[neighbor->index]) {
                    // not a better option , do nothing
                } else {
                    // the new path is a better option, do update
                    dist[neighbor->index] = distance;
                    openSet.emplace(neighbor, dist[neighbor->index]);
                    cameFrom[neighbor->index] = current->index;
                }

            // if neighbor not in open set yet
            } else {
                if (distance < dist[neighbor->index]) dist[neighbor->index] = distance;
                openSet.emplace(neighbor, dist[neighbor->index]);
                opensetFlag[neighbor->index] = true;
                cameFrom[neighbor->index] = current->index;
            }
        }
    }

    if (!openSet.size()) {
        throw runtime_error("Error: No unvisited is found");
    }

    constructPath(path, cameFrom, current);
};

void RobotMap::findClosestUnvisitedToR(vector<Cell*> &path, int lastIncIndex) {

    DijkData &dijkData = *(allDijkData[lastIncIndex]);
    Cell* closestUnvisited;
    while(true) {
        closestUnvisited = dijkData.unvisited.front();
        if (closestUnvisited->visited) dijkData.unvisited.pop_front();
        else break;
        if (dijkData.unvisited.size() == 0) {
            string errormessage = "dijkData.unvisited.size() == 0";
            throw runtime_error(errormessage);
        }
    }

    DqPathWrapper* d = findMinimumPathToR(closestUnvisited, lastIncIndex);
    deque<Cell*> &dq = d->path;

    // convert deque path to path for robot
    path.clear();
    while(dq.size() > 0) {
        path.push_back(dq.front());
        dq.pop_front();
    }

    delete d;
};

void RobotMap::randomShortestWayHome(vector<Cell*> &path, Cell* current) {

    int lastIncIndex = -1;

    DqPathWrapper* d = findMinimumPathToR(current, lastIncIndex);
    deque<Cell*> &dq = d->path;

    // select one with minimum visitedSum
    // convert deque path to path for robot
    path.clear();
    while(dq.size() > 0) {
        path.push_back(dq.back());
        dq.pop_back();
    }
    
    delete d;
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
    int* &dist = allDijkData[-1]->distanceToR;
    vector<Cell*> &adjs = adjCells[cell->index];

    vector<Cell*> adjs_S, adjs_EG;
    for (short i = 0 ; i < adjs.size() ; ++i) {
        if (adjs[i]->visited) continue;
        if (dist[adjs[i]->index] < dist[cell->index]) {
            adjs_S.push_back(adjs[i]);
        } else {
            adjs_EG.push_back(adjs[i]);
        }
    }

    if (adjs_S.empty() && adjs_EG.empty()) return nullptr;

    vector<Cell*> *v;
    if (!adjs_EG.empty()) v = &adjs_EG;
    else v = &adjs_S;

    short index = 0;
    if ((*v).size() > 1) {
        index = rndFrom0To((*v).size()-1);
    }
    return (*v)[index];
};

Cell* RobotMap::unvisitedAdj_v2(Cell* source) {
    int* &dist = allDijkData[-1]->distanceToR;
    
    vector<Cell*> &adjs = adjCells[source->index];

    adjs_with_visitedAdjs.clear();
    adjs_without_visitedAdjs.clear();

    for (short i = 0 ; i < adjs.size() ; ++i) {
        Cell* adj = adjs[i];
        if (adj->visited) continue;
        bool hasVisitedAdj = false;
        vector<Cell*> &adjsOfadj = adjCells[adj->index];
        for (short j = 0 ; j < adjsOfadj.size() ; ++j) {
            if (adjsOfadj[j] == source) continue;
            if (adjsOfadj[j]->visited) {
                hasVisitedAdj = true;
                break;
            }
        }
        if (hasVisitedAdj) {
            adjs_with_visitedAdjs.push_back(adj);
        } else {
            adjs_without_visitedAdjs.push_back(adj);
        }
    }

    vector<Cell*> *v;
    if (!adjs_with_visitedAdjs.empty()) {
        v = &adjs_with_visitedAdjs;
    } else {
        v = &adjs_with_visitedAdjs;
    }

    adjs_S.clear();
    adjs_EG.clear();
    for (short i = 0 ; i < (*v).size() ; ++i) {
        if ((*v)[i]->visited) continue;
        if (dist[(*v)[i]->index] < dist[source->index]) {
            adjs_S.push_back((*v)[i]);
        } else {
            adjs_EG.push_back((*v)[i]);
        }
    }

    if (adjs_S.empty() && adjs_EG.empty()) return nullptr;
    if (!adjs_EG.empty()) v = &adjs_EG;
    else v = &adjs_S;

    short index = 0;
    if ((*v).size() > 1) {
        index = rndFrom0To((*v).size()-1);
    }

    return (*v)[index];
}

Cell* RobotMap::unvisitedAdj_v3(Cell* source) {
    vector<Cell*> &adjs = adjCells[source->index];
    int* &dist = allDijkData[-1]->distanceToR;

    Cell* candidate[3];
    short score[3] {0, 0, 0};
    short index = 0;
    for (short k = 0 ; k < adjs.size() ; ++k) {
        if (!adjs[k]->visited) {
            if (index > 2) {
                string message = "index > 2";
                throw runtime_error(message);
            }
            candidate[index++] = adjs[k];
            if (dist[adjs[k]->index] > dist[source->index]) {
                ++score[index];
            }
        }
    }

    if (index == 0) return nullptr;
    return nullptr;
    short highestIndex = 0, highestScore = 0;
    for (short i = 0 ; i < index ; ++i) {
        if (score[i] > highestScore) {
            highestIndex = i;
            highestScore = score[i];
        }
    }
    return candidate[highestIndex];
}

Cell* RobotMap::unvisitedAdj_fatest(Cell* source) {
    vector<Cell*> &adjs = adjCells[source->index];
    for (short k = 0 ; k < adjs.size() ; ++k) {
        if (!adjs[k]->visited) return adjs[k];
    }
    return nullptr;
};

DijkData::DijkData() {};

DijkData::DijkData (int _indexAdjToR, int totalCells) {
    indexAdjToR = _indexAdjToR;
    distanceToR = new int[totalCells];
    cameFrom = new int[totalCells];
    for (int i = 0 ; i < totalCells; ++i) {
        distanceToR[i] = MAXINT;
        cameFrom[i] = -1;
    }
};
DijkData::~DijkData() {
    if (distanceToR) delete [] distanceToR;
    if (cameFrom) delete [] cameFrom;
};

void RobotMap::calcDijkRespectTo(int indexAdjToR) {

    clearPQ(openSet);
    // priority_queue<Qmember> openSet;

    DijkData* dijkData = new DijkData(indexAdjToR, totalCells);
    allDijkData[indexAdjToR] = dijkData;
    int* dist = dijkData->distanceToR;
    int* cameFrom = dijkData->cameFrom;
    deque<Cell*> &unvisited = dijkData->unvisited;
    
    for (int i = 0 ; i < totalCells; ++i) {
        opensetFlag[i] = false;
        closedSetFlag[i] = false;
    }

    Cell* adjToR;
    dist[recharger->index] = 0;

    if (indexAdjToR == -1) {
        openSet.emplace(recharger, 0);
        opensetFlag[recharger->index] = true;
    } else {
        adjToR = cells->cells[indexAdjToR];
        closedSetFlag[recharger->index] = true;
        dist[adjToR->index] =1;
        openSet.emplace(adjToR, dist[adjToR->index]);
        opensetFlag[adjToR->index] = true;
    }

    Cell *current, *neighbor;

    while(openSet.size()) {
        const Qmember &q = openSet.top();
        if (!opensetFlag[q.cell->index]) {
            openSet.pop();
            continue;
        }

        current = q.cell;
        openSet.pop();
        opensetFlag[current->index] = false;
        closedSetFlag[current->index] = true;
        unvisited.push_back(current);
        
        // for each neighbor of current
        for (int i = 0 ; i < adjCells[current->index].size() ; ++i) {
            neighbor = adjCells[current->index][i];

            // if neighbor in closed set
            if (closedSetFlag[neighbor->index]) continue;

            // the distance from R to neighbor via current
            int distance = dist[current->index] + 1;
            
            // if neighbor is in open set and 
            if (opensetFlag[neighbor->index]) {
                if (distance >= dist[neighbor->index]) {
                    // not a better option , do nothing
                } else {
                    // the new path is a better option, do update
                    dist[neighbor->index] = distance;
                    openSet.emplace(neighbor, dist[neighbor->index]);
                    cameFrom[neighbor->index] = current->index;
                }

            // if neighbor not in open set yet
            } else {
                if (distance < dist[neighbor->index]) dist[neighbor->index] = distance;
                openSet.emplace(neighbor, dist[neighbor->index]);
                opensetFlag[neighbor->index] = true;
                cameFrom[neighbor->index] = current->index;
            }
        }
    }
};

void RobotMap::findRandomShortestWayHome(_tmpPathWrapper* pathWrapper, Cell* source) {

    int* dist = allDijkData[-1]->distanceToR;

    Cell *next, *current = source;
    vector<Cell*> cellsMinToR; // adj cells with min dist to R with respect to curent cell
    int minDistance;
    pathWrapper->path.resize(dist[source->index]);
    int pathStoreIndex = dist[source->index] - 1;

    while(current != recharger) {
        cellsMinToR.clear();
        minDistance = dist[current->index];

        vector<Cell*> &adjs = adjCells[current->index];
        for (short i = 0 ; i < adjs.size(); ++i) {
            if (dist[adjs[i]->index] > minDistance) continue;
            if (dist[adjs[i]->index] < minDistance) {
                minDistance = dist[adjs[i]->index];
                cellsMinToR.clear();
            }
            cellsMinToR.push_back(adjs[i]);
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

void RobotMap::findWayHomeViaInc(DqPathWrapper* dqPathWrapper, 
    Cell* source, const int lastIncIndex) {

    int* &dist = allDijkData[lastIncIndex]->distanceToR;
    Cell *next, *current = source;
    vector<Cell*> cellsMinToR; // adj cells with min dist to R with respect to curent cell
    int minDistance;
    int count = 0;
    dqPathWrapper->path.push_back(source);
    
    while(current != recharger) {
        cellsMinToR.clear();
        minDistance = dist[current->index];

        vector<Cell*> &adjs = adjCells[current->index];
        for (short i = 0 ; i < adjs.size(); ++i) {
            
            if (dist[adjs[i]->index] > minDistance) continue;
            if (dist[adjs[i]->index] < minDistance) {
                if (adjs[i] == recharger && lastIncIndex != -1 && current->index != lastIncIndex) continue;
                minDistance = dist[adjs[i]->index];
                cellsMinToR.clear();
            }
            cellsMinToR.push_back(adjs[i]);
        }

        short index = 0;
        if (cellsMinToR.size() > 1) {
            if (!hasMultiplePaths) hasMultiplePaths = true;
            index = rndFrom0To(cellsMinToR.size()-1);
        }

        next = cellsMinToR[index];
        dqPathWrapper->path.push_back(next);
        dqPathWrapper->visitedSum += next->visited;
        current = next;
    }
};

DqPathWrapper* RobotMap::findMinimumPathToR(Cell* source, int indexViaCellAdjToR) {
    vector<DqPathWrapper*> pathsWithLeastVisited;
    int minVisited = MAXINT;
    hasMultiplePaths = false;
    for (short i = 0 ; i < randomHomingPaths ; ++i) {
        DqPathWrapper* dqPathWrapper = new DqPathWrapper();
        findWayHomeViaInc(dqPathWrapper, source, indexViaCellAdjToR);
        if (dqPathWrapper->visitedSum < minVisited) {
            minVisited = dqPathWrapper->visitedSum;
            for (int i = 0 ; i < pathsWithLeastVisited.size() ; ++i) {
                delete pathsWithLeastVisited[i];
            }
            pathsWithLeastVisited.clear();
        }
        if (dqPathWrapper->visitedSum <= minVisited) {
            pathsWithLeastVisited.push_back(dqPathWrapper);
        }
        if (!hasMultiplePaths) break;
    }

    // select random one with minimum visitedSum
    uniform_int_distribution<int> unidist(0, pathsWithLeastVisited.size()-1);
    int index = unidist(randomGenerator);
    DqPathWrapper* d = pathsWithLeastVisited[index];
    pathsWithLeastVisited[index] = nullptr;

    for (int i = 0 ; i < pathsWithLeastVisited.size() ; ++i) {
        delete pathsWithLeastVisited[i];
    }

    return d;
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
    if (num == 1) return unidist0to1(randomGenerator);
    if (num == 2) return unidist0to2(randomGenerator);
    if (num == 3) return unidist0to3(randomGenerator);
    throw std::runtime_error("Error: input num > 3, which is not allowed.");
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

void RobotMap::printDijkData() {
    Cell* cell;
    int dist;
    for (map<int, DijkData*>::iterator it = allDijkData.begin();
            it != allDijkData.end() ; ++it) {
        DijkData &dijk = *(it->second);

        cout << "indexAdjToR : " << dijk.indexAdjToR << ", There are " << dijk.unvisited.size() << " unvisited in the list." << endl;
        cout << ", distance to R: ";

        for (short i = 0 ; i < rows; ++i) {
            for (short j = 0 ; j < columns ; ++j) {
                cell = cells->get(i, j);
                if (!cell) {
                    cout << "XXX ";
                    continue;
                }
                //cellInfo(cell);
                dist = dijk.distanceToR[cell->index];
                if (dist < 10) {
                    cout << "  " << dist << " ";
                } else if (dist < 100) {
                    cout << " " << dist << " ";
                } else {
                    cout << dist;
                }
            }
            cout << endl;
        }
        cout << endl;
    }
};

void RobotMap::cellInfo(Cell* cell) {
    cout << "cell["<<cell->index<<"]: "
        << "("<<cell->i<<", "<<cell->j<<"), "
        << "visited("<<(cell->visited? "True" : "False") << ").";
};