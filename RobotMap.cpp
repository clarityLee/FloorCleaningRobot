#include <iostream>
#include <vector>
#include <map>
#include <string>
#include <random>
#include <chrono>
#include <queue>
#include <deque>
#include "RobotMap.hpp"
using namespace std;

bool PathCompare::operator() (_tmpPathWrapper* const & lhs, _tmpPathWrapper* const & rhs) const {
    return lhs->visitedSum > rhs->visitedSum;
};
bool DqPathCompare::operator() (DqPathWrapper* const & lhs, DqPathWrapper* const & rhs) const {
    return lhs->visitedSum > rhs->visitedSum;
};

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

template <class T, class S, class C>
void shrinkToFit(priority_queue<T, S, C>& q) {
    struct HackedQueue : private priority_queue<T, S, C> {
        static S& Container(priority_queue<T, S, C>& q) {
            return q.*&HackedQueue::c;
        }
    };
    HackedQueue::Container(q).shrink_to_fit();
};

spQueue::spQueue(int _capacity) : capacity(_capacity) {
    q = new Cell*[capacity];
};
spQueue::~spQueue() {
    delete [] q;
};
void spQueue::push(Cell* c) {
    q[++last] = c;
    ++size;
}
void spQueue::pop() {
    if (!size) return;
    ++first;
    --size;
};
Cell* spQueue::front() {
    if (!size) return nullptr;
    return q[first];
};
bool spQueue::empty() {
    return !size;
}

RobotMap::RobotMap(short r, short c) : rows(r), columns(c),
        randomGenerator(chrono::high_resolution_clock::now().time_since_epoch().count()),
        unidist0to1(0, 1),
        unidist0to2(0, 2),
        unidist0to3(0, 3) {
    rawData = new char*[rows];
    cells.reserve(r*c);
    for (short i = 0 ; i < rows ; ++i) {
        rawData[i] = new char[columns];
    }
};

RobotMap::~RobotMap() {
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
                // cells.push_back(new Cell(cellIndex++));
                cells.emplace_back(cellIndex++);
                cellCoords.emplace_back(i ,j);
            } else if (rawData[i][j] == 'R') {
                // cells.push_back(new Cell(cellIndex++));
                cells.emplace_back(cellIndex++);
                cellCoords.emplace_back(i ,j);
                recharger = &(cells.back());
            }
        }
    }
    totalCells = cells.size();
    chrono::duration<double> elapsed = now() - phaseStartTime;
    cout << "    - RobotMap::readFloorData(), total Cells: " << totalCells << ", total used time : " << (int) (elapsed.count() * 1000) << " ms." << endl;
};

void RobotMap::processData() {
    constexpr auto &&now = std::chrono::high_resolution_clock::now;

    // construct adjacency list
    cout << "    - RobotMap::processData : construct adjacency list of all cells..." << endl;
    auto phaseStartTime = now();
    for (short i = 0 ; i < rows ; ++i) {
        for (short j = 0 ; j < columns ; ++j) {
            if (rawData[i][j] == '1') continue;
            short index = 0;
            Cell &cell = cells[getIndex(i ,j)];
            if (i>0 && rawData[i-1][j] != '1') {
                cell.adjCells[index++] = &(cells[getIndex(i-1, j)]);
                ++edges;
            }
            if (j<columns-1 && rawData[i][j+1] != '1') {
                cell.adjCells[index++] = &(cells[getIndex(i, j+1)]);
                ++edges;
            }
            if (i<rows-1 && rawData[i+1][j] != '1') {
                cell.adjCells[index++] = &(cells[getIndex(i+1, j)]);
                ++edges;
            }
            if (j>0 && rawData[i][j-1] != '1') {
                cell.adjCells[index++] = &(cells[getIndex(i, j-1)]);
                ++edges;
            }
        }
    }
    edges/=2;
    chrono::duration<double> elapsed = now() - phaseStartTime;
    cout << "    - RobotMap::processData : adjacency list of all cells completed in : "
        << (int) (elapsed.count() * 1000) << " ms." << endl;

    // using dijkstra to calculate distance
    phaseStartTime = now();
    calcAllDijk();
    elapsed = now() - phaseStartTime;
    cout << "    - RobotMap::processData : calculating all cell distances to recharger. / Completed in : "
         << (int) (elapsed.count() * 1000) << " ms." << endl;

    // deallocate raw floor data
    for (short i = 0 ; i < rows ; ++i) {
        delete rawData[i];
    }
    delete [] rawData;
};

void RobotMap::resetForRefine() {
    bfsTraversedNodes = 0;
    for (int i = 0 ; i < cells.size() ; ++i) {
        cells[i].visited = false;
    }
    calcAllDijk();
};

void RobotMap::findClosestUnvisited(vector<Cell*> &path, Cell* source) {

    bool* opensetFlag = new bool[totalCells];
    bool* closedSetFlag = new bool[totalCells];
    clearPQ(openSet);

    int* dist = new int[totalCells];
    int* cameFrom = new int[totalCells];
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

        for (short i = 0 ; i < 4 && current->adjCells[i] != nullptr; ++i) {
            neighbor = current->adjCells[i];

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
    delete [] opensetFlag;
    delete [] closedSetFlag;
    delete [] dist;
    delete [] cameFrom;
};

void RobotMap::findClosestUnvisitedv3(vector<Cell*> &path, Cell* source) {
    spQueue q(totalCells);
    bool* added = new bool[totalCells];
    int* cameFrom = new int[totalCells];
    for (int i = 0 ; i < totalCells ; ++i) {
        added[i] = false;
        cameFrom[i] = -1;
    }
    
    q.push(source);
    added[source->index] = true;
    cameFrom[source->index] = -1;

    Cell* goal;
    while(!q.empty()) {
        Cell* current = q.front();
        q.pop();
        ++bfsTraversedNodes;
        if (!current->visited) {
            goal = current;
            break;
        }

        for (short i = 0 ; i < 4 && current->adjCells[i] != nullptr ; ++i) {
            Cell* adj = current->adjCells[i];
            if (added[adj->index]) continue;
            if (adj == recharger) continue;

            q.push(adj);
            cameFrom[adj->index] = current->index;
            added[adj->index] = true;
        }
    }

    constructPath(path, cameFrom, goal);

    delete [] added;
    delete [] cameFrom;
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

void RobotMap::findFarestUnvisitedToR(vector<Cell*> &path, int lastIncIndex) {

    DijkData &dijkData = *(allDijkData[lastIncIndex]);
    Cell* farestUnvisited;
    while(true) {
        farestUnvisited = dijkData.unvisited.back();
        if (farestUnvisited->visited) dijkData.unvisited.pop_back();
        else break;
        if (dijkData.unvisited.size() == 0) {
            string errormessage = "dijkData.unvisited.size() == 0";
            throw runtime_error(errormessage);
        }
    }

    DqPathWrapper* d = findMinimumPathToR(farestUnvisited, lastIncIndex);
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

Cell* RobotMap::getRecharger() {
    return recharger;
};

Cell* RobotMap::closestUnvisited(int lastIncIndex) {
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
    return closestUnvisited;
};

Cell* RobotMap::randomStart() {
    short maxIndex = 3;
    for (short i = 0 ; i < 4; ++i) {
        if (recharger->adjCells[i] == nullptr) {
            maxIndex = i - 1;
            break;
        }
    }

    int useNeighborIndex = 0;
    if (maxIndex > 0) {
        useNeighborIndex = rndFrom0To(maxIndex);
    }

    return recharger->adjCells[useNeighborIndex];
};

Cell* RobotMap::unvisitedAdjacent(Cell* cell) {
    int* &dist = allDijkData[-1]->distanceToR;
    Cell* (&adjs) [4] = cell->adjCells;

    vector<Cell*> adjs_S, adjs_EG;
    for (short i = 0 ; i < 4 && adjs[4] != nullptr ; ++i) {
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
    
    Cell* (&adjs) [4] = source->adjCells;

    adjs_with_visitedAdjs.clear(); // better choice
    adjs_without_visitedAdjs.clear();

    for (short i = 0 ; i < 4 && adjs[i] != nullptr ; ++i) {
        Cell* adj = adjs[i];
        if (adj->visited) continue;
        bool hasVisitedAdj = false;
        Cell* (&adjsOfadj)[4] = adj->adjCells;
        for (short j = 0 ; j < 4 && adjsOfadj[j] != nullptr ; ++j) {
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
        v = &adjs_without_visitedAdjs;
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

    if (adjs_S.empty() && adjs_EG.empty()) {
        return nullptr;
    }
    if (!adjs_EG.empty()) {
        v = &adjs_EG;
    } else {
        v = &adjs_S;
    }

    short index = 0;
    if ((*v).size() > 1) {
        index = rndFrom0To((*v).size()-1);
    }

    return (*v)[index];
}

Cell* RobotMap::unvisitedAdj_v3(Cell* source) {
    Cell* (&adjs)[4] = source->adjCells;
    int* &dist = allDijkData[-1]->distanceToR;

    Cell* candidate[3];
    short score[3] {0, 0, 0};
    short index = 0;
    for (short k = 0 ; k < 4 && adjs[k] != nullptr ; ++k) {
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
    for (short k = 0 ; k < 4 && source->adjCells[k] != nullptr ; ++k) {
        if (source->adjCells[k] == recharger) continue;
        if (!source->adjCells[k]->visited) return source->adjCells[k];
    }
    return nullptr;
};

Cell* RobotMap::unvisitedAdj_min(Cell* source) {
    int* &dist = allDijkData[-1]->distanceToR;
    int minDist = MAXINT;
    int useIndex = -1;
    for (short k = 0 ; k < 4 && source->adjCells[k] != nullptr ; ++k) {
        if (source->adjCells[k] == recharger) continue;
        if (source->adjCells[k]->visited) continue;
        if (dist[source->adjCells[k]->index] < minDist) {
            minDist = dist[source->adjCells[k]->index];
            useIndex = k;
        }
    }
    if (useIndex == -1) return nullptr;
    else return source->adjCells[useIndex];
};

int RobotMap::getTotalCells() {
    return totalCells;
};

int RobotMap::distanceToRecharger(int cellIndex) {
    return distToR[cellIndex];
};

int RobotMap::distanceToRecharger(Cell* cell) {
    return distToR[cell->index];
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

void RobotMap::calcAllDijk() {
    allDijkData.clear();
    resizePQ(openSet, totalCells);
    calcDijkRespectTo(-1); // respect to nothing
    distToR = allDijkData[-1]->distanceToR;
    for (short k = 0 ; k < 4 && recharger->adjCells[k] != nullptr; ++k) {
        calcDijkRespectTo(recharger->adjCells[k]->index);
    }
    clearPQ(openSet); shrinkToFit(openSet);
};

void RobotMap::calcDijkRespectTo(int indexAdjToR) {

    clearPQ(openSet);
    bool* opensetFlag = new bool[totalCells];
    bool* closedSetFlag = new bool[totalCells];

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
        adjToR = &(cells[indexAdjToR]);
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
        if (indexAdjToR != -1) {
            unvisited.push_back(current);
        }
        
        for (short i = 0 ; i < 4 && current->adjCells[i] != nullptr ; ++i) {
            neighbor = current->adjCells[i];

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

    delete [] opensetFlag;
    delete [] closedSetFlag;
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

        // vector<Cell*> &adjs = adjCells[current->index];
        Cell* (&adjs)[4] = current->adjCells;
        for (short i = 0 ; i < 4 && adjs[i] != nullptr ; ++i) {
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

        Cell* (&adjs) [4] = current->adjCells;
        for (short i = 0 ; i < 4 && adjs[i] != nullptr ; ++i) {
            
            if (adjs[i] == recharger && lastIncIndex != -1 && current->index != lastIncIndex) continue;
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

inline void RobotMap::constructPath(vector<Cell*> &path, int* cameFrom, Cell* current) {
    int currentIndex = current->index;
    path.clear();
    path.push_back(current);
    
    while(true) {
        // if current does not exist in cameFrom
        if (cameFrom[currentIndex] == -1) break;

        currentIndex = cameFrom[currentIndex];
        path.push_back(&(cells[currentIndex]));
    }
};

inline void RobotMap::constructPath(vector<Cell*> &path, vector<int> &cameFrom, Cell* current) {
    cout << endl << "-- using constructPath vector cameFrom version" << endl;
    int currentIndex = current->index;
    path.clear();
    path.push_back(current);
    
    while(true) {

        // if current does not exist in cameFrom
        if (cameFrom[currentIndex] == -1) break;

        currentIndex = cameFrom[currentIndex];
        path.push_back(&(cells[currentIndex]));
    }
};

inline short RobotMap::rndFrom0To(short num) {
    if (num == 1) return unidist0to1(randomGenerator);
    if (num == 2) return unidist0to2(randomGenerator);
    if (num == 3) return unidist0to3(randomGenerator);
    throw std::runtime_error("Error: input num > 3, which is not allowed.");
};

int RobotMap::getIndex(short i, short j) {
    // using binary search
    int left = 0, right = cellCoords.size() - 1;
    int mid;
    while(left <= right) {
        mid = left + (right-left)/2;
        coordinate &cm = cellCoords[mid];
        if (i == cm.i && j == cm.j) return mid;
        if (i < cm.i || (i == cm.i && j < cm.j)) {
            right = mid - 1;
        } else if (i > cm.i || (i == cm.i && j > cm.j)) {
            left = mid + 1;
        }
    }
    return -1;
};

void RobotMap::printPath(vector<Cell*> &path) {
    cout << "           ";
    cout << "There are " << path.size() << " cells in the path : ";
    for (int i = path.size() - 1, j = 0 ; i >= 0 ; --i, ++j) {
        if (j%4 == 0) cout << endl << "           ";
        coordinate &c = cellCoords[path[i]->index];
        cout << "(" << c.i << ", " << c.j << ")";
        if (i>0) cout << "  ->  ";
    }
    cout << endl;
};

void RobotMap::printPath(deque<Cell*> path) {
    cout << "           ";
    cout << "There are " << path.size() << " cells in the path : ";
    int count = 0;
    while(!path.empty()) {
        Cell* cell = path.back();
        path.pop_back();
        if (count % 10 == 0) cout << endl << "           ";
        coordinate &c = cellCoords[cell->index];
        cout << "(" << c.i << ", " << c.j << ")";
        if (!path.empty()) cout << "  ->  ";
        ++count;
    }
    cout << endl;
}

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
                cell = &(cells[getIndex(i ,j)]);
                if (!cell) {
                    cout << "XXX ";
                    continue;
                }
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
    coordinate &c = cellCoords[cell->index];
    cout << "cell["<<cell->index<<"]: "
        << "("<<c.i<<", "<<c.j<<"), "
        << "visited("<<(cell->visited? "True" : "False") << ").";
};