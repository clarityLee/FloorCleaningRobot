#include <fstream>
#include <vector>
#include <string>
#include <map>
#include <chrono>
#include <random>
#include <queue>
#include <deque>
using namespace std;
const int MAXINT = 2147483647;

class Cell {
public:
    Cell(int _index) : index(_index) {};
    const int index;
    Cell* adjCells[4] {nullptr, nullptr, nullptr, nullptr};
    bool visited = false;
};

class coordinate {
public:
    coordinate(short _i, short _j) : i(_i), j(_j) {};
    short i, j;
};

class Qmember {
public:
    Qmember() {};
    Qmember(Cell* _c, int _d) : cell(_c), distance(_d) {};
    Cell* cell;
    int distance;
    bool operator< (const Qmember &rhs) const;
};

class _tmpPathWrapper {
public:
    vector<Cell*> path;
    int visitedSum = 0;
};
class DqPathWrapper {
public:
    deque<Cell*> path; // source is at front, dest is at end
    int visitedSum = 0;
};

/* This is a special queue that can only do "push" for "capacity" times
   If you execute "push" for more than "capacity" times, it will explode. */
class spQueue {
public:
    spQueue(int _capacity);
    ~spQueue();
    void push(Cell*);
    void pop();
    bool empty();
    Cell* front();
private:
    spQueue();
    int capacity;
    int size = 0;
    int first = 0;
    int last = -1;
    Cell** q;
};

class DijkData;
class RobotMap {
public:
    RobotMap(short rows, short columns);
    ~RobotMap();
    void readFloorData(std::ifstream&);
    void processData();
    void resetForRefine();
    
    void findClosestUnvisitedv3(vector<Cell*> &path, Cell* source); // bfs, using !!
    void findClosestUnvisitedToR(vector<Cell*> &path, int lastIncIndex); // using !!
    void randomShortestWayHome(vector<Cell*> &path, Cell* current); // using !!
    Cell* getRecharger(); // using!!
    Cell* randomStart(); // using

    // using !! higher priority for those with smaller distance to R
    Cell* unvisitedAdj_min(Cell* source); 

    // using !! : higher priority for those with farer distance to R and cling to visited
    Cell* unvisitedAdj_v2(Cell* source);

    int getTotalCells();
    int distanceToRecharger(int cellIndex); // using !!
    vector<coordinate> cellCoords;
    int bfsTraversedNodes = 0;
    int edges = 0;

    /* --- bellowing are functions not using currently, just for future reference ---*/
    void findClosestUnvisited(vector<Cell*> &path, Cell* source); // dijk, (not using)
    void findFarestUnvisitedToR(vector<Cell*> &path, int lastIncIndex); // (not using)
    Cell* closestUnvisited(int lastIncIndex); // (not using)
    // (not using) : higher priority for those with farer distance to R
    Cell* unvisitedAdjacent(Cell* source); 
    Cell* unvisitedAdj_v3(Cell* source); // (not using) I forget what this is ....
    Cell* unvisitedAdj_fatest(Cell* source); // (not using) just pick first unvisited
    int distanceToRecharger(Cell* cell); // (not using)

private:
    int totalCells = 0;
    short randomHomingPaths = 40;
    short rows = 0, columns = 0;
    char** rawData;
    bool hasMultiplePaths = false;
    vector<Cell> cells;
    Cell* recharger = nullptr;

    vector<Cell*> adjs_with_visitedAdjs;
    vector<Cell*> adjs_without_visitedAdjs;
    vector<Cell*> adjs_S;
    vector<Cell*> adjs_EG;

    // important !! do not deallocate this one!! it's handled in destruct map already.
    int* distToR;

    mt19937 randomGenerator;
    uniform_int_distribution<short> unidist0to1, unidist0to2, unidist0to3;
    inline short rndFrom0To(short num);

    priority_queue<Qmember> openSet;
    map<int, DijkData*> allDijkData;
    void calcAllDijk();
    void calcDijkRespectTo(int indexAdjToR); // using !!
    void findRandomShortestWayHome(_tmpPathWrapper* pathWrapper, Cell* source); // (not using)
    void findWayHomeViaInc(DqPathWrapper*, Cell* source, const int lastIncIndex); // using !!
    DqPathWrapper* findMinimumPathToR(Cell* source, int indexViaCellAdjToR); // using !!
    inline void constructPath(vector<Cell*> &path, int* cameFrom, Cell* current); // using !!
    inline void constructPath(vector<Cell*> &path, vector<int> &cameFrom, Cell* current); // (not using)
    int getIndex(short i, short j); // using !!

    // for test:
    void printPath(vector<Cell*> &path);
    void printPath(deque<Cell*> path);
    void printDijkData();
    void cellInfo(Cell* cell);
};

class DijkData {
public:
    DijkData();
    DijkData(int indexAdjToR, int totoalCells);
    ~DijkData();
    int indexAdjToR;
    int* distanceToR;
    int* cameFrom;
    
    deque<Cell*> unvisited; // front is closest.
};

class PathCompare {
public: bool operator() (_tmpPathWrapper* const & lhs, _tmpPathWrapper* const & rhs) const;
};
class DqPathCompare {
public: bool operator() (DqPathWrapper* const &lhs, DqPathWrapper* const & rhs) const;
};