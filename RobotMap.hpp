#include <fstream>
#include <vector>
#include <string>
#include <map>
#include <queue>
#include <chrono>
#include <random>
#include <deque>
using namespace std;
const int MAXINT = 2147483647;
const short randomHomingPaths = 20;

class Cell {
public:
    Cell(short theI, short theJ, int theIndex) : i(theI), j(theJ), index(theIndex) {};
    const int index;
    const short i;
    const short j;
    bool visited = false;
};

class Cells {
public:
    Cells(int reserveSize);
    ~Cells();
    vector<Cell*> cells;
    int size();
    void push_back(Cell* cell);
    Cell* back();
    Cell* get(short i, short j);
    Cell* get(int index);
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

class DijkData;
class RobotMap {
friend class CleaningRobot;
public:
    RobotMap(short rows, short columns);
    ~RobotMap();
    void readFloorData(std::ifstream&);
    void processData();
    void findClosestUnvisited(vector<Cell*> &path, Cell* source);
    void findClosestUnvisited(vector<Cell*> &path, Cell* source, Cell* lastIncToR);
    void findClosestUnvisitedToR(vector<Cell*> &path, int lastIncIndex);
    void randomShortestWayHome(vector<Cell*> &path, Cell* current);
    Cell* randomStart(); // returns random one of recharger's neighbor;
    Cell* unvisitedAdjacent(Cell* source);
    Cell* unvisitedAdj_v2(Cell* source);
    Cell* unvisitedAdj_v3(Cell* source);
    Cell* unvisitedAdj_fatest(Cell* source);
    Cell* nextCell();

private:
    short rows = 0, columns = 0;
    // short rx = 0, ry = 0; // rx, ry : x, y positoin of recharger.
    int totalCells = 0;
    int edges = 0;
    char** rawData;
    bool hasMultiplePaths = false;
    Cells* cells;
    Cell* recharger = nullptr;
    vector<vector<Cell*>> adjCells;

    vector<Cell*> adjs_with_visitedAdjs;
    vector<Cell*> adjs_without_visitedAdjs;
    vector<Cell*> adjs_S;
    vector<Cell*> adjs_EG;

    bool* opensetFlag = nullptr;
    bool* closedSetFlag = nullptr;

    int* tmpCameFrom = nullptr;
    int* tmpDist = nullptr;
    int* distToR; // do not deallocate this one

    mt19937 randomGenerator;
    uniform_int_distribution<short> unidist0to1, unidist0to2, unidist0to3;
    inline short rndFrom0To(short num);

    short pathDirection = 1; // 1 or -1
    deque<Cell*> miniPath;
    priority_queue<Qmember> openSet;
    map<int, DijkData*> allDijkData;
    void calcDijkRespectTo(int indexAdjToR);
    void findRandomShortestWayHome(_tmpPathWrapper* pathWrapper, Cell* source);
    void findWayHomeViaInc(DqPathWrapper*, Cell* source, const int lastIncIndex);
    DqPathWrapper* findMinimumPathToR(Cell* source, int indexViaCellAdjToR);
    void constructPath(vector<Cell*> &path, int* cameFrom, Cell* current);

    // for test:
    // void printAllDistance();
    void printPath(vector<Cell*> &path);
    void printDijkData();
    void cellInfo(Cell* cell);
    int DijkMs = 0;
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

class CellCompare {
public: bool operator() (Cell* const & lhs, Cell* const & rhs) const;
};
class PathCompare {
public: bool operator() (_tmpPathWrapper* const & lhs, _tmpPathWrapper* const & rhs) const;
};
class DqPathCompare {
public: bool operator() (DqPathWrapper* const &lhs, DqPathWrapper* const & rhs) const;
};