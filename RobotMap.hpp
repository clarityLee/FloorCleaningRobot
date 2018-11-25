#include <fstream>
#include <vector>
#include <string>
#include <map>
#include <queue>
#include <chrono>
#include <random>
using namespace std;
const int MAXINT = 2147483647;
const short randomHomingPaths = 10;

class Cell {
public:
    Cell(short theI, short theJ, int theIndex) : i(theI), j(theJ), index(theIndex) {};
    const int index;
    const short i;
    const short j;
    int distance = MAXINT;
    int _tempDistance = 0;
    bool visited = false;
    string info();
};

class Cells {
public:
    Cells(int reserveSize);
    ~Cells();
    vector<Cell*> cells;
    int size();
    void push_back(Cell* cell);
    void resetTempDistance();
    Cell* back();
    Cell* get(short i, short j);
    Cell* get(int index);
};

class _tmpPathWrapper {
public:
    vector<Cell*> path;
    int visitedSum = 0;
};

class RobotMap {
friend class CleaningRobot;
public:
    RobotMap(short rows, short columns);
    ~RobotMap();
    void readFloorData(std::ifstream&);
    void processData();
    void findClosestUnvisited(vector<Cell*> &path, Cell* source);
    void findClosestUnvisited(vector<Cell*> &path, Cell* source, Cell* lastIncToR);
    void randomShortestWayHome(vector<Cell*> &path, Cell* current);
    Cell* randomStart(); // returns random one of recharger's neighbor;
    Cell* unvisitedAdjacent(Cell* cell);

private:
    
    short rows = 0, columns = 0;
    // short rx = 0, ry = 0; // rx, ry : x, y positoin of recharger.
    int totalCells = 0;
    char** rawData;
    bool hasMultiplePaths = false;
    Cells* cells;
    Cell* recharger = nullptr;
    vector<vector<Cell*>> adjCells;

    bool* opensetFlag = nullptr;
    bool* closedSetFlag = nullptr;
    int* cameFrom = nullptr;

    mt19937 randomGenerator;
    uniform_int_distribution<short> unidist0to1, unidist0to2, unidist0to3;
    inline short rndFrom0To(short num);

    void calculateDistanceToR();
    /* TODO: */ void calculateDistanceToR(Cell* adjToR);
    void findRandomShortestWayHome(_tmpPathWrapper* pathWrapper, Cell* source);
    void constructPath(vector<Cell*> &path, int* cameFrom, Cell* current);

    // for test:
    void printAllDistance();
    void printPath(vector<Cell*> &path);
};

class CellCompareForInit {
public: bool operator() (Cell* const & lhs, Cell* const & rhs) const;
};
class CellCompare {
public: bool operator() (Cell* const & lhs, Cell* const & rhs) const;
};
class PathCompare {
public: bool operator() (_tmpPathWrapper* const & lhs, _tmpPathWrapper* const & rhs) const;
};