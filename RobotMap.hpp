#include <fstream>
#include <vector>
#include <string>
#include <map>
using namespace std;
const int MAXINT = 2147483647;
const short randomHomingPaths = 2;

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

class CellCompareForInit {
public: bool operator() (Cell* const & lhs, Cell* const & rhs) const;
};
class CellCompare {
public: bool operator() (Cell* const & lhs, Cell* const & rhs) const;
};
class PathCompare {
public: bool operator() (_tmpPathWrapper* const & lhs, _tmpPathWrapper* const & rhs) const;
};

class Coordinate {
public:
    Coordinate(short _i, short _j) : i(_i), j(_j) {};
    short i, j;
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
    void recordPath(Cell* cell);
    Cell* unvisitedAdjacent(Cell* cell);
private:
    
    short rows = 0, columns = 0,
        rx = 0, ry = 0; // rx, ry : x, y positoin of recharger.
    int totalCells = 0;
    char** rawData;
    bool hasMultiplePaths = false;
    // bool** visited;
    Cells* cells;
    Cell* recharger = nullptr;
    vector<vector<Cell*>> adjCells;
    vector<Coordinate> path;
    // int* distance;
    void calculateDistanceToR();
    void a_star(vector<Cell*> &path, Cell* source, Cell* destination);
    void findRandomShortestWayHome(_tmpPathWrapper* pathWrapper, Cell* source);
    void constructPath(vector<Cell*> &path, map<Cell*, Cell*> &cameFrom, Cell* current);

    // for test:
    void printAllDistance();
    void printPath(vector<Cell*> &path);

    
};
