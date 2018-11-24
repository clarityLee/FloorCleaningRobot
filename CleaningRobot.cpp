#include <vector>
#include <chrono>
#include "CleaningRobot.hpp"
using namespace std;
constexpr auto &&now = std::chrono::high_resolution_clock::now;

CleaningRobot::CleaningRobot() {};
CleaningRobot::~CleaningRobot() {
    if (map) delete map;
};

void CleaningRobot::readFloorData(int argc, char* argv[]) {
    if (argc < 2) {
        errorFlag = true;
        errorMessage = "Missing command argument. Please specify the path to floor.data.";
        return;
    }

    ifstream readInFile(string(argv[1]) + "/floor.data");
    if (!readInFile.is_open()) {
        errorFlag = true;
        errorMessage = "Unable to open \"" + string(argv[1]) + "/floor.data\". Either file does not exist or wrong path.";
        return;
    }

    short rows = 0 , columns = 0;
    readInFile >> rows >> columns >> maxBattery;

    map = new RobotMap(rows, columns);

    auto phaseStartTime = now();
    map->readFloorData(readInFile);
    chrono::duration<double> elapsed = now() - phaseStartTime;
    cout << "RobotMap::readFloorData() completed in : " << elapsed.count() * 1000 << " ms." << endl;
    phaseStartTime = now();
    map->processData();
    elapsed = now() - phaseStartTime;
    cout << "RobotMap::processData() completed in : " << elapsed.count() * 1000 << " ms." << endl;
}; 

bool CleaningRobot::hasError() {
    return errorFlag;
};

int CleaningRobot::printError() {
    cout << "Error: " << errorMessage << endl;
    return 1;
};

void CleaningRobot::clean() {

    int unvisitedCount = map->totalCells;
    bool reCalculate = false;
    bool isGoingHome = false;
    bool needToSetHome = false;
    int battery = 0;
    Cell* current = map->recharger;
    map->recordPath(current);
    Cell* next = nullptr;
    Cell* lastIncToR = nullptr;
    Cell* destination = nullptr;
    vector<Cell*> path; // contains current -> goal after set
    // int loopcount = 1;
    // const int destcount = 10000;

    cout << endl;
    while(current != map->recharger || unvisitedCount) {

        if (!current->visited) {
            // cout << "--current: (" << current->i << ", " << current->j << ") is not visited, setting current visited."  << endl;
            current->visited = true;
            --unvisitedCount;
        } else {
            // cout << "--current: (" << current->i << ", " << current->j << ") is visited already."  << endl;
        }

        // cout << "--current: (" << current->i << ", " << current->j << "). distance to R: " 
        // << current->distance << ", remaining battery: " << battery 
        // << ", total cells: " << map->totalCells << ", unvisited: " 
        // << unvisitedCount << ", Using step: " << usingSteps << ", loopcount:" << loopcount << endl;

        // 1. determine next movement

        // 1.1 robot is at recharger
        if (current == map->recharger) {
            battery = maxBattery;
            // cout << "           battery recharged. reamining battery :" << battery << endl;
            isGoingHome = false;

            path.clear();
            if (lastIncToR == nullptr) {
                // cout << "           fresh start. current->visited: " << (current->visited ? "TRUE" : "FALSE") << endl;
                path.push_back(map->randomStart());
            } else {
                // cout << "           finding closest unvisited cell. lastIncToR: (" << lastIncToR->i << ", " << lastIncToR->j << ")" << endl;
                map->findClosestUnvisited(path, map->recharger, lastIncToR);
                // map->printPath(path);
            }

            if (path.back() == current) path.pop_back();
            next = path.back();

            /*
            if (reCalculate) {
            reCalculate = false;
            */
            

            /* TODO: 
            recalculate partial Dijkstra for inc direction until closest unvisited
            Set distance table;
            Set route table;
            */

            /* TODO: 
            set destination to random one of the closest unvisited cell according to distanceTale & routeTable
            set path
            next = path.front();
            path.pop() // ??
            */

        // 1.2 robot is not at recharger
        } else {

            // TODO: need to check battery for current step to reduce some process.

            if (next != nullptr) {
                // cout << "           has next target. next: " << next << endl;

                // robot is on the way to the closest unvisited cell
                if (!isGoingHome) {
                // (using normal dijkstra distance)
                    // cout << "           robot is not going home. check battery for next." << endl;
                    // cout << "           next: (" << next->i << ", " << next-> j << "), distance to R :" << next->distance << endl;
                    needToSetHome = !enoughBattery(next, battery-1);
                    // cout << "           Battery check complete. needToSetHome: " << (needToSetHome ? "TRUE" : "FALSE") << endl;
                
                // robot is homing
                } else {
                    // cout << "           robot is going home." << endl;
                    if (next == map->recharger) {
                        if (lastIncToR != current) {
                            // TODO: recalculate?
                            reCalculate = true;
                            lastIncToR = current;
                        }
                    }			
                }

            // robot has no goal now, robot is roaming
            } else {
                // cout << "           having no next target." << endl;
                
                // all the cells are visited
                if (unvisitedCount == 0) {
                    // cout << "           set needToSetHome = true." << endl;
                    needToSetHome = true;

                // still some cells not visited
                } else {
                    // has unvisited neighbor?
                    next = map->unvisitedAdjacent(current);
                    if (next) {
                        // cout << "           one of unvisitedAdjacent is choosed." << endl;
                        // cout << "           next: (" << next->i << ", " << next-> j << "), distance to R :" << next->distance << ", visited: " << (next->visited ? "TRUE" : "FALSE") << endl;
                        needToSetHome = !enoughBattery(next, battery-1);
                    
                    } else {
                        // cout << "           There is no unvisitedAdjacent." << endl;
                        path.clear();
                        map->findClosestUnvisited(path, current);
                        // map->printPath(path);
                        if (path.back() == current) path.pop_back();
                        next = path.back();
                        needToSetHome = !enoughBattery(next, battery-1);
                    }
                }
            }
        } 

        if (needToSetHome) {
            // cout << "           need to set home" << endl;
            map->randomShortestWayHome(path, current);
            // map->printPath(path);
            if (path.back() == current) path.pop_back();
            next = path.back();
            isGoingHome = true;
            needToSetHome = false;
        } else {
            // cout << "           No need to set home" << endl;
        }

        // 2. move !
        // cout << "           start moving!" << endl;
        if (next == map->recharger) lastIncToR = current;
        current = next;
        if (path.back() == current) path.pop_back();
        if (path.empty()) {
            next = nullptr;
        } else {
            next = path.back();
        }
        map->recordPath(current);

        ++usingSteps;
        battery -= 1;

        // if (loopcount == destcount) break;
        // ++loopcount;
    }
};

void CleaningRobot::outputPath(int argc, char* argv[]) {
    ofstream o(string(argv[1]) + "/final.path");
    o << usingSteps << ' ' << endl;
    for (int i = 0 ; i < map->path.size() ; ++i) {
        o << map->path[i].i << ' ' << map->path[i].j << endl;
    }
};

inline bool CleaningRobot::enoughBattery(Cell* cell, int battery) {
    // cout << "           check: battery: " << battery << ", distance: " << cell->distance << endl;
    return battery >= cell->distance;
};

void CleaningRobot::test() {

    // map->calculateDistanceToR();
    map->printAllDistance();

    /*
    cout << "Printing test data of Robot: " << endl;
    cout << "map->adjCells.size() : " << map->adjCells.size() << endl;
    for (int i = 0 ; i < map->adjCells.size() ; ++i) {
        Cell* cell = map->cells->get(i);
        cout << "Position(" << cell->i << ", " << cell->j << ") : ";
        cout << "adjCells[" << i << "].size() : " << map->adjCells[i].size() << ", ";
        vector<Cell*> & cells = map->adjCells[i];
        cout << ", capacity : " << cells.capacity();
        for (int j = 0 ; j < cells.size() ; ++j) {
            Cell* cell = cells[j];
            cout << "| index: " << cell->index << ", (" << cell->i << ", " << cell->j << ") |";
        }
        cout << endl;
    }
    */
};