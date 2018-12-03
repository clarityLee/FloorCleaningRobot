#include <vector>
#include <chrono>
#include <functional>
#include <sstream>
#include <cstdio>
// #include <map>
#include "CleaningRobot.hpp"
using namespace std;

CleaningRobot::CleaningRobot() {};
CleaningRobot::~CleaningRobot() {
    if (rmap) delete rmap;
};

void CleaningRobot::readFloorData(int argc, char* argv[]) {
    if (argc < 2) {
        errorFlag = true;
        errorMessage = "Missing command argument. Please specify the path to floor.data.";
        return;
    }

    filePath = string(argv[1]);
    ifstream readInFile(filePath + "/floor.data");
    if (!readInFile.is_open()) {
        errorFlag = true;
        errorMessage = "Unable to open \"" + filePath + "/floor.data\". Either file does not exist or wrong path.";
        return;
    }


    short rows = 0 , columns = 0;
    readInFile >> rows >> columns >> maxBattery;

    rmap = new RobotMap(rows, columns);
    rmap->readFloorData(readInFile);
    rmap->processData();

    finalPath.reserve(101000);
}; 

bool CleaningRobot::hasError() {
    return errorFlag;
};

int CleaningRobot::printError() {
    cout << "Error: " << errorMessage << endl;
    return 1;
};

void CleaningRobot::clean() {
    constexpr auto &&now = std::chrono::high_resolution_clock::now;

    // ofstream o(filePath + "/path.log");
    int unvisitedCount = rmap->getTotalCells();
    bool isGoingHome = false;
    bool setPathToRecharger = false;
    int battery = 0;
    Cell* recharger = rmap->getRecharger();
    Cell* current = recharger;
    Cell* next = nullptr;
    Cell* lastIncToR = nullptr;
    Cell* destination = nullptr;
    vector<Cell*> path;
    Cell*(RobotMap::*unvisitedAdj)(Cell*);

    if (rmap->edges > 600000) {
        unvisitedAdj = &RobotMap::unvisitedAdj_min;
        if (!isRefine)
            cout << endl << "   edges: " << rmap->edges << ", switching to faster algorithm... " << endl;
    } else {
        unvisitedAdj = &RobotMap::unvisitedAdj_v2;
        if (!isRefine)
            cout << endl << "   edges: " << rmap->edges << ", use default algorithm..." << endl;
    }
    
    path.reserve(battery);

    recharger->visited = true;
    --unvisitedCount;
    // int count = 0;
    // string m;
    auto start = now();
    chrono::duration<double> elapsed = now() - start;
    while(unvisitedCount || current != recharger) {
        bool statistic_atRecharger = false;
        bool statistic_doingPath = false;
        bool statistic_roaming = false;
        bool statistic_findUnvClosest = false;
        if (!current->visited) {
            current->visited = true;
            --unvisitedCount;
        }

        // m = string("step ") + to_string(count) + ", "
        //     + "current("+to_string(current->index)+")(" + to_string(rmap->cellCoords[current->index].i) + ", " + to_string(rmap->cellCoords[current->index].j) + "), "
        //     + "unvisited: " + to_string(unvisitedCount) + ", "
        //     + "battery: " + to_string(battery) + ", "
        //     + "distToR: " + to_string(rmap->distanceToRecharger(current->index));
        // o << m;
        // o.flush();

        // 1. determine next movement
        // 1.1 robot is at recharger
        if (current == recharger) {
            statistic_atRecharger = true;
            // o << " -- At Recharger "; o.flush();
            battery = maxBattery;
            isGoingHome = false;
            // o << " -- start if (lastIncToR == nullptr)"; o.flush();
            if (lastIncToR == nullptr) {
                // o << " --randomStart"; o.flush();
                path.clear();
                path.push_back(rmap->randomStart());
            } else {
                // o << " -- findClosestUnvisitedToR with lastIncToR->index:" << lastIncToR->index; o.flush();
                rmap->findClosestUnvisitedToR(path, lastIncToR->index);
                ++cFCUTR;
            }
            // o << " -- At Recharger complete"; o.flush();

        // 1.2 robot is not at recharger
        } else {
            // o << " -- not at recharger"; o.flush();
            // if robot has no goal now, robot is roaming
            if (next == nullptr) {
                // o <<  " -- roaming "; o.flush();

                // all the cells are visited
                if (unvisitedCount == 0)  setPathToRecharger = true;

                // still some cells not visited
                else {
                    statistic_roaming = true;
                    // has unvisited neighbor?
                    next = (rmap->* unvisitedAdj)(current);
                    // if (next) {
                    //     o << " -- unvisitedAdj: (" << rmap->cellCoords[next->index].i << ", " << rmap->cellCoords[next->index].j << ")"; o.flush();
                    // } else {
                    //     o << " -- unvisitedAdj: " << next; o.flush();
                    // }

                    if (next) {
                        // o << " -- has unvisited Adj "; o.flush();

                        setPathToRecharger = !enoughBattery(next->index, battery-1);
                        // o << " -- sptR:" << (setPathToRecharger ? "True" : "False");
                        ++noUnvClosestCount;
                    } else {
                        // o << " -- unvClosest "; o.flush();
                        statistic_findUnvClosest = true;
                        // rmap->findClosestUnvisitedv3(path, current);
                        // setPathToRecharger = !enoughBattery(path.front()->index, battery-path.size()+1);

                        setPathToRecharger = !rmap->findClosestUnvisitedv4(path, current, battery);

                        ++unvClosestCount;
                    }
                }
            } else {
                statistic_doingPath = true;
                setPathToRecharger = !enoughBattery(next->index, battery-1);
            }
        }
        
        if (setPathToRecharger) {
            // o << " -- setPathToRecharger "; o.flush();
            rmap->randomShortestWayHome(path, current);
            next = nullptr;
            isGoingHome = true;
            setPathToRecharger = false;
            lastIncToR = path[1];
        }

        // 2. move !
        // o << " --start move"; o.flush();
        // Cell* dest = path[0];
        // o << ", dest(" << dest->index << "):"; o.flush();
        // o << "(" << rmap->cellCoords[dest->index].i << ", " << rmap->cellCoords[dest->index].j << ") "; o.flush();

        while(!path.empty() &&path.back() == current) {path.pop_back();}
        if (!next) {
            next = path.back();
        }
        
        if (next == recharger) lastIncToR = current;
        current = next;
        if (!path.empty()) path.pop_back();

        // o << " --get next."; o.flush();
        if (path.empty()) next = nullptr;
        else next = path.back();
        
        // record current cell into final path.
        finalPath.push_back(current->index);

        if (finalPath.size() >= 100000) {
            saveTmp();
        }
        
        ++usingSteps;
        --battery;
        // ++count;
        // o << " --move complete" << endl; o.flush();
        elapsed = now() - start;
        if (statistic_atRecharger) {
            sAtRecharger += elapsed.count() * 1000;
        } else if (statistic_roaming) {
            sRoaming += elapsed.count() * 1000;
            if (statistic_findUnvClosest) {
                sGoingNoneAdj += elapsed.count() * 1000;
            } else {
                sGoingAdj += elapsed.count() * 1000;
            }
        } else if (statistic_doingPath) {
            sDoingPath += elapsed.count() * 1000;
        } else {
            sElse += elapsed.count() * 1000;
        }
        start = now();
    }

    if (!isRefine) saveTmp();
    else if (hasTmpFile) saveTmp();
};

void CleaningRobot::refine() {
    resetForRefine();
    clean();
};

void CleaningRobot::outputPath() {
    ifstream i(filePath + "/tmpfile");
    ofstream o(filePath + "/final.path");
    o << usingSteps << endl << i.rdbuf();
};

void CleaningRobot::refineOutput() {
    ofstream o(filePath + "/final.path");
    if (hasTmpFile) {
        ifstream i(filePath + "/tmpfile");
        o << usingSteps << endl << i.rdbuf();
    } else {
        o << usingSteps << endl;
        stringstream ss;
        for (int i = 0 ; i < finalPath.size(); ++i) {
            ss << rmap->cellCoords[finalPath[i]].i << ' ' << rmap->cellCoords[finalPath[i]].j << endl;
        }
        o << ss.rdbuf();
        finalPath.clear();
    }
};

void CleaningRobot::cleanTmpFile() {
    if (hasTmpFile) {
        string fileName = filePath + "/tmpfile";
        remove(fileName.c_str());
        hasTmpFile = false;
    }
};

inline bool CleaningRobot::enoughBattery(Cell* cell, int battery) {
    return battery >= rmap->distanceToRecharger(cell);
};
inline bool CleaningRobot::enoughBattery(int cellIndex, int battery) {
    return battery >= rmap->distanceToRecharger(cellIndex);
};


void CleaningRobot::saveTmp() {
    hasTmpFile = true;
    ofstream o(filePath+"/tmpfile", ios_base::app);
    stringstream ss;
    for (int i = 0 ; i < finalPath.size(); ++i) {
        ss << rmap->cellCoords[finalPath[i]].i << ' ' << rmap->cellCoords[finalPath[i]].j << endl;
    }
    o << ss.rdbuf();
    finalPath.clear();
};

void CleaningRobot::resetForRefine() {
    hasTmpFile = false;
    isRefine = true;
    errorFlag = false;
    usingSteps = 0;
    errorMessage.clear();;

    sAtRecharger = 0;
    sDoingPath = 0;
    sRoaming = 0;
    sGoingAdj = 0;
    sGoingNoneAdj = 0;
    sElse = 0;
    cFCUTR = 0;
    noUnvClosestCount = 0;
    unvClosestCount = 0;
    finalPath.clear();

    rmap->resetForRefine();
};

int CleaningRobot::totalSteps() {
    return usingSteps;
}

void CleaningRobot::test() {


};

void CleaningRobot::analysis() {
    cout << "   ---  sAtRecharger : " << cFCUTR << " counts, "<< (int) sAtRecharger << " ms" << endl;
    cout << "   ---  sDoingPath : " << (int) sDoingPath << " ms" << endl;
    cout << "   ---  sRoaming : " << (int) sRoaming << " ms" << endl;
    cout << "        -- sGoingAdj: " << noUnvClosestCount << " counts, " << (int) sGoingAdj << " ms" << endl;
    cout << "        -- sGoingNoneAdj: " << unvClosestCount << " counts, " << (int) sGoingNoneAdj << " ms, bfs traversed nodes:" << rmap->bfsTraversedNodes << endl;
    cout << "   ---  sElse : " << (int) sElse << " ms" << endl;
    cout << "   ---  sTotal : " << (int) (sAtRecharger+sDoingPath+sRoaming+sElse) << " ms" << endl;
};