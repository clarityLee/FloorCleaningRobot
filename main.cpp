#include <iostream>
#include <chrono>
#include "CleaningRobot.hpp"
using namespace std;

int main(int argc, char* argv[]) {
    constexpr auto &&now = std::chrono::high_resolution_clock::now;

    auto programStartTime = now();
    int oneTime = 0, totalTime = 0, count = 1;
    int minSteps;
    const int limitTime = 25000;

    CleaningRobot robot;
    cout << "** Robot is starting to do CleaningRobot::readFloorData() ...." << endl;
    auto phaseStartTime = now();
    robot.readFloorData(argc, argv);
    if (robot.hasError()) {
        robot.printError();
        return 0;
    }
    chrono::duration<double> elapsed = now() - phaseStartTime;
    cout << "    CleaningRobot::readFloorData() completed in : " 
        << (int) (elapsed.count() * 1000) << " ms." << endl;
    

    cout << "** Robot is cleaning floor now (generating path) ....";
    phaseStartTime = now();
    robot.clean();
    elapsed = now() - phaseStartTime;
    cout << "   completed in : " << (int) (elapsed.count() * 1000) << " ms." << endl;
    robot.analysis();

    cout << "** Robot is saving file.....";
    phaseStartTime = now();
    robot.outputPath();
    robot.cleanTmpFile();
    elapsed = now() - phaseStartTime;
    cout << "completed in : " << (int) (elapsed.count() * 1000) << " ms." << endl;

    elapsed = now() - programStartTime;
    oneTime = totalTime = (int) (elapsed.count() * 1000);
    minSteps = robot.totalSteps();
    cout << "** Total used time : " << totalTime << " ms, total steps: " << minSteps << endl;
    cout << endl;

    
    if (limitTime - totalTime > oneTime) {
        cout << "** Robot is now refining the result," << endl 
            << "   the refining process will be limited within 25 sec or 100 rounds," << endl
            << "   please wait";

        while (limitTime - totalTime > oneTime && count <= 100) {
            cout << ".";
            robot.refine();
            if (robot.totalSteps() < minSteps) {
                minSteps = robot.totalSteps();
                robot.outputPath();
            }
            robot.cleanTmpFile();
            ++count;
            elapsed = now() - programStartTime;
            totalTime = (int) (elapsed.count() * 1000);
        }
        cout << endl << "Complete ! result is output with best steps: " << minSteps << endl;
    }
    
    return 0;
};