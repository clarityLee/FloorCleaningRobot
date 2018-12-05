#include <iostream>
#include <chrono>
#include "CleaningRobot.hpp"
using namespace std;

int getDigit(int n);
int main(int argc, char* argv[]) {
    constexpr auto &&now = std::chrono::high_resolution_clock::now;
    const bool refine = true;
    const short refindRound = 10000;

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
    // robot.analysis();

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

    
    if (refine && limitTime - totalTime > oneTime) {
        cout << "** Robot is now refining the result," << endl 
            << "   the refining process will be limited within 25 sec or " << refindRound << " rounds," << endl
            << "   please wait... refining round : " << count << flush;

        while (limitTime - totalTime > oneTime && count <= refindRound) {
            int prev_digit = getDigit(count-1);
            prev_digit = prev_digit ? prev_digit : 1;
            cout << string(prev_digit, '\b') << count << flush;
            robot.refine();
            if (robot.totalSteps() < minSteps) {
                minSteps = robot.totalSteps();
                robot.refineOutput();
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

inline int getDigit(int n) {
    int digits = 0;
    while (n) {
        n /= 10;
        digits++;
    }
    return digits;
};