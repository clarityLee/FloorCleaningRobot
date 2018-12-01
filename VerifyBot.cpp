#include <iostream>
#include <string>
#include <fstream>
#include <vector>
using namespace std;

class Coordinate {
public:
    Coordinate() {};
    Coordinate(short _i, short _j) : i(_i), j(_j) {};
    short i, j;
};

template <class T>
class Array2D {
public:
    Array2D();
    Array2D(int _rows, int _columns);
    Array2D(int _rows, int _columns, T initValue);
    ~Array2D();
    inline void init(int _rows, int _columns);
    inline void init(T initValue);
    inline void init(int _rows, int _columns, T initValue);
    inline T get(int i, int j);
    inline void set(int i , int j, T value);
    
private:
    bool initialized = false;
    int rows = 0, columns = 0, size = 0;
    T* arr = 0;
};

class VerifyBot {
public:
    void readFloorData(int argc, char* argv[]);
    void readFinalPath(char* argv[]);
    void verify(char* argv[]);
private:
    bool hasError = false;
    string errorMessage;
    short ri = 0, rj = 0,
        rows = 0, columns = 0;
    int maxBattery = 0, steps;
    Array2D<char> rawData;
    vector<Coordinate> path;
};

int main(int argc, char* argv[]) {
    VerifyBot bot;
    bot.readFloorData(argc, argv);
    bot.readFinalPath(argv);
    bot.verify(argv);
};

void VerifyBot::readFloorData(int argc, char* argv[]) {
    if (argc < 2) {
        cout << "Missing command argument. Please specify the path to floor.data." << endl;
        hasError = true;
        return;
    }

    ifstream readInFile(string(argv[1]) + "/floor.data");
    if (!readInFile.is_open()) {
        cout << "Unable to open \"" << string(argv[1])
            << "/floor.data\". Either file does not exist or wrong path." << endl;
        hasError = true;
        return;
    }

    readInFile >> rows >> columns >> maxBattery;
    rawData.init(rows, columns);
    char c;
    for (short i = 0 ; i < rows ; ++i) {
        for (short j = 0 ; j < columns ; ++j) {
            readInFile >> c;
            rawData.set(i, j, c);
            if (c == 'R') {
                ri = i;
                rj = j;
            }
        }
    }
    cout << "VerifyBot : complete reading : \"" << string(argv[1]) << "/floor.data\"." << endl;
};

void VerifyBot::readFinalPath(char* argv[]) {
    if (hasError) return;
    ifstream readInFile(string(argv[1]) + "/final.path");
    if (!readInFile.is_open()) {
        cout << "Unable to open \"" << string(argv[1])
            << "/final.path\". Either file does not exist or wrong path." << endl;
        hasError = true;
        return;
    }

    readInFile >> steps;
    path.reserve(steps);

    short i = 0, j = 0;
    for (int k = 0 ; k < steps ; ++k) {
        readInFile >> i >> j;
        path.emplace_back(i, j);
    }

    if (steps != path.size()) {
        cout << "Error: steps size in final.path is: " << steps
            << ", but final.path has " << path.size() << " coordinates." << endl;
        hasError = true;
        return;
    }
    cout << "VerifyBot : complete reading : \"" << string(argv[1]) << "/final.path\"." << endl;
};

void VerifyBot::verify(char* argv[]) {
    if (hasError) return;

    ofstream o(string(argv[1]) + "/verify.log");

    int battery = 0;
    short i = ri, j = rj;
    Array2D<bool> visited(rows, columns, false);
    bool hasError = true;
    string m;
    for (int k = 0 ; k <= steps ; ++k) {
        if (i == ri && j == rj) {
            battery = maxBattery;
        }
        visited.set(i, j, true);
        hasError = true;

        m = string("-- ")
            + "steps: " + to_string(k)
            + ", current(" + to_string(i) + ", " + to_string(j) + ")"
            + ", battery remain: " + to_string(battery);
        o << m << endl;

        if (k == steps) {
            hasError = false;
            break;
        }

        if (battery < 0) {
            o << "   Error: not enough batery." << endl;
            break;
        }

        Coordinate next = path[k];
        m = "   next(" + to_string(next.i) + ", " + to_string(next.j) + ")";
        if (next.i < 0 || next.i >= rows
            || next.j < 0 || next.j >= columns) {
            hasError = true;
            o << "   Error: Coordinate of next step (" << next.i << ", " << next.j << ") is out of bound." << endl;
            break;
        }
        if ((next.i == i && (next.j == j-1 || next.j == j+1)) ||
            (next.j == j && (next.i == i-1 || next.i == i+1))) {
            // is contiguous
        } else {
            hasError = true;
            o << "   Error: Coordinate of next step (" << next.i << ", " << next.j << ") is not adjacent to current position." << endl;
            break;
        }
        if (rawData.get(i, j) != '0' && rawData.get(i, j) != 'R') {
            hasError = true;
            o << "   Error: Coordinate of next step (" << next.i << ", " << next.j << ") is not passable." << endl;
            break;
        }

        if (i == ri && j == rj && k-2 >= 0
            && (path[k-2].i != next.i || path[k-2].j != next.j)) {
            o << "   Error: Current is at recharger, but next step  (" << next.i << ", " << next.j << ")  is going out via illeage direction." << endl;
            o << "          You shall not pass recharger!!" << endl;
            break;
        }

        // no problem , move to next step
        i = next.i;
        j = next.j;
        --battery;

        hasError = false;
    }

    int unvisited = 0;
    for (short i = 0 ; i < rows; ++i) {
        for (short j = 0 ; j < columns; ++j) {
            if (rawData.get(i, j) == '1') continue;
            if (!visited.get(i, j)) ++unvisited;
        }
    }
    if (unvisited) {
        o << "   Error: There are " << unvisited << " unvisited cell by the path." << endl;
        hasError = true;
    }

    if (hasError) {
        cout << "NO ~~ !! Steps verification failed. There are errors. Please see verify.log" << endl;
    } else {
        cout << "OK!! Good !! Steps verification complete. final.path is legal." << endl;
    }

};


template <class T>
Array2D<T>::Array2D() {};

template <class T>
Array2D<T>::Array2D(int _rows, int _columns)
        :rows(_rows) , columns(_columns), size(_rows*_columns) {
    if (_rows < 1 || _columns < 1) {
        throw runtime_error("rows and colums must be greater than 1.");
    }
    arr = new T[size];
    initialized = true;
};

template <class T>
Array2D<T>::Array2D(int _rows, int _columns, T initValue)
        :rows(_rows) , columns(_columns), size(_rows*_columns) {
    if (_rows < 1 || _columns < 1) {
        throw runtime_error("rows and colums must be greater than 1.");
    }
    arr = new T[size];
    for (int i = 0 ; i < size ; ++i) {
        arr[i] = initValue;
    }
    initialized = true;
};

template <class T>
Array2D<T>::~Array2D() {
    if (arr) delete [] arr;
};

template <class T>
inline void Array2D<T>::init(int _rows, int _columns) {
    if (initialized) return;
    if (_rows < 1 || _columns < 1) {
        throw runtime_error("rows and colums must be greater than 1.");
    }
    rows = _rows;
    columns = _columns;
    size = rows*columns;
    arr = new T[size];
    initialized = true;
};

template <class T>
inline void Array2D<T>::init(T initValue) {
    for (int i = 0 ; i < size ; ++i) {
        arr[i] = initValue;
    }
};

template <class T>
inline void Array2D<T>::init(int _rows, int _columns, T initValue) {
    init(_rows, _columns);
    init(initValue);
};

template <class T>
inline T Array2D<T>::get(int i, int j) {
    if (!initialized) {
        throw runtime_error("Array2D has not been initialized.");
    }
    return arr[i*columns+j];
};

template <class T>
inline void Array2D<T>::set(int i , int j, T value) {
    if (!initialized) {
        throw runtime_error("Array2D has not been initialized.");
    }
    arr[i*columns+j] = value;
}