#include <iostream>
#include <fstream>
#include <vector>
#include <ctime>
#include "manager.h"
using namespace std;

int main(int argc, char** argv)
{
    fstream input, output;
    if (argc == 3) {
        input.open(argv[1], ios::in);
        output.open(argv[2], ios::out);
        if (!input) {
            cerr << "Cannot open the input file \"" << argv[1]
                 << "\". The program will be terminated..." << endl;
            exit(1);
        }
        if (!output) {
            cerr << "Cannot open the output file \"" << argv[2]
                 << "\". The program will be terminated..." << endl;
            exit(1);
        }
    }
    else {
        cerr << "Usage: ./interchangeManager <input file> <output file>" << endl;
        exit(1);
    }

    Manager* man = new Manager();
    man->parseInput(input);
    man->plan();
    man->writeOutput(output);
    cout << "Finish!\n";
    return 0;
}
