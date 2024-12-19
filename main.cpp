#include <iostream>
#include <vector>
#include <limits>
#include <string>
#include <chrono>
#include "utils.h"
#include "algorithms.h"
#include "grid_search.h"

using namespace std;

class MetaheuristicSolver {
public:
    void run() {
        int m, n;
        string filename;
        //string filename = "";

        cout << "Enter the path to the input file (e.g., scp.txt, scp510.txt): ";
        cin >> filename;

        readSubsetsFromFile(filename, m, n);

        // Create an initial feasible solution
        vector<int> initialSolution = constructInitialFeasibleSolution(m, n);
        if (initialSolution.empty()) {
            cerr << "Initial solution could not be constructed." << endl;
            return;
        }

        cout << "=== Initial Solution ===" << endl;
        displayResults(initialSolution, m);
        cout << "=========================" << endl;

        // General parameters
        int maxIterations = 1000;
        int destroySize = 5;
        double alpha = 1.0;
        int maxNoImprovement = 50;
        int tabuTenure = 10;

        bool continueExecution = true;

        while (continueExecution) {
            displayMenu();
            int choice = getChoice();

            if (choice == 9) {
                cout << "Exiting the program." << endl;
                break;
            }

            cout << "Do you want to run a grid search for this model? (y/n): ";
            char gridSearchResponse;
            cin >> gridSearchResponse;
            bool runGridSearch = (gridSearchResponse == 'y' || gridSearchResponse == 'Y');

            vector<int> bestSolution;

            if (runGridSearch) {
                performGridSearch(choice, m, n, initialSolution);
            } else {
                bestSolution = executeModel(choice, m, n, initialSolution, maxIterations, destroySize, alpha, maxNoImprovement, tabuTenure);
                displayResults(bestSolution, m);
            }

            cout << "Do you want to test another model? (y/n): ";
            char repeatResponse;
            cin >> repeatResponse;
            continueExecution = (repeatResponse == 'y' || repeatResponse == 'Y');
        }
    }

private:
    vector<int> constructInitialFeasibleSolution(int m, int n) {
        cout << "Constructing an initial feasible solution..." << endl;
        vector<int> solution = constructFeasibleSolution(m, n);

        if (isFeasibleSolution(solution, m)) {
            cout << "Initial solution is feasible." << endl;
        } else {
            cerr << "Initial solution is not feasible. Exiting..." << endl;
            solution.clear();
        }

        return solution;
    }

    void displayMenu() {
        cout << "Choose the model to execute:" << endl;
        cout << "0 = Simple Tabu Search" << endl;
        cout << "1 = Tabu Search with Meta-learning" << endl;
        cout << "2 = Large Neighborhood Search (LNS)" << endl;
        cout << "3 = Adaptive Large Neighborhood Search (ALNS)" << endl;
        cout << "4 = Variable Neighborhood Search (VNS)" << endl;
        cout << "5 = Greedy Algorithm" << endl;
        cout << "6 = Grasp Algorithm" << endl;
        cout << "7 = Grasp + VNS" << endl;
        cout << "8 = VNS with Meta-learning" << endl;
        cout << "9 = Exit" << endl;
        cout << "Enter your choice: ";
    }

    int getChoice() {
        int choice;
        cin >> choice;

        if (cin.fail()) {
            cin.clear();
            cin.ignore(numeric_limits<streamsize>::max(), '\n');
            cerr << "Invalid input. Please try again." << endl;
            return -1;
        }
        return choice;
    }

    void performGridSearch(int choice, int m, int n, const vector<int>& initialSolution) {
        switch (choice) {
            case 0:
                cout << "Grid Search for Simple Tabu Search..." << endl;
                gridSearchTabuSimple(m, n, initialSolution);
                break;
            case 1:
                cout << "Grid Search for Tabu Search with Meta-learning..." << endl;
                gridSearchTabuMetaLearning(m, n, initialSolution);
                break;
            case 2:
                cout << "Grid Search for LNS..." << endl;
                gridSearchLNS(m, n, initialSolution);
                break;
            case 3:
                cout << "Grid Search for ALNS..." << endl;
                gridSearchALNS(m, n, initialSolution);
                break;
            case 4:
                cout << "Grid Search for VNS..." << endl;
                gridSearchVNS(m, n, initialSolution);
                break;
            case 5:
                cerr << "Grid Search for Greedy not implemented." << endl;
                break;
            case 6:
                cerr << "Grid Search for Grasp not implemented." << endl;
                gridSearchGRASP(m, n);
                break;
            case 7:
                cerr << "Grid Search for Grasp+VNS not implemented." << endl;
                gridSearchGRASP_VNS(m, n);
                break;
            case 8:
                cerr << "Grid Search for Meta+VNS not implemented." << endl;
                gridSearchVNSMetaLearning(m, n, initialSolution);
                break;
            default:
                cerr << "Invalid choice. Please try again." << endl;
                break;
        }
    }

    vector<int> executeModel(int choice, int m, int n, const vector<int>& initialSolution,
                             int maxIterations, int destroySize, double alpha, int maxNoImprovement, int tabuTenure) {
        vector<int> solution;
        auto start = chrono::high_resolution_clock::now(); // Start timing

        switch (choice) {
            case 0:
                cout << "Running Simple Tabu Search..." << endl;
                solution = tabuSearch(m, n, maxIterations, tabuTenure, maxNoImprovement);
                break;
            case 1:
                cout << "Running Tabu Search with Meta-learning..." << endl;
                solution = tabuSearchMetaLearning(m, n, maxIterations, tabuTenure, maxNoImprovement);
                break;
            case 2:
                cout << "Running Large Neighborhood Search (LNS)..." << endl;
                solution = largeNeighborhoodSearch(m, n, initialSolution, maxIterations, destroySize);
                break;
            case 3:
                cout << "Running Adaptive Large Neighborhood Search (ALNS)..." << endl;
                solution = adaptiveLargeNeighborhoodSearch(m, n, initialSolution, maxIterations, destroySize, alpha, maxNoImprovement);
                break;
            case 4:
                cout << "Running Variable Neighborhood Search (VNS)..." << endl;
                solution = vnsSearchCustom(m, n, initialSolution, maxIterations, 3, 50);
                break;
            case 5:
                cout << "Running Greedy Algorithm..." << endl;
                solution = greedySolution(m, n);
                break;
            case 6:
                cout << "Running Grasp Algorithm..." << endl;
                solution = GRASP(m, n, maxIterations, alpha);
                break;
            case 7:
                cout << "Running Hybrid VNS with GRASP..." << endl;
                solution = hybridVNSWithGRASP(m, n, maxIterations, 3, 50, 10, alpha); // kMax=3, localSearchAttempts=50, graspIterations=10
                break;
            case 8:
                cout << "Running Hybrid VNS with Meta..." << endl;
                solution = vnsSearchMetaLearning(m, n, initialSolution, maxIterations, 5, 50);
                break;
            default:
                cerr << "Invalid choice. Please try again." << endl;
                break;
        }

        auto end = chrono::high_resolution_clock::now(); // End timing
        chrono::duration<double> elapsed = end - start;
        cout << "Execution time: " << elapsed.count() << " seconds" << endl;

        return solution;
    }

    void displayResults(const vector<int>& solution, int m) {
        if (!solution.empty()) {
            cout << "Solution found: ";
            for (int s : solution) {
                cout << s << " ";
            }
            cout << endl;
            cout << "Total Weight: " << calculateWeight(solution) << endl;
            cout << "Number of Subsets: " << solution.size() << endl;

            if (isFeasibleSolution(solution, m)) {
                cout << "The solution is feasible." << endl;
            } else {
                cout << "The solution is not feasible." << endl;
            }
        }
    }
};

int main() {
    MetaheuristicSolver solver;
    solver.run();
    return 0;
}
