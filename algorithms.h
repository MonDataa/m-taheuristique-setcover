#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include <vector>
#include <set>
#include <unordered_set>
#include <string>
#include <random>

using namespace std;

// Structure pour ALNS
struct Method {
    string name;
    double score;
};

// Vérification si une solution est faisable
bool isFeasibleSolution(const vector<int>& solution, int m);

// Calcul de la couverture des éléments
vector<int> calculateCoverage(const vector<int>& solution, int m);

// Calcul du poids total d'une solution
int calculateWeight(const vector<int>& solution);

// Algorithme Greedy
vector<int> greedySolution(int m, int n);

// Recherche Tabou
vector<int> tabuSearch(int m, int n, int maxIterations, int tabuTenure, int maxNoImprovement);

// Recherche Tabou avec méta-learning
vector<int> diversifySolution(const vector<int>& solution, int destroySize, int m, int n, mt19937& rng);
vector<int> tabuSearchMetaLearning(int m, int n, int maxIterations, int tabuTenure, int maxNoImprovement);

// LNS et ALNS
vector<int> reconstructSolution(const vector<int>& partialSolution, int m, int n);
vector<int> largeNeighborhoodSearch(int m, int n, const vector<int>& initialSolution, int maxIterations, int destroySize);
int selectMethod(const vector<Method>& methods, mt19937& rng);
void updateScores(vector<Method>& destructionMethods, vector<Method>& reconstructionMethods, int dIdx, int rIdx, double alpha, bool improved);
vector<int> adaptiveLargeNeighborhoodSearch(int m, int n, const vector<int>& initialSolution, int maxIterations, int destroySize, double alpha, int maxNoImprovement);

// Variable Neighborhood Search (VNS)
vector<int> localSearchVNS(const vector<int>& solution, int m, int n, int localSearchAttempts);
vector<int> shakingVNS(const vector<int>& solution, int k, int m, int n);
vector<int> vnsSearchCustom(int m, int n, const vector<int>& initialSolution, int maxIterations, int kMax, int localSearchAttempts);

#endif // ALGORITHMS_H
