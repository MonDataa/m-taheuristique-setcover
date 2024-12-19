#ifndef UTILS_H
#define UTILS_H

#include <vector>
#include <string>
#include <set>

// Déclaration des variables globales
extern std::vector<std::vector<int>> aij; // Matrice binaire aij
extern std::vector<int> weights;         // Poids des sous-ensembles

void readSubsetsFromFile(const std::string& filename, int& m, int& n);
void displayMatrix(const std::vector<std::vector<int>>& matrix);
std::vector<int> constructFeasibleSolution(int m, int n);
bool verifyCoverage(const std::vector<int>& solution, int m, int n);
void addSubset(std::vector<int>& currentSolution, const std::vector<int>& uncoveredElements);
void removeSubset(std::vector<int>& currentSolution);
void exchangeSubsets(std::vector<int>& currentSolution);

#endif // UTILS_H

