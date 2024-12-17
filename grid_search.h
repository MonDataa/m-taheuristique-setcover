#ifndef GRID_SEARCH_H
#define GRID_SEARCH_H

#include <vector>
#include <string>

// Déclarations des fonctions de grid search
void gridSearchTabuSimple(int m, int n, const std::vector<int>& initialSolution);
void gridSearchTabuMetaLearning(int m, int n, const std::vector<int>& initialSolution);
void gridSearchLNS(int m, int n, const std::vector<int>& initialSolution);
void gridSearchALNS(int m, int n, const std::vector<int>& initialSolution);
void gridSearchVNS(int m, int n, const std::vector<int>& initialSolution);

#endif // GRID_SEARCH_H

