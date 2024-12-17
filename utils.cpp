#include "utils.h"
#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <set>
#include <algorithm>
#include <numeric>
#include <cstdlib>

using namespace std;

// D�finition des variables globales
vector<vector<int>> aij;
vector<int> weights;

void readSubsetsFromFile(const string& filename, int& m, int& n) {
    ifstream file(filename);
    if (!file.is_open()) {
        cerr << "Impossible d'ouvrir le fichier." << endl;
        exit(EXIT_FAILURE);
    }

    file >> m >> n; // Lire m (univers) et n (nombre de sous-ensembles)
    cout << "Dimensions trouv�es dans le fichier : " << m << " x " << n << endl;

    // Ajustement dynamique des dimensions
    aij.assign(m, vector<int>(n, 0));
    weights.assign(n, 0);

    set<int> invalidElements; // Pour les �l�ments hors de l'univers

    for (int j = 0; j < n; ++j) {
        int subsetSize;
        file >> subsetSize;

        for (int k = 0; k < subsetSize; ++k) {
            int element;
            file >> element;

            if (element >= 1 && element <= m) {
                aij[element - 1][j] = 1;
            } else {
                invalidElements.insert(element);
            }
        }
        file >> weights[j]; // Poids du sous-ensemble
    }

    file.close();

    if (!invalidElements.empty()) {
        cout << "�l�ments ignor�s hors de l'univers : ";
        for (int elem : invalidElements) cout << elem << " ";
        cout << endl;
    }
}

void displayMatrix(const vector<vector<int>>& matrix) {
    cout << "Affichage de la matrice aij : " << endl;
    for (const auto& row : matrix) {
        for (int val : row) cout << val << " ";
        cout << endl;
    }
    cout << "Dimensions : " << matrix.size() << " x " << matrix[0].size() << endl;
}

// Fonction pour construire une solution r�alisable
vector<int> constructFeasibleSolution(int m, int n) {
    vector<int> covered(m, 0);
    vector<int> solution;
    set<int> uncovered;

    for (int i = 0; i < m; ++i) {
        uncovered.insert(i);
    }

    while (!uncovered.empty()) {
        int bestSubset = -1;
        double bestScore = -1.0;  // Score le plus �lev� (nombre d'�l�ments couverts / poids)

        // Trouver le meilleur sous-ensemble
        for (int j = 0; j < n; ++j) {
            int coveredCount = 0;

            for (int i : uncovered) {
                if (aij[i][j] == 1) {
                    ++coveredCount;
                }
            }

            if (coveredCount > 0) {
                double score = static_cast<double>(coveredCount) / weights[j];
                if (score > bestScore) {
                    bestScore = score;
                    bestSubset = j;
                }
            }
        }

        // V�rifier si un sous-ensemble a �t� trouv�
        if (bestSubset == -1) {
            cerr << "Erreur : Impossible de couvrir tous les �l�ments." << endl;
            exit(EXIT_FAILURE);
        }

        // Ajouter le meilleur sous-ensemble � la solution
        solution.push_back(bestSubset);

        // Mettre � jour les �l�ments couverts
        for (int i = 0; i < m; ++i) {
            if (aij[i][bestSubset] == 1) {
                uncovered.erase(i);
            }
        }
    }

    return solution;
}

// V�rification de la couverture de tous les �l�ments
bool verifyCoverage(const vector<int>& solution, int m, int n) {
    vector<int> covered(m, 0);

    for (int subset : solution) {
        for (int i = 0; i < m; ++i) {
            if (aij[i][subset] == 1) {
                covered[i] = 1;
            }
        }
    }

    for (int i = 0; i < m; ++i) {
        if (covered[i] == 0) {
            cerr << "Erreur : L'�l�ment " << i + 1 << " n'est pas couvert." << endl;
            return false;
        }
    }

    cout << "Tous les �l�ments de l'univers sont couverts." << endl;
    return true;
}

void addSubset(vector<int>& currentSolution, const vector<int>& uncoveredElements) {
    int bestSubset = -1;
    int maxCoverage = 0;

    // Rechercher le sous-ensemble qui couvre le maximum d'�l�ments non couverts
    for (int j = 0; j < aij[0].size(); ++j) {
        int coverage = 0;
        for (int i : uncoveredElements) {
            if (aij[i][j] == 1) {
                coverage++;
            }
        }

        // Trouver le sous-ensemble avec la couverture maximale
        if (coverage > maxCoverage) {
            maxCoverage = coverage;
            bestSubset = j;
        }
    }

    // Ajouter le meilleur sous-ensemble
    if (bestSubset != -1) {
        currentSolution.push_back(bestSubset);
        cout << "Sous-ensemble ajout� : " << bestSubset << endl;
    }
}


void removeSubset(vector<int>& currentSolution) {
    for (auto it = currentSolution.begin(); it != currentSolution.end(); ++it) {
        int subsetToRemove = *it;

        // V�rifier si tous les �l�ments restent couverts apr�s le retrait
        bool feasible = true;
        for (int i = 0; i < aij.size(); ++i) {
            bool stillCovered = false;
            for (int j : currentSolution) {
                if (j != subsetToRemove && aij[i][j] == 1) {
                    stillCovered = true;
                    break;
                }
            }
            if (!stillCovered) {
                feasible = false;
                break;
            }
        }

        // Retirer le sous-ensemble si faisable
        if (feasible) {
            currentSolution.erase(it);
            cout << "Sous-ensemble retir� : " << subsetToRemove << endl;
            return;
        }
    }

    cout << "Aucun sous-ensemble ne peut �tre retir� sans rendre la solution non faisable." << endl;
}


void exchangeSubsets(vector<int>& currentSolution) {
    for (int j = 0; j < aij[0].size(); ++j) {
        // V�rifier si le sous-ensemble est d�j� dans la solution
        if (find(currentSolution.begin(), currentSolution.end(), j) != currentSolution.end()) {
            continue;
        }

        for (auto it = currentSolution.begin(); it != currentSolution.end(); ++it) {
            int subsetToReplace = *it;

            // V�rifier si l'�change am�liore la couverture ou le poids
            bool validExchange = true;
            for (int i = 0; i < aij.size(); ++i) {
                if (aij[i][subsetToReplace] == 1 && aij[i][j] == 0) {
                    validExchange = false;
                    break;
                }
            }

            // �changer si valide
            if (validExchange) {
                *it = j;
                cout << "Sous-ensemble �chang� : " << subsetToReplace << " remplac� par " << j << endl;
                return;
            }
        }
    }

    cout << "Aucun �change valide n'a �t� trouv�." << endl;
}



