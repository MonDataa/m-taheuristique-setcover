#include "grid_search.h"
#include "algorithms.h"
#include <iostream>
#include <vector>
#include <limits>
#include <string>
#include <chrono>

using namespace std;


void gridSearchTabuSimple(int m, int n, const vector<int>& initialSolution) {
    // Grilles de valeurs à tester
    vector<int> iterationsGrid = {500, 1000, 2000};
    vector<int> tabuTenureGrid = {5, 10, 20};
    vector<int> maxNoImprovementGrid = {20, 50, 100};

    int bestWeight = numeric_limits<int>::max();
    vector<int> bestSolution;
    int bestIterations = 0, bestTenure = 0, bestNoImp = 0;

    for (int maxIt : iterationsGrid) {
        for (int tenure : tabuTenureGrid) {
            for (int noImp : maxNoImprovementGrid) {
                cout << "Test Tabu Simple: maxIt=" << maxIt
                     << ", tabuTenure=" << tenure
                     << ", maxNoImp=" << noImp << endl;

                vector<int> sol = tabuSearch(m, n, maxIt, tenure, noImp);
                int w = calculateWeight(sol);
                if (isFeasibleSolution(sol, m) && w < bestWeight) {
                    bestWeight = w;
                    bestSolution = sol;
                    bestIterations = maxIt;
                    bestTenure = tenure;
                    bestNoImp = noImp;
                }
            }
        }
    }

    cout << "Meilleurs paramètres (Tabu Simple): "
         << "maxIterations=" << bestIterations
         << ", tabuTenure=" << bestTenure
         << ", maxNoImprovement=" << bestNoImp
         << ", Poids=" << bestWeight << endl;
}

void gridSearchTabuMetaLearning(int m, int n, const vector<int>& initialSolution) {
    // Grilles de valeurs à tester
    vector<int> iterationsGrid = {500, 1000, 2000};
    vector<int> tabuTenureGrid = {5, 10, 20};
    vector<int> maxNoImprovementGrid = {20, 50, 100};

    // Paramètres méta-learning additionnels
    vector<int> diversificationFrequencyGrid = {10, 25, 50};
    vector<int> perturbationSizeGrid = {3, 5, 10};

    int bestWeight = numeric_limits<int>::max();
    vector<int> bestSolution;
    int bestIterations = 0, bestTenure = 0, bestNoImp = 0;
    int bestDivFreq = 0, bestPertSize = 0;

    for (int maxIt : iterationsGrid) {
        for (int tenure : tabuTenureGrid) {
            for (int noImp : maxNoImprovementGrid) {
                for (int divFreq : diversificationFrequencyGrid) {
                    for (int pertSize : perturbationSizeGrid) {
                        cout << "Test Tabu Méta-learning: maxIt=" << maxIt
                             << ", tabuTenure=" << tenure
                             << ", maxNoImp=" << noImp
                             << ", divFreq=" << divFreq
                             << ", pertSize=" << pertSize << endl;

                        vector<int> sol = tabuSearchMetaLearning(m, n, maxIt, tenure, noImp /* , divFreq, pertSize ... si nécessaire */);
                        int w = calculateWeight(sol);
                        if (isFeasibleSolution(sol, m) && w < bestWeight) {
                            bestWeight = w;
                            bestSolution = sol;
                            bestIterations = maxIt;
                            bestTenure = tenure;
                            bestNoImp = noImp;
                            bestDivFreq = divFreq;
                            bestPertSize = pertSize;
                        }
                    }
                }
            }
        }
    }

    cout << "Meilleurs paramètres (Tabu Méta-learning): "
         << "maxIterations=" << bestIterations
         << ", tabuTenure=" << bestTenure
         << ", maxNoImprovement=" << bestNoImp
         << ", diversificationFrequency=" << bestDivFreq
         << ", perturbationSize=" << bestPertSize
         << ", Poids=" << bestWeight << endl;
}

void gridSearchLNS(int m, int n, const vector<int>& initialSolution) {
    // Listes de valeurs à tester
    vector<int> maxIterationsList = {500, 1000};
    vector<int> destroySizeList = {5, 10, 20};

    int bestWeight = std::numeric_limits<int>::max();
    vector<int> bestSolution;

    cout << "Début du Grid Search LNS..." << endl;

    for (int maxIter : maxIterationsList) {
        for (int dSize : destroySizeList) {
            cout << "Test LNS avec maxIterations=" << maxIter << ", destroySize=" << dSize << endl;
            vector<int> solution = largeNeighborhoodSearch(m, n, initialSolution, maxIter, dSize);

            if (isFeasibleSolution(solution, m)) {
                int w = calculateWeight(solution);
                cout << "Solution faisable trouvée, poids=" << w << ", taille=" << solution.size() << endl;
                if (w < bestWeight) {
                    bestWeight = w;
                    bestSolution = solution;
                }
            } else {
                cout << "Solution non faisable." << endl;
            }
        }
    }

    cout << "Fin du Grid Search LNS." << endl;
    if (!bestSolution.empty()) {
        cout << "Meilleure solution trouvée : poids=" << bestWeight << ", taille=" << bestSolution.size() << endl;
    } else {
        cout << "Aucune solution faisable n'a été trouvée." << endl;
    }
}

void gridSearchALNS(int m, int n, const vector<int>& initialSolution) {
    vector<int> maxIterationsList = {500, 1000};
    vector<int> destroySizeList = {5, 10};
    vector<double> alphaList = {0.5, 1.0, 2.0};
    vector<int> maxNoImprovementList = {50, 100};

    int bestWeight = std::numeric_limits<int>::max();
    vector<int> bestSolution;

    cout << "Début du Grid Search ALNS..." << endl;

    for (int maxIter : maxIterationsList) {
        for (int dSize : destroySizeList) {
            for (double a : alphaList) {
                for (int maxNoImp : maxNoImprovementList) {
                    cout << "Test ALNS avec maxIterations=" << maxIter
                         << ", destroySize=" << dSize
                         << ", alpha=" << a
                         << ", maxNoImprovement=" << maxNoImp << endl;

                    vector<int> solution = adaptiveLargeNeighborhoodSearch(m, n, initialSolution, maxIter, dSize, a, maxNoImp);

                    if (isFeasibleSolution(solution, m)) {
                        int w = calculateWeight(solution);
                        cout << "Solution faisable trouvée, poids=" << w << ", taille=" << solution.size() << endl;
                        if (w < bestWeight) {
                            bestWeight = w;
                            bestSolution = solution;
                        }
                    } else {
                        cout << "Solution non faisable." << endl;
                    }
                }
            }
        }
    }

    cout << "Fin du Grid Search ALNS." << endl;
    if (!bestSolution.empty()) {
        cout << "Meilleure solution trouvée : poids=" << bestWeight << ", taille=" << bestSolution.size() << endl;
    } else {
        cout << "Aucune solution faisable n'a été trouvée." << endl;
    }
}

void gridSearchVNS(int m, int n, const vector<int>& initialSolution) {
    vector<int> maxIterationsList = {500, 1000};
    vector<int> kMaxList = {2, 3, 5};
    vector<int> localSearchAttemptsList = {10, 50};

    int bestWeight = std::numeric_limits<int>::max();
    vector<int> bestSolution;

    cout << "Début du Grid Search VNS..." << endl;

    for (int maxIter : maxIterationsList) {
        for (int kMax : kMaxList) {
            for (int localSearchAttempts : localSearchAttemptsList) {
                cout << "Test VNS avec maxIterations=" << maxIter
                     << ", kMax=" << kMax
                     << ", localSearchAttempts=" << localSearchAttempts << endl;

                // Modification du comportement de VNS pour accepter localSearchAttempts
                vector<int> solution = vnsSearchCustom(m, n, initialSolution, maxIter, kMax, localSearchAttempts);

                if (isFeasibleSolution(solution, m)) {
                    int w = calculateWeight(solution);
                    cout << "Solution faisable trouvée, poids=" << w << ", taille=" << solution.size() << endl;
                    if (w < bestWeight) {
                        bestWeight = w;
                        bestSolution = solution;
                    }
                } else {
                    cout << "Solution non faisable." << endl;
                }
            }
        }
    }

    cout << "Fin du Grid Search VNS." << endl;
    if (!bestSolution.empty()) {
        cout << "Meilleure solution trouvée : poids=" << bestWeight << ", taille=" << bestSolution.size() << endl;
    } else {
        cout << "Aucune solution faisable n'a été trouvée." << endl;
    }
}

void gridSearchGRASP(int m, int n) {
    vector<double> alphaValues = {0.1, 0.3, 0.5, 0.7, 0.9}; // Différents niveaux de randomisation
    vector<int> iterationsValues = {50, 100, 200};           // Nombre d'itérations GRASP

    int bestWeight = numeric_limits<int>::max();
    vector<int> bestSolution;
    double bestAlpha = 0.0;
    int bestIterations = 0;

    cout << "Démarrage de Grid Search pour GRASP..." << endl;

    for (double alpha : alphaValues) {
        for (int maxIterations : iterationsValues) {
            cout << "Test avec alpha=" << alpha << ", maxIterations=" << maxIterations << endl;

            auto start = chrono::high_resolution_clock::now();
            vector<int> solution = GRASP(m, n, maxIterations, alpha);
            auto end = chrono::high_resolution_clock::now();
            int weight = calculateWeight(solution);
            chrono::duration<double> duration = end - start;

            cout << "Poids obtenu : " << weight << ", Temps : " << duration.count() << "s" << endl;

            if (weight < bestWeight) {
                bestWeight = weight;
                bestSolution = solution;
                bestAlpha = alpha;
                bestIterations = maxIterations;
            }
        }
    }

    cout << "Meilleur résultat trouvé : " << endl;
    cout << "Alpha = " << bestAlpha << ", maxIterations = " << bestIterations << ", Poids = " << bestWeight << endl;
    cout << "Solution : ";
    for (int s : bestSolution) cout << s << " ";
    cout << endl;
}


void gridSearchGRASP_VNS(int m, int n) {
    vector<double> alphaValues = {0.1, 0.3, 0.5, 0.7};          // Niveau de randomisation
    vector<int> graspIterationsValues = {50, 100, 200};         // Nombre d'itérations pour GRASP
    vector<int> vnsIterationsValues = {100, 200, 500};          // Nombre d'itérations pour VNS
    vector<int> kMaxValues = {2, 3, 5};                         // Taille maximale des voisinages
    vector<int> localSearchAttemptsValues = {10, 20, 50};       // Nombre d'essais pour la recherche locale

    int bestWeight = numeric_limits<int>::max();
    vector<int> bestSolution;
    double bestAlpha = 0.0;
    int bestGraspIterations = 0;
    int bestVnsIterations = 0;
    int bestKMax = 0;
    int bestLocalSearchAttempts = 0;

    cout << "Démarrage de Grid Search pour GRASP+VNS..." << endl;

    for (double alpha : alphaValues) {
        for (int graspIterations : graspIterationsValues) {
            for (int vnsIterations : vnsIterationsValues) {
                for (int kMax : kMaxValues) {
                    for (int localSearchAttempts : localSearchAttemptsValues) {
                        cout << "Test avec alpha=" << alpha
                             << ", graspIterations=" << graspIterations
                             << ", vnsIterations=" << vnsIterations
                             << ", kMax=" << kMax
                             << ", localSearchAttempts=" << localSearchAttempts << endl;

                        auto start = chrono::high_resolution_clock::now();
                        vector<int> solution = hybridVNSWithGRASP(m, n, vnsIterations, kMax, localSearchAttempts, graspIterations, alpha);
                        auto end = chrono::high_resolution_clock::now();
                        int weight = calculateWeight(solution);
                        chrono::duration<double> duration = end - start;

                        cout << "Poids obtenu : " << weight << ", Temps : " << duration.count() << "s" << endl;

                        if (weight < bestWeight) {
                            bestWeight = weight;
                            bestSolution = solution;
                            bestAlpha = alpha;
                            bestGraspIterations = graspIterations;
                            bestVnsIterations = vnsIterations;
                            bestKMax = kMax;
                            bestLocalSearchAttempts = localSearchAttempts;
                        }
                    }
                }
            }
        }
    }

    cout << "Meilleur résultat trouvé : " << endl;
    cout << "Alpha = " << bestAlpha
         << ", graspIterations = " << bestGraspIterations
         << ", vnsIterations = " << bestVnsIterations
         << ", kMax = " << bestKMax
         << ", localSearchAttempts = " << bestLocalSearchAttempts
         << ", Poids = " << bestWeight << endl;
    cout << "Solution : ";
    for (int s : bestSolution) cout << s << " ";
    cout << endl;
}

// Fonction pour exécuter Grid Search sur le modèle VNS avec Meta-Learning
void gridSearchVNSMetaLearning(int m, int n, const vector<int>& initialSolution) {
    // Plages de valeurs pour chaque paramètre
    vector<int> maxIterationsValues = {500, 1000};
    vector<int> kMaxValues = {3, 5, 7};
    vector<int> localSearchAttemptsValues = {10, 20, 50};
    vector<double> alphaValues = {0.05, 0.1, 0.2};

    vector<int> bestSolution;
    int bestWeight = numeric_limits<int>::max();

    cout << "===== Début du Grid Search pour VNS avec Meta-Learning =====" << endl;

    // Boucles imbriquées pour tester toutes les combinaisons de paramètres
    for (int maxIterations : maxIterationsValues) {
        for (int kMax : kMaxValues) {
            for (int localSearchAttempts : localSearchAttemptsValues) {
                for (double alpha : alphaValues) {
                    cout << "\n[Test] maxIterations=" << maxIterations
                         << ", kMax=" << kMax
                         << ", localSearchAttempts=" << localSearchAttempts
                         << ", alpha=" << alpha << endl;

                    auto start = chrono::high_resolution_clock::now();
                    vector<int> solution = vnsSearchMetaLearning(m, n, initialSolution, maxIterations, kMax, localSearchAttempts);
                    auto end = chrono::high_resolution_clock::now();
                    chrono::duration<double> elapsed = end - start;

                    int weight = calculateWeight(solution);

                    // Affichage des résultats pour cette combinaison
                    cout << "  Poids obtenu : " << weight
                         << ", Temps d'exécution : " << elapsed.count() << " secondes" << endl;

                    // Mise à jour de la meilleure solution
                    if (weight < bestWeight) {
                        bestWeight = weight;
                        bestSolution = solution;
                        cout << "  ** Nouvelle meilleure solution trouvée **" << endl;
                    }
                }
            }
        }
    }

    cout << "\n===== Fin du Grid Search =====" << endl;
    cout << "Meilleur poids trouvé : " << bestWeight << endl;
    cout << "Solution : ";
    for (int s : bestSolution) {
        cout << s << " ";
    }
    cout << endl;
}



