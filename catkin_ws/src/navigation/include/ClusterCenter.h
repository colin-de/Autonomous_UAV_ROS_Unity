#ifndef CLUSTER_CENTER_H
#define CLUSTER_CENTER_H

// Based on https://stackoverflow.com/a/20039017/14969444

#include <stdlib.h>
#include <vector>

class ClusterCenter
{
public:
    ClusterCenter(std::vector<std::vector<int>> binaryMatrix, int M, int N);

    typedef struct
    {
        int one;
        int two;
    } Pair;

    Pair location_of_most_unknowns();
    void push(int a, int b);
    void pop(int *a, int *b);
    void update_cache(int row);

private:
    std::vector<std::vector<int>> binaryMatrix;
    int M, N; /* Dimension of input; M is length of a row. */
    Pair best_ll;
    Pair best_ur;
    int best_area;

    int *c;  /* Cache */
    Pair *s; /* Stack */
    int top; /* Top of stack */
};

#endif // CLUSTER_CENTER_H
