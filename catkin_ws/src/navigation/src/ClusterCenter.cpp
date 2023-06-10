#include <ClusterCenter.h>

ClusterCenter::ClusterCenter(std::vector<std::vector<int>> binaryMatrix, int M, int N)
    : binaryMatrix(binaryMatrix), M(M), N(N), top(0), best_ll{0, 0}, best_ur{-1, -1}, best_area(0)
{
}

ClusterCenter::Pair ClusterCenter::location_of_most_unknowns()
{
    int m, n;
    c = (int *)malloc((M + 1) * sizeof(int));
    s = (Pair *)malloc((M + 1) * sizeof(Pair));
    for (m = 0; m != M + 1; ++m)
    {
        c[m] = s[m].one = s[m].two = 0;
    }
    /* Main algorithm: */
    for (n = 0; n != N; ++n)
    {
        int open_width = 0;
        update_cache(n);
        for (m = 0; m != M + 1; ++m)
        {
            if (c[m] > open_width)
            { /* Open new rectangle? */
                push(m, open_width);
                open_width = c[m];
            }
            else /* "else" optional here */
                if (c[m] < open_width)
                { /* Close rectangle(s)? */
                    int m0, w0, area;
                    do
                    {
                        pop(&m0, &w0);
                        area = open_width * (m - m0);
                        if (area > best_area)
                        {
                            best_area = area;
                            best_ll.one = m0;
                            best_ll.two = n;
                            best_ur.one = m - 1;
                            best_ur.two = n - open_width + 1;
                        }
                        open_width = w0;
                    } while (c[m] < open_width);
                    open_width = c[m];
                    if (open_width != 0)
                    {
                        push(m0, w0);
                    }
                }
        }
    }
    int col = (best_ll.one + best_ur.one) / 2;
    int row = (best_ll.two + best_ur.two) / 2;
    return {row, col};
}

void ClusterCenter::push(int a, int b)
{
    s[top].one = a;
    s[top].two = b;
    ++top;
}

void ClusterCenter::pop(int *a, int *b)
{
    --top;
    *a = s[top].one;
    *b = s[top].two;
}

void ClusterCenter::update_cache(int row)
{
    int m;
    for (m = 0; m != M; ++m)
    {
        if (binaryMatrix[row][m] == 0)
        {
            c[m] = 0;
        }
        else
        {
            ++c[m];
        }
    }
}