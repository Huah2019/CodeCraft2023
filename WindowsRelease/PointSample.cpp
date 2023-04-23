#include <bits/stdc++.h>
using namespace std;
pair<int, int> grid_center(int i, int j) { return {2 * i + 1, 2 * j + 1}; }
void solve()
{
    // freopen("yyz_tests1/5.txt", "r", stdin);
    freopen("maps/4_p.txt", "r", stdin);
    // freopen("maps_fu/4.txt", "r", stdin);
    // freopen("maps_fu_exp/4.txt", "r", stdin);
    freopen("1.out", "w", stdout);
    int n = 100;
    vector<string> g(n);
    for (int i = 0; i < n; ++i)
        cin >> g[i];

    int gn = 2 * n + 1;
    vector<vector<int>> grid = vector<vector<int>>(gn, vector<int>(gn));
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            if (g[i][j] == '#')
            {
                int gi, gj;
                tie(gi, gj) = grid_center(i, j);
                for (int dx = -1; dx <= 1; ++dx)
                    for (int dy = -1; dy <= 1; ++dy)
                        grid[gi + dx][gj + dy] = -1;
            }
    for (int i = 0; i < gn; ++i)
        for (int j = 0; j < gn; ++j)
            if (i == 0 || i == gn - 1 || j == 0 || j == gn - 1)
                grid[i][j] = -1;
    auto newGrid = grid;
    for (int i = 1; i < gn - 1; ++i)
    {
        for (int j = 1; j < gn - 1; ++j)
            if (grid[i][j] != -1)
            {
                int k = j;
                while (k + 1 < gn - 1 && grid[i][k + 1] != -1)
                    ++k;
                int len = k - j + 1;
                for (int h = j; h <= k; h += 2)
                {
                    if (len == 5 && h == j + 2)
                        continue;
                    newGrid[i][h] = 1;
                }
                j = k;
            }
    }
    for (int j = 1; j < gn - 1; ++j)
        for (int i = 1; i < gn - 1; ++i)
        {
            if (grid[i][j] != -1)
            {
                int k = i;
                while (k + 1 < gn - 1 && grid[k + 1][j] != -1)
                    ++k;
                int len = k - i + 1;
                for (int h = i; h <= k; h += 2)
                {
                    if (len == 5 && h == i + 2)
                        continue;
                    newGrid[h][j] = 1;
                }
                i = k;
            }
        }
    int sum1 = 0;
    for (int i = 0; i < gn; ++i)
        for (int j = 0; j < gn; ++j)
            if (newGrid[i][j] == 0)
                ++sum1;

    auto cal = [&](int len)
    {
        auto checkrow = [&](int r, int c1, int c2)
        {
            if (grid[r][c1] != -1 || grid[r][c2] != -1)
                return false;
            for (int c = c1 + 1; c < c2; ++c)
                if (grid[r][c] != 0)
                    return false;
            return true;
        };
        auto checkcol = [&](int c, int r1, int r2)
        {
            if (grid[r1][c] != -1 || grid[r2][c] != -1)
                return false;
            for (int r = r1 + 1; r < r2; ++r)
                if (grid[r][c] != 0)
                    return false;
            return true;
        };
        auto ok = [&](int r1, int r2, int c1, int c2)
        {
            for (int r = r1; r <= r2; ++r)
                for (int c = c1; c <= c2; ++c)
                    if (grid[r][c] != 0)
                        return false;
            return true;
        };
        for (int i = 0; i + len - 1 < gn; ++i)
            for (int j = 0; j + len - 1 < gn; ++j)
            {
                if (!ok(i + 1, i + len - 2, j + 1, j + len - 2))
                    continue;
                if ((checkrow(i, j, j + len - 1) || checkrow(i + len - 1, j, j + len - 1)) && (ok(i + 1, i + len - 2, j, j) || ok(i + 1, i + len - 2, j + len - 1, j + len - 1)))
                {
                    newGrid[i + len / 2][j + len / 2] = 0;
                }
                if ((checkcol(j, i, i + len - 1) || checkcol(j + len - 1, i, i + len - 1)) && (ok(i, i, j + 1, j + len - 2) || ok(i + len - 1, i + len - 1, j + 1, j + len - 2)))
                {
                    newGrid[i + len / 2][j + len / 2] = 0;
                }
            }
    };
    cal(5);
    cal(7);

    int sum2 = 0;
    for (int i = 0; i < gn; ++i)
    {
        for (int j = 0; j < gn; ++j)
            if (newGrid[i][j] == -1)
                cout << '#';
            else if (newGrid[i][j] == 0)
                cout << 'X', ++sum2;
            else
                cout << ".";
        cout << '\n';
    }
    freopen("CON", "w", stdout);
    cout << sum1 << ' ' << sum2 << ' ' << sum2 - sum1 << '\n';
}
int main()
{
    solve();
}