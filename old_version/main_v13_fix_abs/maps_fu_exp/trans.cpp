#include <bits/stdc++.h>
using namespace std;
int main()
{
    fstream in("3.txt");
    fstream out("3_t.txt");
    string s;
    while (in >> s)
    {
        for (char &c : s)
            if (c == '.')
                c = '*';
        out << s << '\n';
    }
}