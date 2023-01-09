#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>
using namespace std;

void func()
{
    int a = 1200;

    std::vector<float>
        v;

    auto start = std::chrono::high_resolution_clock::now();

    for (int y = 0; y < a + a; y++)
    {

        // std::cout << "y = " << y << std::endl;
        v.clear();
        for (int i = 0; i < a; i++)
            v.push_back(0.0f);
        int size = v.size();

        int pref_size = y;
        float dif = v.size() - (float)pref_size;
        float ratio = v.size() / abs(dif);
        float count = 0;
        for (int i = v.size() - 1; i >= 0; i--)
        {
            count += 1;
            while (count > ratio)
            {
                count -= ratio;
                if (dif > 0)
                    v.erase(v.begin() + i);
                else
                    v.insert(v.begin() + i, 1.0f);
            }
        }
        if (v.size() > pref_size)
        {
            v.pop_back();
        }
        else if (v.size() < pref_size)
        {
            v.push_back(0.0f);
        }

        if (y % 1 == 0)
        {
            auto stop = std::chrono::high_resolution_clock::now();

            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
            std::cout << duration.count() / 1 << " from: " << y - 1 << " to: " << y << std::endl;

            start = std::chrono::high_resolution_clock::now();
        }

        if (pref_size != v.size())
        {
            std::cout << "error";
            std::cout << "dif = " << dif << std::endl;
            std::cout << pref_size << std::endl;
            std::cout << v.size() << std::endl;
            std::cout << "real = " << (float)v.size() / abs(dif) << std::endl;
            std::cout << std::endl;
        }
        // print vector v
        // for (int i = 0; i < v.size(); i++)
        //     std::cout << v[i] << " ";
        // std::cout << std::endl;
        // std::cout << "pref" << pref_size << std::endl;
        // break;
    }
}

int main(void)
{
    auto start = std::chrono::high_resolution_clock::now();
    func();
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    std::cout << duration.count() << std::endl;
}
