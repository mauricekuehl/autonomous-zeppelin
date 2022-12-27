#include <iostream>
#include <functional>

void function_with_callback(int y, std::function<void(int)> callback)
{
    int i = 1;
    std::cout << "y: " << y << std::endl;
    int t = 999;
    callback(i, t);
}

void callback(int i, int t, int z, int y)
{

    int res = i + y + z;
    std::cout << res << std::endl;
}

int main(int argc, char **argv)
{
    int imported_obj = 9;
    int y = 2;
    int z = 99;
    // call function with callback and bind imported_obj and z to be available in the callback
    function_with_callback(y, std::bind(&callback, std::placeholders::_1, std::placeholders::_2, imported_obj, z));
    std::cout << imported_obj << std::endl;
}
