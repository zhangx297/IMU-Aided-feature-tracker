#include <iostream>
#include <map>

int main() {
    std::map<std::string, int> my_map;

    // 插入元素
    my_map["apple"] = 1;
    my_map["banana"] = 2;
    my_map["orange"] = 3;

    // 访问元素
    std::cout << "apple: " << my_map["apple"] << std::endl;
    std::cout << "banana: " << my_map["banana"] << std::endl;
    std::cout << "orange: " << my_map["orange"] << std::endl;

    // 遍历map
    for (const auto &pair : my_map) {
        std::cout << pair.first << ": " << pair.second << std::endl;
    }

    return 0;
}
