#include "geometry.hpp"

int main() {
    Geometry::Triangle first{{1, 0, 0}, {0, 1, 0}, {0, 0, 7}};
    
    Geometry::Triangle second{{0, 0, -2}, {2, 2, 0}, {0, 0, 10}};
    
    std::cout << "first -> second: " << first.is_intersect(second)  << "\n";
    std::cout << "second -> first: " << second.is_intersect(first)  << "\n";
    return 0;
}