#include <gtest/gtest.h>
#include <set>

#include "../src/geometry.hpp"


std::set<int> get_intersections(const std::vector<Geometry::Triangle>& triangles) {
    std::set<int> intersections;

    for (int i = 0; i < triangles.size() - 1; ++i) {
        for (int j = i + 1; j < triangles.size(); ++j) {
            if (Geometry::is_intersect(triangles[i], triangles[j])) {
                intersections.insert(i + 1);
                intersections.insert(j + 1);
            }
        }
    }
    return intersections;
}

TEST(BasicTests, PairIntersect) {
    Geometry::Triangle first{{1, 0, 0}, {0, 1, 0}, {0, 0, 7}};

    Geometry::Triangle second{{0, 0, -2}, {2, 2, 0}, {0, 0, 10}};

    EXPECT_TRUE(Geometry::is_intersect(first, second));
}

TEST(BasicTests, MultipleIntersect) {
    std::vector<Geometry::Triangle> triangles{
        {{1, 0, 0}, {0, 1, 0}, {0, 0, 7}},
        {{-2.63, 1.17, 0}, {2.76, -0.93, 4}, {2.78, 3.8, 0}},
        {{-2.19, -1.06, 0}, {-1.22, -3.81, 0}, {-3.96, -3.4, 0}}};

    std::set<int> correct{1, 2};
    std::set<int> intersections = get_intersections(triangles);

    EXPECT_TRUE(intersections == correct);
}

TEST(BasicTests, OneDotIntersect) {
    std::vector<Geometry::Triangle> triangles{
        {{-2.19, -1.06, 0}, {-1.22, -3.81, 0}, {-3.96, -3.4, 0}},
        {{0, 1, 0}, {0, -3, 0}, {-2.19, -1.06, 0}}};

    std::set<int> correct{1, 2};
    std::set<int> intersections = get_intersections(triangles);

    EXPECT_TRUE(intersections == correct);
}

TEST(BasicTests, NestingTest) {
    std::vector<Geometry::Triangle> triangles{
        {{-4, 0, 0}, {0, -4, 0}, {0, 0, 0}},
        {{-2, -1, 0}, {-1, -2, 0}, {-1, -1, 0}}};

    std::set<int> correct{1, 2};
    std::set<int> intersections = get_intersections(triangles);

    EXPECT_TRUE(intersections == correct);
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}