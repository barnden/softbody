#pragma once

#include <functional>
#include <iostream>
#include <string>
#include <vector>

constexpr std::string GREEN = "\e[0;32m";
constexpr std::string RED = "\e[0;31m";
constexpr std::string RESET = "\e[0m";

struct Test {
    std::string name;
    std::function<bool()> testfn;
};

struct TestManager {
    std::vector<Test> tests;

    void add(Test test) { tests.push_back(test); }

    void run()
    {
        auto num_passed = 0;
        for (auto&& test : tests) {
            if (test.testfn()) {
                std::cout << GREEN << "PASS" << RESET;
                num_passed++;
            } else {
                std::cout << RED << "FAIL" << RESET;
            }

            std::cout << ": " << test.name << '\n';
        }
    }
};
