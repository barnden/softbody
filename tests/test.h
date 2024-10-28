/*
 * Copyright (c) 2024, Brandon G. Nguyen <brandon@nguyen.vc>
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */
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
    size_t total = 0;
    size_t total_passed = 0;
    std::vector<Test> tests;

    void add(Test test) { tests.push_back(test); }

    void run()
    {
        for (auto&& test : tests) {
            if (test.testfn()) {
                std::cout << GREEN << "PASS: " << RESET;
                total_passed++;
            } else {
                std::cout << RED << "FAIL: " << RESET;
            }

            std::cout << test.name << '\n';
        }

        total += tests.size();
        tests.clear();
    }

    void statistics()
    {
        std::cout.precision(4);
        std::cout << "Passed " << 100. * ((double)total_passed) / ((double)total) << "%\n";
    }
};
