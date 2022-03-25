#pragma once

#include <iostream>
#include <vector>
using namespace std;

struct Edge {
    uint32_t send;  // ����վ��
    uint32_t recv;  // ����վ��
    uint32_t dist;  // ����

    int cap;
    int flow;
    int cost;
};

using Route = vector<uint32_t>;