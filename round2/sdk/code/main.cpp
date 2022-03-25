#include <iostream>
#include <fstream>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <queue>
#include <unordered_set>
#include <algorithm>
#include "data.h"

using namespace std;
typedef pair<uint32_t, uint32_t> PAIR;

namespace Config {
    static const uint32_t INF = 0x3f3f3f3f;
}

class Solution {
    vector<vector<Edge>> graph;   // 邻接表
    vector<uint32_t> stations;    // 基站的编号列表
    vector<uint32_t> satellites;  // 卫星的编号列表
    vector<uint32_t> mask_nodes;
    vector<double> avg_cost;    // 每个卫星设为接收卫星的平均代价
    vector<bool> visit;                 // 最短路计算时，用于dijkstra标记
    vector<vector<uint32_t>> dist;      // 以基站i为起点时，到卫星j的最短距离
    vector<vector<uint32_t>> prev;      // 以基站i为起点时，到卫星j的路径前驱

    vector<vector<uint32_t>> sate_stations;//每个卫星可以接受到的基站

public:
    vector<Route> Plan(uint32_t N, uint32_t C, uint32_t D, uint32_t PS, const vector<bool> &typeVec, const vector<int> &pwrVec, const vector<Edge> &edgeVec) {
        // 初始化
        graph.resize(N);
        avg_cost.reserve(N);
        visit.resize(N);
        dist.resize(N);
        prev.resize(N);
        for (uint32_t i = 0; i < typeVec.size(); i++) {
            if (typeVec[i] == 0) {
                stations.push_back(i);
                dist[i].resize(N);
                prev[i].resize(N);
            } else {
                satellites.push_back(i);
            }
        }
        mask_nodes.resize(N, false);
        sate_stations.resize(N);

        vector<Route> retRouteVec;

        // 基于邻接表建图
        for (auto &e:edgeVec) {
            graph[e.send].push_back(e);
            graph[e.recv].push_back({e.recv, e.send, e.dist});
        }

        // 计算每个基站出发的单源最短路径
        for (auto &s:stations) {
            dijkstra(s);
        }

        //统计每个卫星可以接受到的基站
        for (auto &sate:satellites) {
            for (auto &s:stations) {
                if (dist[s][sate] <= D)
                    sate_stations[sate].push_back(s);
            }
        }

        // 根据平均代价，找出尽量少的卫星作为接收卫星

        unordered_set<uint32_t> init_receivers;
        unordered_set<uint32_t> station_set;
        vector<uint32_t> satellite_count(N, 0); //每个卫星接收的基站个数
        while (station_set.size() < stations.size()) {
            uint32_t sate = find_best_satellite(C, PS, D, pwrVec);
            if (sate == -1u)
                break;

            bool flag = false;
            for (auto &s:stations) {
                if (!station_set.count(s) && dist[s][sate] <= D) {
                    flag = true;
                    station_set.insert(s);
                    mask_nodes[s] = true;
                    satellite_count[sate]++;
                }
            }
            if (flag) {
                init_receivers.insert(sate);
                mask_nodes[sate] = true;
            }
        }


        // 选中的接收卫星进行排序，按照基站数
        vector<uint32_t> selected_satellites(init_receivers.begin(), init_receivers.end());
        sort(selected_satellites.begin(), selected_satellites.end(), [satellite_count](int x, int y) {
            return satellite_count[x] > satellite_count[y]; // 将选中的接收卫星按连接的基站数排序
        });

        // 根据最终选定的接收卫星集合final_receivers，为每个基站选一个接收卫星
        init_receivers.clear();
        for (auto &s:stations) {
            for (auto &sate:selected_satellites) {
                if (dist[s][sate] <= D) {
                    init_receivers.insert(sate);
                    break;
                }
            }
        }

        // 根据最终选定的接收卫星集合，为每个基站选一个接收卫星
        vector<uint32_t> station_map(N);  // 每个基站的接收卫星是谁
        for (auto &s:stations) {
            uint32_t min_dist = Config::INF;
            for (auto &sate:init_receivers) {
                if (min_dist > dist[s][sate] && dist[s][sate] <= D) {
                    min_dist = dist[s][sate];
                    station_map[s] = sate;
                }
            }
        }

        //记录路径
        for (auto &s:stations) {
            retRouteVec.emplace_back();
            for (uint32_t i = station_map[s]; typeVec[i] != 0; i = prev[s][i]) {
                retRouteVec.back().push_back(i);
            }
            retRouteVec.back().push_back(s);  //基站
            reverse(retRouteVec.back().begin(), retRouteVec.back().end());
        }
        return retRouteVec;
    }

    // 最短路径+记录路径 O(n logn)
    void dijkstra(const uint32_t source) {
        auto &cost = dist[source];
        auto &pre = prev[source];
        cost.assign(cost.size(), Config::INF);
        cost[source] = 0;
        visit.assign(visit.size(), false);
        priority_queue<PAIR, vector<PAIR>, greater<>> q;
        q.push(PAIR(0, source));
        while (!q.empty()) {
            uint32_t u = q.top().second;
            q.pop();
            if (visit[u]) continue;
            visit[u] = true;
            for (auto &e:graph[u]) {
                if (cost[e.recv] > cost[u] + e.dist) {
                    cost[e.recv] = cost[u] + e.dist;
                    pre[e.recv] = e.send;
                    q.push(PAIR(cost[e.recv], e.recv));
                }
            }
        }
    }

    // 从剩余卫星+剩余基站中，找一个平均代价最低的卫星
    uint32_t find_best_satellite(uint32_t C, uint32_t PS, uint32_t distance_limit, const vector<int> &pwrVec) {
        double min_avg_cost = 1e36;
        uint32_t ret = -1u;
        for (auto &sate:satellites) {
            if (mask_nodes[sate])
                continue;

            int num_stations = 0;
            double temp_avg_cost = 0;
            for (auto &s:sate_stations[sate]) {
                if (!mask_nodes[s] && dist[s][sate] <= distance_limit*0.96666) {
                    num_stations++;
                    temp_avg_cost += C * dist[s][sate] * pwrVec[s];
                }
            }
            temp_avg_cost += PS;
            temp_avg_cost /= num_stations;

            if (min_avg_cost > temp_avg_cost) {
                min_avg_cost = temp_avg_cost;
                ret = sate;
            }
        }
        return ret;
    }
};


int main(int argc, char *argv[]) {
//    ios::sync_with_stdio(false);
//    ifstream rin("samples/Example.case");
//    ifstream rin("../../judge/cases/TestData_24.case");
//    cin.rdbuf(rin.rdbuf());

    uint32_t N;             // 表示所有卫星和发射基站的总数
    uint32_t E;             // 表示网络中可使用的路径总数
    uint32_t C;             // 表示路径距离L与功耗P的系数，以编号为i的发射基站为起点的传输功耗计算公式为：P=C*L*pwrVec[i]
    uint32_t D;             // 表示全网最大路径长度限制??
    uint32_t PS;            // 表示接收卫星所需要的站点功??
    vector<bool> typeVec;   // 下标为i的值代表ID为i的站点身份，卫星为true，发射基站为false
    vector<int> pwrVec;     // 下标为i的值代表ID为i的发射基站的功耗系数，如果不是发射基站，则??10000
    vector<Edge> edgeVec;   // 包含E条边

    cin >> N >> E >> C >> D >> PS;

    // 输入站点身份
    typeVec = vector<bool>(N);
    for (uint32_t i = 0; i < N; i++) {
        bool type;
        cin >> type;
        typeVec[i] = type;
    }

    // 输入发射基站的额功耗系
    pwrVec = vector<int>(N);
    for (uint32_t i = 0; i < N; i++) {
        cin >> pwrVec[i];
    }

    // 输入边集
    edgeVec = vector<Edge>(E);
    for (auto &edge : edgeVec) {
        cin >> edge.send >> edge.recv >> edge.dist;
    }

    Solution solution;
    vector<Route> retRouteVec = solution.Plan(N, C, D, PS, typeVec, pwrVec, edgeVec);
    for (const auto &route : retRouteVec) {
        for (const auto &siteId : route) {
            cout << siteId << " ";
        }
        cout << "\n";
    }
    return 0;
}