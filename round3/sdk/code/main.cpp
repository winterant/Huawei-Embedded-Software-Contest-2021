#include <iostream>
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

    vector<bool> visit;                 // 最短路计算时，用于dijkstra标记
    vector<vector<uint32_t>> dist;      // 以基站i为起点时，到卫星j的最短距离
    vector<vector<uint32_t>> prev;      // 以基站i为起点时，到卫星j的路径前驱

    vector<bool> mask_nodes;

    vector<vector<uint32_t>> satellites_stations;        //每个卫星连接的基站
    vector<vector<uint32_t>> stations_satellites;  //每个基站可到达的卫星

    vector<Route> retRouteVec;

    void init(uint32_t N, const vector<bool> &typeVec) {
        // 初始化
        graph.resize(N);
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
        satellites_stations.resize(N);
        stations_satellites.resize(N);
    }

public:
    vector<Route> Plan(uint32_t N, uint32_t C, uint32_t D, const vector<bool> &typeVec, const vector<int> &pwrVec, const vector<Edge> &edgeVec) {
        // 初始化所有变量
        init(N, typeVec);

        // 基于邻接表建图
        for (auto &e:edgeVec) {
            graph[e.send].push_back(e);
            graph[e.recv].push_back({e.recv, e.send, e.dist});
        }

        // 计算每个基站出发的单源最短路径
        for (auto &s:stations) {
            dijkstra(s);
        }

        //统计每个卫星的可达基站
        for (auto &sate:satellites) {
            for (auto &stat:stations) {
                if (dist[stat][sate] <= D)
                    satellites_stations[sate].push_back(stat);
            }
        }

        //统计每个基站的可达卫星
        for (auto &stat:stations) {
            for (auto &sate:satellites) {
                if (dist[stat][sate] <= D)
                    stations_satellites[stat].push_back(sate);
            }
        }

        // 将基站排序；按照每个基站的可达卫星数
        sort(stations.begin(), stations.end(), [this](uint32_t x, uint32_t y) {
            return stations_satellites[x].size() < stations_satellites[y].size();
        });
        /********************** 以上是预处理 *************************/

        // 根据平均代价，找出尽量少的卫星作为接收卫星
        unordered_set<uint32_t> selected_receivers;
        for (auto &stat:stations) {
            if (mask_nodes[stat])  // 已经寻到可达卫星的基站，不再考虑
                continue;

            // 找到一颗最优卫星作为接收卫星
            uint32_t sate = find_best_satellite(stations_satellites[stat], C, D, pwrVec);
//            cout << "New satellite: " << sate << endl;

            // 标记掉当前卫星可以连接的那些基站
            for (auto &s:stations) {
                uint32_t remain_keda = 0;
                for (auto &ss:stations_satellites[s]) {
                    if (!mask_nodes[ss])
                        remain_keda++;
                }

                if (!mask_nodes[s] && dist[s][sate] <= D && (dist[s][sate] * pwrVec[s] <= 6000||remain_keda<=1)) {
                    mask_nodes[s] = true;  // 标记基站已可行
                }
            }
            mask_nodes[sate] = true;  // 标记卫星已使用
            selected_receivers.insert(sate);
        }

        /*************************** 以下处理路径 ******************************/
        // 根据最终选定的接收卫星集合selected_receivers，为每个基站选一个接收卫星
        vector<uint32_t> station_map(N);  // 每个基站的接收卫星是谁
        for (auto &stat:stations) {
            uint32_t min_dist = Config::INF;
            for (auto &sate:selected_receivers) {
                if (min_dist > dist[stat][sate] && dist[stat][sate] <= D) {
                    min_dist = dist[stat][sate];
                    station_map[stat] = sate;
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

    // 从给定卫星列表中，找一个代价最低的卫星
    uint32_t find_best_satellite(const vector<uint32_t> &given_satellites, uint32_t C, uint32_t D, const vector<int> &pwrVec) {
        double min_avg_cost = 1e36;
        uint32_t ret = -1u;
        for (auto &sate:given_satellites) {
            if (mask_nodes[sate])  // 已被使用的接收卫星不再考虑
                continue;

            // 对于当前卫星sate，找出所有可达基站
            static vector<uint32_t> selected_stations;
            selected_stations.clear();
            for (auto &stat:satellites_stations[sate]) {
                uint32_t remain_keda = 0;
                for (auto &ss:stations_satellites[stat]) {
                    if (!mask_nodes[ss])
                        remain_keda++;
                }
                if (!mask_nodes[stat] && (dist[stat][sate] * pwrVec[stat] <= 6000 || remain_keda <= 1))
                    selected_stations.push_back(stat);
            }

            // 根据题目约束，计算设当前卫星作为接收卫星时的代价
            double temp_avg_cost = get_temp_cost(sate, selected_stations, C, D, pwrVec);

            // 记录最小代价，以及对应的卫星编号
            if (min_avg_cost > temp_avg_cost) {
                min_avg_cost = temp_avg_cost;
                ret = sate;
            }
        }
        return ret; // 一定不会返回-1u，那样就无解了，即出现了无法与任何卫星连接的基站
    }

    double get_temp_cost(const uint32_t satellite, const vector<uint32_t> &selected_stations, uint32_t C, uint32_t D, const vector<int> &pwrVec) {
        double temp_avg_cost = 0;
        int num_stations = 0; // 防止初以0
        for (auto &stat:selected_stations) {
            if (dist[stat][satellite] <= D /* * 0.975 */) {
                temp_avg_cost += C * dist[stat][satellite] * pwrVec[stat];
                num_stations++;
            }
        }
        temp_avg_cost += pwrVec[satellite];
        temp_avg_cost /= (num_stations + 1);
        return temp_avg_cost;
    }
};

int main(int argc, char *argv[])
{
    uint32_t N;             // 表示所有卫星和发射基站的总数
    uint32_t E;             // 表示网络中可使用的路径总数
    uint32_t C;             // 表示路径距离L与功耗P的系数，以编号为i的发射基站为起点的传输功耗计算公式为：P=C*L*pwrVec[i]
    uint32_t D;             // 表示全网最大路径长度限制。
    vector<bool> typeVec;   // 下标为i的值代表ID为i的站点身份，卫星为true，发射基站为false
    vector<int> pwrVec;     // 下标为i的值代表ID为i的站点功耗信息：如果是发射基站，则为路径功耗系数；否则，则为接收卫星的功耗。
    vector<Edge> edgeVec;   // 包含E条边

    cin >> N >> E >> C >> D;

    // 输入站点身份
    typeVec = vector<bool>(N);
    for (uint32_t i = 0; i < N; i++) {
        bool type;
        cin >> type;
        typeVec[i] = type;
    }

    // 读入站点的功耗信息
    pwrVec = vector<int>(N);
    for (uint32_t i = 0; i < N; i++) {
        cin >> pwrVec[i];
    }

    // 输入边集
    edgeVec = vector<Edge>(E);
    for (auto& edge : edgeVec) {
        cin >> edge.send >> edge.recv >> edge.dist;
    }

    Solution solution;
    vector<Route> retRouteVec = solution.Plan(N, C, D, typeVec, pwrVec, edgeVec);
    for (const auto& route : retRouteVec) {
        for (const auto& siteId : route) {
            cout << siteId << " ";
        }
        cout << "\n";
    }
    return 0;
}