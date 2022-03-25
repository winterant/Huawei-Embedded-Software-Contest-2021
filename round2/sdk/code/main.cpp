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
    vector<vector<Edge>> graph;   // �ڽӱ�
    vector<uint32_t> stations;    // ��վ�ı���б�
    vector<uint32_t> satellites;  // ���ǵı���б�
    vector<uint32_t> mask_nodes;
    vector<double> avg_cost;    // ÿ��������Ϊ�������ǵ�ƽ������
    vector<bool> visit;                 // ���·����ʱ������dijkstra���
    vector<vector<uint32_t>> dist;      // �Ի�վiΪ���ʱ��������j����̾���
    vector<vector<uint32_t>> prev;      // �Ի�վiΪ���ʱ��������j��·��ǰ��

    vector<vector<uint32_t>> sate_stations;//ÿ�����ǿ��Խ��ܵ��Ļ�վ

public:
    vector<Route> Plan(uint32_t N, uint32_t C, uint32_t D, uint32_t PS, const vector<bool> &typeVec, const vector<int> &pwrVec, const vector<Edge> &edgeVec) {
        // ��ʼ��
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

        // �����ڽӱ�ͼ
        for (auto &e:edgeVec) {
            graph[e.send].push_back(e);
            graph[e.recv].push_back({e.recv, e.send, e.dist});
        }

        // ����ÿ����վ�����ĵ�Դ���·��
        for (auto &s:stations) {
            dijkstra(s);
        }

        //ͳ��ÿ�����ǿ��Խ��ܵ��Ļ�վ
        for (auto &sate:satellites) {
            for (auto &s:stations) {
                if (dist[s][sate] <= D)
                    sate_stations[sate].push_back(s);
            }
        }

        // ����ƽ�����ۣ��ҳ������ٵ�������Ϊ��������

        unordered_set<uint32_t> init_receivers;
        unordered_set<uint32_t> station_set;
        vector<uint32_t> satellite_count(N, 0); //ÿ�����ǽ��յĻ�վ����
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


        // ѡ�еĽ������ǽ������򣬰��ջ�վ��
        vector<uint32_t> selected_satellites(init_receivers.begin(), init_receivers.end());
        sort(selected_satellites.begin(), selected_satellites.end(), [satellite_count](int x, int y) {
            return satellite_count[x] > satellite_count[y]; // ��ѡ�еĽ������ǰ����ӵĻ�վ������
        });

        // ��������ѡ���Ľ������Ǽ���final_receivers��Ϊÿ����վѡһ����������
        init_receivers.clear();
        for (auto &s:stations) {
            for (auto &sate:selected_satellites) {
                if (dist[s][sate] <= D) {
                    init_receivers.insert(sate);
                    break;
                }
            }
        }

        // ��������ѡ���Ľ������Ǽ��ϣ�Ϊÿ����վѡһ����������
        vector<uint32_t> station_map(N);  // ÿ����վ�Ľ���������˭
        for (auto &s:stations) {
            uint32_t min_dist = Config::INF;
            for (auto &sate:init_receivers) {
                if (min_dist > dist[s][sate] && dist[s][sate] <= D) {
                    min_dist = dist[s][sate];
                    station_map[s] = sate;
                }
            }
        }

        //��¼·��
        for (auto &s:stations) {
            retRouteVec.emplace_back();
            for (uint32_t i = station_map[s]; typeVec[i] != 0; i = prev[s][i]) {
                retRouteVec.back().push_back(i);
            }
            retRouteVec.back().push_back(s);  //��վ
            reverse(retRouteVec.back().begin(), retRouteVec.back().end());
        }
        return retRouteVec;
    }

    // ���·��+��¼·�� O(n logn)
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

    // ��ʣ������+ʣ���վ�У���һ��ƽ��������͵�����
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

    uint32_t N;             // ��ʾ�������Ǻͷ����վ������
    uint32_t E;             // ��ʾ�����п�ʹ�õ�·������
    uint32_t C;             // ��ʾ·������L�빦��P��ϵ�����Ա��Ϊi�ķ����վΪ���Ĵ��书�ļ��㹫ʽΪ��P=C*L*pwrVec[i]
    uint32_t D;             // ��ʾȫ�����·����������??
    uint32_t PS;            // ��ʾ������������Ҫ��վ�㹦??
    vector<bool> typeVec;   // �±�Ϊi��ֵ����IDΪi��վ����ݣ�����Ϊtrue�������վΪfalse
    vector<int> pwrVec;     // �±�Ϊi��ֵ����IDΪi�ķ����վ�Ĺ���ϵ����������Ƿ����վ����??10000
    vector<Edge> edgeVec;   // ����E����

    cin >> N >> E >> C >> D >> PS;

    // ����վ�����
    typeVec = vector<bool>(N);
    for (uint32_t i = 0; i < N; i++) {
        bool type;
        cin >> type;
        typeVec[i] = type;
    }

    // ���뷢���վ�Ķ��ϵ
    pwrVec = vector<int>(N);
    for (uint32_t i = 0; i < N; i++) {
        cin >> pwrVec[i];
    }

    // ����߼�
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