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
    vector<vector<Edge>> graph;   // �ڽӱ�
    vector<uint32_t> stations;    // ��վ�ı���б�
    vector<uint32_t> satellites;  // ���ǵı���б�

    vector<bool> visit;                 // ���·����ʱ������dijkstra���
    vector<vector<uint32_t>> dist;      // �Ի�վiΪ���ʱ��������j����̾���
    vector<vector<uint32_t>> prev;      // �Ի�վiΪ���ʱ��������j��·��ǰ��

    vector<bool> mask_nodes;

    vector<vector<uint32_t>> satellites_stations;        //ÿ���������ӵĻ�վ
    vector<vector<uint32_t>> stations_satellites;  //ÿ����վ�ɵ��������

    vector<Route> retRouteVec;

    void init(uint32_t N, const vector<bool> &typeVec) {
        // ��ʼ��
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
        // ��ʼ�����б���
        init(N, typeVec);

        // �����ڽӱ�ͼ
        for (auto &e:edgeVec) {
            graph[e.send].push_back(e);
            graph[e.recv].push_back({e.recv, e.send, e.dist});
        }

        // ����ÿ����վ�����ĵ�Դ���·��
        for (auto &s:stations) {
            dijkstra(s);
        }

        //ͳ��ÿ�����ǵĿɴ��վ
        for (auto &sate:satellites) {
            for (auto &stat:stations) {
                if (dist[stat][sate] <= D)
                    satellites_stations[sate].push_back(stat);
            }
        }

        //ͳ��ÿ����վ�Ŀɴ�����
        for (auto &stat:stations) {
            for (auto &sate:satellites) {
                if (dist[stat][sate] <= D)
                    stations_satellites[stat].push_back(sate);
            }
        }

        // ����վ���򣻰���ÿ����վ�Ŀɴ�������
        sort(stations.begin(), stations.end(), [this](uint32_t x, uint32_t y) {
            return stations_satellites[x].size() < stations_satellites[y].size();
        });
        /********************** ������Ԥ���� *************************/

        // ����ƽ�����ۣ��ҳ������ٵ�������Ϊ��������
        unordered_set<uint32_t> selected_receivers;
        for (auto &stat:stations) {
            if (mask_nodes[stat])  // �Ѿ�Ѱ���ɴ����ǵĻ�վ�����ٿ���
                continue;

            // �ҵ�һ������������Ϊ��������
            uint32_t sate = find_best_satellite(stations_satellites[stat], C, D, pwrVec);
//            cout << "New satellite: " << sate << endl;

            // ��ǵ���ǰ���ǿ������ӵ���Щ��վ
            for (auto &s:stations) {
                uint32_t remain_keda = 0;
                for (auto &ss:stations_satellites[s]) {
                    if (!mask_nodes[ss])
                        remain_keda++;
                }

                if (!mask_nodes[s] && dist[s][sate] <= D && (dist[s][sate] * pwrVec[s] <= 6000||remain_keda<=1)) {
                    mask_nodes[s] = true;  // ��ǻ�վ�ѿ���
                }
            }
            mask_nodes[sate] = true;  // ���������ʹ��
            selected_receivers.insert(sate);
        }

        /*************************** ���´���·�� ******************************/
        // ��������ѡ���Ľ������Ǽ���selected_receivers��Ϊÿ����վѡһ����������
        vector<uint32_t> station_map(N);  // ÿ����վ�Ľ���������˭
        for (auto &stat:stations) {
            uint32_t min_dist = Config::INF;
            for (auto &sate:selected_receivers) {
                if (min_dist > dist[stat][sate] && dist[stat][sate] <= D) {
                    min_dist = dist[stat][sate];
                    station_map[stat] = sate;
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

    // �Ӹ��������б��У���һ��������͵�����
    uint32_t find_best_satellite(const vector<uint32_t> &given_satellites, uint32_t C, uint32_t D, const vector<int> &pwrVec) {
        double min_avg_cost = 1e36;
        uint32_t ret = -1u;
        for (auto &sate:given_satellites) {
            if (mask_nodes[sate])  // �ѱ�ʹ�õĽ������ǲ��ٿ���
                continue;

            // ���ڵ�ǰ����sate���ҳ����пɴ��վ
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

            // ������ĿԼ���������赱ǰ������Ϊ��������ʱ�Ĵ���
            double temp_avg_cost = get_temp_cost(sate, selected_stations, C, D, pwrVec);

            // ��¼��С���ۣ��Լ���Ӧ�����Ǳ��
            if (min_avg_cost > temp_avg_cost) {
                min_avg_cost = temp_avg_cost;
                ret = sate;
            }
        }
        return ret; // һ�����᷵��-1u���������޽��ˣ����������޷����κ��������ӵĻ�վ
    }

    double get_temp_cost(const uint32_t satellite, const vector<uint32_t> &selected_stations, uint32_t C, uint32_t D, const vector<int> &pwrVec) {
        double temp_avg_cost = 0;
        int num_stations = 0; // ��ֹ����0
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
    uint32_t N;             // ��ʾ�������Ǻͷ����վ������
    uint32_t E;             // ��ʾ�����п�ʹ�õ�·������
    uint32_t C;             // ��ʾ·������L�빦��P��ϵ�����Ա��Ϊi�ķ����վΪ���Ĵ��书�ļ��㹫ʽΪ��P=C*L*pwrVec[i]
    uint32_t D;             // ��ʾȫ�����·���������ơ�
    vector<bool> typeVec;   // �±�Ϊi��ֵ����IDΪi��վ����ݣ�����Ϊtrue�������վΪfalse
    vector<int> pwrVec;     // �±�Ϊi��ֵ����IDΪi��վ�㹦����Ϣ������Ƿ����վ����Ϊ·������ϵ����������Ϊ�������ǵĹ��ġ�
    vector<Edge> edgeVec;   // ����E����

    cin >> N >> E >> C >> D;

    // ����վ�����
    typeVec = vector<bool>(N);
    for (uint32_t i = 0; i < N; i++) {
        bool type;
        cin >> type;
        typeVec[i] = type;
    }

    // ����վ��Ĺ�����Ϣ
    pwrVec = vector<int>(N);
    for (uint32_t i = 0; i < N; i++) {
        cin >> pwrVec[i];
    }

    // ����߼�
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