# coding=gbk
import os
import random


def generate_in(filename, num_vertexes, C=1, PS=10000, num_stations=None, num_edges=None):
    D = random.randint(2, 2000)
    if num_stations is None:
        num_stations = random.randint(1, min(4000, num_vertexes - 1))  # 最多4000个基站，超过则因边不足而无解
    if num_edges is None:
        num_edges = random.randint(num_stations, min(4000, num_stations + (num_vertexes - num_stations) * (num_vertexes - num_stations - 1) // 2))  # 边的数量，最多4000个边
    with open(filename, 'w') as f:
        f.write(f'{num_vertexes} {num_edges} {C} {D} {PS}\n')

        # 前num_stations个设为基站，其余为卫星
        for i in range(num_vertexes):
            f.write(str(0 if i < num_stations else 1) + (' ' if i < num_vertexes - 1 else '\n'))

        # 一行基站的功耗系数
        for i in range(num_vertexes):
            f.write(str(random.randint(1, 10)) + (' ' if i < num_vertexes - 1 else '\n'))

        # 先产生基站到卫星的边
        for i in range(num_stations):
            to = random.randint(num_stations, num_vertexes - 1)
            dist = random.randint(1, D)
            f.write(f'{i} {to} {dist}\n')

        # 在产生卫星与卫星之间的边
        visit = [[False] * num_vertexes for i in range(num_vertexes)]
        for i in range(num_edges - num_stations):
            source, to = -1, -1
            while source == to or visit[source][to]:
                source = random.randint(num_stations, num_vertexes - 1)
                to = random.randint(num_stations, num_vertexes - 1)
            visit[source][to] = visit[to][source] = True
            dist = random.randint(1, 1000)
            f.write(f'{source} {to} {dist}\n')


def generate_random_data(dir):
    os.makedirs(dir, exist_ok=True)

    def work(idx, N, C, PS):
        filename = f'{dir}/test{idx:02d}.case'
        print(f'Generating test {idx} to {filename} with N={N}')
        generate_in(filename, N, C, PS)

    test_id = 1
    for i in range(10):
        work(test_id, N=random.randint(2, 10), C=1, PS=10000)
        test_id += 1
    for i in range(8):
        work(test_id, N=random.randint(10, 200), C=1, PS=10000)
        test_id += 1
    for i in range(4):
        work(test_id, N=random.randint(200, 2000), C=1, PS=10000)
        test_id += 1
    for i in range(2):
        work(test_id, N=random.randint(2000, 5000), C=1, PS=10000)
        test_id += 1

    work(test_id, N=5000, C=1, PS=10000)


if __name__ == '__main__':
    data_dir = 'cases25'

    # generate_in(f'{data_dir}/test_sp01.case', num_vertexes=5000, C=1, PS=10000, num_stations=2500, num_edges=2500)

    generate_random_data(data_dir)
