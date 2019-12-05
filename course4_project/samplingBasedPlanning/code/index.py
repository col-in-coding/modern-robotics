import pandas as pd
import numpy as np
import rrt


def main():
    obstacle_list = pd.read_csv(
        '../results/obstacles.csv', sep=',', comment='#',
        dtype={'x': 'float', 'y': 'float', 'diameter': 'float'},
        header=None, names=['x', 'y', 'd']
    ).sort_index()
    obstacle_list = obstacle_list.to_numpy()
    # initiate the sampler
    sampler = rrt.RrtSampling(12, obstacle_list, 0.03)

    # maximum try time
    n = 20
    while n > 0:
        res = sampler.run(100)
        print('success: ', res['success'])
        if res['success']:
            nodes = np.array(res['nodes'])
            edges = np.array(res['edges'])
            path = np.array(res['path']).transpose()
            print(path)
            np.savetxt(fname='../results/nodes.csv', fmt="%d,%.4f,%.4f,%.4f", delimiter=',', newline='\n', X=nodes)
            np.savetxt(fname='../results/edges.csv', fmt="%d,%d,%.4f", delimiter=',', newline='\n', X=edges)
            np.savetxt(fname='../results/path.csv', fmt="%d", delimiter=',', newline=",", X=path)
            n = 0
        n -= 1


if __name__ == '__main__':
    main()
