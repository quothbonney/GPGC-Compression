import numpy as np
import matplotlib.pyplot as plt

full_matrix = np.empty([int(256), int(256)])
buffer = [0]
count = [0]
xOff, yOff = 0, 0

if __name__ == '__main__':
    with open('encoded.txt') as f:
        lines = f.readlines()

    sizes = []

    for line in lines:
        a = line.split(" ")
        i, j, k, n, xOff, yOff = [float(a[0]), float(a[1]), float(a[2]), int(a[3]), int(a[4]), int(a[5])]

        elem, row = 0, 0
        while row < n:
            while elem < n:
                full_matrix[row + yOff][elem + xOff] = i * row + j * elem + k
                elem += 1
            row += 1
            elem = 0

        sizes.append(n)
    print(sizes)


    plt.imshow(full_matrix)
    plt.colorbar()
    plt.show()
