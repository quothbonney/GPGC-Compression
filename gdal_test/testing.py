b = []
S = 8

x0 = []
y0 = []
Mem = [1,2,2,2,2,1,1]


if __name__ == '__main__':
    b.append(0)
    x0.append(0)
    y0.append(0)
    Index = 0
    while Index < len(Mem):
        while b[Index] < Mem[Index]:
            b[Index] = b[Index]+1
            for i in range(3):
                b.insert(Index+1, b[Index])
            x0.insert(Index + 1, x0[Index] + (1 / (2 ** b[Index])))
            x0.insert(Index + 2, x0[Index])
            x0.insert(Index + 3, x0[Index] + (1 / (2 ** b[Index])))

            y0.insert(Index + 1, y0[Index])
            y0.insert(Index + 2, y0[Index] + (1 / (2 ** b[Index])))
            y0.insert(Index + 3, y0[Index] + (1 / (2 ** b[Index])))
        Index += 1

    print(x0)
    print(y0)


