journey = [[ 0.,      -6.     ],[ 2.90625, -2.03125],[ 6.15625, -1.46875],[7.21875,  2.46875],[ 7.90625,  5.96875],[ 8., 8.]]

total = 0
for i in range(len(journey)):
    if (i != 0):
        x_diff = (journey[i][0]-journey[i-1][0])
        y_diff = (journey[i][1]-journey[i-1][1])
        total += pow(pow(x_diff,2)+pow(y_diff,2),(1/2))
    # print(total)

# print(pow(pow(20,2)+pow(20,2),(1/2)))

james = 'james'=='james'

print(james)



