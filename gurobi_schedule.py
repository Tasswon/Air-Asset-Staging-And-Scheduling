from gurobipy import *
import csv
import time
import numpy as np
from haversine import haversine
import statistics
import random


def extract(link, step):
    rows = []
    unique_vals = []
    with open('TSP_CASES/test.csv', 'r') as csv_file:
        for row in csv_file:
            row = row.split(',')
            if 'D1' not in row:
                rows.append([float(i) for i in row])
                unique_vals.append([float(row[i]) for i in range(len(row) - 5, len(row) - 2)])
        csv_file.close()
    rows = np.array(rows)
    unique_vals = np.unique(np.array(unique_vals), axis=0)

    # allow only so many missions
    rows = np.array(rows[1::step])

    return rows, unique_vals


def haversine_distance(s_1, s_2, d_1, d_2, p_1, p_2):
    return haversine([s_1, s_2], [p_1, p_2]) + haversine([p_1, p_2], [d_1, d_2])


def generate_distances(vehicle_limit, rows, bases, plane_only_mission, helicopter_speed, plane_speed):
    # initialize distance
    distances = np.full((plane_only_mission[1] + 1, plane_only_mission[1] + 1, 2), 0, dtype="float")
    bases = np.delete(bases, 2, 1)
    rows = np.delete(rows, [4, 5, 6, 7], 1)

    temp = np.concatenate([np.concatenate([bases, bases], axis=1), rows])

    # generate distances and convert to time matrix
    for i in range(len(distances)):
        for j in range(len(distances[i])):
            if i < vehicle_limit and j < vehicle_limit:
                continue

            dis = haversine_distance(temp[i, 0], temp[i, 1], temp[j, 0], temp[j, 1], temp[j, 2], temp[j, 3])
            distances[i, j, 0] = dis / helicopter_speed
            distances[i, j, 1] = dis / plane_speed
    return distances


#####################################################################################################################################################################
#####################################################################################################################################################################
#####################################################################################################################################################################


# extract csv data (D1, D2, P1, P2, V1, V2, Base_Type, Mission_Type, Limit)
missions = 30
samples = 180
rows, bases = extract('TSP_CASES/test.csv', int(samples / missions))

# variables for calculations

plane_limit = 4
hours_allowed = 10
vehicle_limit = 12
base_range = [0, vehicle_limit - 1]
helicopter_only_mission = [vehicle_limit, np.sum(rows[:, 7] == 6) + vehicle_limit - 1]
plane_only_mission = [np.sum(rows[:, 7] == 6) + vehicle_limit,
                      np.sum(rows[:, 7] == 7) + np.sum(rows[:, 7] == 6) + vehicle_limit - 1]
helicopter_speed = 300
plane_speed = 500

# sort the bases by which have a plane, sort missions by helicopter only, generate time windows
rows = rows[rows[:, 7].argsort()]
bases = bases[bases[:, 2].argsort()]
bases = np.flip(bases, 0)
windows = np.array(rows[:, 8])

windows = np.concatenate([np.full(vehicle_limit, 24), np.array(windows)])
rows = np.delete(rows, 8, 1)

# generate distance matrices
distances = generate_distances(vehicle_limit, rows, bases, plane_only_mission, helicopter_speed, plane_speed)

# initialize time track and solution matrix
tracker = np.zeros(12)
time_track = np.zeros(12)


#######################################################################################################################
#######################################################################################################################
#######################################################################################################################
# model properties
#######################################################################################################################
#######################################################################################################################
######################################################################################################################

mod = Model("schedule")
mod.setParam('TimeLimit', 60*60*4)

num_missions = 42
num_bases = 12
options = 2
p = 10

# whether a helicopter (0) or plane (1) is at a base
base_type = []
for i in range(len(bases)):
    if bases[i, 2] == 3 or bases[i, 2] == 4:
        base_type.append(0)
    else:
        base_type.append(1)

# states the type of mission
mission_type = []
for i in range(12):
    mission_type.append(0)

for i in range(len(rows)):
    if rows[i,7] == 6:
        mission_type.append(0)
    else:
        mission_type.append(1)

# Add variables
x = {} # x_ijk
u = {} # u_i

# distance matrix
d = {} # d_ijl

# instantiate the distance matrix
for i in range(num_missions):
    for j in range(num_missions):
        for l in range(options):
            d[(i, j, l)] = distances[i, j, l]

# declare x variable
for i in range(num_missions):
    for j in range(num_missions):
        for k in range(num_bases):
            x[(i, j, k)] = mod.addVar(vtype=GRB.BINARY, name="x_{}_{}_{}".format(i, j, k))

# declare u variable
for i in range(num_missions):
    u[(i)] = mod.addVar(vtype=GRB.INTEGER, name="u_{}".format(i))

mod.update()

################################################+#######################################################################
# add constraints
#######################################################################################################################
# each pickup is completed exactly once
for j in range(num_missions):
    mod.addConstr(quicksum(quicksum(x[(i, j, k)] for k in range(num_bases)) for i in range(num_missions)) == 1)

# each pickup is completed exactly once
for i in range(num_missions):
    mod.addConstr(quicksum(quicksum(x[(i, j, k)] for k in range(num_bases)) for j in range(num_missions)) == 1)
########################################################################################################################################################
# mirrored i and j
for n in range(num_missions):
    for k in range(num_bases):
        mod.addConstr(quicksum(x[(i, n, k)] for i in range(num_missions)) == quicksum(x[(n, j, k)] for j in range(num_missions)))
########################################################################################################################################################
# subtour eliminatation
for i in range(num_missions):
    for j in range(num_missions):
        for k in range(num_bases):
            if i != j:
                mod.addConstr((u[(i)] - u[(j)] + (num_missions - 1) * x[(i, j, k)]) <= (num_missions - 1) - 1)
########################################################################################################################################################
# each pickup is completed exactly once
for j in range(num_bases):
    for k in range(num_bases):
        mod.addConstr(quicksum(x[(i, j, k)] for i in range(num_missions)) <= 1)

# each pickup is completed exactly once
for i in range(num_bases):
    for k in range(num_bases):
        mod.addConstr(quicksum(x[(i, j, k)] for j in range(num_missions)) <= 1)
########################################################################################################################################################
for i in range(num_bases):
    for j in range(num_bases):
        for k in range(num_bases):
            mod.addConstr(x[(i, j, k)] == 0)
########################################################################################################################################################
# mission and base compatibility
for i in range(num_missions):
    for j in range(num_missions):
        for k in range(num_bases):
            mod.addConstr((x[(i, j, k)] * (base_type[k]) - mission_type[i]) <= 0)
########################################################################################################################################################
# how far pilots can travel in a day
for k in range(num_bases):
    mod.addConstr(quicksum(quicksum(x[(i, j, k)] * d[(i, j, base_type[k])] for i in range(num_missions)) for j in range(num_missions)) <= p)
########################################################################################################################################################
# time must be less than current from i to j
for i in range(num_missions):
    for j in range(num_missions):
        for k in range(num_bases):
            if i >= num_bases:
                mod.addConstr((x[(i, j, k)] * (d[(i, j, base_type[k])] + windows[i])) <= x[(i, j, k)] * windows[j])
########################################################################################################################################################
# time must be less than current from i to j
for i in range(num_missions):
    for j in range(num_missions):
        for k in range(num_bases):
            if i < num_bases:
                mod.addConstr(x[(i, j, k)] * d[(i, j, base_type[k])] <= x[(i, j, k)] * windows[j])
########################################################################################################################################################
"""
########################################################################################################################################################
# mirrored i and j
for n in range(num_missions):
    for k in range(num_bases):
        mod.addConstr(quicksum(x[(i, n, k)] for i in range(num_missions)) == quicksum(x[(n, j, k)] for j in range(num_missions)))
########################################################################################################################################################
# mission and base compatibility
for i in range(num_missions):
    for j in range(num_missions):
        for k in range(num_bases):
            mod.addConstr((x[(i, j, k)] * (base_type[k]) - mission_type[i]) <= 0)
########################################################################################################################################################
# how far pilots can travel in a day
for k in range(num_bases):
    mod.addConstr(quicksum(quicksum(x[(i, j, k)] * d[(i, j, base_type[k])] for i in range(num_missions)) for j in range(num_missions)) <= p)
########################################################################################################################################################
# time must be less than current from i to j
for i in range(num_missions):
    for j in range(num_missions):
        for k in range(num_bases):
            if i != j:
                mod.addConstr((x[(i, j, k)] * (d[(i, j, base_type[k])] + windows[i])) <= windows[j])

for i in range(num_bases):
    for j in range(num_bases):
        for k in range(num_bases):
            mod.addConstr(x[(i, j, k)] == 0)

########################################################################################################################################################


########################################################################################################################################################


# each will leave and return to its home depot at most once
# each will leave and return to its home depot at most once
for k in range(num_bases):
    mod.addConstr(quicksum(quicksum(x[(i, j, k)] for i in range(num_missions)) for j in range(num_missions)) <= 1)

########################################################################################################################################################
########################################################################################################################################################
########################################################################################################################################################
# can't have a depot going to another depot
for i in range(num_bases):
    for j in range(num_bases):
        for k in range(num_bases):
            mod.addConstr(x[(i, j, k)] == 0)

########################################################################################################################################################
# time must be less than current from i to j
#for i in range(num_missions):
#    for j in range(num_missions):
#        for k in range(num_bases):
#            if i != j:
#                mod.addConstr((x[(i, j, k)] * (d[(i, j)] + windows[i])) <= windows[j])


########################################################################################################################################################


print()
print(windows)
print()
"""
########################################################################################################################################################

########################################################################################################################################################
########################################################################################################################################################
########################################################################################################################################################
########################################################################################################################################################
########################################################################################################################################################
########################################################################################################################################################
########################################################################################################################################################

# set the objective function
mod.setObjective(quicksum(quicksum(quicksum((x[(i, j, k)] * d[(i, j, base_type[k])]) for k in range(num_bases)) for j in range(num_missions)) for i in range(num_missions)))

########################################################################################################################################################
mod.optimize()
print('\nObj: %g' % mod.objVal)
print('Runtime: %g' % mod.Runtime)

f = open('output.txt','w')

for v in mod.getVars():
    f.write('%s %g \n' % (v.varName, v.x))