import numpy as np
from haversine import haversine


missions = 20
max_time = 24

def extract(link, step):
    rows = []
    unique_vals = []
    with open('saved_csvs/test_15.csv', 'r') as csv_file:
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

    #print(rows)
    
    return rows, unique_vals


def haversine_distance(s_1, s_2, d_1, d_2, p_1, p_2):
    return haversine([s_1, s_2], [p_1, p_2]) + haversine([p_1, p_2], [d_1, d_2])


def generate_distances(vehicle_limit, rows, bases, plane_only_mission, helicopter_speed, plane_speed):
    print(bases)

    # initialize distance
    distances = np.full((plane_only_mission[1] + 1, plane_only_mission[1] + 1, 2), -1, dtype="float")
    original = np.full((plane_only_mission[1] + 1, plane_only_mission[1] + 1, 2), -1, dtype="float")
    bases = np.delete(bases, 2, 1)
    rows = np.delete(rows, [4, 5, 6, 7], 1)
    
    temp = np.concatenate([np.concatenate([bases, bases], axis=1), rows])
    
    # generate distances and convert to time matrix
    for i in range(len(distances)):
        for j in range(len(distances[i])):
            if i < vehicle_limit and j < vehicle_limit:
                continue
                
            dis = haversine_distance(temp[i, 0], temp[i, 1], temp[j, 0], temp[j, 1], temp[j, 2], temp[j, 3])
            original[i,j,0] = dis
            original[i,j,1] = dis
            distances[i,j,0] = dis / helicopter_speed
            distances[i,j,1] = dis / plane_speed
    return distances, original

#####################################################################################################################################################################
#####################################################################################################################################################################
#####################################################################################################################################################################

def generate_initial(solution, tracker, windows, distances, hours_allowed, plane_limit, helicopter_only_mission, plane_only_mission, small_heli, small_any):
    n = 0
    global_min = np.zeros(4)
    count = 0

    while True:
        ###################################################################################################################
        # attempt to find a better organization for an improved solution
        ###################################################################################################################
        if count >= plane_limit:
            current_tracker = sum(tracker)
            for i in range(len(solution)):
                for j in range(len(solution)):
                    if solution[i, j] == -1:
                        solution[i, j] = i
                        break
            # try 10 times
            attempt = 10
            while True:
                if attempt == 0:
                    raise Exception('Solution may not be possible!')
                solution, tracker = fix(solution, tracker, windows, distances, hours_allowed, plane_limit, helicopter_only_mission, helicopter_only_mission[0])
                if current_tracker <= sum(tracker):
                    attempt -= 1
                else:
                    break
            count = 0
            for i in range(len(solution)):
                for j in range(1, len(solution[0])):
                    if solution[i, j] == i:
                        solution[i, j] = -1
                        break
        ###################################################################################################################

        # assign helicopter missions first
        for i in range(plane_limit, helicopter_only_mission[0]):
            # extract the smallest index
            min_index = small_heli[n]

            # -1 index value
            limit = 0
            for x in solution[i]:
                if x == -1:
                    break
                limit += 1

            # ensure it does not pass the window
            if limit == 1:
                if windows[i] < distances[min_index, i, 0] + windows[min_index]:
                    count += 1
                    continue
                if windows[min_index] < distances[solution[i, 0], min_index, 0]:
                    count += 1
                    continue
            else:
                if windows[i] < distances[min_index, i, 0] + windows[min_index]:
                    count += 1
                    continue
                if windows[min_index] < distances[solution[i, limit - 1], min_index, 0] + windows[solution[i, limit - 1]]:
                    count += 1
                    continue

            # ensure it does not pass the time for flight
            if hours_allowed < (distances[solution[i, limit - 1], min_index, 0] + tracker[i]) or hours_allowed < (distances[solution[i, limit - 1], min_index, 0] + tracker[i] + distances[min_index, i, 0]):
                count += 1
                continue

            # figure out smallest and add smallest to it
            if global_min[0] == 0 and global_min[1] == 0:
                count = 0
                global_min[0] = min_index
                global_min[1] = i
                global_min[2] = distances[solution[i, limit - 1], min_index, 0] + tracker[i] + distances[min_index, i, 0]
                global_min[3] = limit
            else:
                if global_min[2] > distances[solution[i, limit - 1], min_index, 0] + tracker[i] + distances[min_index, i, 0]:
                    count = 0
                    global_min[0] = min_index
                    global_min[1] = i
                    global_min[2] = distances[solution[i, limit - 1], min_index, 0] + tracker[i] + distances[min_index, i, 0]
                    global_min[3] = limit

        # iterate n to the next and add value to solution
        if global_min[2] != 0:
            count = 0
            min_index = int(global_min[0])
            i = int(global_min[1])
            tracker[i] = global_min[2]
            n += 1
            limit = int(global_min[3])
            solution[i, limit] = min_index

            global_min[0] = 0
            global_min[1] = 0
            global_min[2] = 0
            global_min[3] = 0

        # perform check for 12-39 to finish heli assignment
        end_flag = True
        for x in range(helicopter_only_mission[0], helicopter_only_mission[1] + 1):
            if x not in solution:
                end_flag = False
                break
        if end_flag == True:
            break

    ###################################################################################################################
    ###################################################################################################################
    n = 0
    count = 0
    global_min = np.zeros(4)

    while True:
        ###################################################################################################################
        # attempt to find a better organization for an improved solution
        ###################################################################################################################
        if count >= helicopter_only_mission[0]:
            current_tracker = sum(tracker)
            for i in range(len(solution)):
                for j in range(len(solution)):
                    if solution[i, j] == -1:
                        solution[i, j] = i
                        break

            # try 10 times
            attempt = 10
            while True:
                if attempt == 0:
                    raise Exception('Solution may not be possible!')
                solution, tracker = fix(solution, tracker, windows, distances, hours_allowed, plane_limit, helicopter_only_mission, helicopter_only_mission[0])
                if current_tracker <= sum(tracker):
                    attempt -= 1
                else:
                    break
            count = 0
            for i in range(len(solution)):
                for j in range(1, len(solution[0])):
                    if solution[i, j] == i:
                        solution[i, j] = -1
                        break
            o = 2
        ###################################################################################################################

        # assign helicopter missions first
        for i in range(helicopter_only_mission[0]):
            # extract the smallest index
            min_index = small_any[n]

            type = 0
            if i < plane_limit:
                type = 1

            # -1 index value
            limit = 0
            for x in solution[i]:
                if x == -1:
                    break
                limit += 1

            # ensure it does not pass the window
            if limit == 1:
                if windows[i] < distances[min_index, i, type] + windows[min_index]:
                    q = windows[i]
                    s = windows[min_index]
                    t = distances[min_index, i, type]
                    count += 1
                    continue
                if windows[min_index] < distances[i, min_index, type]:
                    q = windows[min_index]
                    s = distances[i, min_index, type]
                    count += 1
                    continue
            else:
                if windows[i] < distances[min_index, i, type] + windows[min_index]:
                    q = windows[i]
                    s = windows[min_index]
                    t = distances[min_index, i, type]
                    count += 1
                    continue
                if windows[min_index] < distances[solution[i, limit - 1], min_index, type] + windows[solution[i, limit - 1]]:
                    q = windows[min_index]
                    s = windows[solution[i, limit - 1]]
                    t = distances[solution[i, limit - 1], min_index, type]
                    count += 1
                    continue

            # ensure it does not pass the time for flight
            if hours_allowed < (distances[solution[i, limit - 1], min_index, type] + tracker[i]) or hours_allowed < (distances[solution[i, limit - 1], min_index, type] + tracker[i] + distances[min_index, i, type]):
                count += 1
                continue

            # figure out smallest and add smallest to it
            if global_min[0] == 0 and global_min[1] == 0:
                count = 0
                global_min[0] = min_index
                global_min[1] = i
                global_min[2] = distances[solution[i, limit - 1], min_index, type] + tracker[i] + distances[min_index, i, type]
                global_min[3] = limit
            else:
                if global_min[2] > distances[solution[i, limit - 1], min_index, type] + tracker[i] + distances[min_index, i, type]:
                    count = 0
                    global_min[0] = min_index
                    global_min[1] = i
                    global_min[2] = distances[solution[i, limit - 1], min_index, type] + tracker[i] + distances[min_index, i, type]
                    global_min[3] = limit

        # iterate n to the next and add value to solution
        if global_min[2] != 0:
            count = 0
            min_index = int(global_min[0])
            i = int(global_min[1])
            tracker[i] = global_min[2]
            n += 1
            limit = int(global_min[3])
            solution[i, limit] = min_index

            global_min[0] = 0
            global_min[1] = 0
            global_min[2] = 0
            global_min[3] = 0

        # perform check for 40-56 to finish heli assignment
        end_flag = True
        for x in range(plane_only_mission[0], plane_only_mission[1] + 1):
            if x not in solution:
                end_flag = False
                break
        if end_flag == True:
            break

    ###################################################################################################################
    ###################################################################################################################

    for i in range(len(solution)):
        for j in range(len(solution)):
            if solution[i, j] == -1:
                solution[i, j] = i
                break

    return solution, tracker
#####################################################################################################################################################################
#####################################################################################################################################################################
#####################################################################################################################################################################
#####################################################################################################################################################################
#####################################################################################################################################################################


def fix(solution, tracker, windows, distances, hours_allowed, plane_limit, helicopter_only_mission, vehicle_limit):
    i_permutation = np.random.permutation(solution.shape[0])
    j_permutation = np.random.permutation(solution.shape[1])
    for i in i_permutation:
        for j in j_permutation:
            # skip -1 or solutions that are from 0-11
            if solution[i,j] == -1 or solution[i,j] < vehicle_limit:
                continue
            # current i vehicle distance value
            type_i = 0
            if type_i < plane_limit:
                type_i = 1
            i_distance_original = tracker[i]
            i_distance_updated = (tracker[i] - distances[solution[i, j - 1], solution[i, j], type_i] - distances[solution[i, j], solution[i, j + 1], type_i]) + distances[solution[i, j - 1], solution[i, j + 1], type_i]
            if i_distance_updated < 0:
                i_distance_updated = 0

            # improvement flag and value array for update
            improvement = False
            values = np.zeros(3)

            for k in range(len(solution)):
                # skip if k and i are the same array
                if k == i:
                    continue
                for l in range(len(solution[k])):
                    # skip -1 or solutions that are from 0-11
                    if l == 0 or solution[k, l] == -1:
                        continue

                    # set vehicle type
                    type_k = 0
                    if type_k < plane_limit:
                        type_k = 1

                    # validate that the movement is compatible, skip if not (plane can't do helicopter mission)
                    if k < plane_limit and (solution[i, j] >= helicopter_only_mission[0] and solution[i, j] <= helicopter_only_mission[1]):
                        continue

                    # check if it actually reduces the time
                    k_distance_original = tracker[k]
                    k_distance_updated = (tracker[k] - distances[solution[k, l - 1], solution[k, l], type_k]) + (distances[solution[k, l - 1], solution[i, j], type_k] + distances[solution[i, j], solution[k, l], type_k])
                    if k_distance_updated < 0:
                        k_distance_updated = 0

                    if (i_distance_original + k_distance_original) < (i_distance_updated + k_distance_updated):
                        continue

                    # check if distance goes over max allowed time for fligtht
                    if k_distance_updated > hours_allowed:
                        continue

                    ##############################################################################################################################
                    # perform time validation check (ensure we don't go over the time window)
                    flag = True
                    for m in range(len(solution[k])):
                        if solution[k, m + 1] == -1:
                            break
                        if m + 1 == l:
                            if m == 0:
                                if windows[solution[i, j]] < distances[solution[k, m], solution[i, j], type_k]:
                                    flag = False
                                    break
                                if windows[solution[k, m + 1]] < windows[solution[i, j]] + distances[solution[i, j], solution[k, m + 1], type_k]:
                                    flag = False
                                    break
                                continue
                            else:
                                if windows[solution[i, j]] < windows[solution[k, m]] + distances[solution[k, m], solution[i, j], type_k]:
                                    flag = False
                                    break
                                if windows[solution[k, m + 1]] < windows[solution[i, j]] + distances[solution[i, j], solution[k, m + 1], type_k]:
                                    flag = False
                                    break
                                continue
                        if m == 0:
                            if windows[solution[k, m + 1]] < distances[solution[k, m], solution[k, m + 1], type_k]:
                                flag = False
                                break
                        else:
                            if windows[solution[k, m + 1]] < windows[solution[k, m]] + distances[solution[k, m], solution[k, m + 1], type_k]:
                                flag = False
                                break
                    if flag == False:
                        continue

                    # check if improvement flag is and either update or validate which is better
                    if improvement == True and (i_distance_updated + k_distance_updated > i_distance_updated + values[0]):
                        continue
                    else:
                        improvement = True
                        values[0] = k_distance_updated
                        values[1] = k
                        values[2] = l
                    ##############################################################################################################################


            # check if there was an improvement and perform the swap
            if improvement == True:
                # update tracker
                tracker[i] = i_distance_updated
                tracker[int(values[1])] = values[0]
                k = int(values[1])
                l = int(values[2])

                len_i = j
                len_k = 0
                for m in range(len(solution[i])):
                    if solution[k, m] != -1:
                        len_k += 1
                # update k
                while len_k != l - 1:
                    solution[k, len_k] = solution[k, len_k - 1]
                    len_k -= 1
                solution[k, l] = solution[i, j]

                # update i
                while solution[i, len_i] != -1:
                    solution[i, len_i] = solution[i, len_i + 1]
                    len_i += 1

    return solution, tracker

#####################################################################################################################################################################
#####################################################################################################################################################################
#####################################################################################################################################################################
#####################################################################################################################################################################
#####################################################################################################################################################################

def update(solution, tracker, windows, distances, hours_allowed, plane_limit, helicopter_only_mission, vehicle_limit, i_permutation, j_permutation, tabu, tabu_len):
    for i in i_permutation:
        for j in j_permutation:
            #####################################################################################################################################################################
            # skip if in tabu list
            #####################################################################################################################################################################
            if tabu[i, j] > 0:
                continue
            #####################################################################################################################################################################

            # skip -1 or solutions that are from 0-11
            if solution[i,j] == -1 or solution[i,j] < vehicle_limit:
                continue
            # current i vehicle distance value
            type_i = 0
            if type_i < plane_limit:
                type_i = 1
            i_distance_original = tracker[i]
            i_distance_updated = (tracker[i] - distances[solution[i, j - 1], solution[i, j], type_i] - distances[solution[i, j], solution[i, j + 1], type_i]) + distances[solution[i, j - 1], solution[i, j + 1], type_i]
            if i_distance_updated < 0:
                i_distance_updated = 0

            # improvement flag and value array for update
            improvement = False
            values = np.zeros(3)

            for k in range(len(solution)):
                # skip if k and i are the same array
                if k == i:
                    continue
                for l in range(len(solution[k])):
                    #####################################################################################################################################################################
                    # skip if in tabu list
                    #####################################################################################################################################################################
                    if tabu[k, l] > 0:
                        continue
                    #####################################################################################################################################################################

                    # skip -1 or solutions that are from 0-11
                    if l == 0 or solution[k, l] == -1:
                        continue

                    # set vehicle type
                    type_k = 0
                    if type_k < plane_limit:
                        type_k = 1

                    # validate that the movement is compatible, skip if not (plane can't do helicopter mission)
                    if k < plane_limit and (solution[i, j] >= helicopter_only_mission[0] and solution[i, j] <= helicopter_only_mission[1]):
                        continue

                    # check if it actually reduces the time
                    k_distance_original = tracker[k]
                    k_distance_updated = (tracker[k] - distances[solution[k, l - 1], solution[k, l], type_k]) + (distances[solution[k, l - 1], solution[i, j], type_k] + distances[solution[i, j], solution[k, l], type_k])
                    if k_distance_updated < 0:
                        k_distance_updated = 0

                    if (i_distance_original + k_distance_original) < (i_distance_updated + k_distance_updated):
                        continue

                    # check if distance goes over max allowed time for fligtht
                    if k_distance_updated > hours_allowed:
                        continue

                    ##############################################################################################################################
                    # perform time validation check (ensure we don't go over the time window)
                    flag = True
                    for m in range(len(solution[k])):
                        if solution[k, m + 1] == -1:
                            break
                        if m + 1 == l:
                            if m == 0:
                                if windows[solution[i, j]] < distances[solution[k, m], solution[i, j], type_k]:
                                    flag = False
                                    break
                                if windows[solution[k, m + 1]] < windows[solution[i, j]] + distances[solution[i, j], solution[k, m + 1], type_k]:
                                    flag = False
                                    break
                                continue
                            else:
                                if windows[solution[i, j]] < windows[solution[k, m]] + distances[solution[k, m], solution[i, j], type_k]:
                                    flag = False
                                    break
                                if windows[solution[k, m + 1]] < windows[solution[i, j]] + distances[solution[i, j], solution[k, m + 1], type_k]:
                                    flag = False
                                    break
                                continue
                        if m == 0:
                            if windows[solution[k, m + 1]] < distances[solution[k, m], solution[k, m + 1], type_k]:
                                flag = False
                                break
                        else:
                            if windows[solution[k, m + 1]] < windows[solution[k, m]] + distances[solution[k, m], solution[k, m + 1], type_k]:
                                flag = False
                                break
                    if flag == False:
                        continue

                    # check if improvement flag is and either update or validate which is better
                    if improvement == True and (i_distance_updated + k_distance_updated > i_distance_updated + values[0]):
                        continue
                    else:
                        improvement = True
                        values[0] = k_distance_updated
                        values[1] = k
                        values[2] = l
                    ##############################################################################################################################

            # check if there was an improvement and perform the swap
            if improvement == True:
                # add to tabu list
                tabu[i, j] = tabu_len
                tabu[k, l] = tabu_len

                # update tracker
                tracker[i] = i_distance_updated
                tracker[int(values[1])] = values[0]
                k = int(values[1])
                l = int(values[2])

                len_i = j
                len_k = 0
                for m in range(len(solution[i])):
                    if solution[k, m] != -1:
                        len_k += 1
                # update k
                while len_k != l - 1:
                    solution[k, len_k] = solution[k, len_k - 1]
                    len_k -= 1
                solution[k, l] = solution[i, j]

                # update i
                while solution[i, len_i] != -1:
                    solution[i, len_i] = solution[i, len_i + 1]
                    len_i += 1

        # decrease the tabu size
        for x in range(tabu.shape[0]):
            for y in range(tabu.shape[1]):
                if tabu[x, y] > 0:
                    tabu[x, y] -= 1

    return solution, tracker, tabu

#####################################################################################################################################################################
#####################################################################################################################################################################
#####################################################################################################################################################################

# extract csv data (D1, D2, P1, P2, V1, V2, Base_Type, Mission_Type, Limit)
samples = 180
rows, bases = extract('TSP_CASES/test.csv', int(samples/missions))

# variables for calculations

plane_limit = 4
hours_allowed = 10
vehicle_limit = 12
base_range = [0, vehicle_limit - 1]
helicopter_only_mission = [vehicle_limit, np.sum(rows[:, 7] == 6) + vehicle_limit - 1]
plane_only_mission = [np.sum(rows[:, 7] == 6) + vehicle_limit, np.sum(rows[:, 7] == 7) + np.sum(rows[:, 7] == 6) + vehicle_limit -1]
helicopter_speed = 300
plane_speed = 500

# sort the bases by which have a plane, sort missions by helicopter only, generate time windows
rows = rows[rows[:,7].argsort()]
bases = bases[bases[:,2].argsort()]
bases = np.flip(bases, 0)
windows = np.array(rows[:, 8])

windows = np.concatenate([np.full(vehicle_limit, max_time), np.array(windows)])
rows = np.delete(rows, 8, 1)

print(windows)
print(base_range)
print(helicopter_only_mission)
print(plane_only_mission)

# generate distance matrices
distances, original = generate_distances(vehicle_limit, rows, bases, plane_only_mission, helicopter_speed, plane_speed)

# initialize time track and solution matrix
tracker = np.zeros(12)
solution = np.full((vehicle_limit, plane_only_mission[1]), -1)

for i in range(vehicle_limit):
    solution[i, 0] = i

# smallest indices for helicopter missions
small_heli = np.argsort(windows[helicopter_only_mission[0]: helicopter_only_mission[1] + 1]) + vehicle_limit
small_any = np.argsort(windows[plane_only_mission[0]: plane_only_mission[1] + 1]) + (helicopter_only_mission[1] + 1)


# # Generate an initial greedy solution
solution, tracker = generate_initial(solution, tracker, windows, distances, hours_allowed, plane_limit, helicopter_only_mission, plane_only_mission, small_heli, small_any)
print("Initial: {}".format(sum(tracker)))

#####################################################################################################################################################################
#####################################################################################################################################################################
#####################################################################################################################################################################

count = 0
current_best = sum(tracker)
tabu = np.zeros((solution.shape[0], solution.shape[1]))
tabu_len = 30
while True:
    count += 1
    i_permutation = np.random.permutation(solution.shape[0])
    j_permutation = np.random.permutation(solution.shape[1])
    solution, tracker, tabu = update(solution, tracker, windows, distances, hours_allowed, plane_limit, helicopter_only_mission, vehicle_limit, i_permutation, j_permutation, tabu, tabu_len)
    if current_best > sum(tracker):
        current_best = sum(tracker)
    else:
        break
    print("Improvement {}: {}".format(count, sum(tracker)))


#####################################################################################################################################################################
#####################################################################################################################################################################
#####################################################################################################################################################################
