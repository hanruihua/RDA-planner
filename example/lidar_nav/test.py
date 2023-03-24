def scan_convex(state, scan_data):
    ranges = np.array(scan_data['ranges'])
    angles = np.linspace(scan_data['angle_min'], scan_data['angle_max'], len(ranges))

    point_list = []
    obstacle_list = []

    for i in range(len(ranges)):
        scan_range = ranges[i]
        angle = angles[i]

        if scan_range < ( scan_data['range_max'] - 0.01):
            point = np.array([ [scan_range * np.cos(angle)], [scan_range * np.sin(angle)]  ])
            point_list.append(point)

    if len(point_list) < 4:
        return obstacle_list
    
    else:

        points = np.hstack(point_list).T

        hull = ConvexHull(points)

        hull_point_list = [point_list[v] for v in hull.vertices]

        if len(hull_point_list) == 4:
            vertices = np.hstack(hull_point_list)
            obstacle_list.append(obs(None, None, vertices, 'Rpositive', 0))
        else:
            x_min = np.min([p[0] for p in hull_point_list])
            x_max = np.max([p[0] for p in hull_point_list])
            y_min = np.min([p[1] for p in hull_point_list])
            y_max = np.max([p[1] for p in hull_point_list])

            vertices = np.array([[x_min, y_min], [x_min, y_max], [x_max, y_max], [x_max, y_min]])
    

        return []