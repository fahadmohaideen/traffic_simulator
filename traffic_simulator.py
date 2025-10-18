
import osmnx as ox
import numpy as np
import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication
import sys
import networkx as nx
from PyQt5.QtCore import QTimer
from scipy.interpolate import interp1d
from geopy import distance
import time
import threading
from PyQt5.QtCore import QObject, pyqtSignal
from PyQt5.QtCore import QThread
from scipy.interpolate import interp1d
import itertools
import matplotlib.pyplot as plt
import matplotlib
# matplotlib.use('Qt5Agg')
import queue
import math
from geopy import distance
import time
import cProfile
import pstats
from math import radians, cos
from geopy.distance import geodesic
from math import degrees
import multiprocessing
import threading
from matplotlib.animation import FuncAnimation
import io
import json
from json import JSONEncoder
import ast
from multiprocessing import Process, Pipe


class CarEncoder(JSONEncoder):
    def default(self, o):
        return o.__dict__


class CarProcess(multiprocessing.Process):
    def __init__(self, shared_dict, target=None, args=()):
        super().__init__(target=target, args=args)
        # self.queue = queue
        self.shared_dict = shared_dict


def normalize(v):
    norm = np.linalg.norm(v)
    return v / norm if norm != 0 else v


def perpendicular_vector(v):
    return np.array([-v[1], v[0]])


def translate_segment(segment, distance):
    start, end = segment
    segment_vector = np.array(end) - np.array(start)
    perp_vector = perpendicular_vector(segment_vector)
    translation_vector = normalize(perp_vector) * distance

    new_start = np.array(start) + translation_vector
    new_end = np.array(end) + translation_vector

    return new_start, new_end


def translate_polyline(polyline, offset_distance):
    # Assuming polyline is a list of (x, y) tuples
    translated_polyline = []
    for i, point in enumerate(polyline[:-1]):
        next_point = polyline[i + 1]
        print(next_point, point)
        direction_vector = np.array(next_point) - np.array(point)
        # Calculate a simple perpendicular vector (not normalized)
        perp_vector = np.array([-direction_vector[1], direction_vector[0]])
        # Determine the offset vector with the desired distance
        if np.linalg.norm(perp_vector) == 0:
            continue
        offset_vector = perp_vector * (offset_distance / np.linalg.norm(perp_vector))
        # Apply the offset
        translated_polyline.append((np.array(point) + offset_vector).tolist())

    # Add the last point with the same offset as the second to last point
    translated_polyline.append((np.array(polyline[-1]) + offset_vector).tolist())
    return np.array(translated_polyline)


def translate_polyline_optimized(polyline, offset_distance):
    points = np.array(polyline)
    # Assuming 'points' is your 2D array of coordinates
    _, indices = np.unique(points, return_index=True, axis=0)
    unique_points = points[np.sort(indices)]
    directions = unique_points[1:] - unique_points[:-1]
    perp_vectors = np.empty_like(directions)
    # perp_vectors = directions
    perp_vectors[:, 0] = -directions[:, 1]
    perp_vectors[:, 1] = directions[:, 0]
    norms = np.linalg.norm(perp_vectors, axis=1)
    valid_indices = norms != 0  # This creates a boolean array, not an array of values
    perp_vectors[valid_indices] /= norms[valid_indices][:, np.newaxis]  # Correct broadcasting
    offset_vectors = perp_vectors * offset_distance
    filtered_offset_vectors = offset_vectors[
        (offset_vectors[:, 0] != 0.00000000) | (offset_vectors[:, 1] != 0.00000000)]
    # print(filtered_offset_vectors)
    translated_points = unique_points[:-1] + filtered_offset_vectors
    translated_polyline = np.vstack([translated_points, unique_points[-1] + filtered_offset_vectors[-1]])
    return translated_polyline


def path_interp1d(g, edge):
    node_coords = ox.graph_to_gdfs(g, nodes=False, edges=True)['geometry'].to_dict()
    edge_geometry = node_coords[(edge[0], edge[1], 0)]
    edge_coords = list(edge_geometry.coords)

    # Detect turning points (sharp angles > 30 degrees)
    turning_points = []
    for i in range(1, len(edge_coords) - 1):
        v1 = (edge_coords[i][0] - edge_coords[i - 1][0], edge_coords[i][1] - edge_coords[i - 1][1])
        v2 = (edge_coords[i + 1][0] - edge_coords[i][0], edge_coords[i + 1][1] - edge_coords[i][1])

        dot = v1[0] * v2[0] + v1[1] * v2[1]
        mag1, mag2 = np.sqrt(v1[0] ** 2 + v1[1] ** 2), np.sqrt(v2[0] ** 2 + v2[1] ** 2)

        if mag1 > 0 and mag2 > 0:
            angle = np.degrees(np.arccos(np.clip(dot / (mag1 * mag2), -1, 1)))
            if angle > 8:
                turning_points.append(i)

    # Process segments or whole edge
    segments = []
    if turning_points:
        points = [0] + turning_points + [len(edge_coords) - 1]
        segments = [edge_coords[points[i]:points[i + 1] + 1] for i in range(len(points) - 1)]
    else:
        segments = [edge_coords]

    # Interpolate each segment
    all_x, all_y = np.array([]), np.array([])
    for segment in segments:
        if len(segment) < 2:
            continue

        xs, ys = np.array(segment).T
        f = interp1d(xs, ys)

        x_segment = np.array([])
        for i in range(1, len(xs)):
            dist = distance.geodesic((ys[i - 1], xs[i - 1]), (ys[i], xs[i])).meters
            x_segment = np.append(x_segment, np.linspace(xs[i - 1], xs[i], max(2, int(dist))))

        all_x = np.append(all_x, x_segment)
        all_y = np.append(all_y, f(x_segment))

    return all_x, all_y


def edge_attributes(g):
    """Given a graph, create a dictionary of all edges and its attributes."""
    edge_dict = {}
    for edge in list(g.edges):
        edge_dict[str((edge[0], edge[1]))] = []
    return edge_dict


def dict_to_json(d, json_file):
    """Given a dictionary, convert it to a json file."""
    import json
    with open(json_file, 'w') as f:
        json.dump(d, f)


# create a function that allows to add car attributes to all edges in the json file

def add_car_attributes_to_edges_json(json_file, edge, car_dict, locks):
    """Given a json file, add car attributes to all edges in the json file."""
    import json
    with locks:
        with open(json_file, 'r') as f:
            data = json.load(f)
            data[str(edge)].append(car_dict)
        with open(json_file, 'w') as f:
            json.dump(data, f)


def update_specific_car_attributes_to_edges_json(json_file, edge, car_attributes, car_id, locks):
    """Given a json file, add specific car attributes to all edges in the json file."""
    import json
    with locks:
        with open(json_file, 'r') as f:
            print(f)
            data = json.load(f)
            car_dicts = data[str(edge)]
            for car_dict in car_dicts:
                if car_dict['id'] == car_id:  # assuming 'id' is the unique identifier
                    car_dict.update(car_attributes)
                    break
        with open(json_file, 'w') as f:
            json.dump(data, f)


def remove_car_attributes_from_edges_json(json_file, edge, car_id, locks):
    """Given a json file, remove car attributes from all edges in the json file."""
    import json
    with locks:
        with open(json_file, 'r') as f:
            data = json.load(f)
            car_dicts = data[str(edge)]
            for i, car_dict in enumerate(car_dicts):
                if car_dict['id'] == car_id:  # assuming 'id' is the unique identifier
                    del car_dicts[i]
                    break
        with open(json_file, 'w') as f:
            json.dump(data, f)


def shortest_path_plotter(car_dict, route, G, json_file, locks):
    """
    Generator function to manage a car's movement along a route.
    """
    nodes = route
    for i in range(len(nodes) - 1):
        current_node = nodes[i]
        next_node = nodes[i + 1]
        current_edge = (current_node, next_node)
        previous_edge = (nodes[i - 1], current_node) if i != 0 else None
        next_next_node = nodes[i + 2] if i + 2 < len(nodes) else None
        next_edge = (next_node, next_next_node) if next_next_node is not None else None
        car_dict['distance_travelled'] = 0.0
        car_dict['distance_to_next_node'] = None
        car_dict['previous_edge'] = str(previous_edge) if previous_edge is not None else None
        car_dict['current_node'] = str(current_node) if current_node is not None else None
        car_dict['current_edge'] = str(current_edge) if current_edge is not None else None
        car_dict['next_node'] = str(next_node) if next_node is not None else None
        car_dict['next_edge'] = str(next_edge) if next_edge is not None else None
        path = path_interp1d(G, current_edge)
        x_new, y_new = path
        car_dict['current_edge_x_coords'] = str(x_new) if x_new is not None else None
        car_dict['current_edge_y_coords'] = str(y_new) if y_new is not None else None
        add_car_attributes_to_edges_json(json_file, current_edge, car_dict, locks)
        car_dict['current_edge_x_coords'] = x_new
        car_dict['current_edge_y_coords'] = y_new
        yield x_new, y_new

def resample_array(arr, target_len):
    return np.interp(
        np.linspace(0, len(arr) - 1, target_len),
        np.arange(len(arr)),
        arr
    )

def ball_animation(car_dict, route, json_file, locks):
    global fig, ax
    # pr = cProfile.Profile()
    # pr.enable()
    shortest_path_graph = shortest_path_plotter(car_dict, route, G, json_file, locks)
    for x_new0, y_new0 in shortest_path_graph:
        car_dict['distance_travelled'] = 0.0
        car_dict['distance_to_next_node'] = None
        #print(car_dict)
        if car_dict['previous_edge'] is not None:
            remove_car_attributes_from_edges_json(json_file, car_dict['previous_edge'], car_dict['id'], locks)
        k = 0
        is_running = True
        is_going = True
        translated = False
        total_current_time = 0.0
        initial_speed = car_dict['speed']
        for x0 in range(1, len(x_new0)):
            x_new = car_dict['current_edge_x_coords']
            y_new = car_dict['current_edge_y_coords']
            k += 1
            # pr = cProfile.Profile()
            # pr.enable()
            start_time = time.time()
            with locks:
                with open(json_file, 'r') as f:
                    shared_dict = json.load(f)
                with open(json_file, 'w') as f:
                    json.dump(shared_dict, f)
            #print(car_dict['distance_to_next_node'])
            if (
                    car_dict['distance_to_next_node']
                    and car_dict['distance_to_next_node'] <= 35.0
                    and car_dict['next_edge'] is not None
            ):
                with locks:
                    with open(json_file, 'r') as f:
                        shared_dict = json.load(f)

                # Get nearby edges
                nearby_edges = list(G.edges(ast.literal_eval(car_dict['next_node'])))
                current_edge = ast.literal_eval(car_dict['current_edge'])
                if current_edge in nearby_edges:
                    nearby_edges.remove(current_edge)
                elif (current_edge[1], current_edge[0]) in nearby_edges:
                    nearby_edges.remove((current_edge[1], current_edge[0]))

                all_nearby_edges = nearby_edges + [(e[1], e[0]) for e in nearby_edges]
                car_should_stop = False

                for edges in all_nearby_edges:
                    edge = str(edges)
                    for other in shared_dict.get(edge, []):
                        if not other or other['id'] == car_dict['id']:
                            continue

                        # Skip invalid or stationary cars
                        if other.get('speed', 0.0) <= 0.0 or car_dict.get('speed', 0.0) <= 0.0:
                            continue
                        if not all(
                                k in other and other[k] is not None
                                for k in ['speed', 'distance_to_next_node', 'next_node']
                        ):
                            continue

                        # Compute ETA (time to reach next node)
                        other_t = other['distance_to_next_node'] / other['speed']
                        cur_t = car_dict['distance_to_next_node'] / car_dict['speed']

                        # Stop if another car will reach the same node sooner
                        if (
                                other['next_node'] == car_dict['next_node']
                                and other_t <= cur_t
                                and other['current_edge'] != car_dict['current_edge']
                        ):
                            car_should_stop = True
                            break
                    if car_should_stop:
                        break
                    if car_should_stop:
                            # Deadlock prevention: priority by ETA or ID
                        same_node_cars = []
                        for edges in all_nearby_edges:
                            edge = str(edges)
                            for other in shared_dict.get(edge, []):
                                if not other or other['id'] == car_dict['id']:
                                    continue
                                if other.get('next_node') == car_dict.get('next_node'):
                                    same_node_cars.append(other)

                        if same_node_cars:
                            my_eta = car_dict['distance_to_next_node'] / max(car_dict['speed'], 0.1)
                            all_cars_with_eta = []
                            for c in same_node_cars + [car_dict]:
                                eta = c['distance_to_next_node'] / max(c['speed'], 0.1)
                                all_cars_with_eta.append((eta, c['id']))

                            all_cars_with_eta.sort(key=lambda x: (x[0], x[1]))
                            leader_id = all_cars_with_eta[0][1]

                            if leader_id == car_dict['id']:
                                car_dict['speed'] = max(car_dict['speed'], 20.0)
                                print(
                                        f"[DEBUG] Car {car_dict['id']} proceeds first at node {car_dict['next_node']}")
                            else:
                                car_dict['speed'] = 0.0
                                update_specific_car_attributes_to_edges_json(
                                        json_file,
                                        car_dict['current_edge'],
                                        {"speed": 0.0},
                                        car_dict['id'],
                                        locks,
                                    )
                                print(
                                        f"[DEBUG] Car {car_dict['id']} yielding to Car {leader_id} at {car_dict['next_node']}")
                                time.sleep(0.5)
                        else:
                            car_dict['speed'] = 0.0
                            update_specific_car_attributes_to_edges_json(
                                    json_file,
                                    car_dict['current_edge'],
                                    {"speed": 0.0},
                                    car_dict['id'],
                                    locks,
                                )
                            print(f"[DEBUG] Car {car_dict['id']} stopped near node {car_dict['next_node']}")
                            time.sleep(0.5)
                    else:
                        car_dict['speed'] = max(car_dict['speed'], 20.0)
                        # if cars and cars['next_node'] == car_dict['next_node'] and car_dict[
                        # 'distance_to_next_node'] and (
                        # cars['distance_to_next_node'] <= 45.0 or (
                        # cars[
                        # 'distance_travelled'] <= 45.0 if cars in next_edge_cars else None)) and \
                        # cars['speed'] != 0.0:
            # else:
            # nearby_edge = str((car_dict['current_edge'][1], car_dict['current_edge'][0]))
            # for cars in shared_dict[nearby_edge]:
            # if cars:
            # distance_between_cars = distance.geodesic((car_dict['y_pos'], car_dict['x_pos']), (cars['y_pos'], cars['x_pos']))
            # distance_between_cars_meters = float(str(distance_between_cars).split(' ')[0]) * 1000
            # if distance_between_cars_meters <= 40.0:
            # car.x_pos -= 5.0
            # car.y_pos -= 5.0
            # cars.x_pos += 5.0
            # cars.y_pos += 5.0
            # car.current_edge_x_coords = x_new[x0: x0 + 30]

            else:
                nearby_edge0 = (ast.literal_eval(car_dict['current_edge'])[1],
                                ast.literal_eval(car_dict['current_edge'])[0])
                nearby_edge = str(nearby_edge0)
                with locks:
                    with open(json_file, 'r') as f:
                        shared_dict = json.load(f)
                    with open(json_file, 'w') as f:
                        json.dump(shared_dict, f)
                # --- BEGIN FIXED DIVERGENCE LOGIC ---
                for cars in shared_dict.get(nearby_edge, []):
                    if not cars or cars['id'] == car_dict['id']:
                        continue

                    dist = distance.geodesic(
                        (car_dict['y_pos'], car_dict['x_pos']),
                        (cars['y_pos'], cars['x_pos'])
                    )
                    dist_m = float(str(dist).split(' ')[0]) * 1000

                    # use hysteresis thresholds to prevent jitter
                    ENTER_OFFSET_DIST = 25.0  # start diverging when closer than this
                    EXIT_OFFSET_DIST = 30.0  # return when farther than this
                    OFFSET_AMOUNT = 0.0001

                    if dist_m < ENTER_OFFSET_DIST and not translated:
                        # Apply the lateral offset once
                        points = np.column_stack(
                            (car_dict['current_edge_x_coords'], car_dict['current_edge_y_coords'])
                        )
                        translated_points = translate_polyline_optimized(points, OFFSET_AMOUNT)
                        current_edge_x_coords, current_edge_y_coords = zip(*translated_points)
                        car_dict.update({
                            'current_edge_x_coords': current_edge_x_coords,
                            'current_edge_y_coords': current_edge_y_coords
                        })
                        translated = True
                        print(f"[DEBUG] Car {car_dict['id']} diverging from path (distance={dist_m:.1f} m)")
                    elif dist_m > EXIT_OFFSET_DIST and translated:
                        old_x = np.array(car_dict['current_edge_x_coords'])
                        old_y = np.array(car_dict['current_edge_y_coords'])
                        main_x = np.array(x_new0)
                        main_y = np.array(y_new0)

                        # Resample old arrays to match main arrays
                        if len(old_x) != len(main_x):
                            old_x = resample_array(old_x, len(main_x))
                            old_y = resample_array(old_y, len(main_y))

                        # Blend gradually
                        blend_ratio = min(1.0, (dist_m - EXIT_OFFSET_DIST) / 10.0)
                        new_x = old_x + (main_x - old_x) * blend_ratio
                        new_y = old_y + (main_y - old_y) * blend_ratio

                        car_dict.update({'current_edge_x_coords': new_x})
                        car_dict.update({'current_edge_y_coords': new_y})
                        if blend_ratio >= 1.0:
                            translated = False
                            print(f"[DEBUG] Car {car_dict['id']} returned to main path (distance={dist_m:.1f} m)")
                    # else: within hysteresis band — keep current offset
                # --- END FIXED DIVERGENCE LOGIC ---

                for cars in shared_dict[car_dict['current_edge']]:
                    if cars['id'] == car_dict['id'] and len(shared_dict[car_dict['current_edge']]) > 1:
                        continue

                    front_car_speed = cars['speed']
                    initial_relative_velocity = car_dict['speed'] - front_car_speed
                    initial_car_speed = car_dict['speed']
                    front_car_distance = car_dict['distance_to_next_node'] - cars['distance_to_next_node'] if cars[
                                                                                                                  'distance_to_next_node'] and \
                                                                                                              car_dict[
                                                                                                                  'distance_to_next_node'] else -1.0
                    catch_up_distance = front_car_distance - 20.0
                    #print(initial_relative_velocity, 44)
                    if 20.0 <= front_car_distance <= 40.0 and catch_up_distance > 0.0:
                        """required_decel = (initial_car_speed ** 2 - front_car_speed ** 2) / (2 * catch_up_distance)
                        new_speed = initial_car_speed - min(3.0, required_decel) * 0.01  # dt=0.1s
                        car_dict['speed'] = max(front_speed, new_speed)"""
                        a0 = -10.0
                        a1 = 12.0
                        # car_dict['speed'] = 0
                        car_speed_ratio = 1 / car_dict['speed'] if car_dict['speed'] != 0.0 else 1
                        car_dict['speed'] = initial_car_speed + (
                                    a0 * initial_relative_velocity + a1 * catch_up_distance) * car_speed_ratio
                        new_speed = car_dict['speed']
                        update_specific_car_attributes_to_edges_json(json_file, car_dict['current_edge'],
                                                                     {"speed": new_speed}, car_dict['id'],
                                                                     locks)

                    if 0.0 <= front_car_distance <= 20.0:
                        car_dict['speed'] = front_car_speed


                    if cars['id'] == car_dict['id'] and len(shared_dict[car_dict['current_edge']]) == 1:
                        a = 0.1
                        initial_car_speed = car_dict['speed']
                        initial_relative_velocity = 20.0 - initial_car_speed
                        car_speed_ratio = 1 / car_dict['speed'] if car_dict['speed'] != 0.0 else 1
                        car_dict['speed'] = initial_car_speed + a * initial_relative_velocity * car_speed_ratio
                        new_speed = car_dict['speed']
                        update_specific_car_attributes_to_edges_json(json_file, car_dict['current_edge'],
                                                                     {"speed": new_speed}, car_dict['id'],
                                                                     locks)
                        continue

            if not is_going:
                car_dict['speed'] = initial_speed
            is_running = True
            is_going = True
            distance_kilometers = distance.geodesic((y_new[x0 - 1], x_new[x0 - 1]), (y_new[x0], x_new[x0]))
            distance_meters = float(str(distance_kilometers).split(' ')[0]) * 1000
            car_dict['distance_travelled'] += distance_meters
            car_dict['distance_to_next_node'] = \
                G.get_edge_data((ast.literal_eval(car_dict['current_edge'])[0]),
                                (ast.literal_eval(car_dict['current_edge'])[1]))[0][
                    'length'] - car_dict['distance_travelled']
            distance_to_next_node = car_dict['distance_to_next_node']
            update_specific_car_attributes_to_edges_json(json_file, car_dict['current_edge'],
                                                         {"distance_to_next_node": distance_to_next_node},
                                                         car_dict['id'], locks)
            # print(car.distance_travelled)
            # time_taken = distance_meters / car.speed if car.speed != 0.0 else 0.0
            ball_x = x_new[x0]
            ball_y = y_new[x0]
            car_dict['x_pos'] = ball_x
            car_dict['y_pos'] = ball_y
            update_specific_car_attributes_to_edges_json(json_file, car_dict['current_edge'], {"x_pos": ball_x},
                                                         car_dict['id'],
                                                         locks)
            update_specific_car_attributes_to_edges_json(json_file, car_dict['current_edge'], {"y_pos": ball_y},
                                                         car_dict['id'], locks)
            # print(ball_x, ball_y, car)
            end_time = time.time()
            elapsed_time = end_time - start_time
            total_current_time += elapsed_time
            if car_dict['id'] == 3:
                print(elapsed_time)
                print(distance_meters)
                print(car_dict['speed'])
            car_time_interval = distance_meters / car_dict['speed'] if car_dict['speed'] != 0.0 else 1.0
            time_period = car_time_interval - elapsed_time if car_time_interval - elapsed_time > 0.0 else 0.0
            #print(time_period, 0)
            # if time_period > 0.0:
            # time.sleep(time_period)
            # pr.disable()
            # s = io.StringIO()
            # sortby = 'cumulative'
            # ps = pstats.Stats(pr, stream=s).sort_stats(sortby)
            # ps.print_stats()
            # print(s.getvalue())
            yield (ball_x, ball_y), time_period

    #remove_car_attributes_from_edges_json(json_file, car_dict['current_edge'], car, locks)
    #print('end')


def manage_cars(cars, routes):
    for car_gen in car_generators:
        car_gen.start()
        time.sleep(0.1)


def run_animation(point, update_signal):
    # Get the first point
    time.sleep(point[1])  # Delay based on second element of tuple
    update_signal.signal.emit(point[0])  # Emit only the first element (point)


class UpdateSignal(QObject):
    signal = pyqtSignal(tuple)


class AnimationThread(QThread):
    def __init__(self, route_generators, update_signals):
        super().__init__()
        self.processes = processes
        self.update_signals = update_signals
        # self.threads = []

    # def start_animations(self, route_generators):
    # for i, route_gen in enumerate(route_generators):
    # point = next(route_gen)
    # thread = threading.Thread(target=run_animation, args=(point, self.update_signals[i]))
    # thread.start()
    # self.threads.append(thread)

    def run(self):
        for point, delay in self.route_gen:
            time.sleep(delay)  # Convert microseconds to seconds
            self.update_signal.signal.emit(point)


class CarAnimation:
    def __init__(self, route_generators, G):
        self.G = G
        self.current_delay = None
        self.current_point = None
        self.app = QApplication(sys.argv)
        self.win = pg.GraphicsLayoutWidget(show=True, title="Car Animation")
        self.win.resize(1200, 800)
        self.plot = self.win.addPlot(title="Moving Car")
        self.route_point = route_generators
        # self.timer = QTimer()
        # self.timer.timeout.connect(self.update)
        self.load_osmnx_graph_to_pyqtgraph(G)
        self.points = [self.plot.plot([], [], symbol='o', symbolSize=5, symbolBrush='r') for _ in self.route_point]
        self.plot.hideAxis('bottom')  # Hide the bottom (x) axis
        self.plot.hideAxis('left')  # Hide the left (y) axis
        self.plot.setXRange(103.7395, 103.7512)  # Set fixed x-axis range
        self.plot.setYRange(1.2313, 1.2371)  # Set fixed y-axis range
        # self.next_point()
        self.processes = []
        self.update_signals = [UpdateSignal() for _ in self.route_point]
        for i in range(len(self.update_signals)):
            self.update_signals[i].signal.connect(lambda point, i=i: self.update(i, point))
        for i0 in range(len(self.route_point)):
            threading.Thread(target=self.animate, args=(i0, self.route_point[i0]), daemon=True).start()

    def load_osmnx_graph_to_pyqtgraph(self, G):

        # Get node positions, transforming them if necessary (e.g., to UTM)
        # pos = {node: (data['x'], data['y']) for node, data in G.nodes(data=True)}

        # Plot edges
        node_coords = ox.graph_to_gdfs(G, nodes=False, edges=True)['geometry'].to_dict()
        for edge in G.edges():
            edge_geometry = node_coords[(edge[0], edge[1], 0)]
            edge_coords = edge_geometry.coords
            x_coords = []
            y_coords = []
            for x, y in list(edge_coords):
                x_coords.append(x)
                y_coords.append(y)
            self.plot.plot(x_coords, y_coords, pen=pg.mkPen('b', width=18))
            self.plot.plot(x_coords, y_coords, pen=pg.mkPen('y', width=1))

        # Optional: Plot nodes
        # for node, (x, y) in pos.items():
        # edges = list(G.edges(node))
        # print(edges)
        node_coordinates = ox.graph_to_gdfs(G, edges=False, nodes=True)['geometry'].to_dict()
        #print(node_coordinates)

    # def next_point(self):
    # try:
    # self.current_point, self.current_delay = next(self.route_gen)
    # self.timer.start(int(self.current_delay * 1000))
    # except StopIteration:
    # self.timer.stop()  # Stop if no more points or delays

    def animate(self, i, route_gen):
        if route_gen:
            for item in route_gen:
                if item is not None:
                    point, delay = item
                    time.sleep(delay)
                    self.update_signals[i].signal.emit(point)

    # def animate(self):
    # point, delay = self.route_point
    # time.sleep(delay)
    # self.update_signal.signal.emit(point)

    def update(self, i, point):
        x, y = point
        #print(x, y)
        self.points[i].setData([x], [y])
        # self.next_point()

    def run(self):
        # self.animation_thread.start()
        sys.exit(self.app.exec_())


if __name__ == '__main__':
    south, west, north, east = 1.231401, 103.739519, 1.237477, 103.750725
    G = ox.graph_from_bbox((west, south, east, north))
    #print(G.nodes)

    car_dict0 = {
        'speed': 0.0,
        'distance_travelled': 0.0,
        'next_edge': None,
        'next_node': None,
        'current_edge': None,
        'current_node': None,
        'previous_edge': None,
        'x_pos': None,
        'y_pos': None,
        'current_edge_x_coords': None,
        'current_edge_y_coords': None,
        'next_edge_x_coords': None,
        'next_edge_y_coords': None,
        'distance_to_next_node': None,
        'moving': False
    }

    lock = multiprocessing.Lock()
    edge_dict = edge_attributes(G)
    shared_dict = 'shared_dict.json'
    dict_to_json(edge_dict, shared_dict)

    route1 = nx.shortest_path(G, 1874855397, 1874854292)
    route2 = nx.shortest_path(G, 1874853489, 1874857171)
    route3 = nx.shortest_path(G, 1874853025, 1874855397)
    route4 = nx.shortest_path(G, 1874855397, 1874852805)
    car1 = 0
    car2 = 0
    car3 = 0
    car4 = 0
    cars = [(car1, 25.0), (car2, 15.0), (car3, 20.0), (car4, 5.0)]  # Assuming car1 and car2 are defined
    cars_dict_list = [car_dict0.copy() for _ in range(len(cars))]
    for i in range(len(cars_dict_list)):
        cars_dict_list[i]['speed'] = cars[i][1]
        cars_dict_list[i]['id'] = i
    #print(cars_dict_list)

    routes = [route1, route2, route3, route4]  # Assuming route1 and route2 are defined
    # car_lines = [ax.plot(int(G.nodes[route[0]]['y']), int(G.nodes[route[0]]['x']), marker='o', color='blue')[0] for
    # route in routes]
    # queues = [multiprocessing.Queue() for _ in cars]
    # connections = [multiprocessing.Pipe() for _ in cars]
    car_generators = [
        CarProcess(shared_dict, target=ball_animation, args=(car_dict, route, shared_dict, lock))
        for
        car_dict, route in
        zip(cars_dict_list, routes)]
    route_gen_list = []
    for n in range(len(cars)):
        route_generator = ball_animation(cars_dict_list[n], routes[n], shared_dict, lock)
        route_gen_list.append(route_generator)
    # manage_cars(cars, routes)  # Assuming G is defined
    # ani = FuncAnimation(fig, animate, frames=1000, interval=1, blit=True)
    # plt.show()
    car_animation = CarAnimation(route_gen_list, G)
    # car_animation.start_animations(route_gen_list)

    # car_animation.animation_thread.start()  # Start the animation thread
    # car_animation.animation_thread.run()
    car_animation.run()
