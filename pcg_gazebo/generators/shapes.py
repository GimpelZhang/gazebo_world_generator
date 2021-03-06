# Copyright (c) 2019 - The Procedural Generation for Gazebo authors
# For information on the respective copyright owner see the NOTICE file
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from .. import random
from shapely.geometry import Polygon, Point, \
    MultiPoint, LineString
from shapely.ops import triangulate, unary_union


def rectangle(x_center, y_center, delta_x, delta_y):
    assert delta_x > 0, 'Width in X must be greater than zero'
    assert delta_y > 0, 'Width in Y must be greater than zero'
    x_min = x_center - delta_x / 2.0
    x_max = x_center + delta_x / 2.0
    y_min = y_center - delta_y / 2.0
    y_max = y_center + delta_y / 2.0
    coords = list()
    coords.append((x_min, y_min))
    coords.append((x_max, y_min))
    coords.append((x_max, y_max))
    coords.append((x_min, y_max))
    coords.append((x_min, y_min))
    return Polygon(coords)


def random_rectangle(x_center=0, y_center=0, delta_x_min=2,
                     delta_x_max=15, delta_y_min=2, delta_y_max=15):
    assert delta_x_min > 0 and delta_x_max > 0 and \
        delta_y_min > 0 and delta_y_max > 0
    assert delta_x_min < delta_x_max
    assert delta_y_min < delta_y_max
    delta_x = max(delta_x_min, random.rand() * delta_x_max)
    delta_y = max(delta_y_min, random.rand() * delta_y_max)
    return rectangle(x_center, y_center, delta_x, delta_y)

def random_points(n_points=10, x_min=-10, x_max=10, y_min=-10,
                  y_max=10):
    assert x_min < x_max, 'x_min must be lower than x_max'
    assert y_min < y_max, 'y_min must be lower than y_max'
    x = random.rand(n_points) * (x_max - x_min) + x_min
    y = random.rand(n_points) * (y_max - y_min) + y_min
    return MultiPoint([(xi, yi) for xi, yi in zip(x, y)])


def triangulate_points(points):
    if isinstance(points, list):
        pp = MultiPoint(points)
    elif isinstance(points, MultiPoint):
        pp = points
    return unary_union(triangulate(pp))


def circle(center, radius=1):
    assert len(center) in [2, 3], \
        'Center of circle must have 2 or 3 elements'
    assert radius > 0, 'Radius must be greater than zero'
    center = Point(*center)
    return center.buffer(radius)

# def random_rect_cir(x_center=0, y_center=0, delta_x_min=2,
#                      delta_x_max=15, delta_y_min=2, delta_y_max=15):
#     from random import choice
#     assert delta_x_min > 0 and delta_x_max > 0 and \
#         delta_y_min > 0 and delta_y_max > 0
#     assert delta_x_min < delta_x_max
#     assert delta_y_min < delta_y_max
#     delta_x = max(delta_x_min, random.rand() * delta_x_max)
#     delta_y = max(delta_y_min, random.rand() * delta_y_max)
#     x_min = x_center - delta_x / 2.0
#     x_max = x_center + delta_x / 2.0
#     y_min = y_center - delta_y / 2.0
#     y_max = y_center + delta_y / 2.0
    
#     centers = [[x_min,random.rand()*(y_max-y_min-1.0)+y_min+0.5],[x_max,random.rand()*(y_max-y_min-1.0)+y_min+0.5],[random.rand()*(x_max-x_min-1.0)+x_min+0.5,y_min],[random.rand()*(x_max-x_min-1.0)+x_min+0.5,y_max]]
#     circle_center = choice(centers)
#     return rectangle(x_center, y_center, delta_x, delta_y),circle(circle_center,radius=0.5),circle(circle_center),[x_center,y_center]

def random_rect_cir(x_center=0, y_center=0, delta_x=10,delta_y=10):
    from random import choice
    assert delta_x > 0, 'Width in X must be greater than zero'
    assert delta_y > 0, 'Width in Y must be greater than zero'
    x_min = x_center - delta_x / 2.0
    x_max = x_center + delta_x / 2.0
    y_min = y_center - delta_y / 2.0
    y_max = y_center + delta_y / 2.0
    
    centers = [[x_min,random.rand()*(y_max-y_min-1.0)+y_min+0.5],[x_max,random.rand()*(y_max-y_min-1.0)+y_min+0.5],[random.rand()*(x_max-x_min-1.0)+x_min+0.5,y_min],[random.rand()*(x_max-x_min-1.0)+x_min+0.5,y_max]]
    circle_center = choice(centers)
    return rectangle(x_center, y_center, delta_x, delta_y),circle(circle_center,radius=0.5),circle(circle_center),[x_center,y_center,centers.index(circle_center)]

def random_points_to_triangulation(n_points=10, x_min=-10, x_max=10,
                                   y_min=-10, y_max=10):
    pp = random_points(n_points, x_min, x_max, y_min, y_max)
    return triangulate_points(pp)


def random_rectangles(
        n_rect=5,
        x_center_min=-10,
        x_center_max=10,
        y_center_min=-10,
        y_center_max=10,
        delta_x_min=2,
        delta_x_max=10,
        delta_y_min=2,
        delta_y_max=10,
        delete_interiors=False):
    assert n_rect > 1, 'Number of rectangles to be generated must be n'
    polygon = None
    rectangles = list()
    while len(rectangles) < n_rect:
        new_rect = random_rectangle(
            random.rand() * (x_center_max - x_center_min) + x_center_min,
            random.rand() * (y_center_max - y_center_min) + y_center_min,
            delta_x_min,
            delta_x_max,
            delta_y_min,
            delta_y_max)
        if len(rectangles) == 0:
            rectangles.append(new_rect)
        else:
            for r in rectangles:
                if r.intersects(new_rect):
                    rectangles.append(new_rect)
                    break
    polygon = unary_union(rectangles)
    return polygon

def random_rectangle_rooms(
        n_rect=6,
        x_center_min=-10,
        x_center_max=10,
        y_center_min=-10,
        y_center_max=10,
        delta_x_min=2,
        delta_x_max=20,
        delta_y_min=2,
        delta_y_max=20,
        delete_interiors=False):
    assert n_rect > 1, 'Number of rectangles to be generated must be n'
    # living room, kitchen, hallway, bedroom, bathroom, dining room:
    room_sizes = [[10,10],[4,8],[8,2],[4,6],[6,3],[8,4]]
    polygon = None
    rectangles = list()
    overlaps = list()
    n_other = 3
    # living room: 10*10
    rectangles.append(rectangle(0, 0, 10, 10))
    room_iter = 1
    # Living room, kitchen, hallway:
    while len(rectangles) < n_other:
        new_rect = rectangle(
            random.rand() * (x_center_max - x_center_min) + x_center_min,
            random.rand() * (y_center_max - y_center_min) + y_center_min,
            room_sizes[room_iter][0],
            room_sizes[room_iter][1])
        if len(rectangles) == 0:
            rectangles.append(new_rect)
        else:
            area_flag = False
            one_overlap = list()
            for r in rectangles:
                if r.intersects(new_rect):
                    # to guarantee the area of a room:
                    if r.intersection(new_rect).area<1.6 and r.intersection(new_rect).area>1:
                        area_flag = True
                        one_overlap.append(r.intersection(new_rect))
                    else:
                        area_flag = False
                        one_overlap.clear()
                        break
            if area_flag:
                rectangles.append(new_rect)
                overlaps.extend(one_overlap)
                room_iter = room_iter+1
 
    # bedroom, dining room, bathroom
    door_rooms = list()
    room_pos = list()
    door_cir = list()
    while len(rectangles) < n_rect:
        new_rect, new_cir,new_cir_l, new_pos = random_rect_cir(
            random.rand() * (x_center_max - x_center_min) + x_center_min,
            random.rand() * (y_center_max - y_center_min) + y_center_min,
            room_sizes[room_iter][0],
            room_sizes[room_iter][1])
        inter_flag = False
        if len(rectangles[n_other:])>0:
            for d in rectangles[n_other:]:
                if d.buffer(0.5).intersects(new_rect.buffer(0.5)) or d.buffer(0.5).intersects(new_cir_l):
                    inter_flag = True
                    break
        if not inter_flag:
            for ol in overlaps:
                if ol.intersects(new_rect) or ol.intersects(new_cir_l):
                    inter_flag = True
                    break
        if not inter_flag:
            area_flag = False    
            for r in rectangles[0:n_other]:
                if r.intersects(new_rect):
                    if r.intersection(new_rect).area<1.4 and r.intersection(new_rect).area>0.01 and r.contains(new_cir):
                        area_flag = True
                    else:
                        area_flag = False
                        break
            if area_flag:
                rectangles.append(new_rect)
                rec_bound = unary_union(new_rect.boundary)
                room_wall = rec_bound.buffer(0.15,cap_style=1,join_style=1)
                door_room = room_wall.difference(new_cir)
                door_rooms.append(door_room)
                room_pos.append(new_pos)
                door_cir.append(new_cir)
                room_iter = room_iter+1
    
    polygon = unary_union(rectangles)
    print("range:  ",polygon.bounds)
    print("overlaps: ",len(overlaps))
    return polygon,rectangles,door_rooms,room_pos,door_cir

def random_orthogonal_lines(
        n_lines=5,
        x_min=-10,
        x_max=10,
        y_min=-10,
        y_max=10,
        line_min_length=1,
        line_max_length=15):
    assert n_lines > 0, 'Number of lines must be greater than zero'
    lines = list()
    while len(lines) < n_lines:
        start_x = random.rand() * (x_max - x_min) + x_min
        start_y = random.rand() * (y_max - y_min) + y_min
        length = max(line_min_length, random.rand() * line_max_length)

        if random.randint(2) == 0:
            end_x = start_x + length
            end_y = start_y
        else:
            end_x = start_x
            end_y = start_y + length

        if end_x <= x_max and end_y <= y_max:
            new_line = LineString([(start_x, start_y), (end_x, end_y)])
            if len(lines) > 0:
                for line in lines:
                    if line.contains(new_line):
                        continue
            lines.append(new_line)

    return unary_union(lines)
