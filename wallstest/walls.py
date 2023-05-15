import cv2
import math
import time

# Floodfill
# A star (A*)

# Point
class CustomPoint:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def toString(self):
        return "CustomPoint: (" + str(self.x) + ", " + str(self.y) + ")"

class CustomLine:
    def __init__(self, p1, p2):
        self.p1 = p1
        self.p2 = p2

    def getBoundingBox(self):
        result = []
        result.append(CustomPoint(min(self.p1.x, self.p2.x), \
                min(self.p1.y, self.p2.y)))
        result.append(CustomPoint(max(self.p1.x, self.p2.x), \
                max(self.p1.y, self.p2.y)))
        return result

# Square 
class Square:
    def __init__(self, step, x1, x2, y1, y2, index):
        self.x1 = x1
        self.x2 = x2
        self.y1 = y1
        self.y2 = y2
        self.step = step

        self.index = index
        
        self.hasWall = False
        self.wallRadius = False
        self.arucoId = -1
        self.path = False
        self.clearance = 1

        self.g = 0
        self.h = 0
        self.f = 0
        self.parent = None

    def getPos(self):
        return ((self.x1+self.x2)/2, (self.y1+self.y2)/2)

    def __eq__(self, other):
        return self.index == other.index

    def checkAruco(self, arucoId, corners):
        if self.arucoId != -1:
            return 
        
        arucoTL = CustomPoint(corners[0][0], corners[0][1])
        arucoTR = CustomPoint(corners[1][0], corners[1][1])
        arucoBR = CustomPoint(corners[2][0], corners[2][1])
        arucoBL = CustomPoint(corners[3][0], corners[3][1])

        # Top left
        tl = CustomPoint(self.x1, self.y1)
        # Top right
        tr = CustomPoint(self.x2, self.y1)
        # Bottom right
        br = CustomPoint(self.x2, self.y2)
        # Bottom left
        bl = CustomPoint(self.x1, self.y2)

        if (arucoTL.x < tl.x < arucoBR.x) and \
                (arucoTL.y < tl.y < arucoBR.y):
            self.arucoId = arucoId
            return
        if (arucoTL.x < tr.x < arucoBR.x) and \
                (arucoTL.y < tr.y < arucoBR.y):
            self.arucoId = arucoId
            return
        if (arucoTL.x < br.x < arucoBR.x) and \
                (arucoTL.y < br.y < arucoBR.y):
            self.arucoId = arucoId
            return
        if (arucoTL.x < bl.x < arucoBR.x) and \
                (arucoTL.y < bl.y < arucoBR.y):
            self.arucoId = arucoId
            return

        return

    def checkWall(self, line_x1, line_x2, line_y1, line_y2):
        if self.hasWall or self.arucoId != -1:
            return

        # Wall line
        p1 = CustomPoint(line_x1, line_y1)
        p2 = CustomPoint(line_x2, line_y2)
        line = CustomLine(p1, p2)

        # Top left
        tl = CustomPoint(self.x1, self.y1)
        # Top right
        tr = CustomPoint(self.x2, self.y1)
        # Bottom right
        br = CustomPoint(self.x2, self.y2)
        # Bottom left
        bl = CustomPoint(self.x1, self.y2)

        # Top line (tl, tr)
        top = CustomLine(tl, tr)

        # Right line (tr, br)
        right = CustomLine(tr, br)

        # Bottom line (bl, br)
        bottom = CustomLine(bl, br)

        # Left line (tl, bl)
        left = CustomLine(tl, bl)
        
        # Top (tl, tr)
        if self.intersection(top, line):
            self.hasWall = True
            self.clearance = 0
            return True
        # Right (tr, br)
        if self.intersection(right, line):
            self.hasWall = True
            self.clearance = 0
            return True
        # Bottom (bl, br)
        if self.intersection(bottom, line):
            self.hasWall = True
            self.clearance = 0
            return True
        # Left (tl, bl)
        if self.intersection(left, line):
            self.hasWall = True
            self.clearance = 0
            return True
        # Edge case
        if self.x1 < line_x1 and line_x1 < self.x2 and \
                self.y1 < line_y1 and line_y1 < self.y2 and \
                self.x1 < line_x2 and line_x2 < self.x2 and \
                self.y1 < line_y2 and line_y2 < self.y2:
            self.hasWall = True
            self.clearance = 0
            return True

        return False
   
    # src: https://martin-thoma.com/how-to-check-if-two-line-segments-intersect/

    # Point a and point b
    def crossProduct(self, a, b):
        return a.x * b.y - b.x * a.y
    
    # a - array of points (bounding box)
    # b - array of points (bounding box)
    def doBoundingBoxesIntersect(self, a, b):
        return a[0].x <= b[1].x and \
                a[1].x >= b[0].x and \
                a[0].y <= b[1].y and \
                a[1].y >= b[0].y
    
    # a - line
    # b - point
    def isPointOnLine(self, a, b):
        aTmp = CustomLine(CustomPoint(0, 0),\
                CustomPoint(a.p2.x - a.p1.x, a.p2.y - a.p1.y))
        bTmp = CustomPoint(b.x - a.p1.x, b.y - a.p1.y)
        r = self.crossProduct(aTmp.p2, bTmp)
        return (abs(r) < 0.000001)
    
    # a - line
    # b - point
    def isPointRightOfLine(self, a, b):
        aTmp = CustomLine(CustomPoint(0, 0), \
                CustomPoint(a.p2.x - a.p1.x, a.p2.y - a.p1.y))
        bTmp = CustomPoint(b.x - a.p1.x, b.y - a.p1.y)
        r = self.crossProduct(aTmp.p2, bTmp)
        return (r < 0)

    # a, b - line
    def lineSegmentTouchesOrCrossesLine(self, a, b):
        return (self.isPointOnLine(a, b.p1) or \
                self.isPointOnLine(a, b.p2) or \
                (self.isPointRightOfLine(a, b.p1) ^ \
                self.isPointRightOfLine(a, b.p2)))
    
    # a, b - lines
    def intersection(self, a, b):
        box1 = a.getBoundingBox()
        box2 = b.getBoundingBox()

        boundingBoxCheck = self.doBoundingBoxesIntersect(box1, box2)

        if boundingBoxCheck:
            abCrossesLine = self.lineSegmentTouchesOrCrossesLine(a, b)

            if abCrossesLine:
                baCrossesLine = \
                        self.lineSegmentTouchesOrCrossesLine(b, a)
                return baCrossesLine
        
        return False

def square_in_list(square_index, _list):
    for square in _list:
        if square.index == square_index:
            return True

    return False

def astar(grid, start_index, end_index):
    open_list = []
    closed_list = []

    # Adding the start node to the open list
    open_list.append(grid[start_index[0]][start_index[1]])
    count = 0

    while len(open_list) > 0:
        open_list.sort(key=lambda x: x.f)

        # Get current square
        current_square = open_list[0]

        if current_square.index == end_index:
            closed_list.append(current_square)
            break
            
        # Remove the current square from the open list
        open_list.pop(0)
        closed_list.append(current_square)

        # Looking at the surrounding squares of the current square
        for n in [(-1, -1), (0, -1), (1, -1), (-1, 0), (1, 0), (-1, 1),\
                (0, 1), (1, 1)]:
            
            #sur_i = (start_index[0]+n[0],\
            #        start_index[1]+n[1])
            
            sur_i = (current_square.index[0]+n[0],\
                    current_square.index[1]+n[1])

            # Boundaries check
            if sur_i[0] < 0 or sur_i[1] < 0 or sur_i[0] > 49 or sur_i[1] > 34:
                continue

            sur_square = grid[sur_i[0]][sur_i[1]] 
            # Check if the sqaure is illegal terrain or it is already in the
            # closed list
            if sur_square.hasWall or sur_square.clearance < 3 or \
                    sur_square.wallRadius or \
                    square_in_list(sur_square.index, closed_list):
                continue

            # Heuristics' calculations
            g = 0
            h = 0
            f = 0

            if n in [(-1, -1), (1, -1), (-1, 1), (1, 1)]:
                g = current_square.g + 14
            else:
                g = current_square.g + 10
            
            # "Manhattan" distances and h calculation
            man_dist_i = abs(end_index[0]-sur_i[0])
            man_dist_j = abs(end_index[1]-sur_i[1])
            h = (man_dist_i+man_dist_j)*10
            
            f = g + h

            if square_in_list(sur_square.index, open_list) and g < sur_square.g:
                sur_square.parent = current_square
                sur_square.g = g
                sur_square.h = h
                sur_square.f = f
                open_list[open_list.index(sur_square)] = sur_square
            # Set surrounding square parent to current square and add the
            # surrounding square to open list
            elif not square_in_list(sur_square.index, open_list):
                sur_square.parent = current_square
                sur_square.g = g
                sur_square.h = h
                sur_square.f = f
                open_list.append(sur_square)

        count += 1
    
    if not square_in_list(end_index, closed_list):
        return
    
    current_square = closed_list[-1]
    while current_square.index != start_index:
        print(current_square.clearance, current_square.parent.clearance)
        print(current_square.index, "->", current_square.parent.index)
        current_square.path = True
        current_square = current_square.parent

grid = []
path = []

in_img = cv2.imread("sample-1-1_small.png", 0)
height, width = in_img.shape

step = 10
width_len = math.floor(width/step)
height_len = math.floor(height/step)

print(width_len)
print(height_len)

for i in range(width_len):
    grid.append([])
    for j in range(height_len):
        current_square = Square(step, i*step, i*step+step,\
                j*step, j*step+step, (i, j)) 
        grid[i].append(current_square)
        #print(current_square.x1, current_square.y1, current_square.x2, current_square.y2)

# LINES DETECTION ------------------------------------------------------------#
lsd = cv2.createLineSegmentDetector(0)
# Position 0 of the returned tuple are the detected lines
lines = lsd.detect(in_img)[0]
# Draw detected lines in the image
lines_img = lsd.drawSegments(in_img, lines)

# ARUCO DETECTION ------------------------------------------------------------#
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_1000)
aruco_params = cv2.aruco.DetectorParameters_create()
(corners, ids, rejected) = cv2.aruco.detectMarkers(in_img, aruco_dict,
        parameters = aruco_params)

cv2.aruco.drawDetectedMarkers(lines_img, corners, ids)

# WALL DETECTION IN GRID -----------------------------------------------------#
print("Starting wall detection...")
start_wall = time.time()
for i in range(width_len):
    for square in grid[i]:
        if square.hasWall or square.arucoId != -1:
            continue

        for line in lines:
            line = line[0]

            for j in range(len(ids)):
                square.checkAruco(ids[j], corners[j][0])
            square.checkWall(line[0], line[2], line[1], line[3])
print("Wall detection done (took " + str(time.time()-start_wall) + ") s")

# CLEARANCE ------------------------------------------------------------------#
print("Starting clearance...")
start_clearance = time.time()
for i in range(width_len):
    for square in grid[i]:
        if square.hasWall:
            continue
        
        radius = 1
        clear = True
        while clear:
            square.clearance = radius
            radius += 1

            # TL, TR, BR, BL
            corners = [(square.index[0]-radius, square.index[1]-radius),
                    (square.index[0]+radius, square.index[1]-radius),
                    (square.index[0]+radius, square.index[1]+radius),
                    (square.index[0]-radius, square.index[1]+radius)]

            for j in range(4):
                if not clear:
                    break

                corner = corners[j]

                if corner[0] < 0 or corner[0] > 49 or corner[1] < 0 or \
                        corner[1] > 34:
                    clear = False
                    break
            
                next_corner = corners[(j+1) % 4]
                
                move_x = next_corner[0] - corner[0]
                move_y = next_corner[1] - corner[1]
                
                mult_x = 1
                mult_y = 1
                if move_x < 0:
                    mult_x = -1
                elif move_y < 0:
                    mult_y = -1
                
                for k in range(abs(move_x)):
                    x = corner[0]+mult_x*k
                    if x < 0 or x > 50 or grid[x][corner[1]].hasWall:
                        clear = False
                        break
                
                for k in range(abs(move_y)):
                    y = corner[1]+mult_y*k
                    if y < 0 or y > 35 or grid[corner[0]][y].hasWall:
                        clear = False
                        break
print("Clearance done (took " + str(time.time()-start_clearance) + ") s")

        
# WALL RADIUS ----------------------------------------------------------------#
'''wall_radius = 2
for i in range(width_len):
    for square in grid[i]:
        if not square.hasWall:
            continue

        # TL, TR, BR, BL
        corners = [(square.index[0]-wall_radius, square.index[1]-wall_radius),
                (square.index[0]+wall_radius, square.index[1]-wall_radius),
                (square.index[0]+wall_radius, square.index[1]+wall_radius),
                (square.index[0]-wall_radius, square.index[1]+wall_radius)]

        if square.index[0] == 26 and square.index[1] == 13:
            print(corners)

        for j in range(4):
            corner = corners[j]

            if corner[0] < 0 or corner[0] > 49 or corner[1] < 0 or corner[1] > 34:
                continue

            next_corner = corners[(j+1) % 4]

            move_x = next_corner[0] - corner[0]
            move_y = next_corner[1] - corner[1]

            mult_x = 1
            mult_y = 1
            if move_x < 0:
                mult_x = -1
            elif move_y < 0:
                mult_y = -1
            
            for k in range(abs(move_x)):
                x = corner[0]+mult_x*k
                if x > -1 and x < 50:
                    grid[x][corner[1]].wallRadius = True
            
            for k in range(abs(move_y)):
                y = corner[1]+mult_y*k
                if y > -1 and y < 35:
                    grid[corner[0]][y].wallRadius = True'''


#sel_square = grid[35][14]
#sel_square.hasWall = True
# TL, TR, BR, BL
#corners = [(sel_square.index[0]-wall_radius, sel_square.index[1]-wall_radius),
#        (sel_square.index[0]+wall_radius, sel_square.index[1]-wall_radius),
#        (sel_square.index[0]+wall_radius, sel_square.index[1]+wall_radius),
#        (sel_square.index[0]-wall_radius, sel_square.index[1]+wall_radius)]

# WALL RADIUS PRACTICE -------------------------------------------------------#
'''grid[corners[0][0]][corners[0][1]].hasWall = True
grid[corners[1][0]][corners[1][1]].hasWall = True
grid[corners[2][0]][corners[2][1]].hasWall = True
grid[corners[3][0]][corners[3][1]].hasWall = True'''

'''for i in range(4):
    corner = corners[i]
    next_corner = corners[(i+1) % 4]

    move_x = next_corner[0] - corner[0]
    move_y = next_corner[1] - corner[1]

    mult_x = 1
    mult_y = 1
    if move_x < 0:
        mult_x = -1
    elif move_y < 0:
        mult_y = -1
    
    for j in range(abs(move_x)):
        grid[corner[0]+mult_x*j][corner[1]].hasWall = True
    
    for j in range(abs(move_y)):
        grid[corner[0]][corner[1]+mult_y*j].hasWall = True'''

    
'''for j in range(wall_radius, 0, -1):

    for k in [(-j, -j), (0, -j), (j, -j), (-j, 0), (j, 0), (-j, j),\
        (0, j), (j, j)]:
            sur_i = (sel_square.index[0]+k[0], sel_square.index[1]+k[1])
            sur_square = grid[sur_i[0]][sur_i[1]] 
            if sur_square.hasWall or square.wallRadius:
                continue

            sur_square.hasWall = True
'''


'''for i in range(width_len):
    for square in grid[i]:
        if not square.hasWall or square.wallRadius:
            continue
        
        for j in range(wall_radius, 0, -1):
            for k in [(-j, -j), (0, -j), (j, -j), (-j, 0), (j, 0), (-j, j),\
                (0, j), (j, j)]:
                sur_i = (square.index[0]+k[0], square.index[1]+k[1])
            
            # Boundaries check
            if sur_i[0] < 0 or sur_i[1] < 0 or sur_i[0] > 49 or sur_i[1] > 34:
                continue

            sur_square = grid[sur_i[0]][sur_i[1]] 
            sur_square.wallRadius = True'''


# PATH FINDING ---------------------------------------------------------------#
print("Starting pathfinding...")
start_path = time.time()
#path_result = astar(grid, (13, 7), (7, 24))
path_result = astar(grid, (13, 7), (37, 29))
#path_result = astar(grid, (2, 2), (10, 8))
print("Pathfinding done (took " + str(time.time()-start_path) + ") s")

# DRAW SQUARES WITH WALLS ----------------------------------------------------#
print("Starting drawing...")
start_drawing = time.time()
wall_img = lines_img
for i in range(width_len):
    for square in grid[i]:
        if square.hasWall or square.wallRadius:
            start_point = (square.x1, square.y1)
            end_point = (square.x2, square.y2)
            cv2.rectangle(wall_img, start_point, end_point, \
                    (255, 0, 0), 2)
        elif square.path == True:
            start_point = (square.x1, square.y1)
            end_point = (square.x2, square.y2)
            cv2.rectangle(wall_img, start_point, end_point, \
                    (0, 255, 255), 2)
        '''elif square.clearance >= 6:
            start_point = (square.x1, square.y1)
            end_point = (square.x2, square.y2)
            cv2.rectangle(wall_img, start_point, end_point, \
                    (255, 255, 255), 2)
        elif square.clearance >= 5:
            start_point = (square.x1, square.y1)
            end_point = (square.x2, square.y2)
            cv2.rectangle(wall_img, start_point, end_point, \
                    (255, 0, 255), 2)
        elif square.clearance >= 4:
            start_point = (square.x1, square.y1)
            end_point = (square.x2, square.y2)
            cv2.rectangle(wall_img, start_point, end_point, \
                    (255, 255, 0), 2)
        elif square.clearance >= 3:
            start_point = (square.x1, square.y1)
            end_point = (square.x2, square.y2)
            cv2.rectangle(wall_img, start_point, end_point, \
                    (0, 255, 255), 2)
        else:
            start_point = (square.x1, square.y1)
            end_point = (square.x2, square.y2)
            cv2.rectangle(wall_img, start_point, end_point, \
                    (0, 0, 0), 2)'''
        
        '''elif square.arucoId != -1:
            start_point = (square.x1, square.y1)
            end_point = (square.x2, square.y2)
            cv2.rectangle(wall_img, start_point, end_point, \
                    (0, 255, 0), 2)'''

        '''for j in range(height_len):
            if i == 5 and j == 20:
                start_point = (grid[i][j].x1, grid[i][j].y1)
                end_point = (grid[i][j].x2, grid[i][j].y2)
                cv2.rectangle(wall_img, start_point, end_point, \
                        (0, 0, 0), 2)
        if i == 5 and j == 21:
            start_point = (grid[i][j].x1, grid[i][j].y1)
            end_point = (grid[i][j].x2, grid[i][j].y2)
            cv2.rectangle(wall_img, start_point, end_point, \
                    (0, 0, 255), 2)'''
print("Drawing done (took " + str(time.time()-start_drawing) + ") s")


#exit(0)
# SHOW RESULT ----------------------------------------------------------------#
cv2.imshow("LSD", wall_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
