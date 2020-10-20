#!/usr/bin/env python3
import tkinter as tk
import random # to generate random balls
from math import cos, sin, radians, tan, pow, sqrt

height = 700            #Window X
width = 700             #Window Y
circles_num = 8                     #Number of circles
startX = random.randint(25, 75)       #Tim starting X
startY = random.randint(600, 650)    #Tim starting Y
tim_rad = 40            #Radius of tim
tim_id = 0              #Id of Tim
lidar_rays = []         #Lidar rays going off of Tim
circles = []            #All of the circles
algorithm_lines = []    #Algorithm lines
endX = random.randint(600, 650)      #Target X
endY = random.randint(25, 75)         #Target Y
target_id = 0           #Target ID


class LIDAR:
    def __init__(self):
        #Create the windwo
        window = tk.Tk()
        window.title("LIDAR Simulation")

        #Create canvas frame
        frame1 = tk.Frame(window)
        frame1.pack()
        self.canvas = tk.Canvas(frame1, width = width, height = height, bg = "#CAD2C5")
        self.canvas.pack()

        #Add the cylinders to map
        self.display()

        #Create Button Frames
        frame2 = tk.Frame(window)
        frame2.pack()
        displayBtn = tk.Button(frame2, text = "Start LIDAR", command = self.begin)
        displayBtn.pack()

        #Bind Keys
        self.canvas.bind_all('<Key>', self.movement)

        #Run it all
        window.mainloop()


    def movement(self, event):
        #Initialize Variables
        x = 0
        y = 0
        #Check for different keys and do different movements based upon it
        if event.char == "a":
            x = -10
            y = 0
        elif event.char == "d":
            x = 10
            y = 0
        elif event.char == "w":
            x = 0
            y = -10
        elif event.char == "s":
            x = 0
            y = 10
        
        #Check Bounds
        global startY
        global startX
        startX += x
        startY += y
        #Check if touching a circle first
        for circle in circles:
            x_diff = abs(startX - circle["x"])
            y_diff = abs(startY - circle["y"])
            if(x_diff <= circle["r"]):
                if(y_diff <= circle["r"]):
                    startX -= x
                    startY -= y
                    x = 0
                    y = 0

        #Check Window Bounds
        if startX < 0 or startX > width:
            startX -= x
            x = 0
        if startY < 0 or startY > height:
            startY -= y
            y = 0

        #Move
        self.canvas.move(tim_id, x, y)
    
        #Clean up 
        global lidar_rays
        global algorithm_lines
        for id in lidar_rays:
            self.canvas.delete(id[0])
            self.canvas.delete(id[1])
        for id in algorithm_lines:
            self.canvas.delete(id[0])
            self.canvas.delete(id[1])

        self.canvas.pack()
        lidar_rays = []
        algorithm_lines = []
        self.lidar()



    def begin(self):
        #Initialize Tim Robot starting points
        x1 = startX - tim_rad/2
        y1 = startY - tim_rad/2
        x2 = startX + tim_rad/2
        y2 = startY + tim_rad/2
        coords = x1, y1, x2, y2
        global tim_id
        global target_id
        tim_id = self.canvas.create_oval(coords, fill = 'blue', tags = "circle")
        target_id = self.canvas.create_rectangle(endX-10, endY-10, endX+10, endY+10, fill='red')
        self.canvas.tag_raise(tim_id)
        
        #Start the lidar
        self.lidar()
        self.canvas.pack()


    
    def drawlines(self, deg):
        #Starting variables
        tim_coords = self.canvas.coords(tim_id)
        tim_x = (tim_coords[0] + tim_coords[2]) / 2
        tim_y = (tim_coords[1] + tim_coords[3]) / 2 
        save_deg = deg
        w = 0
        h = 0
        r = 6
        end_x = 0
        end_y = 0
        left_side = (deg >= 90) and (deg < 270)
        right_side = (deg < 90) or (deg >= 270)

        #Left quadrant of unit circle
        if(left_side):
            #Top left qudrant
            if(deg <= 180) and (deg >= 90):
                #Assign correct height
                h = tim_y - r
                w = tim_x - r
                deg = deg - 90
                #Try first calculation
                calc_x = h*tan(radians(deg))
                calc_y = h
                if(w - calc_x) < 0:
                    deg = 90 - deg
                    calc_y = w*tan(radians(deg))
                    calc_x = w

                end_x = tim_x - calc_x
                end_y = tim_y - calc_y
            #Bottom left quadrant
            else:
                h = height - tim_y - r
                w = tim_x - r
                deg = 270 - deg
                #Try first calculation
                calc_x = h*tan(radians(deg))
                calc_y = h
                if(w - calc_x) < 0:
                    deg = 90 - deg
                    calc_y = w*tan(radians(deg))
                    calc_x = w

                end_x = tim_x - calc_x
                end_y = tim_y + calc_y
        #Right side of unit circle
        elif(right_side):
            #Bottom right quadrant
            if(deg <= 360) and (deg >= 270):
                #Assign correct height
                h = height - tim_y - r
                w = width - tim_x - r
                deg = deg - 270
                #Try first calculation
                calc_x = h*tan(radians(deg))
                calc_y = h
                if(tim_x + calc_x) > width:
                    deg = 90 - deg
                    calc_y = w*tan(radians(deg))
                    calc_x = w

                end_x = tim_x + calc_x
                end_y = tim_y + calc_y
            #Top right quadrant
            else:
                h = tim_y - r
                w = width - tim_x - r
                #Try first calculation
                calc_x = h*tan(radians(deg))
                calc_y = h
                if(tim_x + calc_x) > width:
                    deg = 90 - deg
                    calc_y = w*tan(radians(deg))
                    calc_x = w
            
                end_x = tim_x+calc_x
                end_y = tim_y-calc_y

        #Check for collision before printing
        self.collision(tim_x, tim_y, end_x, end_y, r, save_deg)

    #Algorithm to detemine quickest path to the Target
    def calculatePath(self):
        #Get Tim Coords
        tim_coords = self.canvas.coords(tim_id)
        tim_x = (tim_coords[0] + tim_coords[2]) / 2
        tim_y = (tim_coords[1] + tim_coords[3]) / 2 

        #Get Target Coords
        target_coords = self.canvas.coords(target_id)
        target_x = (target_coords[0] + target_coords[2]) / 2
        target_y = (target_coords[1] + target_coords[3]) / 2 

        #Calcualte distance
        dist_target_x = abs(tim_x - target_x)
        dist_target_y = abs(tim_y - target_y)

        #Count variables
        count_up = 0
        count_down = 0

        #Loop through each degree and compare which half of quadrant is touching least amount of circles
        #Send Tim down the path of least resistance
        for ray in lidar_rays:
            if ray[2] <= 45 and ray[2] >= 0 and ray[3] == True:
                count_down += 1
            elif ray[2] <= 90 and ray[2] >= 45 and ray[3] == True:
                count_up += 1

        #Draw the lines
        if (count_up <count_down):
            alg_x = self.canvas.create_line(tim_x, tim_y, tim_x+dist_target_x, tim_y, dash=(4,2), width = 4)
            alg_y = self.canvas.create_line(tim_x+dist_target_x, tim_y, tim_x+dist_target_x, tim_y-dist_target_y, dash=(2, 4), width = 4)
            algorithm_lines.append([alg_x, alg_y])
        else:
            alg_y = self.canvas.create_line(tim_x, tim_y, tim_x, tim_y-dist_target_y, dash=(4,2), width = 4)
            alg_x = self.canvas.create_line(tim_x, tim_y-dist_target_y, tim_x+dist_target_x, tim_y-dist_target_y, dash=(2, 4), width = 4)
            algorithm_lines.append([alg_x, alg_y])
        
    


    def collision(self, x1, y1, x2, y2, rad, deg):
        #Compare to each circle to see if we have a collision
        closest_dist = height*2
        closest_x = x2
        closest_y = y2
        drawLine = False

        #Detect a collision for each circle
        for circle in circles:
            h = circle["x"]
            k = circle["y"]
            r = circle["r"]
            delta_x = x2-x1
            delta_y = y2-y1

            A = pow(delta_x, 2) + (pow(delta_y, 2))
            B = 2*delta_x*(x1-h) + 2*delta_y*(y1-k)
            C = pow((x1-h), 2) + pow((y1-k),2) - pow(r, 2)

            top = 2*C
            bottom_left = -1*B
            disc = pow(B, 2) - 4*A*C

            #No intersection
            if(disc < 0):
                continue

            t = top / (bottom_left + sqrt(disc))

            if(t >= 0) and (t <=1):
                drawLine = True
            else:
                continue

            x_end = (x2-x1)*(t) + x1
            y_end = (y2-y1)*(t) + y1

            dist_x = abs(x_end - x1)
            dist_y = abs(y_end - y1)
            dist = sqrt(pow(dist_x, 2) + pow(dist_y, 2))

            #Store current closest
            if(dist < closest_dist):
                closest_dist = dist
                closest_x = x_end
                closest_y = y_end
            
        
        #Draw the ray with the closest point (intersection of wall)
        ray = self.canvas.create_line(x1, y1, closest_x, closest_y)
        hit = self.canvas.create_oval(closest_x-rad/2, closest_y-rad/2, closest_x+rad/2, closest_y+rad/2, fill='#84A98C')
        self.canvas.tag_lower(ray)
        lidar_rays.append([ray, hit, deg, drawLine])



    def lidar(self):
        #Loop through each ray, draw, then calcualte path
        step = 5
        end = 360 + step
        for degree in range(0, end, step):
            self.drawlines(degree)
        self.calculatePath()
        self.canvas.pack()


            
    #Draw the circles and store information
    def display(self):
        for i in range(0, circles_num):
            x = random.randint(40, width-40)
            y = random.randint(0, height-100)
            circ_width = random.randint(10, 40)
            coords = x-circ_width, y-circ_width, x+circ_width, y+circ_width

            print("Circle number %i " % i)
            self.canvas.create_oval(coords, fill = '#354F52', tags = "circle")
            circles.append(
                {"x":x,
                 "y":y,
                 "r":circ_width
                }
            )
        
        
    
#RUN THE PROGRAM
LIDAR()


