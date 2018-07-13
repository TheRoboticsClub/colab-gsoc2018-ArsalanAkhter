from referee import distanceWidget
from sensors import sensor
import numpy as np
import cv2
import math

import threading
import time
from datetime import datetime
from ompl_solution.Point2DPlanning import Plane2DEnvironment
import sys
from scipy.ndimage import gaussian_filter1d
from purePursuit import State, calc_target_index, PIDControl, pure_pursuit_control, update_state
import matplotlib.pyplot as plt


time_cycle = 80

class MyAlgorithm(threading.Thread):

    def __init__(self, grid, sensor, vel):
        self.sensor = sensor
        self.grid = grid
        self.vel = vel
        sensor.getPathSig.connect(self.generatePath)

        self.stop_event = threading.Event()
        self.kill_event = threading.Event()
        self.lock = threading.Lock()
        threading.Thread.__init__(self, args=self.stop_event)
        self.worldPathIdx = 0;
        self.worldPathArrayExecuting = 0;


    def run (self):

        while (not self.kill_event.is_set()):
           
            start_time = datetime.now()

            if not self.stop_event.is_set():
                self.execute()

            finish_Time = datetime.now()

            dt = finish_Time - start_time
            ms = (dt.days * 24 * 60 * 60 + dt.seconds) * 1000 + dt.microseconds / 1000.0
            #print (ms)
            if (ms < time_cycle):
                time.sleep((time_cycle - ms) / 1000.0)

    def stop (self):
        self.stop_event.set()

    def play (self):
        if self.is_alive():
            self.stop_event.clear()
        else:
            self.start()

    def kill (self):
        self.kill_event.set()

    """ Write in this method the code necessary for looking for the shorter
        path to the desired destiny.
        The destiny is chosen in the GUI, making double click in the map.
        This method will be call when you press the Generate Path button. 
        Call to grid.setPath(path) mathod for setting the path. """
    def generatePath(self, list):
        print("LOOKING FOR SHORTER PATH")
        mapIm = self.grid.getMap()      
        dest = self.grid.getDestiny()   
        gridPos = self.grid.getPose()

        size = mapIm.shape
        h = size[0]
        w = size[1]
        print (h)
        print (w)
        mapppm = np.zeros((h,w,3))
        kernel = np.ones((5,5),np.uint8)    
        erosion = cv2.erode(mapIm,kernel,iterations = 1) 
        #cv2.imshow('image',erosion)
        for i in range(h):
            for j in range(w):
                mapppm[i,j,0] = erosion[i,j]
                mapppm[i,j,1] = erosion[i,j]
                mapppm[i,j,2] = erosion[i,j]

        
        cv2.imwrite('map_image.ppm',mapppm)

        planner = str(list[0])
        #objective = str(list[1])
        runtime = str(list[1])

        print planner
        #print objective
        print runtime

        fname = sys.path[0]+'/map_image.ppm'
        print fname
        env = Plane2DEnvironment(fname)

        if env.plan(gridPos[0],gridPos[1], dest[0],dest[1],planner,float(runtime)):
            env.recordSolution()
            env.save("result_demo.ppm")
            pathlist = [[] for i in range(2)]
            pathlist = env.getPath()
            worldPathList = [[] for i in range(2)]

            patharray = np.array(pathlist)
            patharray = np.rint(patharray)
            print patharray
            size = patharray.shape
            print size
            num = 0
            pathX_old = -1
            pathY_old = -1
            for i in range(size[1]):
                pathX = patharray[0][i]
                pathY = patharray[1][i]
                if pathX != pathX_old or pathY != pathY_old:
                    self.grid.setPathVal (int(pathX), int(pathY), num)
                    tmp = self.grid.gridToWorld(pathX,pathY)
                    worldPathList[0].append(tmp[0])
                    worldPathList[1].append(tmp[1])
                    num += 1
                    pathX_old = pathX
                    pathY_old = pathY

            self.grid.setPathFinded()
            worldPathArraySmoothTmp = np.array(worldPathList)
            #sigma = 10
            #worldPathArraySmoothTmp[0] = gaussian_filter1d(worldPathArraySmoothTmp[0], sigma)
            #worldPathArraySmoothTmp[1] = gaussian_filter1d(worldPathArraySmoothTmp[1], sigma)
            worldPathArraySmoothTmp[0] = worldPathArraySmoothTmp[0]
            worldPathArraySmoothTmp[1] = worldPathArraySmoothTmp[1]

            print worldPathArraySmoothTmp
            self.grid.setWorldPathArray(worldPathArraySmoothTmp)

        #Represent the Gradient Field in a window using cv2.imshow

    """ Write in this mehtod the code necessary for going to the desired place,
        once you have generated the shorter path.
        This method will be periodically called after you press the GO! button. """


    def findTargetPose(self):
        worldPathArrayExecuting1 = self.grid.getWorldPathArray()
        worldPathIdx1 =  self.grid.worldPathArrayIdx
        return (worldPathArrayExecuting1[0][worldPathIdx1], worldPathArrayExecuting1[1][worldPathIdx1])

    def correct_yaw(self, req_yaw):
        robot_yaw = self.sensor.getRobotTheta()
        while abs(req_yaw - robot_yaw) > 0.1:
            clearscreen()
            print "correcting robot yaw:"
            print "current yaw: " + str(self.sensor.getRobotTheta())
            print "required yaw: " + str(req_yaw)
            self.vel.setV(0)
            yaw_diff = req_yaw - self.sensor.getRobotTheta()
            if yaw_diff > 3.14:
                yaw_diff -= 6.28
            if yaw_diff < -3.14:
                yaw_diff += 6.28
            print "yaw_diff: " + str(yaw_diff)
            # self.vel.setW(0.3*yaw_diff*state.w)
            if (yaw_diff) > 0:
                self.vel.setW(0.2 * yaw_diff)
            else:
                self.vel.setW(0.2 * yaw_diff)
            # robot_yaw = self.sensor.getRobotTheta()
            if abs(req_yaw - self.sensor.getRobotTheta()) <= 0.05:
                print "Robot yaw corrected."
                self.vel.setW(0)
                return

    def execute(self):
        # Add your code here
        target_speed = 0.5  # [m/s]
        T = 100.0  # max simulation time
        show_animation = True
        worldPathArrayExecuting1 = self.grid.getWorldPathArray()
        cx = worldPathArrayExecuting1[0]
        cy = worldPathArrayExecuting1[1]
        # find the inflection points
        # by taking 2nd differential of both cx and cy
        cx_diff = np.diff(cx,2)
        cy_diff = np.diff(cy,2)
        # and then taking only non-zero points
        # find indicies of all points that re non-zero in cx_diff
        cx_nonzero_indicies = np.reshape(np.nonzero(cx_diff), -1)
        cy_nonzero_indicies = np.reshape(np.nonzero(cy_diff), -1)
        # Take a union of above indicies
        print cx_nonzero_indicies
        print cy_nonzero_indicies

        all_nonzero_indicies = np.union1d(cx_nonzero_indicies,cy_nonzero_indicies)
        # And then take all the points with all_non_zero_indicies
        cx = cx[all_nonzero_indicies]
        cy = cy[all_nonzero_indicies]

        # Obtain the current position of the vehicle
        x = self.sensor.getRobotX()
        y = self.sensor.getRobotY()
        yaw = self.sensor.getRobotTheta()

        state = State(x, y, yaw, v=0.0, w=0.0)
        prev_state = state

        # Find the goal point
        # Calculate the point on the path closest to the vehicle
        # We already have a certain look-ahead distance (Lfc)
        # search look ahead target point index
        target_ind = calc_target_index(state, cx, cy)



        print("GOING TO DESTINATION")

        lastIndex = len(cx)-1
        time = 0.0
        x = [state.x]
        y = [state.y]
        yaw = [state.yaw]
        v = [state.v]
        t = [0.0]
        dt = 0.01
        while T >= time and lastIndex > target_ind:
            clearscreen()
            prev_yaw = state.yaw
            prev_state = State(state.x, state.y, state.yaw, state.v, state.w)
            print "time" + str(time)
            print "last_ind: " + str(lastIndex)
            print "target_ind: " + str(target_ind)
            ai = PIDControl(target_speed, state.v)
            print "target_speed: " + str(target_speed)
            print "curr_speed: " + str(state.v)
            di, target_ind = pure_pursuit_control(state, cx, cy, target_ind)
            print "di: " + str(di)
            print "prev_yaw: "+ str(prev_yaw)
            print "new_target_ind: " + str(target_ind)
            state = update_state(state, ai, di)
            print "state.yaw: " + str(state.yaw)
            print "state: " + str(state)
            time = time + dt
            if lastIndex == target_ind:
                self.vel.setV(0)
                self.vel.setW(0)
                print "Target Location Reached."
                #break

            # Ensure that the robot reaches this new updated state
            # First Ensure the Yaw
            '''req_heading = math.atan2(state.y - prev_state.y, state.x - prev_state.x)
            curr_heading = math.atan2(state.y - self.sensor.getRobotY(), state.x - self.sensor.getRobotX())
            while req_heading-curr_heading < 0:
                clearscreen()
                print "correcting robot yaw:"
                print "current yaw: " + str(self.sensor.getRobotTheta())
                print "required yaw: " + str(state.yaw)
                self.vel.setV(0)
                self.vel.setW(0.5*(req_heading-curr_heading))
                req_heading = math.atan2(state.y - prev_state.y, state.x - prev_state.x)
                curr_heading = math.atan2(state.y - self.sensor.getRobotY(), state.x - self.sensor.getRobotX())
                if req_heading == curr_heading:
                    self.vel.setW(0)'''

            robot_yaw = self.sensor.getRobotTheta()
            while abs(state.yaw - robot_yaw) > 0.1:
                clearscreen()
                print "correcting robot yaw:"
                print "current yaw: " + str(self.sensor.getRobotTheta())
                print "required yaw: " + str(state.yaw)
                self.vel.setV(0)
                yaw_diff = state.yaw - self.sensor.getRobotTheta()
                if yaw_diff > 3.14:
                    yaw_diff -= 6.28
                if yaw_diff < -3.14:
                    yaw_diff += 6.28
                print "yaw_diff: " + str(yaw_diff)
                #self.vel.setW(0.3*yaw_diff*state.w)
                if (yaw_diff) > 0:
                    self.vel.setW(0.2*yaw_diff)
                else:
                    self.vel.setW(0.2*yaw_diff)
                #robot_yaw = self.sensor.getRobotTheta()
                if abs(state.yaw - self.sensor.getRobotTheta()) <= 0.1:
                    print "Robot yaw corrected."
                    self.vel.setW(0)
                    break
                    #self.vel.setV(target_speed)
            old_distance = 0
            distance = 0
            # Then ensure the complete position
            distance = math.sqrt(
                    math.pow((state.x - self.sensor.getRobotX()), 2) + math.pow((state.y - self.sensor.getRobotY()), 2))
            req_heading = math.atan2(state.y - prev_state.y, state.x - prev_state.x)
            curr_heading =  math.atan2(state.y - self.sensor.getRobotY(), state.x - self.sensor.getRobotX())
            #while abs(state.x - self.sensor.getRobotX()) > 0.05 and abs(state.y - self.sensor.getRobotY()) > 0.05:
            while distance > 0.1:
                clearscreen()  # For better printing
                print "Now Correcting robot location."
                # Measure the distance between current location and target location
                print "distance: " + str(distance)
                # should we move forward or backward?
                curr_vel = 0.5*state.v
                print "velocity: " + str(curr_vel)
                print "req_heading: " + str(req_heading)
                print "curr_heading: " + str(curr_heading)
                print "diff_heading: " + str(req_heading-curr_heading)
                print "curr_x: " + str (self.sensor.getRobotX())
                print "curr_y: " + str(self.sensor.getRobotY())
                print "req_x: " + str(state.x)
                print "req_y: " + str(state.y)
                print "prev_x: " + str(prev_state.x)
                print "prev_y: " + str(prev_state.y)
                self.vel.setV(curr_vel)
                if abs(req_heading-curr_heading) > math.pi/2 and abs(req_heading-curr_heading) < 3*math.pi/2:
                    # Correct Heading
                    print "Coming back a bit."
                    curr_vel = -0.1
                    self.vel.setV(curr_vel)
                    self.correct_yaw(req_heading)
                distance = math.sqrt(
                        math.pow((state.x - self.sensor.getRobotX()), 2) + math.pow((state.y - self.sensor.getRobotY()),
                                                                                    2))
                curr_heading = math.atan2(state.y - self.sensor.getRobotY(), state.x - self.sensor.getRobotX())

                #if req_heading-curr_heading<0:
                #    curr_vel = -0.05
                #    self.vel.setV(curr_vel)
                #    distance = math.sqrt(
                #        math.pow((state.x - self.sensor.getRobotX()), 2) + math.pow((state.y - self.sensor.getRobotY()),
                #                                                                    2))
                #    curr_heading = math.atan2(state.y - self.sensor.getRobotY(), state.x - self.sensor.getRobotX())

                #while heading < 0:
                #    self.vel.setV(-0.2)
                #    remaining_distance = math.sqrt(math.pow((state.x - self.sensor.getRobotX()), 2) + math.pow((state.y - self.sensor.getRobotY()),2))
                #    heading = math.atan2((state.y - self.sensor.getRobotY()), (state.x - self.sensor.getRobotX()))
                #    clearscreen()  # For better printing
                #    print "remaining_distance: " + str(remaining_distance)
                #    print "heading: " + str(heading)
                '''if self.sensor.getRobotX() > 0 and self.sensor.getRobotY() > 0: # 4th Quadrant
                    self.vel.setV(state.v)
                elif self.sensor.getRobotX() < 0 and self.sensor.getRobotY() > 0: # 3rd Quadrant:
                    self.vel.setV(state.v * -np.sign(heading))
                elif self.sensor.getRobotX() < 0 and self.sensor.getRobotY() > 0: # 2nd Quadrant:
                    self.vel.setV(state.v * -np.sign(heading))
                else: # 1st Quadrant:
                    self.vel.setV(state.v * np.sign(heading))
                remaining_distance = math.sqrt(math.pow((state.x - self.sensor.getRobotX()),2) + math.pow((state.y - self.sensor.getRobotY()),2))
                heading = math.atan2((state.y-self.sensor.getRobotY()), (state.x-self.sensor.getRobotX()))'''


                # Check distance again
                #distance = math.sqrt(math.pow((state.x - self.sensor.getRobotX()),2) + math.pow((state.y - self.sensor.getRobotY()),2))
                #old_distance = distance
                '''while distance > 0.1 and self.vel.getV() > 0:
                    self.vel.setV(-0.2)
                    #old_distance = distance
                    distance = math.sqrt(math.pow((state.x - self.sensor.getRobotX()), 2) + math.pow((state.y - self.sensor.getRobotY()),2))
                    print "Going back a bit."
                    #print "old_distance = " + str(old_distance)
                    print "distance = " + str(distance)
                    #print "distance difference = " + str(old_distance - distance)

                    #self.vel.setV(0)
                while distance > 0.1 and self.vel.getV() < 0:
                    self.vel.setV(target_speed)
                    old_distance = distance
                    distance = math.sqrt(math.pow((state.x - self.sensor.getRobotX()), 2) + math.pow((state.y - self.sensor.getRobotY()),2))
                    print "Moving forward a bit."
                    print "old_distance = " + str(old_distance)
                    print "distance = " + str(distance)
                    print "distance difference = " + str(old_distance - distance)

                    #self.vel.setV(0)
                while old_distance - distance < 0.0 and self.vel.getV() > 0:
                    self.vel.setV(target_speed)
                    old_distance = distance
                    distance = math.sqrt(math.pow((state.x - self.sensor.getRobotX()), 2) + math.pow((state.y - self.sensor.getRobotY()),2))
                    print "Moving forward a bit."
                    print "old_distance = " + str(old_distance)
                    print "distance = " + str(distance)
                    print "distance difference = " + str(old_distance - distance)

                    #self.vel.setV(0)
                while old_distance - distance < 0.0 and self.vel.getV() < 0:
                    self.vel.setV(-0.2)
                    old_distance = distance
                    distance = math.sqrt(math.pow((state.x - self.sensor.getRobotX()), 2) + math.pow((state.y - self.sensor.getRobotY()),2))
                    print "Going back a bit."
                    print "old_distance = " + str(old_distance)
                    print "distance = " + str(distance)
                    print "distance difference = " + str(old_distance - distance)

                    #self.vel.setV(0)'''
                if abs(state.x - self.sensor.getRobotX()) <= 0.1 and abs(state.y - self.sensor.getRobotY()) <= 0.1:
                    print "Robot location corrected."
                    self.vel.setV(0)
                    break

            x.append(state.x)
            y.append(state.y)
            yaw.append(state.yaw)
            v.append(state.v)
            t.append(time)
            #self.vel.setV(state.v)
            #if abs(di) > 0.05:
            #    self.vel.setW(state.v*(di))
            #else:
            #    self.vel.setW(0.00)
            if show_animation:
                plt.cla()
                plt.plot(cx, cy, ".r", label="course")
                plt.plot(x, y, "-b", label="trajectory")
                plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
                plt.axis("equal")
                plt.grid(True)
                plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
                plt.pause(0.001)
        if abs(cx[lastIndex] - self.sensor.getRobotX()) < 0.15 and abs(cy[lastIndex] - self.sensor.getRobotY()) < 0.15:
            self.vel.setV(0)
            self.vel.setW(0)
            print "Target Location Reached."

        if show_animation:
            plt.plot(cx, cy, ".r", label="course")
            plt.plot(x, y, "-b", label="trajectory")
            plt.legend()
            plt.xlabel("x[m]")
            plt.ylabel("y[m]")
            plt.axis("equal")
            plt.grid(True)

            flg, ax = plt.subplots(1)
            plt.plot(t, [iv * 3.6 for iv in v], "-r")
            plt.xlabel("Time[s]")
            plt.ylabel("Speed[km/h]")
            plt.grid(True)
            plt.show()

        '''TargetPose = self.findTargetPose()
        print TargetPose

        print TargetPose
        #TargetPose = self.findTargetPose(x,y)
        yaw = math.atan2(TargetPose[1]-y,TargetPose[0]-x)
        yaw_diff = yaw - theta
        dis = math.sqrt(pow(x-TargetPose[0],2)+pow(y-TargetPose[1],2))

        epsilon = 0.1
        Kp = 0.35
        if yaw_diff > 3.14:
            yaw_diff -= 6.28
        if yaw_diff < -3.14:
            yaw_diff += 6.28

        # Apply controller here
        if yaw_diff > epsilon or yaw_diff < -epsilon:
            v = 0
            w = Kp*yaw_diff
        else:
            v = dis*1
            if v < 0.1:
                v = 0.1
            w = 0
        print "Target Pose: " + str(TargetPose)
        print "Yaw Difference: " + str(yaw_diff)
        # self.vel.setV(v)

        if abs(x - TargetPose[0]) < 0.01 and abs(y - TargetPose[1]) < 0.01:
            if self.worldPathIdx < len(self.grid.worldPathArraySmooth)-2:
                self.grid.setWorldPathArrayIdx(self.grid.worldPathArrayIdx+1)
                TargetPose = self.findTargetPose()
                print "Target Pose Updated."
            else:
                v = 0
                w = 0
                print "Target Reached."'''


        #EXAMPLE OF HOW TO SEND INFORMATION TO THE ROBOT ACTUATORS


def clearscreen(numlines=10):
    """Clear the console.
    numlines is an optional argument used only as a fall-back.
    """
    import os
    if os.name == "posix":
        # Unix/Linux/MacOS/BSD/etc
        os.system('clear')
    elif os.name in ("nt", "dos", "ce"):
        # DOS/Windows
        os.system('CLS')
    else:
        # Fallback for other operating systems.
        print '\n' * numlines


