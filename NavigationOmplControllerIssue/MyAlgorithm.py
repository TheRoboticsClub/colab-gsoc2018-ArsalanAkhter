from sensors import sensor
import numpy as np
import cv2
import math

import threading
import time
from datetime import datetime
#from ompl_solution.OptimalPlanning import optimalPlanning
from ompl_solution.Point2DPlanning import Plane2DEnvironment
import sys
from scipy.ndimage import gaussian_filter1d
from purePursuit import State, calc_target_index, PIDControl, pure_pursuit_control, update_state
import matplotlib.pyplot as plt


time_cycle = 80


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

        self.worldPathIdx = 0
        self.worldPathArrayExecuting = 0

        self.curr_state = State()
        self.prev_state = State()

        self.targetV = 0.5
        self.desiredR = 0
        self.desiredPhi = 0
        # Pure_pursuit related variables
        self.ai = 0
        self.di = 0
        self.cx = []
        self.cy = []
        self.target_ind = 0
        self.lastIndex = 0
        self.showAnimation = True
        self.yawCorrected = False
        self.posCorrected = False
        self.prevDistDiff = 0
        self.currV = 0
        self.prevV = 0
        self.prevPositionalError = 0
        self.currPositionalError = 0
        self.dt = 0
        self.negVelNeeded = False
        self.posVelNeeded = False

        # Debugging Info Variables
        self.printK_V = 0
        self.printK_W = 0
        self.printDistanceDiff = 0
        self.printCurrV = 0
        self.printCurrW = 0
        self.printYawDiff = 0
        self.printCurrHeading = 0
        self.printReqHeading = 0
        self.printPhiDiff = 0
        self.printDesiredR = 0
        self.printDesiredPhi = 0

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
            self.dt = ms/1000

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
            #print patharray
            size = patharray.shape
            #print size
            num = 0
            pathX_old = -1
            pathY_old = -1
            for i in range(size[1]):
                pathX = patharray[0][i]
                pathY = patharray[1][i]
                if pathX != pathX_old or pathY != pathY_old:
                    self.grid.setPathVal(int(pathX), int(pathY), num)
                    tmp = self.grid.gridToWorld(pathX, pathY)
                    worldPathList[0].append(tmp[0])
                    worldPathList[1].append(tmp[1])
                    num += 1
                    pathX_old = pathX
                    pathY_old = pathY

            self.grid.setPathFinded()
            worldPathArraySmoothTmp = np.array(worldPathList)
            #sigma = 3
            #orldPathArraySmoothTmp[0] = gaussian_filter1d(worldPathArraySmoothTmp[0], sigma)
            #worldPathArraySmoothTmp[1] = gaussian_filter1d(worldPathArraySmoothTmp[1], sigma)
            worldPathArraySmoothTmp[0] = worldPathArraySmoothTmp[0]
            worldPathArraySmoothTmp[1] = worldPathArraySmoothTmp[1]

            print worldPathArraySmoothTmp
            self.grid.setWorldPathArray(worldPathArraySmoothTmp)
            self.initializePurePursuit()

        #Represent the Gradient Field in a window using cv2.imshow



    def cartesian_to_polar(self, x, y):
        r = np.sqrt(x ** 2 + y ** 2)
        phi = np.arctan2(y, x)
        return (r, phi)

    def polar_to_cartesian(self, r, phi):
        x = r * np.cos(phi)
        y = r * np.sin(phi)
        return (x, y)


    def initializePurePursuit(self):
        worldPathArrayExecuting = self.grid.getWorldPathArray()
        self.cx = worldPathArrayExecuting[0]
        self.cy = worldPathArrayExecuting[1]
        # find the inflection points
        # by taking 2nd differential of both cx and cy
        cx_diff = np.diff(self.cx, 2)
        cy_diff = np.diff(self.cy, 2)
        # and then taking only non-zero points
        # find indicies of all points that are non-zero in cx_diff
        cx_nonzero_indicies = np.reshape(np.nonzero(cx_diff), -1)
        cy_nonzero_indicies = np.reshape(np.nonzero(cy_diff), -1)
        # Take a union of above indicies
        all_nonzero_indicies = np.union1d(cx_nonzero_indicies, cy_nonzero_indicies)
        # And then take all the points with all_non_zero_indicies
        self.cx = self.cx[all_nonzero_indicies]
        self.cy = self.cy[all_nonzero_indicies]

        # Obtain the current position of the vehicle
        x = self.sensor.getRobotX()
        y = self.sensor.getRobotY()
        yaw = self.sensor.getRobotTheta()
        r, phi = self.cartesian_to_polar(x, y)
        self.curr_state = State(x, y, yaw, r, phi, v=0.0, w=0.0)
        self.prev_state = State(x=0.0, y=0.0, yaw=0.0, r=0.0, phi=0.0, v=0.0, w=0.0)

        # Find the goal point
        # Calculate the point on the path closest to the vehicle
        # We already have a certain look-ahead distance_diff (Lfc)
        # search look ahead target point index
        self.target_ind = calc_target_index(self.curr_state, self.cx, self.cy)
        self.lastIndex = len(self.cx) - 1
        self.computeNextState()

        print("Pure Persuit Initialized")


    def computeNextState(self):
        clearscreen()
        print "prev_x: " + str(self.prev_state.x)
        print "prev_y: " + str(self.prev_state.y)
        print "prev_yaw: " + str(self.prev_state.yaw)
        print "req_x: " + str(self.curr_state.x)
        print "req_y: " + str(self.curr_state.y)
        print "req_yaw: " + str(self.curr_state.yaw)
        print "After Computation:"

        self.ai = PIDControl(self.targetV, self.prev_state.v)
        self.di, self.target_ind = pure_pursuit_control(self.prev_state, self.cx, self.cy, self.target_ind)
        self.prev_state = State(self.curr_state.x, self.curr_state.y, self.curr_state.yaw, self.curr_state.r,
                                self.curr_state.phi, self.curr_state.v, self.curr_state.w)
        self.curr_state = update_state(self.curr_state, self.ai, self.di)
        '''prevTargetIdx = self.target_ind
        print "prev_target_ind: " + str(prevTargetIdx)

        self.di, self.target_ind = pure_pursuit_control(self.prev_state, self.cx, self.cy, self.target_ind)
        #tempYaw = math.atan2(self.cy[self.target_ind] - self.cy[prevTargetIdx], self.cx[self.target_ind] - self.cx[prevTargetIdx])
        tempYaw = math.atan2(self.curr_state.y - self.prev_state.y,  self.curr_state.x - self.prev_state.x)
        self.prev_state = State(self.curr_state.x, self.curr_state.y, self.curr_state.yaw, self.curr_state.r,
                                self.curr_state.phi, self.curr_state.v, self.curr_state.w)
        self.curr_state = State(self.cx[self.target_ind], self.cy[self.target_ind], tempYaw)'''


        #print "ai: " + str(self.ai)
        #print "di: " + str(self.di)
        print "prev_x: " + str(self.prev_state.x)
        print "prev_y: " + str(self.prev_state.y)
        print "prev_yaw: " + str(self.prev_state.yaw)
        print "req_x: " + str(self.curr_state.x)
        print "req_y: " + str(self.curr_state.y)
        print "req_yaw: " + str(self.curr_state.yaw)
        print "target_ind: " + str(self.target_ind)
        time.sleep(3)


    def printDebugInfo(self):
        clearscreen()
        if self.yawCorrected:
            print "Yaw Corrected!"
        else:
            print "Yaw Not Corrected!"
        if self.posCorrected:
            print "Position Corrected!"
        else:
            print "Position Not Corrected!"
        print "K_V: " + str(self.printK_V)
        print "K_W: " + str(self.printK_W)
        print "curr_pos_error: "+ str(self.currPositionalError)
        print "prev_pos_error: "+ str(self.prevPositionalError)
        #print "distance_diff: " + str(self.printDistanceDiff)
        print "desired_r: " + str(self.printDesiredR)
        print "desired_phi: " + str(self.printDesiredPhi)
        print "phi_diff: " + str(self.printPhiDiff)
        print "prev_dist_diff: " + str(self.prevDistDiff)
        print "curr_Heading: " + str(self.printCurrHeading)
        print "req_Heading: " + str(self.printReqHeading)
        print "curr_vel: " + str(self.printCurrV)
        print "last_ind: " + str(self.lastIndex)
        print "target_ind: " + str(self.target_ind)
        print "target_speed: " + str(self.targetV)
        print "curr_speed: " + str(self.curr_state.v)
        print "di: " + str(self.di)
        print "prev_yaw: " + str(self.prev_state.yaw)
        print "curr_x: "  + str(self.sensor.getRobotX())
        print "curr_y: "  + str(self.sensor.getRobotY())
        print "req_x: "  + str(self.curr_state.x)
        print "req_y: "  + str(self.curr_state.y)
        print "prev_x: "  + str(self.prev_state.x)
        print "prev_y: "  + str(self.prev_state.y)

        print "correcting robot yaw:"
        print "current yaw: " + str(self.sensor.getRobotTheta())
        print "required yaw: " + str(self.curr_state.yaw)
        print "yaw_diff: " + str(self.printYawDiff)
        print "curr_W: " + str(self.printCurrW)
        print "state: " + str(self.curr_state)


    def correct_yaw(self):
        self.vel.setV(0)
        self.vel.setW(0)
        self.printCurrV = 0
        self.printCurrW = 0
        self.yawCorrected = False
        e = self.curr_state.yaw - self.sensor.getRobotTheta()
        e = math.atan2(math.sin(e), math.cos(e))
        if abs(e) > 0.05:
            self.vel.setV(0)
            alpha = 0.9
            K = 2*(1 - math.exp(-alpha*math.pow(e,2)/abs(e)))
            #K = 0.5
            self.vel.setW(K * e)
            self.printCurrW = K * e
            e = self.curr_state.yaw - self.sensor.getRobotTheta()
            e = math.atan2(math.sin(e), math.cos(e))
            self.printYawDiff = e
            self.printK_W = K

        if abs(e) <= 0.05:
            #print "Robot yaw corrected."
            self.vel.setW(0)
            self.printCurrW = 0
            self.yawCorrected = True
            K = 0
            self.printK_W = K

    '''def correctPosition(self):
        self.posCorrected = False
        self.vel.setW(0)
        self.vel.setV(0)
        self.printCurrV = 0
        self.printCurrW = 0
        self.computeDesiredRPhi()
        curr_r, curr_phi = self.cartesian_to_polar(self.sensor.getRobotX(), self.sensor.getRobotY())
        end_loc_r, end_loc_phi = self.cartesian_to_polar(self.cx[self.target_ind], self.cy[self.target_ind])
        distance_diff = end_loc_r - curr_r
        #distance_diff = self.desiredR
        #phi_diff = end_loc_phi - curr_phi
        #self.printPhiDiff = phi_diff
        curr_heading = math.atan2( (self.curr_state.y-self.sensor.getRobotY()) , (self.curr_state.x-self.sensor.getRobotX()) )
        req_heading = math.atan2( (self.curr_state.y-self.prev_state.y) , (self.curr_state.x-self.prev_state.x) )
        if abs(distance_diff) > 0.05:
            alpha = 0.9
            K = 7 * (1 - math.exp(-alpha * math.pow(distance_diff, 2))) / abs(distance_diff)
            self.currV = K * abs(distance_diff)
            #if abs(self.desiredPhi) > math.pi:
            #    self.vel.setV(-1 * self.curr_vel)
            #else:
            #    self.vel.setV(self.curr_vel)
            self.vel.setV(self.currV)

            self.computeDesiredRPhi()
            # for Debugging
            self.printK_V = K
            self.printDistanceDiff = distance_diff
            self.printCurrV = self.currV
            self.printCurrHeading = curr_heading
            self.printReqHeading = req_heading

        if abs(distance_diff) <= 0.05:
            self.vel.setV(0)
            self.printCurrV = 0
            self.posCorrected = True
            K = 0
            self.printK_V = K
            curr_heading = 0
            self.printCurrHeading = curr_heading
            self.printReqHeading = req_heading

        #distance_diff, angle_diff = self.cartesian_to_polar(self.cx[self.target_ind] - self.sensor.getRobotX(),
        #                                                   self.cy[self.target_ind] - self.sensor.getRobotY())

    def computeDesiredRPhi(self):
        #r1, phi1 = self.cartesian_to_polar(self.sensor.getRobotX(), self.sensor.getRobotY())
        #r2, phi2 = self.cartesian_to_polar(self.cx[self.target_ind], self.cy[self.target_ind])
        #self.desiredR = math.sqrt(r1 ** 2 + r2 ** 2 + 2*r1*r2*math.cos(phi2-phi1))
        #self.desiredPhi = phi1 + math.atan2(r2*math.sin(phi2-phi1), r1+r2*math.cos(phi2-phi1))

        self.desiredR, self.desiredPhi = self.cartesian_to_polar(self.cx[self.target_ind] - self.sensor.getRobotX(),
                                                            self.cy[self.target_ind] - self.sensor.getRobotY())
        self.printDesiredR = self.desiredR
        self.printDesiredPhi = self.desiredPhi
        #desiredPhi = math.atan2(math.sin(desiredPhi), math.cos(desiredPhi))
        #return self.desiredR, self.desiredPhi'''

    def computePositionalError(self):
        self.prevPositionalError = self.currPositionalError
        self.currPositionalError = math.sqrt((self.cx[self.target_ind] - self.sensor.getRobotX())**2 +
                                              (self.cy[self.target_ind] - self.sensor.getRobotY())**2 ) - 0.5
        self.printDistanceDiff = self.currPositionalError


    def computeV(self):
        self.prevV = self.currV
        self.computePositionalError()
        alpha = 0.1
        Kp = 0.1 * (1 - math.exp(-alpha * math.pow(self.currPositionalError, 2)) / abs(self.currPositionalError))
        #if Kp < 0: Kp = 0
        #alpha = 0.5
        #Ki = 0.0001 * (1 - math.exp(-alpha * math.pow(self.currPositionalError, 2))) / abs(self.currPositionalError)
        #Kd = 1000 * ( math.exp(alpha * math.pow(self.currPositionalError, 2))) / abs(self.currPositionalError)
        #Kp = 0.1
        Ki = 0.01
        Kd = 0.2
        #self.currV = Kp*self.currPositionalError + Ki*(
        #        self.currPositionalError + self.prevPositionalError)*self.dt + Kd*(
        #        self.currPositionalError - self.prevPositionalError) * self.dt
        self.currV = abs(Kp*self.currPositionalError)
        if self.negVelNeeded:
            self.currV = -self.currV
            self.computePositionalError()
            if abs(self.currPositionalError) <= 0.05:
                self.negVelNeeded = False
                self.currV = abs(Kp * self.currPositionalError)


        if abs(self.prevPositionalError) >= abs(self.currPositionalError) and abs(self.currPositionalError) > 0.05:
            if self.prevV > 0 :
                self.currV = -self.currV
                self.negVelNeeded = True
                self.computePositionalError()

            if self.prevV < 0 :
                self.currV = abs(Kp * self.currPositionalError)
                self.negVelNeeded = False
                self.computePositionalError()


        self.printK_V = Kp
        self.printCurrV = self.currV

    def correctPosition2(self):
        self.posCorrected = False
        self.vel.setW(0)
        self.vel.setV(0)
        self.printCurrV = 0
        self.printCurrW = 0
        self.computeV()
        if abs(self.currPositionalError) > 0.05:
            self.vel.setV(self.currV)
            self.computePositionalError()
        if abs(self.currPositionalError) <= 0.05:
            self.vel.setV(0)
            print "Target Location Reached"
            self.printCurrV = 0
            self.posCorrected = True
            self.computeNextState()

    """ W rite in this mehtod the code necessary for going to the desired place,
        once you have generated the shorter path.
        This method will be periodically called after you press the GO! button. """

    def execute(self):
        #self.computeDesiredRPhi()
        self.correct_yaw()
        if self.yawCorrected:
            self.correctPosition2()
            #self.yawCorrected = False
            #self.correctRelativeDistDiff()
        if self.yawCorrected and self.posCorrected and self.lastIndex!=self.target_ind:
           self.computeNextState()
        if self.yawCorrected and self.posCorrected and self.lastIndex == self.target_ind:
            print "last Index Reached!"
            self.sensor.setV(0)
            self.sensor.setW(0)
        else:
            self.computePositionalError()
            self.printDebugInfo()
            self.yawCorrected = False







        '''curr_r, curr_phi = self.cartesian_to_polar(self.sensor.getRobotX(), self.sensor.getRobotY())
            end_loc_r, end_loc_phi = self.cartesian_to_polar(cx[lastIndex], cy[lastIndex])
            distance_diff = end_loc_r - curr_r
            if lastIndex == target_ind:
                while distance_diff > 0.01:
                    alpha = 0.9
                    K = 5 * (1 - math.exp(-alpha * math.pow(distance_diff, 2))) / abs(distance_diff)
                    curr_vel = K * distance_diff
                    self.vel.setV(curr_vel)
                    curr_r, curr_phi = self.cartesian_to_polar(self.sensor.getRobotX(), self.sensor.getRobotY())
                    end_loc_r, end_loc_phi = self.cartesian_to_polar(cx[lastIndex], cy[lastIndex])
                    distance_diff = end_loc_r - curr_r
                    clearscreen()
                    print "Runnig last mile."
                    print "Distance left: " + str(distance_diff)
            # Then ensure the complete position
            # convert to polar
            curr_r, curr_phi = self.cartesian_to_polar(self.sensor.getRobotX(), self.sensor.getRobotY())
            distance_diff = state.r - curr_r
            while abs(distance_diff) > 0.1:
                clearscreen()  # For better printing
                print "In 2nd While loop, distance correction."
                print "Now Correcting robot location."
                # Measure the distance_diff between current location and target location
                print "distance_diff: " + str(distance_diff)
                alpha = 0.9
                K = 10 * (1 - math.exp(-alpha * math.pow(distance_diff, 2))) / abs(distance_diff)
                curr_vel = K * distance_diff
                if curr_vel < -0.5: curr_vel = -0.01
                self.vel.setV(curr_vel)
                print "last_ind: " + str(lastIndex)
                print "target_ind: " + str(target_ind)
                print "velocity: " + str(curr_vel)
                print "req_heading: " + str(state.phi)
                print "curr_heading: " + str(curr_phi)
                print "diff_heading: " + str(state.phi - curr_phi)
                print "curr_x: " + str(self.sensor.getRobotX())
                print "curr_y: " + str(self.sensor.getRobotY())
                print "req_x: " + str(state.x)
                print "req_y: " + str(state.y)
                print "prev_x: " + str(prev_state.x)
                print "prev_y: " + str(prev_state.y)
                print "curr_r: " + str(state.r)

                curr_r, curr_phi = self.cartesian_to_polar(self.sensor.getRobotX(), self.sensor.getRobotY())
                distance_diff = state.r - curr_r

                if abs(distance_diff) <= 0.1:
                    print "Robot location corrected."
                    self.vel.setV(0)
                    self.vel.setW(0)
                    break

            curr_r, curr_phi = self.cartesian_to_polar(self.sensor.getRobotX(), self.sensor.getRobotY())
            end_loc_r, end_loc_phi = self.cartesian_to_polar(cx[lastIndex], cy[lastIndex])
            distance_diff = end_loc_r - curr_r

            if lastIndex == target_ind and abs(distance_diff) <= 0.1 and abs(distance_diff) > 0.01:
                self.vel.setV(0)
                self.vel.setW(0)
                self.correct_yaw(state.yaw)
                while abs(distance_diff) > 0.01:
                    self.correct_yaw(state.yaw)
                    alpha = 0.9
                    K = 5 * (1 - math.exp(-alpha * math.pow(distance_diff, 2))) / abs(distance_diff)
                    curr_vel = K * distance_diff
                    if curr_vel < -0.5: curr_vel = -0.1
                    self.vel.setV(curr_vel)
                    curr_r, curr_phi = self.cartesian_to_polar(self.sensor.getRobotX(), self.sensor.getRobotY())
                    end_loc_r, end_loc_phi = self.cartesian_to_polar(cx[lastIndex], cy[lastIndex])
                    distance_diff = end_loc_r - curr_r
                    clearscreen()
                    print "Runnig last mile."
                    print "Distance left: " + str(distance_diff)

                self.vel.setV(0)
                self.correct_yaw(state.yaw)
                self.vel.setW(0)
                print "Target Location Reached."

            x.append(state.x)
            y.append(state.y)
            yaw.append(state.yaw)
            v.append(state.v)
            t.append(time)

            if show_animation:
                plt.cla()
                plt.plot(cx, cy, ".r", label="course")
                plt.plot(x, y, "-b", label="trajectory")
                plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
                plt.axis("equal")
                plt.grid(True)
                plt.title("Speed[m/s]:" + str(state.v)[:4])
                plt.pause(0.001)

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
            plt.show()'''












        '''TargetPose = self.findTargetPose(x,y)
        yaw = math.atan2(TargetPose[1]-y,TargetPose[0]-x)
        yaw_dis = yaw - theta
        dis = math.sqrt(pow(x-TargetPose[0],2)+pow(y-TargetPose[1],2))

        if yaw_dis > 3.14:
            yaw_dis -= 6.28
        if yaw_dis < -3.14:
            yaw_dis += 6.28 

        if yaw_dis > 1 or yaw_dis < -1:
            v = 0
            w = yaw_dis
        else:
            v = dis*0.1
            if v<0.1:
                v = 0.1
            w = yaw_dis
        
        print TargetPose
        # self.vel.setV(v)
        print x
        print y
        print theta
        print yaw_dis

        if (TargetPose[2] == 1):
            if dis<1:
                print "get target"
                v = 0
                w = 0
        
        self.vel.setV(v)
        self.vel.setW(w)'''

        #EXAMPLE OF HOW TO SEND INFORMATION TO THE ROBOT ACTUATORS
