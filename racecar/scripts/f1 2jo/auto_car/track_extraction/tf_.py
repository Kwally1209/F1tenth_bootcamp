import numpy as np
import math
import matplotlib.pyplot as plt
import statistics
import yaml
from scipy.optimize import curve_fit
from scipy import interpolate

class reference():
    def __init__(self, filename,seg_dis,sc):

        wpts = np.loadtxt(filename,  delimiter=",", dtype = float)        
        # plt.plot(wpts[:,0], wpts[:,1],'*')
        # self.tf(wpts,'*')
        curv = self.calc_curv(wpts)
        psi = self.calc_psi(wpts)

        self.wpts = np.zeros((len(wpts),4))
        self.wpts[:,0:2] = wpts[:,0:2]
        self.wpts[:,2] = psi
        self.wpts[:,3] = curv

        self.seg_dis = seg_dis
        self.sc = sc
    def devide_segmentation(self):
        

        #Devide to segementation
        s_wpts = []
        idx = 0
        for j in range(0,len(self.wpts),self.seg_dis):
            if j != 0:
                if j < len(self.wpts) - 3:
                    s_wpts.append(self.wpts[idx:j+1])
                    idx = j
                else:
                    wp = np.zeros((len(self.wpts)-idx+1,4))
                    wp[0:-1] = self.wpts[idx:]
                    wp[-1] = self.wpts[0]
                    s_wpts.append(wp)
                    idx = len(self.wpts)
                    break
                

        if idx != len(self.wpts):
            wp = np.zeros((len(self.wpts)-idx+1,4))
            wp[0:-1] = self.wpts[idx:]
            wp[-1] = self.wpts[0]
            s_wpts.append(wp)


        self.s_wpts = s_wpts

    

    def fitting(self,ds):
        
        self.traj = []
        
        for i in range(len(self.s_wpts)):
            # self.tf(self.s_wpts[i][:,0:2], 'b')
            curv_m = np.mean(self.s_wpts[i][1:-1,3])
            psi_m = np.mean(self.s_wpts[i][1:-1,2])
            print("mean of curv is", curv_m)
            print("mean of psi is", math.degrees(psi_m))

            old_dis_h = 0
            old_dis_v = 0
            for j in range(1,len(self.s_wpts[i])):
                dis_h = math.sqrt((self.s_wpts[i][0,0] - self.s_wpts[i][j,0])**2)
                if old_dis_h < dis_h:
                    old_dis_h = dis_h
                
                dis_v = math.sqrt((self.s_wpts[i][0,1] - self.s_wpts[i][j,1])**2)
                if old_dis_v < dis_v:
                    old_dis_v = dis_v
            
            # print("dis:", dis)

            # Determine the road structure
            if abs(curv_m) > 0.25:
                mode = "C"
            elif abs(math.degrees(psi_m)) > 70 and abs(math.degrees(psi_m)) < 110:
                mode = "V"
            else:
                mode = "H"

            if old_dis_h <=self.sc and old_dis_v > old_dis_h:
                mode = "V"
            elif old_dis_v <= self.sc:
                mode = "H"

            CheckVert = True
            CheckHori = True
            Rev = False
            for j in range(1,len(self.s_wpts[i])):
                if self.s_wpts[i][j-1,0] > self.s_wpts[i][j,0]:
                    CheckHori = False
                if self.s_wpts[i][j-1,1] > self.s_wpts[i][j,1]:
                    CheckVert = False
            
            if CheckHori == False and CheckVert == False:
                CheckVert, CheckHori, Rev = True, True, True
                old_j = len(self.s_wpts[i])-1
                for j in reversed(range(len(self.s_wpts[i])-1)): 
                    if self.s_wpts[i][old_j,0] > self.s_wpts[i][j,0]:
                        CheckHori = False
                    if self.s_wpts[i][old_j,1] > self.s_wpts[i][j,1]:
                        CheckVert = False
                    old_j = j
            
            if mode == "H" and CheckHori == False:
                Rev = True
            if mode ==  "V" and CheckVert == False:
                Rev = True

            print("direction is: CheckHori ", CheckHori, "CheckVert ", CheckVert, "mode", mode, "rev", Rev)
            
            
            if CheckHori == True or mode == "H":
                idxs = []
                for j in range(1,len(self.s_wpts[i])):
                    if self.s_wpts[i][j-1,0] == self.s_wpts[i][j,0]:
                        idxs.append(j)
                
                for w in range(len(idxs)):
                    if Rev:
                        self.s_wpts[i][idxs[w],0] = self.s_wpts[i][idxs[w]-1,0] - 0.0001
                    else:
                        self.s_wpts[i][idxs[w],0] = self.s_wpts[i][idxs[w]-1,0] + 0.0001

            elif CheckVert == True or mode == "V":
                idxs = []
                for j in range(1,len(self.s_wpts[i])):
                    if self.s_wpts[i][j-1,1] == self.s_wpts[i][j,1]:
                        idxs.append(j)
                
                for w in range(len(idxs)):

                    if Rev:
                        self.s_wpts[i][idxs[w],1] = self.s_wpts[i][idxs[w]-1,1] - 0.0001
                    else:
                        self.s_wpts[i][idxs[w],1] = self.s_wpts[i][idxs[w]-1,1] + 0.0001


            if mode == "C":
                if Rev:
                    if CheckHori == True:
                        self.s_wpts[i] = self.s_wpts[i][np.argsort(self.s_wpts[i][:,0])]
                        f = interpolate.CubicSpline(self.s_wpts[i][:,0], self.s_wpts[i][:,1])
                        x_new = np.arange(self.s_wpts[i][0,0], self.s_wpts[i][-1,0], ds)
                        y_new = f(x_new)
                    elif CheckVert == True:
                        self.s_wpts[i] = self.s_wpts[i][np.argsort(self.s_wpts[i][:,1])]
                        f = interpolate.CubicSpline(self.s_wpts[i][:,1], self.s_wpts[i][:,0])
                        y_new = np.arange(self.s_wpts[i][0,1], self.s_wpts[i][-1,1], ds)
                        x_new = f(y_new)
                else:
                    if CheckHori == True:
                        f = interpolate.CubicSpline(self.s_wpts[i][:,0], self.s_wpts[i][:,1])
                        x_new = np.arange(self.s_wpts[i][0,0], self.s_wpts[i][-1,0], ds)
                        y_new = f(x_new)
                        
                    elif CheckVert == True:
                        f = interpolate.CubicSpline(self.s_wpts[i][:,1], self.s_wpts[i][:,0])
                        # y_new = np.arange(self.s_wpts[i-1][-1,1], self.s_wpts[i][-1,1], ds)
                        y_new = np.arange(self.s_wpts[i][0,1], self.s_wpts[i][-1,1], ds)
                        x_new = f(y_new)

            elif mode == "H":
                if Rev:
                    self.s_wpts[i] = self.s_wpts[i][np.argsort(self.s_wpts[i][:,0])]
                    popt,pcov = curve_fit(self.func_l,self.s_wpts[i][:,0], self.s_wpts[i][:,1])
                    print(pcov)
                    # y_new = np.arange(self.s_wpts[i][0,1], self.s_wpts[i-1][-1,1], ds)
                    x_new = np.arange(self.s_wpts[i][0,0], self.s_wpts[i][-1,0], ds)
                    y_new = self.func_l(x_new, *popt)
                else:
                    popt,pcov = curve_fit(self.func_l,self.s_wpts[i][:,0], self.s_wpts[i][:,1])
                    print(pcov)
                    # x_new = np.arange(self.s_wpts[i-1][-1,0], self.s_wpts[i][-1,0], ds)
                    x_new = np.arange(self.s_wpts[i][0,0], self.s_wpts[i][-1,0], ds)
                    y_new = self.func_l(x_new, *popt)
                
            else:
                if Rev:
                    self.s_wpts[i] = self.s_wpts[i][np.argsort(self.s_wpts[i][:,1])]
                    popt,pcov = curve_fit(self.func_l,self.s_wpts[i][:,1], self.s_wpts[i][:,0])
                    # x_new = np.arange(self.s_wpts[i][0,0], self.s_wpts[i-1][-1,0], ds)
                    y_new = np.arange(self.s_wpts[i][0,1], self.s_wpts[i][-1,1], ds)
                    x_new = self.func_l(y_new, *popt)
                else:
                    popt,pcov = curve_fit(self.func_l, self.s_wpts[i][:,1], self.s_wpts[i][:,0])
                    # y_new = np.arange(self.s_wpts[i-1][-1,1], self.s_wpts[i][-1,1], ds)
                    y_new = np.arange(self.s_wpts[i][0,1], self.s_wpts[i][-1,1], ds)
                    x_new = self.func_l(y_new, *popt)

            self.insert(self.traj,x_new,y_new,Rev)
            
            plottf = np.zeros((len(x_new),2))
            plottf[:,0] = x_new
            plottf[:,1] = y_new
            # self.tf(plottf,'r*')
        
        self.traj = np.array(self.traj)


    def func(self,x,a,b,c,d,e,f):
        return  (a * x) +  (b* x**2) + (c* x**3) + (d* x**4) + (e* x**5) + f

    def func_c(self,x,a,b,c):
        return  a*np.exp(b*x) + c

    def func_l(self,x,a,b,c):
        return  (a * x) + (b * x**2) + c 

    def insert(self,traj, x,y,rev):
        if rev: 
            for i in reversed(range(len(x))):
                A = [x[i], y[i]]
                traj.append(A)
        else:
            for i in range(len(x)):
                A = [x[i], y[i]]
                traj.append(A)

    def calc_psi(self,traj):
        psi_ref = []

        y = traj[0,1] - traj[-1,1] 
        x = traj[0,0] - traj[-1,0]
        psi = math.atan2(y,x)
        psi_ref.append(psi)

        old_psi = psi
        for i in range(1,len(traj)):
            y = traj[i,1] - traj[i-2,1] 
            x = traj[i,0] - traj[i-2,0]
            psi = math.atan2(y,x)

            # if abs(old_psi) < 3.0 and abs(old_psi - psi) > 3:
            #     psi = old_psi
            # elif abs(old_psi - psi) > 6 and psi > old_psi:
            #     psi = old_psi
            
            old_psi = psi
            psi_ref.append(psi)


        return psi_ref
    
    def calc_curv(self,traj):
        dx = np.gradient(traj[:,0])
        dy = np.gradient(traj[:,1])

        d2x = np.gradient(dx)
        d2y = np.gradient(dy)

        curv_ref = (dx * d2y - d2x * dy) / (dx * dx + dy * dy)**1.5

        return curv_ref


    def save_reference(self, filename, smooth=True):
        
        self.reference = []
        curvature = self.calc_curv(self.traj)
        psi = self.calc_psi(self.traj)
        
        #Check CW or CCW
        n = len(psi)//3
        n1 = np.mean(psi[:n])
        n2 = np.mean(psi[n:])
        if n1 > n2:
            mode = "CCW"
        else:
            mode = "CW"

        if mode == "CW":
            self.traj = np.flip(self.traj, axis = 0)
            curvature = np.flip(curvature)
            psi = np.flip(psi)

        if smooth:
            
            psi_old = 0
            for i in range(len(self.traj)):
                if i != 0 and abs(psi_old)< 3.0 and abs(psi_old-psi[i])> 0.5:
                    psi_old = psi[i]
                    continue    
                ref = [self.traj[i,0], self.traj[i,1], psi[i], curvature[i]]
                self.reference.append(ref)

            self.reference = np.array(self.reference)
            np.savetxt(filename, self.reference,  delimiter = ',')   
        else:
        
            psi_old = 0
            for i in range(len(self.traj)):
                if i != 0 and abs(psi_old)<3 and abs(psi_old-psi[i])> 3:
                    psi_old = psi[i]  
                    continue
                  
                ref = [self.traj[i,0], self.traj[i,1], psi[i], curvature[i]]
                self.reference.append(ref)

            self.reference = np.array(self.reference)
            np.savetxt(filename, self.reference,  delimiter = ',')    

    # def tf(self, traj, color):
    #     tf_wpts = np.zeros((len(traj),2))
    #     tf_wpts[:,1] = 1*traj[:,1]
    #     tf_wpts[:,0] = traj[:,0] + 51.224998
    #     tf_wpts[:,1] = traj[:,1] + 51.224998
        
    #     tf_wpts[:,0] *= 10
    #     tf_wpts[:,1] *= 10
        
    #     plt.plot(tf_wpts[:,0], tf_wpts[:,1], color)
    
    def tf(self, traj, color):
   
        tf_wpts = np.zeros((len(traj),2))

        # Need to change !!
        tf_wpts[:,0] = traj[:,0] + (-51.224998)
        tf_wpts[:,1] = traj[:,1] + (-51.224998)
        
        tf_wpts[:,0] *= 20
        tf_wpts[:,1] *= 20
        
        plt.plot(tf_wpts[:,0], tf_wpts[:,1], color)
    
    
    
    
        


def main():


    filename = 'map_data2-centerline.csv'
    ref = reference(filename,3,1.5)
    ref.devide_segmentation()
    ref.fitting(0.2)
    ref.save_reference("centerline_txt")
    ref.tf(ref.reference,'r')
    print("centerline_done")

    filename = 'map_data2-outerwall.csv'
    ref = reference(filename,4,2.5)
    ref.devide_segmentation()
    ref.fitting(0.2)
    ref.save_reference("outerline_txt", False)
    ref.tf(ref.reference,'k')
    print("done")


    filename = 'map_data2-innerwall.csv'
    ref = reference(filename,3,1)
    ref.devide_segmentation()
    ref.fitting(0.2)
    ref.save_reference("innerline_txt", False)
    ref.tf(ref.reference,'k')
    print("innerline_done")


    

    map = plt.imread("map_data2.pgm")
    plt.imshow(map)
    
    
    plt.show()

main()