import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
from matplotlib.figure import Figure

import matplotlib.animation as animation

from Tkinter import *
import ttk
import numpy as np
import gridmap as MAP
import pathplanner as PP

def display_map(m):
    """
    This function takes a gridmap object m and display it in the gui
    """
    pass

class startPrompt:


    def __init__(self, master):
        
        #Frames
        self.master = master
        self.topF = Frame(master)
        self.botF = Frame(master)
        self.lbF = Frame(self.botF)
        self.rbF = Frame(self.botF)

        self.topF.pack(side=TOP)
        self.botF.pack(side=BOTTOM)
        self.rbF.pack(side=RIGHT)
        self.lbF.pack(side=LEFT)

        #instruction
        self.instrL = Label(self.topF, text='Planning Simulator', font=16)
        self.expL = Label(self.topF, text="""The graphical interface currently
        only supports 2D holonomic and nonholonomic planning.
        3D planning can be done via terminal commands""")

        self.instrL.pack()
        self.expL.pack()

        #Dropdown list
        self.options = [
            "2D planning (GUI)",
            "3D planning (No GUI)"
        ]
        self.actionVar = StringVar()
        self.actionCB = ttk.Combobox(self.lbF, textvariable=self.actionVar, \
                                     values=self.options, justify="left")
        self.actionCB.grid(sticky=W) 

        #Buttons
        self.proceedB = Button(self.rbF, text='Proceed')
        self.exitB = Button(self.rbF, text='Exit', command=self.safe_exit)

        self.proceedB.grid(row=0, sticky=W)
        self.exitB.grid(row=1, sticky=W)

        self.proceedB.bind("<Button-1>", self.proceed)

    def proceed(self, event):
        
        #Check the value of chosen action
        if self.actionCB.get() == self.options[0]:
            print "Launching map creator..."
            newLv = Toplevel(self.master)
            newLv.wm_title("Map Creator")
            newLv.resizable(width=FALSE, height=FALSE)
            mC = mapCreator(newLv)
        elif self.actionCB.get() == self.options[1]:
            print "Exiting Gui, entering terminal UX"
            self.threeDplan()
       
        return

    def safe_exit(self):
        self.master.eval('::ttk::CancelRepeat')
        self.master.destroy()
        
    def threeDplan(self):
        
        xdim = input("Enter your desired x dimension of the map: ")
        ydim = input("Enter your desired y dimension of the map: ")
        zdim = input("Enter your desired z dimension of the map: ")

        start = np.array([0, 0, 0])
        dim = np.array([int(xdim), int(ydim), int(zdim)])
        goal = dim - 1

        my3Dmap = MAP.GridMapD(dim)
        my3Dmap.rand_obs_gen(0.1)
        Planner3D = PP.PlannerHolo(my3Dmap)


        print \
"""
The system randomly generated obstacles with probability
of any given cell of being an obstacle = 0.1
"""

        print \
"""
The system picked [0, 0, 0] to be your start point and [%d, %d, %d]  for
your end point since 3D planning is unintuitive and has a lot of cells.
Let's see if we can find a path"
"""%tuple(goal)

        path = Planner3D.plan(start, goal)
        
        if path is not None:
            print "We found a path! your waypoints are:\n"
            Planner3D.printpath(path)
        else:
            print \
"I am sorry that you can't find a path, let's try again by pressing proceed on the prompt window"

        return

class mapCreator:


    def __init__(self, master):


        #Frames
        self.master = master
        self.leftF = Frame(master)
        self.rightF = Frame(master)
        self.rtF = Frame(self.rightF)
        self.rbF = Frame(self.rightF)

        self.leftF.pack(side=LEFT)
        self.rightF.pack(side=RIGHT)
        self.rtF.pack(side=TOP)
        self.rbF.pack(side=BOTTOM)

        #GridMap Object
        self.my2Dmap = MAP.GridMapD()

        #matplotlib objects
        self.f = Figure(figsize=(5,5), dpi=100)
        self.a = self.f.add_subplot(111)
        self.a.imshow(self.my2Dmap._map())
        
        self.canvas = FigureCanvasTkAgg(self.f, self.leftF)
        self.canvas.show()
        self.canvas.get_tk_widget().bind("<Button-1>", self.place_obs)
        self.canvas.get_tk_widget().bind("<Button-3>", self.remove_obs)
        self.canvas.get_tk_widget().pack(fill=BOTH, expand=True)

        toolbar = NavigationToolbar2TkAgg(self.canvas, self.leftF)
        toolbar.update()
        self.canvas._tkcanvas.pack(fill=BOTH, expand=True)

        #Labels
        self.expL = Label(self.rtF, text="left click on the map to add obstacles\n right click to remove\nred is free space", font=20)
        self.expL.pack()

        #Entry
        self.mapX = Entry(self.rtF)
        self.mapY = Entry(self.rtF)
        self.mapX.insert(0, "10")
        self.mapY.insert(0, "10")

        self.mapX.pack()
        self.mapY.pack()
        self.xres=10
        self.yres=10

        #Buttons
        self.planB = Button(self.rbF, text="Plan", command=self.planBclick)
        self.resetB = Button(self.rbF, text="reset", command=self.resetBclick)
        self.randomB = Button(self.rbF, text="random", command=self.randomBclick)

        self.planB.grid(row=2, column=0, sticky=S)
        self.resetB.grid(row=0, column=0, sticky=N)
        self.randomB.grid(row=1, column=0, sticky=N)

        #temporary variables
        self.start = np.array([0, 0])
        self.goal = np.array([1, 1])
        self.forcewait = BooleanVar()

    def update_map(self):
        self.a.clear()
        self.a.imshow(self.my2Dmap._map())
        self.canvas.show()
        return

    def randomBclick(self):
        self.my2Dmap.rand_obs_gen()
        self.update_map()
        return

    def resetBclick(self):
        self.xres = int(self.mapX.get())
        self.yres = int(self.mapY.get())
        self.my2Dmap = MAP.GridMapD([self.xres, self.yres])
        self.update_map()
        return

    def planBclick(self):
        self.update_map()
        #switch into path mode
        self.forcewait.set(True)
        self.expL.config(text="Please click on the map\n to select starting point", font=20)
        self.canvas.get_tk_widget().bind("<Button 1>", self.get_start)
        self.master.wait_variable(self.forcewait)
        
        self.forcewait.set(True)
        self.expL.config(text="Please click on the map\n to select goal", font=20)
        self.canvas.get_tk_widget().bind("<Button 1>", self.get_goal)
        self.master.wait_variable(self.forcewait)
        
        Planner = PP.PlannerHolo(self.my2Dmap)
        path = Planner.plan(self.start, self.goal)
        
        if path is not None:
            #plot path
            print "Way points passed:\n"
            Planner.printpath(path)
            xdata = path[:, 0]
            ydata = path[:, 1]
            self.a.plot(ydata, xdata, 'y')
            self.canvas.show()
        else:
            print "I am sorry, no path can be found"

        #back to map mode
        self.canvas.get_tk_widget().bind("<Button-1>", self.place_obs)
        self.expL.config(text="left click on the map to add obstacles\n right click to remove\n red is free space")

    def canvas2plot(self, eventx, eventy):
        plotx = round(float(eventy - 60)/400*self.yres - 0.5)
        ploty = round(float(eventx - 65)/400*self.xres - 0.5)
        return [plotx, ploty]

    def place_obs(self, event):
        point = self.canvas2plot(event.x, event.y)
        self.my2Dmap.add_obs(point)
        self.update_map()
        return

    def remove_obs(self,event):
        point = self.canvas2plot(event.x, event.y)
        self.my2Dmap.rm_obs(point)
        self.update_map()
        return
        
    def get_start(self, event):
        self.start = np.array(self.canvas2plot(event.x, event.y), dtype=np.intp)
        if self.my2Dmap.access(self.start):
            self.a.autoscale(False)
            self.a.plot(self.start[1], self.start[0], 'yx')
            self.canvas.show()
            self.forcewait.set(False)
        return

    def get_goal(self, event):
        self.goal = np.array(self.canvas2plot(event.x, event.y), dtype=np.intp)
        if self.my2Dmap.access(self.goal):
            self.a.autoscale(False)
            self.a.plot(self.goal[1], self.goal[0], 'yx')
            self.canvas.show()
            self.forcewait.set(False)
        return
    

def run_program():
    root = Tk()
    root.wm_title("Prompt")
    SP = startPrompt(root)
    root.mainloop()
