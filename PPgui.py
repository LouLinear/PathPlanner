import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

from Tkinter import *
import tkMessageBox
import ttk
import numpy as np
import gridmap as MAP
import pathplanner as PP

class startPrompt:


    """The prompt window that gets launched when one runs the path planner"""

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
        only supports 2D holonomic planning. To plan in 3D, select 3D Planning
        and follow the prompts in the terminal""")

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
            newLv.grab_set()
        elif self.actionCB.get() == self.options[1]:
            self.master.withdraw()
            print "Exiting GUI, Launcing terminal 3D planning UX"
            self.threeDplanUX()
        return

    
    def safe_exit(self):
        self.master.eval('::ttk::CancelRepeat')
        self.master.destroy()
        
    def threeDplanUX(self):
        
        axis_str = ["x", "y", "z"]
        dim = [0, 0, 0]
        
        for i in range(0, 3):
            while True:
                try:
                    tempdim = int(input("Please enter desired %s size(positive integer less than 100 please): " %axis_str[i]))
                    if tempdim > 100 or tempdim < 1:
                        raise ValueError
                    break
                except:
                    print "Entered value must be an integer less than 100"
                    pass
            dim[i] = tempdim

        start = np.array([0, 0, 0])
        dim = np.array(dim)
        goal = dim - 1

        my3Dmap = MAP.GridMapD(dim)
        my3Dmap.rand_obs_gen(0.1)
        Planner3D = PP.PlannerHolo(my3Dmap)


        print \
"""The simulator randomly generated
obstacles with probability
of any given cell of being an obstacle = 0.1
"""

        print \
"""The simulator picked [0, 0, 0] to be
your start point and [%d, %d, %d]  for
your end point since 3D planning is unintuitive and has a lot of cells.
Let's see if we can find a path
"""%tuple(goal)

        
        raw_input("Press enter to start planning...")

        print("Planning, please wait")

        path = Planner3D.plan(start, goal)
        
        if path is not None:
            print "We found a path! your waypoints are:\n"
            Planner3D.printpath(path)
        else:
            print\
"""I am sorry that you can't find a path, 
let's try again
"""

        to_plot = raw_input(\
"""Would you like me to plot the result for you?\n
Warning: It could be very slow for large maps (y/n):"""\
                               )

        if to_plot in ['y', 'Y', 'yes', 'Yes', 'YES']:
            #plot here
            obs_sub = my3Dmap.obs_sparse()
            f = plt.figure()
            a = f.add_subplot(111, projection='3d')
            a.scatter(obs_sub[0], obs_sub[1], obs_sub[2])

            if path is not None:
                x_line = path[:, 0]
                y_line = path[:, 1]
                z_line = path[:, 2]
                a.plot(x_line, y_line, z_line, 'r')

            plt.draw()
            plt.pause(0.01)
        raw_input("Press enter to continue...")
        self.master.deiconify()
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
        self.expL = Label(self.rtF, text="""left click on the map to add obstacles 
        right click to remove obstacles
        red indicates free space
        blue indicates obstacles
        To change the size of the map,
        please enter desired size in the box and press reset
        The size of the map can't go more than 1000\n""", font=20)
        self.expL.grid(row=0, columnspan=2, sticky=N)

        self.xysizeL = Label(self.rtF, text="map size")
        self.xysizeL.grid(row=1, column=0, sticky=E)

        #Entry
        self.mapXYE = Entry(self.rtF)
        self.mapXYE.insert(0, "10")
        self.mapXYE.grid(row=1, column=1, sticky=W)

        #Buttons
        self.planB = Button(self.rbF, text="Plan", command=self.planBclick)
        self.resetB = Button(self.rbF, text="reset", command=self.resetBclick)
        self.randomB = Button(self.rbF, text="random", command=self.randomBclick)

        self.planB.grid(row=2, column=0, sticky=N)
        self.resetB.grid(row=0, column=0, sticky=N)
        self.randomB.grid(row=1, column=0, sticky=N)

        # variables
        self.start = np.array([0, 0])
        self.goal = np.array([1, 1])
        self.forcewait = BooleanVar()
        self.xyres=10

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

        try:
            xyres_temp = int(self.mapXYE.get())
            if xyres_temp > 1000 or xyres_temp < 1:
                raise ValueError
            self.xyres = xyres_temp
        except ValueError:
            tkMessageBox.showwarning("ValueError", \
                                     "Entered value must be an positive integer less than 1000")
            return

        self.my2Dmap = MAP.GridMapD([self.xyres, self.xyres])
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
        self.expL_default()

    def canvas2plot(self, eventx, eventy):
        plotx = round(float(eventy - 60)/400*self.xyres - 0.5)
        ploty = round(float(eventx - 65)/400*self.xyres - 0.5)
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
            self.expL.config(text="Planning, please be patient...\n", font=30)
            self.master.update()
            self.forcewait.set(False)
        return

    def expL_default(self):
        self.expL.config(text="""left click on the map to add obstacles 
        right click to remove
        red indicates free space
        blue indicates obstacles
        To change the size of the map,
        please enter desired size in the box and press reset\n""")
        return

    
    

def run_program():
    root = Tk()
    root.wm_title("Prompt")
    SP = startPrompt(root)
    root.mainloop()
