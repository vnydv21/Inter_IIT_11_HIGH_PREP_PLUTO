from Omen.pluto import Pluto
from Omen.pid import PID
from Omen.aruco import track_aruco
import time
import cv2
import numpy as np
import threading
from PIL import Image, ImageTk
import json
import customtkinter
import math

controls = {
    "rollLeft": "Left",
    "rollRight": "Right",
    "pitchUp": "Up",
    "pitchDown": "Down",
    "yawClock": "d",
    "yawAntiClock": "a",
    "throttleUp": "w",
    "throttleDown": "s",
    "connect": "F1",
    "arm": "F2",
    "takeOffLand": "space"
}

file = open('./Omen/callibration_parameters.txt','r')
cal = json.loads(file.read())
file.close()

zavg = cal['zavg']
xavg = cal['xavg']
yavg = cal['yavg']
defPitch = cal['defPitch']
defRoll = cal['defRoll']
defThrottle = cal['defThrottle']

drone = Pluto(zavg=zavg, xavg=xavg,yavg=yavg,defPitch=defPitch,defRoll=defRoll,defThrottle=defThrottle)

customtkinter.set_appearance_mode("Dark")  # Modes: "System" (standard), "Dark", "Light"
customtkinter.set_default_color_theme("blue")  # Themes: "blue" (standard), "green", "dark-blue"

DEF = "#3B8ED0"
DEFD = "#36719F"

UI_REFRESH = 1

class PlusMinus(customtkinter.CTkFrame):
    def __init__(self, parent, label, val, step, action):
        customtkinter.CTkFrame.__init__(self, parent)
        self.action= action
        self.step = step

        self.grid(row=2, column=1, rowspan=1, sticky="nsew")
        self.grid_columnconfigure((0,2), weight=0)
        self.grid_columnconfigure((1), weight=0)

        self.label = customtkinter.CTkLabel(self, text=label)
        self.label.grid(row=0, column=0, padx=20, pady=1, columnspan=3)

        self.min = customtkinter.CTkButton(self, text="-", width=30, command=self.minf)
        self.min.grid(row=1, column=0, padx=20, pady=1)

        self.val = customtkinter.CTkLabel(self, text=val, width=30)
        self.val.grid(row=1, column=1, padx=2, pady=1)

        self.plus = customtkinter.CTkButton(self, text="+", width=30, command=self.plusf)
        self.plus.grid(row=1, column=2, padx=20, pady=1)

    def plusf(self):
        newval = int(self.val.cget("text"))+self.step
        self.val.configure(text=newval)
        self.action(self.label.cget("text"),newval)

    def minf(self):
        newval = int(self.val.cget("text"))-self.step
        self.val.configure(text=newval)
        self.action(self.label.cget("text"),newval)

    

class App(customtkinter.CTk):
    def __init__(self):
        super().__init__()

        # Var
        self.keys_pressed = []
        self.stream_req = 0
        self.controller_req = 0
        self.frame = None
        self.mode_active = "FreeFly"
        self.camflyspeed = 1
        self.autoflyspeed = 1
        self.camw = 1
        self.camh = 1
        self.frame_size = 500
        self.path_points = []
        self.mousex = 0
        self.mousey = 0
        self.inautofly = 0
        self.drone1_id = 10
        self.drone2_id = 85

        # configure window
        self.title("IIT Ropar | Pluto Controller")
        self.geometry(f"{1100}x{580}")

        # configure grid layout (4x4)
        self.grid_columnconfigure((1), weight=1)
        self.grid_columnconfigure((2, 3), weight=0)
        self.grid_rowconfigure((0, 1, 2), weight=1)

        # create sidebar frame with widgets
        self.sidebar_frame = customtkinter.CTkFrame(self, corner_radius=0)
        self.sidebar_frame.grid(row=0, column=0, rowspan=2, sticky="nsew")
        self.sidebar_frame.grid_rowconfigure(4, weight=1)
        self.logo_label = customtkinter.CTkLabel(self.sidebar_frame, text="Drone 1", font=customtkinter.CTkFont(size=20, weight="bold"))
        self.logo_label.grid(row=0, column=0, padx=20, pady=(20, 10))

        self.drone1_id_label = customtkinter.CTkLabel(self.sidebar_frame, text="Drone 1 Aruco ID")
        self.drone1_id_label.grid(row=0, column=0, padx=20, pady=1)
        self.drone1_id_input = customtkinter.CTkEntry(self.sidebar_frame)
        self.drone1_id_input.insert(0,str(self.drone1_id))
        self.drone1_id_input.grid(row=1, column=0, padx=20, pady=1)
        self.drone1_id_update = customtkinter.CTkButton(self.sidebar_frame, text="Update", command=self.drone1idupdatef)
        self.drone1_id_update.grid(row=2, column=0, padx=20, pady=1)
        
        self.connect_button_1 = customtkinter.CTkButton(self.sidebar_frame)
        self.connect_button_1.grid(row=3, column=0, padx=20, pady=10)

        self.arm_button = customtkinter.CTkButton(self.sidebar_frame)
        self.arm_button.grid(row=4, column=0, padx=20, pady=10)

        self.TakeOffLand = customtkinter.CTkButton(self.sidebar_frame)
        self.TakeOffLand.grid(row=5, column=0, padx=20, pady=10)

        self.appearance_mode_label = customtkinter.CTkLabel(self.sidebar_frame, text="Appearance Mode:", anchor="w")
        self.appearance_mode_label.grid(row=6, column=0, padx=20, pady=(10, 0))
        self.appearance_mode_optionemenu = customtkinter.CTkOptionMenu(self.sidebar_frame, values=["Light", "Dark", "System"], command=self.change_appearance_mode_event)
        self.appearance_mode_optionemenu.grid(row=6, column=0, padx=20, pady=(10, 10))
        self.scaling_label = customtkinter.CTkLabel(self.sidebar_frame, text="UI Scaling:", anchor="w")
        self.scaling_label.grid(row=7, column=0, padx=20, pady=(10, 0))
        self.scaling_optionemenu = customtkinter.CTkOptionMenu(self.sidebar_frame, values=["80%", "90%", "100%", "110%", "120%"],command=self.change_scaling_event)
        self.scaling_optionemenu.grid(row=8, column=0, padx=20, pady=(10, 20))

        # Callibration
        self.callibration_frame = customtkinter.CTkFrame(self, corner_radius=0)
        self.callibration_frame.grid(row=7, column=0, sticky="nsew")
        self.cal_label = customtkinter.CTkLabel(self.callibration_frame, text="Callibration", font=customtkinter.CTkFont(size=20, weight="bold"))
        self.cal_label.grid(row=0, column=0, padx=20, pady=(20, 10))
        self.pitch = PlusMinus(self.callibration_frame, "defPitch", defPitch, 1, self.callibrationf)
        self.pitch.grid(row=1, column=0, padx=20, pady=10)
        self.roll = PlusMinus(self.callibration_frame, "defRoll", defRoll, 1, self.callibrationf)
        self.roll.grid(row=2, column=0, padx=20, pady=10)
        self.yaw = PlusMinus(self.callibration_frame, "defThrottle", defThrottle, 1, self.callibrationf)
        self.yaw.grid(row=3, column=0, padx=20, pady=10)

        # create main entry and button
        # self.entry = customtkinter.CTkEntry(self, placeholder_text="CTkEntry")
        # self.entry.grid(row=3, column=1, columnspan=2, padx=(20, 0), pady=(20, 20), sticky="nsew")

        # self.main_button_1 = customtkinter.CTkButton(master=self, fg_color="transparent", border_width=2, text_color=("gray10", "#DCE4EE"))
        # self.main_button_1.grid(row=3, column=3, padx=(20, 20), pady=(20, 20), sticky="nsew")

        # create Videobox
        # self.Vid = customtkinter.CTkLabel(self, text="", height=400)
        # self.Vid.grid(row=0, column=1, padx=20, pady=20, sticky="nsew")
        # self.VidCan = customtkinter.CTkCanvas(self.Vid, bg='black', height=600)
        # self.VidCan.grid(row=0, column=0, sticky='nswe')
        # self.VidCan.pack(anchor='nw', fill='both', expand=1)

        # create tabview
        # self.mode = customtkinter.CTkTabview(self, command=self.modef)
        # self.mode.grid(row=1, column=1, padx=20, pady=20, sticky="nsew", rowspan=10)
        # self.mode.add("FreeFly")
        # self.mode.add("CamFly")
        # self.mode.add("AutoFly")

        # self.tabview.tab("CTkTabview").grid_columnconfigure(0, weight=1)  # configure grid of individual tabs
        # self.tabview.tab("Tab 2").grid_columnconfigure(0, weight=1)

        # Freefly
        # self.controller = customtkinter.CTkFrame(self.mode.tab("FreeFly"), fg_color="transparent")
        # self.controller.grid(row=0, column=0, padx=(20, 0), pady=(20, 0), sticky="nsew")
        # self.controller.grid_columnconfigure((0,1,2), weight=1)
        # self.up = customtkinter.CTkButton(self.controller, text="u", command=self.upf)
        # self.up.grid(row=0, column=1, padx=20, pady=10)
        # self.left = customtkinter.CTkButton(self.controller, text="l", command=self.leftf)
        # self.left.grid(row=1, column=0, padx=20, pady=10)
        # self.right = customtkinter.CTkButton(self.controller, text="r", command=self.rightf)
        # self.right.grid(row=1, column=2, padx=20, pady=10)
        # self.down = customtkinter.CTkButton(self.controller, text="d", command=self.downf)
        # self.down.grid(row=2, column=1, padx=20, pady=10)

        # Camfly
        # self.camflyframe = customtkinter.CTkFrame(self.mode.tab("CamFly"), fg_color="transparent")
        # self.camflyframe.grid(row=0, column=0, padx=(20, 0), pady=(20, 0), sticky="nsew")
        # self.camflyspeedbutton = PlusMinus(self.camflyframe, "Speed", self.camflyspeed, 1, self.camflyspeedf)
        # self.camflyspeedbutton.grid(row=0, column=0, padx=20, pady=10)

        # AutoFly
        # self.autoflyframe = customtkinter.CTkFrame(self.mode.tab("AutoFly"), fg_color="transparent")
        # self.autoflyframe.grid(row=0, column=0, padx=(20, 0), pady=(20, 0), sticky="nsew")

        # self.clear_path = customtkinter.CTkButton(self.autoflyframe, text="Clear Path", command=self.clearpathf)
        # self.clear_path.grid(row=0, column=0, padx=20, pady=10)

        # self.label_x = customtkinter.CTkLabel(self.autoflyframe, text="X")
        # self.label_x.grid(row=0, column=1, padx=20, pady=2)
        # self.x_point = customtkinter.CTkEntry(self.autoflyframe)
        # self.x_point.grid(row=0, column=2, padx=20, pady=2)
        # self.label_y = customtkinter.CTkLabel(self.autoflyframe, text="Y")
        # self.label_y.grid(row=1, column=1, padx=20, pady=2)
        # self.y_point = customtkinter.CTkEntry(self.autoflyframe)
        # self.y_point.grid(row=1, column=2, padx=20, pady=2)
        # self.add_point = customtkinter.CTkButton(self.autoflyframe, text="Add Point", command=self.addpointf)
        # self.add_point.grid(row=2, column=1, padx=20, pady=5, columnspan=2)

        # self.autoflyspeedbutton = PlusMinus(self.autoflyframe, "Speed", self.autoflyspeed, 1, self.autoflyspeedf)
        # self.autoflyspeedbutton.grid(row=0, column=3, padx=20, pady=10)

        # self.startautoflybutton = customtkinter.CTkButton(self.autoflyframe, text="Start", command=self.startautoflyf)
        # self.startautoflybutton.grid(row=2, column=4, padx=20, pady=5, columnspan=2)


        # self.optionmenu_1 = customtkinter.CTkOptionMenu(self.tabview.tab("FreeFly"), dynamic_resizing=False,values=["Value 1", "Value 2", "Value Long Long Long"])
        # self.optionmenu_1.grid(row=0, column=0, padx=20, pady=(20, 10))
        # self.combobox_1 = customtkinter.CTkComboBox(self.tabview.tab("CTkTabview"),
        #                                             values=["Value 1", "Value 2", "Value Long....."])
        # self.combobox_1.grid(row=1, column=0, padx=20, pady=(10, 10))
        # self.string_input_button = customtkinter.CTkButton(self.tabview.tab("CTkTabview"), text="Open CTkInputDialog",
        #                                                    command=self.open_input_dialog_event)
        # self.string_input_button.grid(row=2, column=0, padx=20, pady=(10, 10))
        # self.label_tab_2 = customtkinter.CTkLabel(self.tabview.tab("Tab 2"), text="CTkLabel on Tab 2")
        # self.label_tab_2.grid(row=0, column=0, padx=20, pady=20)

        # # create radiobutton frame
        # self.radiobutton_frame = customtkinter.CTkFrame(self)
        # self.radiobutton_frame.grid(row=0, column=3, padx=(20, 20), pady=(20, 0), sticky="nsew")
        # self.radio_var = tkinter.IntVar(value=0)
        # self.label_radio_group = customtkinter.CTkLabel(master=self.radiobutton_frame, text="CTkRadioButton Group:")
        # self.label_radio_group.grid(row=0, column=2, columnspan=1, padx=10, pady=10, sticky="")
        # self.radio_button_1 = customtkinter.CTkRadioButton(master=self.radiobutton_frame, variable=self.radio_var, value=0)
        # self.radio_button_1.grid(row=1, column=2, pady=10, padx=20, sticky="n")
        # self.radio_button_2 = customtkinter.CTkRadioButton(master=self.radiobutton_frame, variable=self.radio_var, value=1)
        # self.radio_button_2.grid(row=2, column=2, pady=10, padx=20, sticky="n")
        # self.radio_button_3 = customtkinter.CTkRadioButton(master=self.radiobutton_frame, variable=self.radio_var, value=2)
        # self.radio_button_3.grid(row=3, column=2, pady=10, padx=20, sticky="n")

        # # create checkbox and switch frame
        # self.checkbox_slider_frame = customtkinter.CTkFrame(self)
        # self.checkbox_slider_frame.grid(row=1, column=3, padx=(20, 20), pady=(20, 0), sticky="nsew")
        # self.checkbox_1 = customtkinter.CTkCheckBox(master=self.checkbox_slider_frame)
        # self.checkbox_1.grid(row=1, column=0, pady=(20, 10), padx=20, sticky="n")
        # self.checkbox_2 = customtkinter.CTkCheckBox(master=self.checkbox_slider_frame)
        # self.checkbox_2.grid(row=2, column=0, pady=10, padx=20, sticky="n")
        # self.switch_1 = customtkinter.CTkSwitch(master=self.checkbox_slider_frame, command=lambda: print("switch 1 toggle"))
        # self.switch_1.grid(row=3, column=0, pady=10, padx=20, sticky="n")
        # self.switch_2 = customtkinter.CTkSwitch(master=self.checkbox_slider_frame)
        # self.switch_2.grid(row=4, column=0, pady=(10, 20), padx=20, sticky="n")


        

    

        # create slider and progressbar frame
        # self.slider_progressbar_frame = customtkinter.CTkFrame(self, fg_color="transparent")
        # self.slider_progressbar_frame.grid(row=1, column=1, columnspan=2, padx=(20, 0), pady=(20, 0), sticky="nsew")
        # self.slider_progressbar_frame.grid_columnconfigure(0, weight=1)
        # self.slider_progressbar_frame.grid_rowconfigure(4, weight=1)
        # self.seg_button_1 = customtkinter.CTkSegmentedButton(self.slider_progressbar_frame)
        # self.seg_button_1.grid(row=0, column=0, padx=(20, 10), pady=(10, 10), sticky="ew")
        # self.progressbar_1 = customtkinter.CTkProgressBar(self.slider_progressbar_frame)
        # self.progressbar_1.grid(row=1, column=0, padx=(20, 10), pady=(10, 10), sticky="ew")
        # self.progressbar_2 = customtkinter.CTkProgressBar(self.slider_progressbar_frame)
        # self.progressbar_2.grid(row=2, column=0, padx=(20, 10), pady=(10, 10), sticky="ew")
        # self.slider_1 = customtkinter.CTkSlider(self.slider_progressbar_frame, from_=0, to=1, number_of_steps=4)
        # self.slider_1.grid(row=3, column=0, padx=(20, 10), pady=(10, 10), sticky="ew")
        # self.slider_2 = customtkinter.CTkSlider(self.slider_progressbar_frame, orientation="vertical")
        # self.slider_2.grid(row=0, column=1, rowspan=5, padx=(10, 10), pady=(10, 10), sticky="ns")
        # self.progressbar_3 = customtkinter.CTkProgressBar(self.slider_progressbar_frame, orientation="vertical")
        # self.progressbar_3.grid(row=0, column=2, rowspan=5, padx=(10, 20), pady=(10, 10), sticky="ns")

        # set default values
        
        # self.checkbox_2.configure(state="disabled")
        # self.switch_2.configure(state="disabled")
        # self.checkbox_1.select()
        # self.switch_1.select()
        # self.radio_button_3.configure(state="disabled")
        self.appearance_mode_optionemenu.set("Dark")
        # self.scaling_optionemenu.set("100%")
        # self.optionmenu_1.set("CTkOptionmenu")
        # self.combobox_1.set("CTkComboBox")
        # self.slider_1.configure(command=self.progressbar_2.set)
        # self.slider_2.configure(command=self.progressbar_3.set)
        # self.progressbar_1.configure(mode="indeterminnate")
        # self.progressbar_1.start()
        # # self.textbox.insert("0.0", "CTkTextbox\n\n" + "Lorem ipsum dolor sit amet, consetetur sadipscing elitr, sed diam nonumy eirmod tempor invidunt ut labore et dolore magna aliquyam erat, sed diam voluptua.\n\n" * 20)
        # self.seg_button_1.configure(values=["CTkSegmentedButton", "Value 2", "Value 3"])
        # self.seg_button_1.set("Value 2")
        self.arm_button_def()
        self.connect_button_def()
        self.takeoffland_button_def()
        # self.stream_def()

        self.bind("<KeyPress>", self.keydown)
        self.bind("<KeyRelease>", self.keyup)
        # self.VidCan.bind("<Button-1>", self.leftButtonPressed)
        # self.VidCan.bind("<Motion>", self.mouseMotion)
        

    def leftButtonPressed(self, event):
        imgx = int((event.x + (self.camw/self.camh*self.frame_size)/2 - self.VidCan.winfo_width()/2)/(self.camw/self.camh*self.frame_size)*self.camw)
        imgy = int((event.y + self.frame_size/2 - self.VidCan.winfo_height()/2)/self.frame_size*self.camh)
        if self.mode_active == "CamFly":
            drone.xavg = imgx
            drone.yavg = imgy
        elif self.mode_active == "AutoFly":
            self.path_points.append((imgx,imgy))

    def mouseMotion(self, event):
        imgx = int((event.x + (self.camw/self.camh*self.frame_size)/2 - self.VidCan.winfo_width()/2)/(self.camw/self.camh*self.frame_size)*self.camw)
        imgy = int((event.y + self.frame_size/2 - self.VidCan.winfo_height()/2)/self.frame_size*self.camh)
        self.mousex = imgx
        self.mousey = imgy
    

    def modef(self):
        self.mode_active = self.mode.get()
        if self.mode_active=="FreeFly":
            self.controller_req = 0
            self.stream_req=0
        elif self.mode_active=="CamFly":
            self.controller_req = 1
            self.stream_req+=1
        elif self.mode_active=="AutoFly":
            self.controller_req = 1
            self.stream_req+=1
        
        if self.stream_req==1:
            stream_t = threading.Thread(target=self.stream, args=(self.VidCan,))
            stream_t.daemon = 1
            stream_t.start()

    def arm_button_def(self):
        self.arm_button.configure(state="disabled", command=self.armf, text="Arm", fg_color=DEF, hover_color=DEFD)

    def connect_button_def(self):
        self.connect_button_1.configure(command=self.connectf, text="Connect", fg_color="green", hover_color="darkgreen")

    def takeoffland_button_def(self):
        self.TakeOffLand.configure(state="disabled", command=self.takeofff, text="Take Off", fg_color=DEF, hover_color=DEFD)

    def stream_def(self):
        tmp = Image.fromarray(np.zeros((400,600)))
        tmp = customtkinter.CTkImage(tmp,size=(600,400))
        self.Vid.configure(image=tmp)

    def open_input_dialog_event(self):
        dialog = customtkinter.CTkInputDialog(text="Type in a number:", title="CTkInputDialog")
        print("CTkInputDialog:", dialog.get_input())

    def change_appearance_mode_event(self, new_appearance_mode: str):
        customtkinter.set_appearance_mode(new_appearance_mode)

    def change_scaling_event(self, new_scaling: str):
        new_scaling_float = int(new_scaling.replace("%", "")) / 100
        customtkinter.set_widget_scaling(new_scaling_float)

    def drone1idupdatef(self):
        self.drone1_id = self.drone1_id_input.get()

    def connectf(self):
        if drone.connect():
            self.connect_button_1.configure(fg_color="red", hover_color="darkred", text="Disconnect", command=self.disconnectf)
            self.arm_button.configure(state="normal")
        

    def disconnectf(self):
        drone.disconnect()
        
        self.connect_button_def()
        self.arm_button_def()
        self.takeoffland_button_def()

    def armf(self):
        drone.arm()

        self.arm_button.configure(fg_color="grey", hover_color="darkgrey", text="Unarm", command=self.unarmf)
        self.TakeOffLand.configure(state="normal")

    def unarmf(self):
        drone.land()
        drone.disarm()

        self.arm_button_def()
        self.arm_button.configure(state="normal")
        self.takeoffland_button_def()


    def takeofff(self):
        drone.takeoff()

        self.TakeOffLand.configure(fg_color="grey", hover_color="darkgrey", text="Land", command=self.landf)

    def landf(self):
        drone.land()

        self.takeoffland_button_def()
        self.arm_button.configure(state="normal")

    def clearpathf(self):
        self.path_points.clear()

    def addpointf(self):
        self.path_points.append((int(self.x_point.get()),int(self.y_point.get())))

    def startautoflyf(self):
        self.inautofly = 1
        self.clear_path.configure(state="disabled")
        self.x_point.configure(state="disabled")
        self.y_point.configure(state="disabled")
        self.add_point.configure(state="disabled")
        self.startautoflybutton.configure(text="Stop", command=self.stopautoflyf)
        # self.label_y = customtkinter.CTkLabel(self.autoflyframe, text="Y")
        # self.label_y.grid(row=1, column=1, padx=20, pady=2)
        # self.y_point = customtkinter.CTkEntry(self.autoflyframe)
        # self.y_point.grid(row=1, column=2, padx=20, pady=2)
        # self.add_point = customtkinter.CTkButton(self.autoflyframe, text="Add Point", command=self.addpointf)
        # self.add_point.grid(row=2, column=1, padx=20, pady=5, columnspan=2)

        # self.autoflyspeedbutton = PlusMinus(self.autoflyframe, "Speed", self.autoflyspeed, 1, self.autoflyspeedf)
        # self.autoflyspeedbutton.grid(row=0, column=3, padx=20, pady=10)

        # self.startautoflybutton = customtkinter.CTkButton(self.autoflyframe, text="Start", command=self.startautoflyf)
        # self.startautoflybutton.grid(row=2, column=4, padx=20, pady=5, columnspan=2)

        t_autoflier = threading.Thread(target=self.autoflier)
        t_autoflier.start()

    def autoflier(self):
        points = self.path_points
        points.append(self.path_points[0])
        if drone.tookoff==0:
            self.TakeOffLand.invoke()
            time.sleep(10)
        drone.xavg = points[0][0]
        drone.yavg = points[0][1]
        time.sleep(10)
        xavg = drone.xavg
        yavg = drone.yavg
        for i,p in enumerate(self.path_points):
            (xi,yi) = points[i]
            (xf,yf) = points[i+1]
            d = math.sqrt((xf-xi)**2+(yf-yi)**2)
            steps = int(d/2)
            xstep = (xf-xi)/steps
            ystep = (yf-yi)/steps
            for j in range(steps):
                xavg+=xstep
                yavg+=ystep
                drone.xavg = int(xavg)
                drone.yavg = int(yavg)
                time.sleep(1/(2*self.autoflyspeed))
                if self.inautofly==0 or drone.msp.stopEvent.is_set():
                    break

            if self.inautofly==0 or drone.msp.stopEvent.is_set():
                break

            drone.xavg = xf
            drone.yavg = yf

            time.sleep(5)

        self.TakeOffLand.invoke()


    def stopautoflyf(self):
        self.inautofly = 0
        self.clear_path.configure(state="normal")
        self.x_point.configure(state="normal")
        self.y_point.configure(state="normal")
        self.add_point.configure(state="normal")
        self.startautoflybutton.configure(text="Start", command=self.startautoflyf)


    def stream(self, label):

        while not drone.msp.stopEvent.is_set() and self.stream_req:
            frame = self.frame

            zavg = drone.zavg
            xavg = drone.xavg
            yavg = drone.yavg
            delta = 50
            xmin = xavg-delta
            xmax = xavg+delta
            ymin = yavg-delta
            ymax = yavg+delta

            cv2.line(img= frame, pt1=(xmin,ymin), pt2= (xmax,ymin), color =(0,0,255),thickness = 2, lineType = 8, shift = 0)
            cv2.line(img= frame, pt1=(xmin,ymin), pt2= (xmin,ymax), color =(0,0,255),thickness = 2, lineType = 8, shift = 0)
            cv2.line(img= frame, pt1=(xmin,ymax), pt2= (xmax,ymax), color =(0,0,255),thickness = 2, lineType = 8, shift = 0)
            cv2.line(img= frame, pt1=(xmax,ymin), pt2= (xmax,ymax), color =(0,0,255),thickness = 2, lineType = 8, shift = 0)

            cv2.circle(frame,(xavg,yavg),2,(0,255),2)

            cv2.circle(frame,(self.mousex,self.mousey),2,(255,0,255),2)
            cv2.putText(frame,"("+str(self.mousex)+","+str(self.mousey)+")",(self.mousex,self.mousey),cv2.FONT_HERSHEY_SIMPLEX,fontScale=0.6,color=(0,0,255),thickness=2)

            

            if self.mode_active == "AutoFly":
                points = np.array(self.path_points)
                cv2.polylines(frame, [points], 1, (255,255,255), thickness=2)
                for i,pt in enumerate(points):
                    cv2.putText(frame,str(i),(int(pt[0]),pt[1]),cv2.FONT_HERSHEY_SIMPLEX,fontScale=0.6,color=(0,0,255),thickness=2)

            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            self.camh,self.camw,c = frame.shape

            tmp = Image.fromarray(frame)

            image = tmp.resize((int(self.camw/self.camh*self.frame_size),self.frame_size), Image.ANTIALIAS)
            image = ImageTk.PhotoImage(image)
            label.create_image(label.winfo_width()/2,label.winfo_height()/2, image=image, anchor="center")

            key = cv2.waitKey(3) & 0xFF

            time.sleep(0.1)
        
        self.stream_def()

    def keydown(self,event):
        key = event.keysym
        if not key in self.keys_pressed :
            self.keys_pressed.append(key)

        if key==controls["connect"]:
            self.connect_button_1.invoke()
        elif key==controls["arm"]:
            self.arm_button.invoke()
        elif key==controls["takeOffLand"]:
            self.TakeOffLand.invoke()
            # if drone.tookoff==0:
            #     drone.takeoff()
            # else:
            #     drone.land()
        
        if self.mode_active=="FreeFly":
            if key==controls["rollLeft"]:
                drone.physics.rcRoll = drone.defRoll - 200
            elif key==controls["pitchUp"]:
                drone.physics.rcPitch = drone.defPitch + 200
            elif key==controls["rollRight"]:
                drone.physics.rcRoll = drone.defRoll + 200
            elif key==controls["pitchDown"]:
                drone.physics.rcPitch = drone.defPitch - 200
            elif key==controls["throttleUp"]:
                drone.physics.rcThrottle = drone.defThrottle + 200
            elif key==controls["throttleDown"]:
                drone.physics.rcThrottle = drone.defThrottle - 200
            elif key==controls["yawAntiClock"]:
                drone.physics.rcYaw = 1300
            elif key==controls["yawClock"]:
                drone.physics.rcYaw = 1700

        elif self.mode_active=="CamFly":
            if controls["rollLeft"] in self.keys_pressed:
                drone.xavg-=self.camflyspeed
            if controls["pitchUp"] in self.keys_pressed:
                drone.yavg-=self.camflyspeed
            if controls["rollRight"] in self.keys_pressed:
                drone.xavg+=self.camflyspeed
            if controls["pitchDown"] in self.keys_pressed:
                drone.yavg+=self.camflyspeed
        

        print(key,"  -- Pressed")

    def keyup(self,event):
        key = event.keysym

        if key==controls["rollLeft"] or key==controls["rollRight"]:
            drone.physics.rcRoll = drone.defRoll
        elif key==controls["pitchUp"] or key==controls["pitchDown"]:
            drone.physics.rcPitch = drone.defPitch
        elif key==controls["throttleUp"] or key==controls["throttleDown"]:
            drone.physics.rcThrottle = drone.defThrottle
        elif key==controls["yawAntiClock"] or key==controls["yawClock"]:
            drone.physics.rcYaw = 1500

        # drone.physics.rcRoll = drone.defRoll
        # drone.physics.rcPitch = drone.defPitch
        # drone.physics.rcThrottle = drone.defThrottle
        # drone.physics.rcYaw = 1500

        if key in self.keys_pressed :
            self.keys_pressed.pop(self.keys_pressed.index(key))

        print(key,"  -- Released")

    def callibrationf(self,key,val):
        file = open('./Omen/callibration_parameters.txt','r')
        cal = json.loads(file.read())
        file.close()

        cal[key] = val

        file = open('./Omen/callibration_parameters.txt','w')
        file.write(json.dumps(cal))
        file.close()

        setattr(drone,key,val)

        drone.physics.rcRoll = drone.defRoll
        drone.physics.rcPitch = drone.defPitch
        drone.physics.rcRoll = drone.defRoll
        drone.physics.rcPitch = drone.defPitch

    def camflyspeedf(self,key,val):
        self.camflyspeed = val

    def autoflyspeedf(self,key,val):
        self.autoflyspeed = val



def queuer():
    while not drone.msp.stopEvent.is_set():
        if drone.connected==1:
            drone.send_MSP_RAW_RC()
        
        time.sleep(0.08)

def camController():

    pidX = PID(1/2, 0, 20, 0, -50, 50)
    pidY = PID(1/2, 0, 20, 0, -50, 50)
    pidZ = PID(30, 0, 50, 0, -100, 100)

    while not drone.msp.stopEvent.is_set():
        aruco = track_aruco()
        frame = aruco["frame"]
        ids = aruco["ids"]
        data = aruco["data"]

        if app.drone1_id in ids:
            i = ids.index(app.drone1_id)
            cv2.putText(frame,"DRONE 1",(int(data[i][0]+20),int(data[i][1]+60)),cv2.FONT_HERSHEY_SIMPLEX,fontScale=0.4,color=(0,0,255),thickness=2)

        if app.drone2_id in ids:
            i = ids.index(app.drone2_id)
            cv2.putText(frame,"DRONE 2",(int(data[i][0]+20),int(data[i][1]+60)),cv2.FONT_HERSHEY_SIMPLEX,fontScale=0.4,color=(0,0,255),thickness=2)

        if app.controller_req:
            det=0
            if app.drone1_id in ids:
                i = ids.index(app.drone1_id)
                det=1
                angle_z = data[i][2]
                size = data[i][3]
                pos_x = data[i][0]
                pos_y = data[i][1]

            if (controls['yawAntiClock'] in app.keys_pressed) or (controls['yawClock'] in app.keys_pressed):
                pass
            elif det:
                if angle_z>5:
                    drone.physics.rcYaw = 1300
                elif angle_z<-5:
                    drone.physics.rcYaw = 1700
                else:
                    drone.physics.rcYaw = 1500
            else:            
                drone.physics.rcYaw = 1500

            if (controls['throttleUp'] in app.keys_pressed) or (controls['throttleDown'] in app.keys_pressed):
                pass
            elif det:
                drone.physics.rcThrottle = defThrottle - pidZ.update(size)

            if (controls['rollLeft'] in app.keys_pressed) or (controls['rollRight'] in app.keys_pressed):
                pass
            elif det:
                drone.physics.rcRoll = defRoll - pidX.update(pos_x)

            if (controls['pitchUp'] in app.keys_pressed) or (controls['pitchDown'] in app.keys_pressed):
                pass
            elif det:
                drone.physics.rcPitch = defPitch + pidY.update(pos_y)

        app.frame=frame

        key = cv2.waitKey(3) & 0xFF

        time.sleep(0.02)

t_transmitter = threading.Thread(target=drone.msp.transmit)
t_transmitter.start()
print('Transmitter started')

t_queuer = threading.Thread(target=queuer)
t_queuer.start()
print('Queuer started')

t_camController = threading.Thread(target=camController)
t_camController.start()
print('Camera Controller started')

app = App()
app.mainloop()
drone.msp.stopEvent.set()