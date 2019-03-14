# -*- coding: utf-8 -*-
"""
Created on Sat Feb 16 09:46:26 2019

@author: TTM
"""

import kivy
import time

from kivy.app import App
kivy.require("1.10.0")

from kivy.core.window import Window
from kivy.clock import Clock

from kivy.graphics import Rectangle

from kivy.uix.screenmanager import ScreenManager, Screen
from kivy.uix.gridlayout import GridLayout
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.label import Label
from kivy.uix.popup import Popup
from kivy.uix.button import Button
from kivy.uix.textinput import TextInput
      

class Home_Screen(Screen):
    def __init__(self):
        super(Home_Screen, self).__init__()
        self.bg_image = r"Assets/home_bg_image.jpg"
        layout = FloatLayout(size=Window.size)
        self.home_bg = Label(text = "Welcome to Changi Airport\n Please scan your boarding pass or enter your flight number below to begin", 
                             pos_hint={"center_x":0.5, "y":0.2},size_hint=(0.5,0.5), font_size= "25dp", halign="center")
        self.clock = Clock_Label()
        self.flight_number = TextInput(text="Enter your flight number here", size_hint=(0.5,0.05), pos_hint={"center_x": 0.5, "center_y":0.2}, 
                                       multiline=False)
        self.flight_number.bind(on_text_validate=self.text_validate)
               
        layout.add_widget(self.home_bg)
        layout.add_widget(self.clock)
        layout.add_widget(self.flight_number)
        
        with self.canvas.before:
            self.bg_holder = Rectangle(size = Window.size, pos = self.pos, source = self.bg_image)
            
        with self.canvas:
            self.add_widget(layout)        
            self.bind(size = self.update_bg)
            
    def update_bg(self,*args):
        self.bg_holder.size = Window.size
    
    def text_validate(self, input_text):
        print(input_text.text)
             
        
class Clock_Label(Label):
    def __init__(self):
        super(Clock_Label,self).__init__()
        self.text = time.strftime("%a, %d %b %Y\n %H:%M:%S" , time.localtime())
        self.pos_hint = {"center_x":0.5, "y":0.5}
        self.size_hint = (0.5,0.5)
        self.font_size="25dp"
        Clock.schedule_interval(self.set_time, 1)
        
    def set_time(self,dt):       
        self.text = time.strftime("%a, %d %b %Y\n %H:%M:%S" , time.localtime())
     
 
sm = ScreenManager()
sm.add_widget(Home_Screen())
       
class GUI(App):
    def build(self):
        return sm
    
if __name__ == "__main__":
    GUI().run()
    
            

        
        