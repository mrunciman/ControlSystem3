from tkinter import *
import time
import threading


class userInterface:

   def __init__(self):
      self.rootWindow = Tk()
      self.rootWindow.title("Test Button")
      self.rootWindow.geometry("700x400")

   def setupGUI(self):
      self.label = Label(self.rootWindow, text="You can click the button or wait")
      self.label.pack(pady=20)
      # self.label.config(text="You can click the button or wait")
      #Create button
      self.b1 = Button(self.rootWindow, text = "Start", command = lambda: threading.Thread(target = self.thread_fun).start())
      self.b1.pack(pady=20)
      self.rootWindow.mainloop()

   #Define the function to start the thread
   def thread_fun(self):
      time.sleep(5)
      self.label.config(text= "5 seconds up!")

   # def startGUI(self):
   #    self.rootWindow.mainloop()



if __name__ == "__main__":
   userInt = userInterface()
   userInt.setupGUI()



