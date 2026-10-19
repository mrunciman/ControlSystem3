from pyPS4Controller.controller import Controller

class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)

    def on_x_press(self):
       print("Hello world")

    def on_x_release(self):
       print("Goodbye world")


if __name__ == "__main__":
    controller = MyController(interface="COM3", connecting_using_ds4drv=False)
    controller.debug = True
    controller.listen(timeout=60)