#!/usr/bin/python
import serial
import time
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class Node ():
    def __init__(self):
        self.rospy = rospy
        self.rospy.init_node("nodo_1",anonymous = True)
        self.initParameters()
        self.initSubscribers()
        self.initPublishers()
        self.main()

    def initParameters(self):
        self.topic_ang = "/angular"
        self.topic_lin = "/linear"
        self.topic_vel = "/cmd_vel"
        self.msg_ang = Float32()
        self.msg_lin = Float32()
        self.msg_vel = Twist()
        self.change_ang = True
        self.change_lin = True
        self.x = []
        self.rate = self.rospy.Rate(50)
        return

    def callback_ang(self,msg):
        # el argumento msg tomara el valor del mensaje
        self.msg_ang = msg.data
        self.change_ang = True
        return

    def callback_lin(self,msg):
        self.msg_lin = msg.data
        self.change_lin = True
        return

    def initSubscribers(self):
        # Inicializacion de los subscriptores
        pass

    def initPublishers(self):
        #Inicializacion de publicadores
        self.vell = self.rospy.Publisher(self.topic_ang,Float32,queue_size = 10)
        self.vela = self.rospy.Publisher(self.topic_lin,Float32,queue_size = 10)

        return

    def main(self):
        #Codigo principal
        print ("OK")
        self.puerto =  serial.Serial('/dev/ttyACM0', 9600, timeout=1)


        while not self.rospy.is_shutdown():
            self.dato = self.puerto.readline()
            #self.dato = String(self.dato)
            self.dato = self.dato.rstrip('\n')
            self.dato = self.dato.rstrip('\r')
            try:
                self.x= self.dato.split("/")
            except:
                self.x = [0,0]
            self.long = len (self.x)
            if self.long == 2:
                if self.change_ang and self.change_lin:
                     # si estan en true haga
                     #self.a =float()
                     #self.b= float()
                     self.a = self.x [0]
                     self.b = self.x [1]
                     self.a =float (self.a)
                     self.b =float (self.b)

                     self.a =self.a/100
                     self.b =self.b/100
                     #self.a =str (self.a)
                     #self.b =str (self.b)

                     print(self.a)
                     print (self.b)
                     self.vell.publish(self.a)
                     self.vela.publish(self.b)

                self.rate.sleep()

if __name__ == "__main__":
    try:
        print("Iniciando nodo")
        object = Node()
    except rospy.ROSInterruptException:
        print("Finalizando Nodo")
        pass
