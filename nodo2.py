#!/usr/bin/python
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import math

class Node ():
    def __init__(self):
        self.rospy = rospy
        self.rospy.init_node("nodo_2",anonymous = True)
        self.initParameters()
        self.initSubscribers()
        self.initPublishers()
        self.main()

    def initParameters(self):
        self.topic_scan = "/scan"
        self.topic_aux = "/aux_topic"
        self.msg_scan = LaserScan()
        self.msg_aux = String()
        self.change_scan = False
        self.rate = self.rospy.Rate(50)
        self.pepe = LaserScan()
        self.bol = bool()

        return

    def callback_scan(self, msg):
        # el argumento msg tomara el valor del mensaje

        self.x = []
        self.y = []
        self.angulo_final = []
        self.puntos =[]
        self.puntos_seg = 10
        self.cont_puntos = []
        self.contador = 0
        self.distancias = msg.ranges
        self.angulo = msg.angle_increment
        self.angulo_min = msg.angle_min

        self.distancias = list(self.distancias)
        # PASAR DE RADIANES A GRADOS
        self.angulo = ((self.angulo)*180)/3.1416
        self.angulo_min = ((self.angulo_min)*180)/3.1416



        self.tamano = len(self.distancias)
        self.angulo_final.append(self.angulo_min)
        for i in range (0,self.tamano-1):
            self.rest = self.angulo_final[i]+self.angulo
            self.angulo_final.append(self.rest)

            # de polares a cartesianas
        for j in range (0,self.tamano):
            test = float ("inf")
            if self.distancias [j] == test :
                self.distancias [j] =float (10)
                #print(self.distancias [i])
            self.x.append((self.distancias [j]) * math.cos(self.angulo_final[j]))
            self.y.append((self.distancias [j]) * math.sin(self.angulo_final[j]))

        for k in range (0,self.tamano):
            self.g = k + 1
            if self.g < self.tamano:
                self.pepito = (((self.x[self.g]-self.x[k])**2)+(self.y[self.g]-self.y[k])**2)**(0.5)
                self.puntos.append(self.pepito)

        for l in range (0,len(self.puntos)):
            self.max = 10
            if self.puntos [l] < self.max : 
                if (self.puntos[l] < 1) :
                    self.contador = self.contador + 1
                else :
                    if self.contador > 0 :
                        self.cont_puntos.append (self.contador)
                        self.contador = 0

        for m in range (0,len(self.cont_puntos)):
            if self.cont_puntos[m] >= self.puntos_seg :
                print("detener")
                self.bol = True

        print(self.cont_puntos)
        print (self.bol)
        self.change_scan = True
        return

    def initSubscribers(self):
        # Inicializacion de los subscriptores
        self.sub_scan = self.rospy.Subscriber(self.topic_scan, LaserScan, self.callback_scan)

        return

    def initPublishers(self):
        #Inicializacion de publicadores
        self.pub_aux = self.rospy.Publisher(self.topic_aux,String,queue_size = 10)
        return

    def main(self):
        #Codigo principal
        print ("OK")
        while not self.rospy.is_shutdown():
            if self.change_scan:
                 # si estan en true haga
                 self.bol = str (self.bol)
                 self.pub_aux.publish(self.bol)
                 self.change_scan = False
                 self.bol =False



            self.rate.sleep()

if __name__ == "__main__":
    try:
        print("Iniciando nodo")
        object = Node()
    except rospy.ROSInterruptException:
        print("Finalizando Nodo")
        pass
