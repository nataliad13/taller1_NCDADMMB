#!/usr/bin/python
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class Node ():
    def __init__(self):
        self.rospy = rospy
        self.rospy.init_node("nodo_3",anonymous = True)
        self.initParameters()
        self.initSubscribers()
        self.initPublishers()
        self.main()

    def initParameters(self):
        self.topic_ang = "/angular"
        self.topic_lin = "/linear"
        self.topic_vel = "/cmd_vel"
        self.topic_aux = "/aux_topic"
        self.msg_ang = Float32()
        self.msg_lin = Float32()
        self.msg_vel = Twist()
        self.change_ang = False
        self.change_lin = False
        self.rate = self.rospy.Rate(50)
        return

    def callback_ang(self, msg):
        # el argumento msg tomara el valor del mensaje
        self.msg_ang = msg.data
        self.change_ang = True
        return

    def callback_lin(self, msg):
        self.msg_lin = msg.data
        self.change_lin = True
        return
    def callback_aux (self , msg):
        self.msg_aux = msg.data
        return

    def initSubscribers(self):
        # Inicializacion de los subscriptores
        self.sub_ang = self.rospy.Subscriber(self.topic_ang, Float32, self.callback_ang)
        self.sub_lin = self.rospy.Subscriber(self.topic_lin, Float32, self.callback_lin)
        self.sub_aux = self.rospy.Subscriber(self.topic_aux, String, self.callback_aux)
        return

    def initPublishers(self):
        #Inicializacion de publicadores
        self.pub_vel = self.rospy.Publisher(self.topic_vel,Twist,queue_size = 10)
        return

    def main(self):
        #Codigo principal
        print ("OK")
        while not self.rospy.is_shutdown():
            if self.change_ang and self.change_lin:
                 # si estan en true haga
                if self.msg_aux =="False":

                     self.msg_vel.linear.x = float(self.msg_lin)
                     self.msg_vel.angular.z = float(self.msg_ang)
                     self.pub_vel.publish(self.msg_vel)
                else:
                    self.msg_lin = 0
                    self.msg_ang = 0
                    self.msg_vel.linear.x = float(self.msg_lin)
                    self.msg_vel.angular.z = float(self.msg_ang)
                    self.pub_vel.publish(self.msg_vel)

                self.change_ang = False
                self.change_lin = False
            self.rate.sleep()

if __name__ == "__main__":
    try:
        print("Iniciando nodo")
        object = Node()
    except rospy.ROSInterruptException:
        print("Finalizando Nodo")
        pass
