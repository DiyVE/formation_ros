import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute, SetPen, Spawn

class TurtleBot:
    def __init__(self, turtle_name):
        # Initialisation du Noeud ROS et anonymous=False pour que le noeud soit unique
        rospy.init_node('%s controller'%turtle_name, anonymous=False)

        # Déclaration du Publisher permettant de publier la vitesse de la tortue
        self.velocity_publisher = rospy.Publisher('/%s/cmd_vel'%turtle_name,
                                                  Twist, queue_size=10)

        # Déclaration du Subscriber permettant de récupérer la position de la tortue
        # A chaque fois que le Subscriber reçoit un message, la fonction update_pose est appelée
        self.pose_subscriber = rospy.Subscriber('/%s/pose'%turtle_name,
                                                Pose, self.update_pose)

        try:
            # Instanciation des services permettant de téléporter la tortue et de changer sa couleur
            self.set_color_srv = rospy.ServiceProxy('/%s/set_pen'%turtle_name, SetPen)
            self.teleport_srv = rospy.ServiceProxy('/%s/teleport_absolute'%turtle_name, TeleportAbsolute)

            # On attend que les services soient disponibles, dans le cas contraire une exception est levée
            self.set_color_srv.wait_for_service()
            self.teleport_srv.wait_for_service()
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        # On initialise l'attribut pose correspondant à la position de la tortue
        self.pose = Pose()

if __name__ == "__main__":
    spawn_srv = rospy.ServiceProxy('/spawn', Spawn)
    turtle_name = spawn_srv(x=4, y=2, theta=0)

    x = TurtleBot(turtle_name)

    rospy.spin()

