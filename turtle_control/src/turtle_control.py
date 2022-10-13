import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute, SetPen, Spawn, Kill

# Fonction appelée lorsque l'on souhaite quitter le programme
def on_shutdown():
    try:
        kill_srv(turtle_name)
        exit(1)
    except rospy.ServiceException:
        pass
    except ValueError:
        print("Turtle cannot be killed")


# Fonction appelée à chaque fois que le Subscriber reçoit un message
def update_pose(data):
    # TODO
    # ...
    # ...
    pass

# Si le fichier est exécuté directement
if __name__ == "__main__":
    # Initialisation du noeud ROS en anonyme afin de démarrer plusieurs fois le même noeud
    rospy.init_node("robot_controller", anonymous=True)

    try:
        # Instanciation du service permettant de créer une nouvelle tortue
        spawn_srv = rospy.ServiceProxy('/spawn', Spawn)

        # On attend que le service soit disponible, dans le cas contraire une exception est levée
        spawn_srv.wait_for_service()

        # On demande à l'utilisateur de saisir la position et la rotation de la nouvelle tortue
        x = float(input("x: "))
        y = float(input("y: "))
        theta = float(input("theta: "))

        # On fait apparaître une nouvelle tortue à la position donnée
        turtle_name = spawn_srv(x=x, y=y, theta=theta).name

        # On envoi un message d'information
        rospy.loginfo("Turtle %s spawned"%turtle_name)

        # Instanciation des services permettant de téléporter la tortue, de changer sa couleur et faire apparaître une nouvelle tortue
        set_color_srv = rospy.ServiceProxy('/%s/set_pen'%turtle_name, SetPen)
        teleport_srv = rospy.ServiceProxy('/%s/teleport_absolute'%turtle_name, TeleportAbsolute)
        kill_srv = rospy.ServiceProxy('/kill', Kill)

        # On attend que les services soient disponibles, dans le cas contraire une exception est levée
        set_color_srv.wait_for_service()
        teleport_srv.wait_for_service()
        kill_srv.wait_for_service()
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

    # Déclaration du Publisher permettant de publier la vitesse de la tortue
    velocity_publisher = rospy.Publisher('/%s/cmd_vel'%turtle_name,
                                                Twist, queue_size=10)

    # Déclaration du Subscriber permettant de récupérer la position de la tortue
    # A chaque fois que le Subscriber reçoit un message, la fonction update_pose est appelée
    pose_subscriber = rospy.Subscriber('/%s/pose'%turtle_name,
                                            Pose, update_pose)

    # On initialise la position de la tortue
    turtle_pose = Pose()

    # On déclare que si le noeud est arrêté, la fonction on_shutdown doit être appelée
    rospy.on_shutdown(on_shutdown)

    # Réglage de la vitesse de la boucle ci-dessous
    rate = rospy.Rate(10)

    # On boucle tant que ctrl+c n'est pas appuyé
    while not rospy.is_shutdown():
        # TODO
        # ...
        # ...

        # On attend 0.1 secondes (donné par le rospy.Rate)
        rate.sleep()

