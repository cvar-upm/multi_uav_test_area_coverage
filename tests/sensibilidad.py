from geometry_msgs.msg import Point, Polygon
from mutac_msgs.msg import Generation, Sweep
from mutac_msgs.srv import GeneratePlan
import rospy
from mutac_msgs.msg import State

counter = 0

def callback(msg):
    global counter
    if (msg.state == State.LOST):
        counter += 1

if __name__ == '__main__':
    rospy.init_node('test')

    rospy.wait_for_service('/mutac/generate_plan')
    client = rospy.ServiceProxy('/mutac/generate_plan', GeneratePlan)

    lost_sub = rospy.Subscriber('/mutac/drone_events', State, callback)

    polygon1 = Polygon(points=[Point(540, 345, 0), Point(635, 120, 0), Point(825, 120, 0), Point(730, 345, 0), Point(540, 345, 0)])
    sweeps = [Sweep(polygon=polygon1, orientation=Point(0, -25, 0))]

    generation = Generation(sweeps)

    client(generation)

    rospy.spin()

    print("\n--------------")
    print("LOST COUNT:", counter)
    print("--------------")
