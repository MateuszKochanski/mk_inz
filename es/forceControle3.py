import robotLib.es5 as es
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
import rospy
def point(name):
    data = {}
    return data[name]
    
def loadData(data):
    var.x = data.pose.position.x
    var.y = data.pose.position.y
    var.z = data.pose.position.z
    var.rx = data.pose.orientation.x
    var.ry = data.pose.orientation.y
    var.rz = data.pose.orientation.z
    var.header = data.header
    var.k = True
    
if (__name__ == '__main__'):
    es.init()
    var = es.Shared_variables()
    main()
    
def callback(dane):
    if (dane.header.seq == 0):
        loadData(dane)
        var.c = True
        
    elif (dane.header.stamp > var.header.stamp) and var.c:
        loadData(dane)
        
    
def main():
    var.k = False
    var.x = 0.42
    var.y = 0.22
    var.z = 0.4
    var.rx = -126.24999
    var.ry = 1.60902
    var.rz = -179.77854
    var.c = False
    pub = rospy.Publisher('done', Header, queue_size=10)
    rospy.Subscriber('/sterowanie', PoseStamped, callback, queue_size=10)
    es.move(type=es.Point, end=es.CartesianPos([var.x, var.y, var.z], [var.rx, var.ry, var.rz], es.InternalPos([193.401, 89.597, 68.518, 110.514, 90.873, -130.335])), speed=20.0, acc=20.0)
    es.execute_move()
    while True:
        if (var.k == True):
            var.header.frame_id = 'accepted'
            pub.publish(var.header)
            es.move(type=es.Point, end=es.CartesianPos([var.x, var.y, var.z], [var.rx, var.ry, var.rz], es.InternalPos([193.401, 89.597, 68.518, 110.514, 90.873, -130.335])), speed=20.0, acc=20.0)
            es.execute_move()
            var.header.frame_id = 'done'
            pub.publish(var.header)
            var.k = False
