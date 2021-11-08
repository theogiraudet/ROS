#!/usr/bin/env python3
# Version obfusquee du noeud d'odometrie
import rospy #line:3
from sensor_msgs .msg import JointState #line:4
from nav_msgs .msg import Odometry #line:5
import math #line:7
from geometry_msgs .msg import Quaternion #line:9
from tf .transformations import quaternion_from_euler #line:10

####### Parametres ########
_O0OOO000O0O00O0O0 = 0.0166 # rayon d'une roue (en m)
_O0OO00OO000O00O00 = 0.15   # voie (distance entre deux roues du meme essieu, en m)
###########################

m=rospy.ROSInterruptException
j=rospy.spin
I=rospy.Subscriber
V=rospy.init_node
N=rospy.Time
T=rospy.Publisher
_00O0OOO0O0O000000 = math.sin
_0OO000O000O0000O0 = math.cos
_00O0OOO0O0O00000O = Odometry
_00O0OOO0O0000O000 = 'odometry'
_00O0OO00O0000O000 = None
_00OOOO00O0000O000 = 0.0
def _OOOOO00000O00OOOO (OO000000O0OOO0O0O ):#line:12
 O0000O0OO000O000O =quaternion_from_euler (0.0 ,0.0 ,OO000000O0OOO0O0O )#line:13
 return Quaternion (*O0000O0OO000O000O )#line:14
_OO0OO00O00OO0OO00 =T (_00O0OOO0O0000O000,_00O0OOO0O0O00000O ,queue_size =10 )#line:17
_O0O00O00O00OOOOOO =_00OOOO00O0000O000 #line:20
_OOOO0OO0O00OOOOO0 =_O0O00O00O00OOOOOO #line:21
_O00OOO0OOOOOOO0OO =_00OOOO00O0000O000 #line:22
_OO0O0O00O000000OO =_00O0OO00O0000O000 #line:29
_O0O0OOOOO000OO000 =_00O0OO00O0000O000 #line:30
def _OOO0000000OO000O0 (O0OO0000OO00OOO00 ):#line:33
 global _O0O00O00O00OOOOOO ,_OOOO0OO0O00OOOOO0 ,_O00OOO0OOOOOOO0OO, _OO0O0O00O000000OO ,_O0O0OOOOO000OO000 #line:35
 OOO0000OOO0O00OO0 =O0OO0000OO00OOO00 .position [0 ]#line:38
 O0OO00O000OO0OO00 =O0OO0000OO00OOO00 .position [1 ]#line:39
 if _OO0O0O00O000000OO ==_00O0OO00O0000O000 or _O0O0OOOOO000OO000 ==_00O0OO00O0000O000 :#line:42
  _OO0O0O00O000000OO =OOO0000OOO0O00OO0 #line:43
  _O0O0OOOOO000OO000 =O0OO00O000OO0OO00 #line:44
 OOOO00O0OO00OOO00 =OOO0000OOO0O00OO0 -_OO0O0O00O000000OO #line:47
 OO0O00OO0O00OO0OO =O0OO00O000OO0OO00 -_O0O0OOOOO000OO000 #line:48
 _OO0O0O00O000000OO =OOO0000OOO0O00OO0 #line:49
 _O0O0OOOOO000OO000 =O0OO00O000OO0OO00 #line:50
 OO000000O000OOOO0 =_O0OOO000O0O00O0O0 *(OOOO00O0OO00OOO00 +OO0O00OO0O00OO0OO )/2 #line:53
 OO00O00O0O00OO000 =_O0OOO000O0O00O0O0 *(OOOO00O0OO00OOO00 -OO0O00OO0O00OO0OO )/_O0OO00OO000O00O00 #line:54
 O0000O0OO000O0OOO =_0OO000O000O0000O0 (_O00OOO0OOOOOOO0OO )#line:57
 OO0O00000O00OOOOO =_00O0OOO0O0O000000 (_O00OOO0OOOOOOO0OO )#line:58
 _O0O00O00O00OOOOOO +=OO000000O000OOOO0 *O0000O0OO000O0OOO #line:59
 _OOOO0OO0O00OOOOO0 +=OO000000O000OOOO0 *OO0O00000O00OOOOO #line:60
 _O00OOO0OOOOOOO0OO +=OO00O00O0O00OO000 #line:61
 O0000OO0O0O000OOO =N.now ()#line:66
 O0O00OO0O0O00O0OO =Odometry ()#line:67
 O00000O00000OO0OO =O0O00OO0O0O00O0OO .header #line:68
 OOO0000OO0O0OOO00 ='odom'#line:69
 OOOOO0OO0O0OO00O0 =O0O00OO0O0O00O0OO .pose #line:70
 OO0O0OO0O0OOOO00O =OOOOO0OO0O0OO00O0 .pose #line:71
 O0O0O0O0000O0OOOO =OO0O0OO0O0OOOO00O .position #line:72
 O00000O00000OO0OO .stamp =O0000OO0O0O000OOO #line:73
 O0O0O0O0000O0OOOO .x =_O0O00O00O00OOOOOO #line:74
 O00000O00000OO0OO .frame_id =OOO0000OO0O0OOO00 #line:75
 O0O0O0O0000O0OOOO .y =_OOOO0OO0O00OOOOO0 #line:76
 OO0O0OO0O0OOOO00O .orientation =_OOOOO00000O00OOOO (_O00OOO0OOOOOOO0OO )#line:77
 _OO0OO00O00OO0OO00 .publish (O0O00OO0O0O00O0OO )#line:78
def _OOO0O0OO0O0O0OO00 ():#line:81
 V (_00O0OOO0O0000O000)#line:83
 OO0OO00OOO0O0OO00 =I ("nxt/encoders",JointState ,_OOO0000000OO000O0 )#line:85
 j ()#line:87
if __name__ =='__main__':#line:89
 try :#line:90
  _OOO0O0OO0O0O0OO00 ()#line:91
 except m :#line:92
  pass #line:93
