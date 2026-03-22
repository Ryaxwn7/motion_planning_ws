#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from std_msgs.msg import UInt8, Bool

class GlobalPlanMonitor:
    def __init__(self, idx=0):
        # 初始化节点，从参数服务器获取话题名称
        if idx == 0:
            rospy.init_node('global_plan_monitor', anonymous=True)
            self.gather_start = False
            # 获取参数（带默认值）
            self.plan_topic = '/move_base/GraphPlanner/plan'
            self.signal_topic = '/gather_signal'
            self.gather_start_topic = '/gather_start'
            self.last_received = rospy.Time.now()
        
            # 初始化发布者和订阅者
            self.signal_pub = rospy.Publisher(self.signal_topic, UInt8, queue_size=10)
            self.plan_sub = rospy.Subscriber(self.plan_topic, Path, self.plan_callback)
            self.signal_sub = rospy.Subscriber(self.gather_start_topic, Bool, self.gather_start_callback)
            rospy.loginfo(f"Monitoring global plan on [{self.plan_topic}]")
            rospy.loginfo(f"Publishing signals to [{self.signal_topic}]")
        else:
            rospy.init_node(f'robot{idx}_global_plan_monitor', anonymous=True)
            self.gather_start = False
            self.plan_topic = f'robot{idx}/move_base/GraphPlanner/plan'
            self.signal_topic = f'/gather_signal'
            self.gather_start_topic = f'/gather_start'
            self.last_received = rospy.Time.now()
            self.signal_pub = rospy.Publisher(self.signal_topic, UInt8, queue_size=10)
            self.plan_sub = rospy.Subscriber(self.plan_topic, Path, self.plan_callback)
            self.signal_sub = rospy.Subscriber(self.gather_start_topic, Bool, self.gather_start_callback)        
            rospy.loginfo(f"Monitoring global plan on [{self.plan_topic}]")
            rospy.loginfo(f"Publishing signals to [{self.signal_topic}]")
            
  
    def plan_callback(self, msg):
        """ 路径消息回调函数 """
        is_valid = len(msg.poses) > 0
        rospy.logdebug(f"[PlanMonitor] Received path with {len(msg.poses)} poses. Valid: {is_valid}")
        # 创建并发布信号
        if(is_valid):
            self.last_received = rospy.Time.now()
        else:
            self.check_timeout()



    def gather_start_callback(self, msg):
        """ 开始采集回调函数 """
        self.gather_start = msg.data
        if self.gather_start:
            rospy.loginfo(f"[PlanMonitor] Gathering")
        else:
            rospy.loginfo(f"[PlanMonitor] Not gathering")

        
    def check_timeout(self):
        rospy.logdebug(f"[PlanMonitor] Checking timeout")
        if self.gather_start and (rospy.Time.now() - self.last_received).to_sec() > 5.0:
            rospy.logwarn(f"[PlanMonitor] Global plan timeout. Last received at {self.last_received.to_sec()}")
            self.signal_pub.publish(UInt8(1))

if __name__ == '__main__':
    try:
        monitor = GlobalPlanMonitor()
        rospy.spin()

    
    except rospy.ROSInterruptException:
        pass
