#!/usr/bin/env python
import sys
import rospy
import json
import mysql.connector
from std_srvs.srv import *
from std_msgs.msg import String
from humans_msgs.srv import Int32Srv

#
'''
def request(req):
    pub = rospy.Publisher('json_listener', String, queue_size=10)

    r = rospy.Rate(10) # 10hz
    id1 = '1'
    id2 = '2'

    ndata1 = { 'data': { 'id': id1, 'foo': 3, 'bar': 5, 'baz': 10, 'label': 'Uema' } }
    ndata2 = { 'data': { 'id': id2, 'foo': 3, 'bar': 5, 'baz': 7, 'label': 'Haruta' } }
    edata1 = { 'data': { 'id': id1+id2, 'weight': 1, 'source': id1, 'target': id2 } }
    nodes = [ ndata1 ,ndata2 ] 
    edges = [ edata1 ] 

    json_data = {
        'nodes': nodes,
	'edges': edges
    }

    json.dumps(json_data)
 
    num = 0
    while num < 2:
        str = json.dumps(json_data) #"hello world %s"%rospy.get_time()
        rospy.loginfo(str)
        pub.publish(str)
        r.sleep()
        print "send!"
        num+=1

    return Int32Response()
'''
#

config = {
    'user': 'root',
    'password': 'robot15',
    'host': '127.0.0.1',
    'db': 'data1',
    'charset':'utf8'
}

class relation:

    def __init__(self):
        self.t_name = '201551'
        self.json_pub = rospy.Publisher('json_listener', String, queue_size=10)
        self.server_srv = rospy.Service('relation', Int32Srv, self.request)
        self.con = mysql.connector.connect(**config)
        self.cur = self.con.cursor()
        self.ndata = []
        self.edata = []

    def request(self,req):
        print req.n
        self.ndata = []
        self.edata = []

        qy = "SELECT DISTINCT tracking_id FROM `"+self.t_name+"` WHERE state = 2 and okao_id = "+str(req.n)
        #print qy
        self.cur.execute(qy)
        res = self.cur.fetchall()
        for row in res:
            #print row[0]
            self.tracking_req(row[0], req.n)

        #print self.ndata

        json_data = {
            'nodes':self.ndata, 
            'edges':self.edata
        }

        print json_data

        json.dumps(json_data)        

        num = 0
        while num < 2:
            json_str = json.dumps(json_data) #"hello world %s"%rospy.get_time()
            #rospy.loginfo(json_str)
            self.json_pub.publish(json_str)
            #r.sleep()
            print "send!"
            num+=1

        return True

    def tracking_req(self,t_id, o_id): 
        print t_id
        #tracking_id = row
        qy = "SELECT DISTINCT time_stamp FROM `"+self.t_name+"` WHERE tracking_id = "+str(t_id)+" ORDER BY time_stamp asc limit 1"
        #print qy
        self.cur.execute(qy)
        res = self.cur.fetchall()
        for row in res:
            start = row[0]

        qy = "SELECT DISTINCT time_stamp FROM `"+self.t_name+"` WHERE tracking_id = "+str(t_id)+" ORDER BY time_stamp desc limit 1"
        #print qy
        self.cur.execute(qy)
        res = self.cur.fetchall()
        for row in res:
            goal = row[0]

        self.datatime_req(start, goal, o_id)

    def datatime_req(self, start, goal, o_id):
        print start
        print goal
        t_id = 0
        #id_relation = {"state":0, "okao_id":0, "hist":0, "time_stamp":0, "t_id":0}
        qy = "SELECT * FROM `"+self.t_name+"` WHERE time_stamp BETWEEN '"+str(start)+"' and '"+str(goal)+"'"
        self.cur.execute(qy)
        res = self.cur.fetchall()
        time = str(start)+' ~ '+str(goal) 
        #print time
        n_buf = {}
        e_buf = {}
        s_o_id = str(2)+'-'+str(o_id)
        for row in res:
            #print 'state:'+str(row[0])+', okao_id:'+str(row[1])+', hist:'+str(row[2])+', '+str(row[3])+', time_stamp:'+str(row[4])+', tracking_id:'+str(row[7])+', (x, y)=('+str(row[8])+', '+str(row[9])+')'
            a_n_id = str(row[0])+'-'+str(row[1]) 

            a_ndata = {'data':{'id':a_n_id, 'hist':str(row[2]), 'time_stamp':str(row[3]), 't_id':str(row[7])}}
            n_buf[row[7]]=a_ndata
            #print t_buf[row[7]]['data']['id']+","+t_buf[row[7]]['data']['hist']
            
        for t_okao_id in n_buf:
            #print t_okao_id
            #print n_buf[row[7]]['data']['id']
            now_node_id = n_buf[t_okao_id]['data']['id']
            if (now_node_id != s_o_id):
                a_edata = {'data':{'id':s_o_id+'+'+now_node_id, 'source': s_o_id, 'target': now_node_id, 'time': time}}
                e_buf[row[7]]=a_edata
            #    self.edata.append(a_edata)
         
        #print t_buf.values()
        self.ndata.extend(n_buf.values())
        self.edata.extend(e_buf.values())
        

def main(args):
    rl = relation()
    rospy.init_node('relation', anonymous=True)
    try:
        rospy.spin()
    except KeybordInterrupt:
        print "Shutting down"
    
    
if __name__ == '__main__':
    main(sys.argv)
