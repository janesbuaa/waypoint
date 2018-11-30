#!/usr/bin/python
# -*- coding: utf-8 -*-

"""按照8个点逆时针飞行，验证机头朝向
            #7←←←←←←←←←#6←←←←←←←←←#5
            ↓                     ↑
            ↓                     ↑
            #8                    #4
            ↓                     ↑
            ↓                     ↑
    start → #1→→→→→→→→→#2→→→→→→→→→#3
"""

from drone_proxy import *

drone = Drone()
ret = drone.unlock()
print "解锁:", ret.success

rospy.init_node('waypoint', anonymous=True)
ori_pos = rospy.client.wait_for_message("/uav/position",Position,2)


def process_state(state):
	if isinstance(state, State):
		if state.landed:
			print "飞机已着陆"
		if state.mode == "WAYPOINT":
			if state.waypoint.status is 0:
				print "执行航点任务中，正飞往", state.waypoint.waypointId
			elif state.waypoint.status is 1:
				print "执行航点任务中，已到达", state.waypoint.waypointId
			elif state.waypoint.status is 2:
				print "执行航点任务中，暂停", state.waypoint.waypointId
			else:
				print "执行航点任务中，未知状态", state.waypoint.waypointId
		if state.mode == "RETURN":
			print "返航中..."


def wpp(latitude, longitude, altitude, heading=360, velocity=0.5, itype=1, hovertime=5, return_flag=False):
	"""航点属性赋值函数"""
	wp = WPItem()
	wp.latitude = latitude
	wp.longitude = longitude
	wp.altitude = altitude
	wp.heading = heading  # 离开该航点时机头指向下一航点
	wp.velocity = velocity  # 离开该航点时的速度
	wp.type = itype  # 起飞点
	if return_flag:
		wp.returnHeading = 360
	else:
		wp.hovertime = hovertime  # 悬停时间
	return wp


def way_point(p):
	# 航点任务基本描述
	bp = WPBasicParam()
	bp.trackId = 1  # 航线编号
	bp.segmentId = 1  # 航段编号
	bp.wpNumber = int(len(p))  # 总共4个航点
	bp.executiveTimes = 1  # 仅执行一次
	bp.traceMode = 0  # 转弯模式为定点转弯
	bp.RCLostAction = 0  # RC失联后执行失联保护
	# 添加航点
	wps = []
	for i in range(len(p)):
		heading_angle = 360  # 机头朝向，360为离开该航点时机头指向下一航点
		itype = 1
		if i == 0:
			itype = 0  # 起飞点
		flag = False
		if i == len(p) - 1:
			itype = 2  # 返航点
			flag = True
        if i == 3:
            heading_angle = 90
        if i == 5:
            heading_angle = 180
        if i == 7:
            heading_angle = 90
		wp = wpp(p[i][0], p[i][1], p[i][2], heading=heading_angle, itype=itype, return_flag=flag)
		wps.append(wp)

	# 装订航点
	ret = drone.wpMission.load(bp, wps)
	print "航点装订:", ret.success
	if ret.success:  # 装订成功
		rospy.Subscriber("/uav/state",State,process_state)  # 订阅/uav/state
		ret = drone.wpMission.start()
		print "启动航点:", ret.success
		# 开始ROS调度
		try:
			rospy.spin()
		except KeyboardInterrupt, e:
			pass


def main():
    if isinstance(ori_pos, Position):
        # 起飞时的坐标
        origin_p = (ori_pos.latitude, ori_pos.longitude, ori_pos.altitude)
        height = origin_p[2]+8
        points = []
        for i in range(3):
            next_p = (origin_p[0], origin_p[1] + 0.00020 * i, height)  # 逆时针飞行，往东飞20米
            points.append(next_p)
        for i in range(1, 3):
            next_p = (origin_p[0] + 0.00015 * i, origin_p[1] + 0.00020 * 2, height)
            points.append(next_p)
        for i in range(1, 3):
            next_p = (origin_p[0] + 0.00015 * 2, origin_p[1] + 0.00020 * (2 - i), height)
            points.append(next_p)
        for i in range(1, 3):
            next_p = (origin_p[0] + 0.00015 * (2 - i), origin_p[1], height)
            points.append(next_p)
        print "总共有{}个点，各点的坐标为\n{}".format(len(points), points)
        way_point(points)


if __name__ == "__main__":
    main()
