#!/usr/bin/python
# -*- coding: utf-8 -*-

from drone_proxy import *

drone = Drone()
ret = drone.unlock()
print "解锁:", ret.success

rospy.init_node('demo_waypoint', anonymous=True)
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


def way_point():
	# 航点任务基本描述
	bp = WPBasicParam()
	bp.trackId = 1  # 航线编号
	bp.segmentId = 1  # 航段编号
	bp.wpNumber = 4  # 总共4个航点
	bp.executiveTimes = 1  # 仅执行一次
	bp.traceMode = 0  # 转弯模式为定点转弯
	bp.RCLostAction = 0  # RC失联后执行失联保护
	# 添加航点
	wps = []
	# 第一个航点为起飞点，起飞悬停10s
	wp1 = WPItem()
	wp1.latitude = ori_pos.latitude
	wp1.longitude = ori_pos.longitude
	wp1.altitude = ori_pos.altitude + 8
	wp1.heading = 360  # 离开该航点时机头指向下一航点
	wp1.velocity = 0.5  # 离开该航点时速度为0.5m/s
	wp1.type = 0  # 起飞点
	wp1.hovertime = 10  # 悬停10s
	wps.append(wp1)

	# 第二个航点为PASS，在北向0.0003方向，不悬停
	wp2 = WPItem()
	wp2.latitude = ori_pos.latitude + 0.0003
	wp2.longitude = ori_pos.longitude
	wp2.altitude = ori_pos.altitude + 8
	wp2.heading = 360  # 离开该航点时机头指向下一航点
	wp2.velocity = 0.5  # 离开该航点时速度为0.5m/s
	wp2.type = 1  # PASS点
	wp2.hovertime = 0  # 不悬停
	wps.append(wp2)

	# 第三个航点为PASS，在北向0.0003,东向0.0002,悬停10s
	wp3 = WPItem()
	wp3.latitude = ori_pos.latitude + 0.0003
	wp3.longitude = ori_pos.longitude + 0.0002
	wp3.altitude = ori_pos.altitude + 8
	wp3.heading = 360  # 离开该航点时机头指向下一航点
	wp3.velocity = 0.5  # 离开该航点时速度为2m/s
	wp3.type = 1  # PASS点
	wp3.hovertime = 10  # 悬停
	wps.append(wp3)

	# 第四个点为返航点，南向0.0003,东向0.0002
	wp4 = WPItem()
	wp4.latitude = ori_pos.latitude - 0.0003
	wp4.longitude = ori_pos.longitude + 0.0002
	wp4.altitude = ori_pos.altitude + 8
	wp4.heading = 360  # 离开该航点时机头指向下一航点
	wp4.velocity = 0.5  # 离开该航点时速度为0.5m/s
	wp4.type = 2  # 自动返航点
	wp4.returnHeading = 360
	wps.append(wp4)

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
		way_point()


if __name__ == "__main__":
	main()
