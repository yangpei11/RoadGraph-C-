# RoadGraph CODE BY C++
参考规范：国家标准GB5768-2009


一、道路路段属性

key                                value(default)                解释
					       	       					       
highway_type                       参考osm的标准               道路类型
					       	       					       
lanes                           > 1                      车道数
					       	       					       
lane_width                          3.5 / 3.75                  车道宽度
					       	       					       
oneway                               true / false                  是否是单行道
							       
offset_to_terrain                      1.5                     路面到地面的高度
	
emergency_lane_width					3.5                    应急车道的宽度，只有当highway_type的为moterway也就是高速公路才会加应急车道
					       

二、道路标线（只考虑线条 用于划分车道的标线）的属性：
							       
road_marker_line_type              (白色虚线、白色实线、黄色虚线、黄色实线、 双白虚线、 双黄实线、黄色虚实线)
							       ( white_dash、white_solid、 yellow_dash、 yellow_solid、 double_white_dash、 double_yellow_solid、 yellow_dash_solid）
 							       
road_marker_line_width             0.15m                       参见网站( http://www.fmxjj.gov.cn/infoshow.asp?ArticleID=306 )
						           
road_marker_line_length            2m、4m、6m                  道路标线为虚线时，其长度, 详情参考上面的网站说明
						           
road_marker_line_gap_length        4m、6m、9m			       道路标线为虚线时，其间隔, 详情参考上面的网站说明
							       
road_marker_line_double_gap        0.15                        双条标线之间的距离

road_marker_line_offset_to_road    0.1						   道路标线的离道路高度的距离



二、道路交叉口属性

road_cross_connection_type         line/arc                    两条道路边缘连接方式，直线或者圆弧

road_cross_intersection_buffer_length 20.0                     路段与路口连接段的长度

road_cross_stop_line_buffer_length    15.0;                    路口停止线的到路口中心的距离

road_cross_arc_granularity			20							弧度的细分级别

road_marker_line_offset_to_road     0.1

road_arc_to_line_angle_thresold 175 ~ 185°                  判断由弧转变成直线的阈值范围



三、道路路口标线， 人行横道

road_marker_cross_width            0.4m（0.45m）                 人行横道斑马线的宽度
								   
road_marker_cross_gap              0.6m                          人行横道斑马线之间的间距
								   
road_marker_cross_length           >= 3m                         人行横道的长度

road_marker_cross_to_stop_line     2                             人行横道到停止线的距离





