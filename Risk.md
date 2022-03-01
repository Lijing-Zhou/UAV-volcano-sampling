# Risk

topic: /vehicle_1/risk_msg_output 用于输出到interface给用户看（例如：是否arm）

topic: /vehicle_1/risk_msg_input 用于risk_management node输入

(interface：(例如)是否arm，若用户选择是，interface需要向此input topic发送‘arm checked'

 controller: 

​				check状态: 发送’arm check'

​				init 结束: 发送'init finished'

)

topic: /vehicle_1/risk_alarm_state 用于向controller node传输risk alarm state

(state:

​		'-1': safe

​		'1': border check failure

​		'2': battery check failure

)
