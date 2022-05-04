# Risk

topic: /vehicle_1/risk_msg_output: show msg to user by outputing to interface（for example: arm?）

topic: /vehicle_1/risk_msg_input: risk_management node input

(interface：(for example) arm? if yes，interface needs send ‘arm checked' to topic

 controller: 

​				check: send ’arm check'

​				init finished: send 'init finished'

)

topic: /vehicle_1/risk_alarm_state: send risk alarm state to controller node 

(state:

​		'-1': safe

​		'1': border check failure

​		'2': battery check failure

)
