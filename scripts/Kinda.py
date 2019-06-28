flag = 0
while True:
	answer = main.answer_question_from_data(msg, kdata)['answer']
	print(answer)
	
	if answer == 'follow':
		flag = 1
	elif answer == 'stop':
		k.move(0, 0)
		break
	
	forward_speed, turn_speed = f.follow(c.depth_image, flag==1)
	k.move(forward_speed, turn_speed)
