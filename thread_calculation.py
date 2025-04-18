'''
根据机械臂的数量，来配置需要启动的线程数
机械臂的配置是根据用户定义来决定的，
4个机械臂应该如何定义，
2个机械臂应该如何定义，
都是从图形界面或者指令行等形式来获取的
例如 1，5，4则对应lowerGroup一组approach所需要的电机，
0,2,3则对应upperGroup一组approach所需要的电机，
9,8则对应pick_place方法所需要的电机
'''



def MultiArmThreads(armsQuantity): 
    '''
if armsQuantity = 4
        # _thread.start_new_thread(self.approach,('lower', [1, 5, 4]))
        # _thread.start_new_thread(self.approach,('upper', [0, 2, 3]))
        # _thread.start_new_thread(self.pick_place,('lower_left', 9))
        # _thread.start_new_thread(self.pick_place,('lower_right', 8))
        # _thread.start_new_thread(self.pick_place,('upper_left', 6))
        # _thread.start_new_thread(self.pick_place,('upper_right', 7))


if armsQuantity = 2
        # _thread.start_new_thread(self.approach,('lower', [1, 5, 4]))
        # _thread.start_new_thread(self.approach,('upper', [0, 2, 3]))
        # _thread.start_new_thread(self.pick_place,('lower_left', 9))
        # _thread.start_new_thread(self.pick_place,('lower_right', 8))

'''

