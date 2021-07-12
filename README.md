# Webots-Stewart-Platform-Game  
通过Webots平台对Stwart机器人进行仿真  
1.运行环境为Webots2020a revision 1。  
2.选择一个模式后，打开worlds文件夹里的stewart_platform.wbt文件即可。  
2.除了ACTION_MODE其他模式必须先连接手柄才能使用，目前可用XBOX 2020控制器控制，其他手柄还没有进行测试。  
3.点击运行之后要用鼠标点一下仿真画面才能开始操作。  
4.每个模式的代码为controllers\stewart_platform里的stewart_platform.c文件。最外面的stewart_platform.c为MOVIE_MODE的代码，可对平台进行六个自由度的操作，并完成一些别的动作。   

模式介绍  
ACTION_MODE: 对应守望先锋里的英雄铁拳  
FLIGHT_MODE: 用于飞行模拟  
GAME_MODE: 做了一个小游戏，通过操控手柄让小球通过迷宫  
MOVIE_MODE: 用于电影特效的拍摄  
RACING_MODE: 用于赛车模拟器
