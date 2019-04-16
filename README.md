 # 2019_huawei_CodeCraft 
成绩： 2019华为软件精英挑战赛，粤港澳赛区，初赛23,复赛15。 此程序是复赛正赛版本。

思路： 
1）将无序crossid地图映射成有序crossid地图。
2）规划路径： 车分为src>dest和src<dest两批,分别为每批车规划最短路径，使用dijstra算法，处理细节： 每个（出发点，目的点）计算一次最短路径，每个（出发点，目的点）的路径唯一; 每计算一次（出发点，目的点）路径，会统计有多少车辆走这些路，这些车辆数会累加，后面其他（出发点，目的点）规划路径时，图的道路权重会乘上累计车辆数的惩罚因子。 
3）时间调度： 初赛正赛：src>dest批次先出发，src<dest后出发，（相当于车辆根据行驶方向进行分流了，一次只行驶一个方向，经验证明，它很好的降低死锁概率）均是按速度从快到慢发送。 复赛正赛：预置车辆先发，然后优先车辆分两个批次发送，然后普通车辆分两个批次发送，跟初赛类似。

不足： 道路利用率，当只允许车辆往一个方向跑的时候，道路利用率不足，虽然车辆数加权的处理增大了道路利用率，但仍有1/3的道路未被充分利用，且这些道路多是被隔断的，难以隔离利用。 
好处： 实现相对容易，大大降低了死锁的概率。

判题器： 进入复赛后我们意识到没有判题器根本不可能走远，因此也做了判题器，但是只对准了论坛的小图，官网的大图一直有100个时间片左右的出入。尽管已经找出了几个bug,还是没有对上，死锁也不一定都对的上。但是对于线下调参还是有一定的帮助，也附在src文件夹中，文件名为map_dispatch_v2.py，独立于其他程序。 判题器在src路径下执行python3 map_dispatch_v2.py即可。

有问题欢迎邮箱chhluo@foxmail.com交流，渣渣代码，大佬请忽略，求轻喷～

如果觉得对您有帮助，麻烦赏颗星星～
