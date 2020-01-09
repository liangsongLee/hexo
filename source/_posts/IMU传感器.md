---
title: IMU传感器
date: 2020-01-09 15:41:00
tags:
mathjax: true
---
#IMU传感器
##IMU误差模型
**误差分类**：加速度计和陀螺仪的误差可以分为：确定性误差、随机误差。
确定性误差可以事先标定，包括：**bias**，**scale** ...
随机误差通常假设噪声服从高斯分布，包括：**高斯白噪声**，**bias**，**随机游走 **...
###确定性误差
**Bias：**理论上，当没有外部作用时，IMU传感器的输出应该为0，但是，实际数据存在一个偏置$\pmb{b}$。加速度计bias对位姿估计的影响：
$$\pmb{v}_{err} = \pmb{b}_at\\
\pmb{p}_{err} = \frac 12\pmb{b}_at^2.$$
**Scale：**实际数值和传感器输出值之间的比值。
###随机误差
**高斯白噪声：**IMU连续时间上受到一个均值为0，方差为$\sigma$，各时刻之间相互独立的高斯过程$n(t)$：
$$E[n(t)]\equiv0\\
E[n(t_1)n(t_2)]=\sigma^2\delta(t_1-t_2)$$
其中$\delta()$表示狄拉克函数。
实际上，IMU传感器获取的数据为**离散采样**，离散和连续高斯白噪声方差之间存在如下关系：
$$n_d[k]=\sigma_dw[k]$$
其中：
$$w[k]\sim\mathcal{N}(0,1)\\
\sigma_d=\sigma \frac 1 {\sqrt{\Delta t}}$$
即高斯白噪声的连续时间到离散时间之间差一个$\frac 1{\sqrt{\Delta t}}$，$\sqrt\Delta t$是传感器的采样时间。
Bias随机游走：通常用维纳过程来建模bias随时间连续变化的过程，离散时间下称之为随机游走。
$$\dot b(t)=n(t)=\sigma_bw(t)$$
其中w是方差为1的白噪声。
离散和连续之间的转换：
$$w[k]\sim\mathcal{N}(0,1)\\
\sigma_{bd}=\sigma_b\sqrt{\Delta t}$$
即bias随机游走的噪声方差从连续时间到离散时间之间需要乘以$\sqrt{\Delta t}$。
##运动模型离散时间处理
###VIO中的IMU模型
忽略scale的影响，只考虑白噪声和bias随机游走：
$$\tilde{\pmb{\omega}}^b=\pmb{\omega}^b+\pmb{b}^g+\pmb{n}^g\\
\tilde{\pmb{a}}^b=\pmb{q}_{bw}(\pmb{a}^w+\pmb{g}^w)+\pmb{b}^a+\pmb{n}^a$$
上标$g$表示陀螺仪，$a$表示加速度计，$w$表示在世界坐标系world，$b$表示IMU机体坐标系body。IMUD真实值为$\pmb{\omega}$，$\pmb{a}$，测量值为$\tilde{\pmb{\omega}}$，$\tilde{\pmb{a}}$。
P(ose),V(elocity),Q(uaternion)对时间的导数可以写成：
$$\dot{\pmb{p}}_{wb_t}=\pmb{v}_l^w\\
\dot{\pmb{v}}_l^w=\pmb{a}_l^w\\
\dot{\pmb{q}_{wb_l}}=\pmb{q}_{wb_l}\otimes\begin{bmatrix}0 \\ {\frac 12\pmb{\omega}^{b_t}} \end{bmatrix}$$
###连续时间下IMU运动模型
根据上述的导数关系，可以从第i时刻的PVQ，通过对IMU的测量值进行积分，得到第j时刻的PVQ：
$$\pmb{p}_{wb_j}=\pmb{p}_{wb_i}+\pmb{v}_i^w\Delta t+\iint_{t\in[i,j]}(\pmb{q}_{wb_t}\pmb{a}^{b_t}-\pmb{g}^w)\delta t^2\\
\pmb{v}_j^w=\pmb{v}_i^w+\int_{t\in[i,j]}(\pmb{q}_{wb_t}\pmb{a}^{b_t}-\pmb{g}^w)\delta t\\
\pmb{q}_{wb_j}=\int_{t\in[i,j]}\pmb{q}_{wb_t}\otimes\begin{bmatrix}0 \\ {\frac 12\pmb{\omega}^{b_t}} \end{bmatrix}\delta t$$
###运动模型的离散积分——欧拉法
使用欧拉法，两个相邻时刻k到k+1的位姿是用第k时刻的测量值$\pmb a$，$\pmb{\omega}$来计算。
$$\pmb{p}_{wb_{k+1}}=\pmb{p}_{wb_k}+\pmb{v}_k^w\Delta t+\frac 12\pmb{a}\Delta t^2\\
\pmb{v}_{k+1}^w=\pmb{v}_k^w+\pmb{a}\Delta t\\
\pmb{q}_{wb_{k+1}}=\pmb{q}_{wb_{k}}\otimes\begin{bmatrix}1 \\ {\frac 12\pmb{\omega}\delta t} \end{bmatrix}
$$
其中，
$$\pmb a=\pmb{q}_{wb_k}(\pmb{a}^{b_k}-\pmb{b}_k^a)-\pmb{g}^w\\
\pmb{\omega}=\pmb{\omega}^{b_k}-\pmb{b}_k^g$$
###运动模型的离散积分——中值法
使用中值法，两个相邻时刻k到k+1的位姿是用两个时刻的测量值$a$，$\omega$的平均值来计算。
$$\pmb{p}_{wb_{k+1}}=\pmb{p}_{wb_k}+\pmb{v}_k^w\Delta t+\frac 12\pmb{a}\Delta t^2\\
\pmb{v}_{k+1}^w=\pmb{v}_k^w+\pmb{a}\Delta t\\
\pmb{q}_{wb_{k+1}}=\pmb{q}_{wb_{k}}\otimes\begin{bmatrix}1 \\ {\frac 12\pmb{\omega}\delta t} \end{bmatrix}
$$
其中，
$$\pmb a=\frac 12[\pmb{q}_{wb_k}(\pmb{a}^{b_k}-\pmb{b}_k^a)-\pmb{g}^w+\pmb{q}_{wb_{k+1}}(\pmb{a}^{b_{k+1}}-\pmb{b}_k^a)-\pmb{g}^w]\\
\pmb{\omega}=\frac 12[(\pmb{\omega}^{b_k}-\pmb{b}_k^g)+(\pmb{\omega}^{b_{k+1}}-\pmb{b}_k^g)]$$
