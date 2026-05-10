# latent-brs 数学与控制原理说明

本文档概述本项目在视觉潜在空间建模、动力学学习与模型预测控制（MPC）上的核心数学原理与控制原理。公式中的行内表达使用$...$，行间公式使用$$...$$。

## 1. 视觉表征与潜在状态构造

项目将图像观测$o_t$映射到潜在嵌入$e_t \in \mathbb{R}^d$。设视觉编码器为$E_\phi(\cdot)$、投影头为$P_\psi(\cdot)$，则
$$
h_t = E_\phi(o_t), \quad e_t = P_\psi(h_t).
$$

为了形成马尔可夫潜在状态，代码将当前嵌入与嵌入差分拼接：
$$
\Delta e_t = e_t - e_{t-1}, \quad s_t = \begin{bmatrix} e_t \\ \Delta e_t \end{bmatrix} \in \mathbb{R}^{2d}.
$$
当不存在$t-1$时，使用$\Delta e_t = 0$。这与训练中“Markov state = embedding + delta”的实现一致。

## 2. JEPA 与潜在动力学学习

项目采用 Joint-Embedding Predictive Architecture（JEPA）思想：编码历史帧得到嵌入序列，并用动作序列驱动潜在动力学预测。记动力学预测器为$f_\theta$，则
$$
\hat{s}_{t+1} = f_\theta(s_t, a_t),
$$
其中$a_t$为动作（在数据集中经均值方差归一化）。

### 2.1 预测损失

训练时使用多步自回归预测，将预测嵌入$\hat{e}_{t+1}$与真实嵌入$e_{t+1}$对齐。核心预测损失为均方误差：
$$
\mathcal{L}_{\text{pred}} = \frac{1}{N} \sum_{t} \left\| \hat{s}_{t+1} - s_{t+1} \right\|_2^2,
$$
其中$s_{t+1} = [e_{t+1},\, e_{t+1}-e_t]^T$。

### 2.2 SIGReg 正则化

SIGReg（Sketch Isotropic Gaussian Regularizer）通过随机投影近似约束嵌入分布接近各向同性高斯。设随机方向向量$a \sim \text{Unif}(\mathbb{S}^{d-1})$（实践中采样$M$个方向并归一化），尺度参数$\tau \in [0,3]$，理想高斯特征函数$\phi(\tau) = \exp(-\tau^2/2)$，则统计量可写为
$$
\mathcal{S}(E) = \mathbb{E}_{a}\left[ \big(\mathbb{E}[\cos(\tau a^\top e)] - \phi(\tau)\big)^2 + \big(\mathbb{E}[\sin(\tau a^\top e)]\big)^2 \right].
$$
SIGReg损失即$\mathcal{L}_{\text{sig}} = \mathcal{S}(E)$。

### 2.3 轨迹“拉直”正则化

为了鼓励潜在轨迹速度平滑，项目引入“时间直线化”损失。设速度$v_t = e_t - e_{t-1}$，则
$$
\mathcal{L}_{\text{straight}} = \mathbb{E}\left[1 - \frac{v_t^\top v_{t+1}}{\|v_t\|_2 \|v_{t+1}\|_2 + \epsilon}\right].
$$

### 2.4 总损失

综合损失为
$$
\mathcal{L} = \mathcal{L}_{\text{pred}} + \lambda_{\text{sig}} \mathcal{L}_{\text{sig}} + \lambda_{\text{straight}} \mathcal{L}_{\text{straight}}.
$$

## 3. Koopman 线性化动力学

项目中还包含 Koopman 模型，用非线性编码器$g(\cdot)$将原始状态$x_t$提升到线性空间$z_t$，使动力学在$z$空间中线性：
$$
z_t = g(x_t), \quad z_{t+1} = A z_t + B u_t.
$$

两种形式：
1. **无解码器形式**：$z_t = [x_t,\, g(x_t)]$，直接在升维空间滚动。
2. **线性解码器形式**：$x_t = C z_t$，提供线性回到原状态的映射。

典型训练目标是最小化一步预测误差：
$$
\min_{A,B,g} \; \sum_t \left\| z_{t+1} - (A z_t + B u_t) \right\|_2^2.
$$

## 4. MPC 控制框架

项目使用**滚动时域控制**（receding horizon MPC）。在每个时间步，求解长度$H$的最优控制序列$\{u_0,\dots,u_{H-1}\}$，仅执行首个动作$u_0$，再将序列平移作为下次初值（warm-start）。

### 4.1 目标函数

在潜在状态空间中定义二次型代价：
$$
J = \sum_{k=0}^{H-1} \left( q_s \|x_k - x_g\|_2^2 + r \|u_k\|_2^2 \right) + q_T \|x_H - x_g\|_2^2,
$$
其中$x_k$为潜在状态，$x_g$为潜在目标，$q_s,q_T,r$为加权系数。

## 5. iLQR-MPC（非线性动力学）

对于 MLP 潜在动力学$x_{k+1}=f(x_k,u_k)$，iLQR使用线性化
$$
x_{k+1} \approx f(x_k,u_k) + A_k \delta x_k + B_k \delta u_k,
$$
并对代价作二次近似。引入局部$Q$函数：
$$
Q_x = l_x + A_k^\top V_x, \quad Q_u = l_u + B_k^\top V_x,
$$
$$
Q_{xx} = l_{xx} + A_k^\top V_{xx} A_k, \quad Q_{ux} = B_k^\top V_{xx} A_k,
$$
$$
Q_{uu} = l_{uu} + B_k^\top V_{xx} B_k + \mu I.
$$
控制更新为
$$
k_k = -Q_{uu}^{-1} Q_u, \quad K_k = -Q_{uu}^{-1} Q_{ux},
$$
并通过线搜索更新轨迹。

### 定理 1（局部二次模型的最优控制增量）
若$Q_{uu} \succ 0$，则在二次近似下，$\delta u_k = k_k + K_k \delta x_k$是使局部增量成本最小的唯一解。

**证明：** 二次近似下局部代价为
$$
\delta J = \frac{1}{2} \delta u_k^\top Q_{uu} \delta u_k + \delta u_k^\top (Q_u + Q_{ux}\delta x_k) + \text{const}.
$$
对$\delta u_k$求梯度并令其为零得
$$
Q_{uu} \delta u_k + Q_u + Q_{ux}\delta x_k = 0,
$$
因此$\delta u_k = -Q_{uu}^{-1}(Q_u + Q_{ux}\delta x_k)$，即$\delta u_k = k_k + K_k \delta x_k$。由于$Q_{uu}\succ 0$，解唯一。∎

### 推论 1（正则化保证可逆）
当在$Q_{uu}$中加入$\mu I$（$\mu>0$）时，$Q_{uu}$至少正定，从而定理1的解存在且唯一。

**证明：** 任意非零向量$v$满足
$$
v^\top Q_{uu} v \ge \mu \|v\|_2^2 > 0,
$$
故$Q_{uu}\succ 0$。∎

## 6. Koopman-ADMM-MPC（线性动力学）

当动力学为线性$z_{k+1}=A z_k + B u_k$时，MPC可转化为带等式约束的二次规划：
$$
\min_{\{u_k,z_k\}} \frac{1}{2}\begin{bmatrix}u\\z\end{bmatrix}^\top P \begin{bmatrix}u\\z\end{bmatrix} + q^\top \begin{bmatrix}u\\z\end{bmatrix}
$$
$$
\text{s.t. } z_{0} = \bar{z}_0,\; z_{k+1} - A z_k - B u_k = 0.
$$

项目构建 KKT 系统并显式求解：
$$
\begin{bmatrix}
P & A_{\text{eq}}^\top \\
A_{\text{eq}} & 0
\end{bmatrix}
\begin{bmatrix}
y \\ \lambda
\end{bmatrix}
=
\begin{bmatrix}
-q \\ b_{\text{eq}}
\end{bmatrix},
$$
其中$y=[u;z]$。

### 定理 2（无约束 ADMM 的一次收敛）
若控制不受额外约束（投影为恒等映射），则一次 KKT 求解即给出全局最优解，ADMM 迭代可在一步内终止。

**证明：** 二次规划带线性等式约束时，KKT 系统的解即为全局最优解。无约束投影不改变该解，因此 ADMM 迭代在第一次“原问题更新”后已满足最优性条件。∎

## 7. 控制原理小结

1. **潜在动力学建模**：通过视觉编码器与 MLP/Koopman 模型，在潜在空间构建可预测动力学。
2. **代价设计**：潜在状态与控制输入的二次型代价提供可微且易优化的目标。
3. **iLQR 与 ADMM MPC**：非线性使用 iLQR 递推+线搜索；线性 Koopman 使用 KKT 闭式解并具备 warm-start 平移。
4. **滚动时域控制**：MPC 以 receding horizon 方式运行，实时更新最优控制序列。
