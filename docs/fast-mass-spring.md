# 1. 前言

Fast Simulation of Mass-Spring Systems 是一篇2013年的布料模拟论文，开辟了一种新的隐式积分思路，并在后续衍生出了Projective Dynamics这样的一般性仿真框架。论文链接 => [Fast Simulation of Mass-Spring Systems](http://graphics.berkeley.edu/papers/Liu-FSM-2013-11/Liu-FSM-2013-11.pdf)
  
先简单过一遍算法

# 2. 算法

这里简述一些论文的核心思想步骤

## 2.1 首先将隐式积分问题转为一个优化问题
   
原本我们要解的隐式积分是: 
$$M(q_{n+1}-2q_n+q_{n-1})=h^2f(q_{n+1})$$
其中:
- n是时间迭代步
- q是所有粒子的位置向量
- M是粒子质量对角矩阵
- h是时间步长
- f是整个系统的保守力

这里的未知状态量是$q_{n+1}$，将其定义为$x$，而$q_n$以及$q_{n-1}$都是已知状态量，将其合并定义为 $y=2q_n-q_{n-1}$

那么就得到如下一个方程:
$$M(x-y)=h^2f(x) \tag 7$$

求解这个方程，等效于求解下面这个方程的极小值:

$$g(x) =\frac{1}{2}(x-y)^TM(x-y)+h^2E(x)$$

其中$E$为系统的势能。(因为$\nabla E=-f$，因此$\nabla g=0$等效于(7)式)

按照胡克定律，弹簧的弹性势能为:

$$E = \frac{1}{2}k(||p_1-p_2||-r)^2$$

其中:
- p1,p2为两个粒子的位置
- r为spring的rest length.

但如果直接采用这个形式，上面的$g(x)$极值问题就不太好解。为了将上式变形为一个方便求解的形式，作者引入了一个辅助变量$d\in R^3$

## 2.2 引入辅助变量d

这是论文的点睛之处。假设d是一个未知的三维向量，那么:

$$(||p_1-p_2||-r)^2=\underset{||d||=r}{min}||p_1-p_2-d||^2$$

这个式子的意思是，将向量d的长度约束为r，而方向可变。那么对右边式子求最小值，结果肯定等于左边。这个在脑海中想象一下，是显然的，且在
$$d=r\frac{p_1-p_2}{||p_1-p_2||}$$
时取得极小值。

引入辅助变量d后,我们就定义$E(x,d)$如下:

$$E(x,d)=\frac{1}{2}\sum_{i=1}^sk_i||p_{i1}-p_{i2}-d_i||^2=\frac{1}{2}x^TLx-x^TJd$$

$L,J$的具体矩阵形式推导参见附录1.

于是 $g(x)$就变成了:

$$g(x,d)=\frac{1}{2}(x-y)^TM(x-y)+h^2(\frac{1}{2}x^TLx-x^TJd)$$

整理，并抛弃常数项，就得到

$$g(x,d)=\frac{1}{2}x^T(M+h^2L)x-x^T(h^2Jd+My)$$

引入外力$f_{ext}$后

$$g(x,d)=\frac{1}{2}x^T(M+h^2L)x-x^T(h^2Jd+My+h^2f_{ext})$$

那么$\underset{x\in R^3}{min}\space g(x)$就变成了:

$$\underset{x\in R^3,d\in U}{min}\space g(x,d)$$
其中 U为{${d_1,...,d_s}$}，且$||d_i||=r_i$

这是一个二元二次带约束的优化问题。

## 2.3 Block Coordinate Descent

使用块坐标下降法来求$g(x,d)$的极值:

1. 首先固定变量x，只对变量d进行优化，得到$\~d$
2. 然后令$d=\~d$，再对x变量进行优化

在论文中步骤1称为local step，步骤2称为global step，之所以这么叫， 也是跟每个步骤所采用的具体优化方法有关的。

### 2.3.1 Local Step

首先步骤1要优化变量d。 这个我们在2.2中已经知道了，d的优化值为:
$$d=r\frac{p_1-p_2}{||p_1-p_2||}$$

在实际工程实现中，我们对每个Spring都按照以上公式计算对应的d。由于这个计算过程每个spring是独立的，因此称为local step。 整个过程完全可以并行。


### 2.3.2 Global Step

通过local step计算出所有的$d=\~d$后，带入$g(x,d)$，得到了一个二次规划问题。 解决二次规划问题，方法就有很多了。
值得注意的是，由于这里的system matrix = $(M+h^2L)$是一个对称正定矩阵，因此有唯一的最小值，求解该二次优化问题等效于求解线性方程组:

$$(M+h^2L)x=h^2Jd+My+h^2f_{ext}$$

论文中采用Cholesky分解对此进行求解。


# 3. 代码实现

- 数学库使用 [nalgebra](https://docs.rs/nalgebra/latest/nalgebra/)
  
核心步骤用代码表示即:
```rust
fn step(&mut self){
    for _ in 0..self.num_iterations {
        self.local_step();
        self.global_step();
    }
}
```

## 3.1 Local Step
首先看local step。 local step主要是利用投影的方式计算每个约束的向量$d$。 我们首先定义:
```rust
let vector_d = DVector::zeros(num_constraints * 3);
```

然后看local step的实现:
```rust
fn local_step(&mut self) {
    compute_vector_d(&self.cloth, &mut self.vector_d);
}

fn compute_vector_d(cloth: &Cloth, vector_d: &mut DVector) {
    debug_assert!(vector_d.len() == cloth.num_constraints() * 3);

    let mut constraint_index = 0;

    // 计算attachment约束的d
    for attachment in &cloth.attachments {
        let d = attachment.target_position;
        vector_d
            .fixed_rows_mut::<3>(constraint_index * 3)
            .copy_from(&d);
        constraint_index += 1;
    }

    // 计算spring约束的d
    for spring in &cloth.springs {
        let p0 = cloth
            .particle_positions
            .fixed_rows::<3>(spring.particle_index_0 * 3);
        let p1 = cloth
            .particle_positions
            .fixed_rows::<3>(spring.particle_index_1 * 3);
        let delta = p0 - p1;
        let d = delta.normalize() * spring.rest_length;
        vector_d
            .fixed_rows_mut::<3>(constraint_index * 3)
            .copy_from(&d);
        constraint_index += 1;
    }
}

```

上面代码里还加入了attachment constraint.该约束的相关公式推导见附录(2)



## 3.2 Global Step

在Global Step的时候我们解线性方程组:

$$(M+h^2L)x=h^2Jd+My+h^2f_{ext}$$

观察里面的几个项:

- $(M+h^2L)$是System Matrix，可以预计算，在仿真过程中不会变化
- $h^2J$也可以预计算，仿真过程中不会变化
- $My+h^2f_{ext}$在每个仿真步中只需要计算一次，我们称其为`inertial_impluse_term`

因此global step实现如下

```rust
fn global_step(&mut self) {
    let b = &self.h2_matrix_j * &self.vector_d + &self.inertial_impluse_term;
    self.cloth.particle_positions = self.cholesky.solve(&b);
}
```

$My+h^2f_{ext}$的计算:

```rust

fn pre_compute_terms(&mut self) {
    let damping = self.damping;
    let positions = &self.cloth.particle_positions;
    let prev_positions = &self.cloth.prev_particle_positions;
    // inertial_impluse_term = M * y + h^2 * f_ext
    self.inertial_impluse_term = &self.matrix_m
        * ((1.0 + damping) * positions - damping * prev_positions)
        + &self.impulse_term;
}
```


Matrix $J$的计算:

```rust
fn compute_matrix_j(cloth: &Cloth) -> DMatrix {
    let i3 = Matrix3::identity();
    let mut matrix_j = DMatrix::zeros(3 * cloth.num_particles(), 3 * cloth.num_constraints());
    let mut constraint_index = 0;
    // attachment对J的贡献
    for attachment in cloth.attachments.iter() {
        let i = attachment.particle_index;
        let k = attachment.stiffness;
        matrix_j
            .fixed_view_mut::<3, 3>(3 * i, 3 * constraint_index)
            .copy_from(&(k * i3));
        constraint_index += 1;
    }

    // spring对J的贡献
    for spring in cloth.springs.iter() {
        let i = spring.particle_index_0;
        let j = spring.particle_index_1;
        let k = spring.stiffness;
        matrix_j
            .fixed_view_mut::<3, 3>(3 * i, 3 * constraint_index)
            .copy_from(&(k * i3));
        matrix_j
            .fixed_view_mut::<3, 3>(3 * j, 3 * constraint_index)
            .copy_from(&(-k * i3));
        constraint_index += 1;
    }
    matrix_j
}

```

Matrix $L$的计算

```rust
fn compute_matrix_l(cloth: &Cloth) -> DMatrix {
    let i3 = Matrix3::identity();
    let mut matrix_l = DMatrix::zeros(3 * cloth.num_particles(), 3 * cloth.num_particles());
    
    // 计算attachment对矩阵L的贡献
    for attachment in &cloth.attachments {
        let k = attachment.stiffness;
        let i = attachment.particle_index;
        matrix_l
            .fixed_view_mut::<3, 3>(3 * i, 3 * i)
            .add_assign(&(k * i3));
    }

    // 计算spring对矩阵L的贡献
    for spring in &cloth.springs {
        let k = spring.stiffness;
        let i = spring.particle_index_0;
        let j = spring.particle_index_1;
        matrix_l
            .fixed_view_mut::<3, 3>(3 * i, 3 * i)
            .add_assign(&(k * i3));
        matrix_l
            .fixed_view_mut::<3, 3>(3 * j, 3 * j)
            .add_assign(&(k * i3));
        matrix_l
            .fixed_view_mut::<3, 3>(3 * i, 3 * j)
            .add_assign(-k * i3);
        matrix_l
            .fixed_view_mut::<3, 3>(3 * j, 3 * i)
            .add_assign(-k * i3);
    }
    matrix_l
}

```

# 4. 其他

本项目只用于验证算法效果，在实际产品中，应当有许多性能优化空间，例如:
- 对local step并行化
- 使用稀疏矩阵格式
- 研究global step的并行化空间
- 利用system matrix的良好特性进行特殊化求解

# 5. 问题

在实践中观察到一些问题，不确定是算法本身的原因，还是我的代码实现有差错。

- 当调大spring stiffness时，阻尼感会明显增强。
- 当调大time step时，阻尼感同样会明显增强。

# 6. 附录

### (1) 矩阵L和J的推导

$$E(x,d)=\frac{1}{2}\sum_{i=1}^sk_i||p_{i1}-p_{i2}-d_i||^2=\frac{1}{2}x^TLx-x^TJd$$

这里s为spring数量，$p_{i1},p_{i2}$为第i个spring所约束的两个粒子的位置。

我们记$A_i^T=[a_{i0},...,a_{im}]$, where:

$$
a_{ij}=
\left \{
\begin{aligned}
1&, \text{if j is 1st particle index of spring i } \\
-1&, \text{if j is 2st particle index of spring i} \\
0&, \text {others}
\end{aligned}
\right.
$$

那么$p_{i1}-p_{i2}$就可以表示为$A_i^Tx$.

于是

$$
\begin{aligned}
E(x,d)=&\frac{1}{2}\sum_{i=1}^sk_i||p_{i1}-p_{i2}-d_i||^2 \\
=&\frac{1}{2}\sum_{i=1}^sk_i||A_i^Tx-d_i||^2 \\
\end{aligned}
$$

由于:

$$
\begin{aligned}
||A_i^Tx-d_i||^2=&(A_i^Tx-d_i)^T(A_i^Tx-d_i) \\
=&x^TA_iA_i^Tx - 2x^TA_id_i + d_i^Td_i
\end{aligned}
$$

抛弃掉常数项$d_i^Td_i$后，代入$E(x,d)$，即有:

$$
E(x,d)=\frac{1}{2}
\sum_{i=1}^s k_i x^TA_iA_i^Tx - 
\sum_{i=1}^s k_i x^TA_id_i 
$$

令

$$
S_i^T=[\delta_{i0},...,\delta_{is}]
$$

其中
$$
\delta_{ij}=
\left \{
\begin{aligned}
1&,  i=j  \\
0&, i\ne j \\
\end{aligned}
\right.
$$

那么就有$d_i=S_i^Td$，因此$E(x,d)$可以写为

$$
E(x,d)=\frac{1}{2}
x^T (\sum_{i=1}^s k_i A_iA_i^T)x - 
x^T (\sum_{i=1}^s k_i A_i S_i^T) d
$$

显然:
$$
\begin{aligned}
L = \sum_{i=1}^s k_i A_iA_i^T \\
J = \sum_{i=1}^s k_i A_i S_i^T
\end{aligned}
$$


### (2) Attachment约束下的$d$,$L$,$J$的影响

Attachment约束用于将粒子固定在空间中的某个位置，单个约束的E可以定义为

$$
E_i = \frac{1}{2}k_i||p_i-d_i||^2
$$

很明显，对于attachment约束:

- $d_i$就是attachment的空间坐标.
- $A_i^T=[\delta_{i0},...,\delta_{im}]$