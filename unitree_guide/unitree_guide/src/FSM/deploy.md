模型对应关节顺序：





**`forward`函数**：
   - `obs_history`是模型的输入，假设是一个包含历史观测数据的张量。
   - `parts = self.estimator(obs_history)[:, 0:19]`：使用estimator对`obs_history`进行编码，并取编码结果的前19列。
   - `vel, z = parts[..., :3], parts[..., 3:]`：将前19列分成两个部分，前3列赋值给`vel`，后16列赋值给`z`。
   - `z = F.normalize(z, dim=-1, p=2.0)`：对`z`进行归一化，使其具有单位范数。
   - `torch.cat((obs_history[:, 0:45], vel, z), dim=1)`：将`obs_history`的前45列与`vel`和`z`连接起来，作为actor的输入。
   - `return self.actor(...)`：将连接后的张量输入actor，得到模型的输出。





self.commands[:, :3] * self.commands_scale：提取并缩放前三个命令。

self.base_ang_vel * self.obs_scales.ang_vel：角速度乘以相应的缩放比例。

self.projected_gravity：投影重力（无缩放）。
(self.dof_pos - self.default_dof_pos) * self.obs_scales.dof_pos：自由度位置与默认位置的差值，并缩放。
self.dof_vel * self.obs_scales.dof_vel：自由度速度，并缩放。
self.actions：当前的动作。
所有这些张量在最后被连接成一个current_obs张量。

+ num_obs = 3 + 3 + 3 + 12 + 12 + 12 = 45
+ num_history = 6




### 整体网络结构

#### 1. `HIMActorCritic` 类

`HIMActorCritic` 类是一个包含 Actor 和 Critic 的神经网络模型。这个模型使用 `HIMEstimator` 来处理时间序列观测数据，并生成动作分布和状态值估计。

**输入和输出**：
- **输入**：
  - `obs_history`：时间序列观测数据（形状 `[batch_size, temporal_steps * num_one_step_obs]`）
  - `critic_observations`：当前时间步的特权观测数据（形状 `[batch_size, num_critic_obs]`）

- **输出**：
  - `actions`：从分布中采样的动作（形状 `[batch_size, num_actions]`）
  - `value`：当前时间步的状态值（形状 `[batch_size, 1]`）

#### 2. `HIMEstimator` 类

`HIMEstimator` 类用于处理时间序列观测数据并提取特征。它包括一个编码器（encoder）、目标网络（target）和原型网络（proto）。

**输入和输出**：
- **输入**：
  - `obs_history`：时间序列观测数据（形状 `[batch_size, temporal_steps * num_one_step_obs]`）
  - `next_critic_obs`：下一个时间步的特权观测数据（形状 `[batch_size, num_one_step_obs + 3]`）

- **输出**：
  - `vel`：编码后的速度特征（形状 `[batch_size, 3]`）
  - `z`：编码后的潜在特征（形状 `[batch_size, enc_hidden_dims[-1]]`）
  - `estimation_loss`：用于训练的估计损失（标量）
  - `swap_loss`：用于训练的交换损失（标量）




### 网络整体处理流程

1. **观测数据输入**：
    - 输入：`obs_history`（形状: `[1, temporal_steps * num_one_step_obs]`）和 `critic_observations`（形状: `[1, num_critic_obs]`）。

2. **特征提取**：
    - `HIMEstimator` 使用 `encode` 方法提取速度特征 `vel`（形状: `[1, 3]`）和潜在特征 `z`（形状: `[1, enc_hidden_dims[-1]]`）。

3. **更新分布**：
    - `HIMActorCritic` 使用 `update_distribution` 方法，将提取的特征与 `obs_history` 拼接，生成 `actor_input`（形状: `[1, num_one_step_obs + 3 + enc_hidden_dims[-1]]`）。
    - `actor_input` 输入到 Actor 网络中，生成动作均值 `actions_mean`（形状: `[1, num_actions]`），并定义动作分布。

4. **选择动作**：
    - `HIMActorCritic` 调用 `act` 方法，从分布中采样动作 `actions`（形状: `[1, num_actions]`）。

5. **状态值评估**：
    - `HIMActorCritic` 调用 `evaluate` 方法，使用 Critic 网络评估状态值 `value`（形状: `[1, 1]`）。





Estimator的encode 
+ 输入：[1, 45 * 6] 最新的Obs在最前面
+ 输出：[1, 3 + 16] vel + latent


Actor得出动作
+ 输入：[1, 45 * 6 + 3 + 16] obs_history + vel +latent
+ 输出：[1, 12] actions