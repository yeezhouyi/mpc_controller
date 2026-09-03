# rl/ — RRBot 关节空间残差 RL（主线 C4 起）

依据《title_ RRBot 关节.docx》第 5–9 周规划。原则：

- **RL 不替代 MPC、不创建第二个关节命令写入者**：策略只输出有限残差
  `tau_raw = tau_MPC + alpha * Delta_tau_RL`，运行时由安全投影/限幅与单一
  command writer 约束（部署契约后续 rl/runtime/）。
- **零残差等价**：`alpha=0` 时快速环境必须与纯 MPC baseline 逐命令一致
  （`test_zero_residual_matches_pure_mpc_baseline` 是 C4 硬门槛）。

## 目录

```
rl/fast_env/
  joint_arm.py      两关节臂 plant（双积分器 + 可选惯量/阻尼/延迟随机化）+ 参考轨迹
  residual_env.py   MDP 环境（残差动作/观测/奖励/终止；baseline=无约束 condensed MPC + 限幅）
rl/tests/test_fast_env.py
```

## MDP 摘要（详见环境 docstring）

- 观测：`[e_q(2), e_qd(2), q_ref_next(2), prev_tau(2)]`（仅运行时可获取量）。
- 动作：归一化 `Delta_tau in [-1,1]^2`（每步满幅 ±1 Nm）。
- 奖励：跟踪误差 + 沿参考进展 + 力矩平滑 + 约束违反惩罚 + 完成奖励。
- 终止：参考结束 / 持续超界 / 最大步数。
- baseline MPC：无约束 condensed 有限时域解 + effort/rate 限幅（fast-env 用）；
  与生产控制器 OSQP 约束解的逐位对齐是后续校准项（记录，不冒充）。

## 运行测试（WSL2 或任意 python3 + numpy + pytest）

```bash
cd <repo>/rl && python3 -m pytest tests -q
```
