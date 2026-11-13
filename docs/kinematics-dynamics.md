# Công thức và vị trí trong code

Ký hiệu: $R$ bán kính bánh, $2L$ khoảng cách hai bánh, $V_m$ và $\omega_m$ vận tốc dài và góc của tâm xe, $V_r, V_l$ vận tốc dài hai bánh, $\omega_r, \omega_l$ vận tốc góc hai bánh, $\tau_r, \tau_l$ mô-men hai bánh.

## Động học thuận

$V_i = R\,\omega_i$, $\quad V_m = \dfrac{V_r + V_l}{2}$, $\quad \omega_m = \dfrac{V_r - V_l}{2L}$

$\dot X = V_m\cos\theta$, $\quad \dot Y = V_m\sin\theta$

| Công thức | Vị trí |
|---|---|
| $V_i = R\,\omega_i$ | `Encoder::update` |
| $V_m$, $\omega_m$ | `Chassis::computeForwardKinematics` |
| $\dot X$, $\dot Y$ | odometry trong `Chassis::update` (θ lấy từ IMU) |
| Ô web | `Chassis::setWheelVelocities` |

## Động học nghịch

$V_r = V_m + L\,\omega_m$, $\quad V_l = V_m - L\,\omega_m$, $\quad \omega_i = V_i / R$

| Công thức | Vị trí |
|---|---|
| $V_r$, $V_l$ | `Chassis::computeInverseKinematics`, gọi qua `applyVW` |
| $\omega_i = V_i/R$ | chia `WHEEL_RADIUS` tại nơi cần dùng (`solveInverseDynamics`, web) |
| Ô web | `Chassis::setVelocityVW` |

## Động lực học thuận

$$\dot V_m = \frac{\tau_r + \tau_l}{m R} - \frac{F_c}{m}, \qquad \dot\omega_m = \frac{L(\tau_r - \tau_l)}{I R} - \frac{M_c}{I}$$

với $F_c = F_{ms} + b_v |V_m|$ và $M_c = F_{ms} L + b_\omega |\omega_m|$, luôn ngược chiều chuyển động.

| Công thức | Vị trí |
|---|---|
| $F_c$, $M_c$ | `resistance()` trong `Chassis.cpp` |
| $\dot V_m$, $\dot\omega_m$ | `Chassis::computeAccelFromTorque` (có xét ma sát tĩnh khi đứng yên) |
| PWM thành gia tốc | `Chassis::computeForwardDynamics` |
| Ô web | `Chassis::setTorques` (tích phân gia tốc rồi PID bám theo) |

Lưu ý: mô-men không đổi cho gia tốc không đổi, nên đây là chuyển động tăng tốc liên tục. Động cơ thật bão hòa rất nhanh khi chạy hở, vì vậy ô này mô phỏng phương trình trong phần mềm (`TORQUE_USE_MODEL`).

## Động lực học nghịch

$$\tau_r = \frac{R}{2}\Big[(m\dot V_m + F_c) + \frac{1}{L}(I\dot\omega_m + M_c)\Big], \qquad
\tau_l = \frac{R}{2}\Big[(m\dot V_m + F_c) - \frac{1}{L}(I\dot\omega_m + M_c)\Big]$$

| Công thức | Vị trí |
|---|---|
| $\tau_r$, $\tau_l$ | `Chassis::solveInverseDynamics` |
| $\dot V_m$, $\dot\omega_m$ từ profile | `Chassis::computeInverseDynamics` |
| Trạng thái duy trì ($\dot V = \dot\omega = 0$) | `Chassis::previewInverseDynamics` |
| Ô web | `Chassis::setDynamicsVW` |

## Mô hình động cơ (ngoài các phương trình trên)

Quy đổi mô-men thành PWM: $U = \dfrac{R_a}{K_t}\tau + K_e\,\omega$, rồi $\text{PWM} = U / V_{\text{eff}} \cdot 255$ với $V_{\text{eff}}$ là điện áp pin trừ sụt áp L298N. Cài đặt trong `Chassis::torqueToPWM` và `pwmToTorque`.

## Hằng số

Các hằng số $m$, $I$, $F_{ms}$, $b_v$, $b_\omega$, $K_t$, $K_e$, $R_a$ nằm trong `include/Config.h`. Đây là giá trị ước lượng, cần đối chiếu với xe thật khi tune.
