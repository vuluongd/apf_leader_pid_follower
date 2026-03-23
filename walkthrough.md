# 📋 Review Project: APF Leader – PID Follower (TurtleSim / ROS 2)

## 1. Tổng quan

Project mô phỏng hệ thống **Leader-Follower** trên **TurtleSim (ROS 2)**:

| Thành phần | File | Mô tả |
|---|---|---|
| **Leader** | [turtle_apf_leader.py](file:///home/lunog/apf_leader_pid_follower/src/my_package/my_package/turtle_apf_leader.py) | Di chuyển đến mục tiêu ngẫu nhiên bằng **Artificial Potential Field (APF)** – lực hút + lực đẩy |
| **Follower** | [turtle_pid_follower.py](file:///home/lunog/apf_leader_pid_follower/src/my_package/my_package/turtle_pid_follower.py) | Bám theo Leader bằng **PID** kết hợp lực đẩy tránh vật cản |
| Config | [pid_and_apf.yaml](file:///home/lunog/apf_leader_pid_follower/src/my_package/config/pid_and_apf.yaml) | **Trống** – chưa được viết |
| Launch | [apf_leader_pid_follower.launch.py](file:///home/lunog/apf_leader_pid_follower/src/my_package/launch/apf_leader_pid_follower.launch.py) | **Trống** – chưa được viết |

> [!CAUTION]
> Project hiện tại **không thể chạy được** vì có nhiều lỗi cú pháp (syntax error), lỗi logic, thiếu file launch/config, và thiếu entry_points trong [setup.py](file:///home/lunog/apf_leader_pid_follower/src/my_package/setup.py).

---

## 2. Các lỗi nghiêm trọng (Bugs)

### 2.1 Lỗi cú pháp (Syntax Error) – Sẽ crash ngay khi import

| # | File | Dòng | Lỗi | Sửa |
|---|---|---|---|---|
| 1 | [turtle_apf_leader.py](file:///home/lunog/apf_leader_pid_follower/src/my_package/my_package/turtle_apf_leader.py) | 23 | `self.pose = Pose \| None = None` – cú pháp sai | `self.pose: Pose \| None = None` |
| 2 | [turtle_pid_follower.py](file:///home/lunog/apf_leader_pid_follower/src/my_package/my_package/turtle_pid_follower.py) | 4 | `from geometry_msgs import Twist` – thiếu `.msg` | `from geometry_msgs.msg import Twist` |
| 3 | [turtle_pid_follower.py](file:///home/lunog/apf_leader_pid_follower/src/my_package/my_package/turtle_pid_follower.py) | 118 | `deriv_angle += (...)` – dùng `+=` nhưng biến chưa khai báo | `deriv_angle = (...)` |

### 2.2 Lỗi sai tên hàm / tham số (Runtime Error)

| # | File | Dòng | Lỗi | Sửa |
|---|---|---|---|---|
| 4 | [turtle_apf_leader.py](file:///home/lunog/apf_leader_pid_follower/src/my_package/my_package/turtle_apf_leader.py) | 56 | `self.get_parameters('ka')` – sai tên method (thêm `s`) | `self.get_parameter('ka')` |
| 5 | [turtle_apf_leader.py](file:///home/lunog/apf_leader_pid_follower/src/my_package/my_package/turtle_apf_leader.py) | 60–61 | Tương tự: `get_parameters` → `get_parameter` cho `'kr'` và `'rho_0'` | `self.get_parameter(...)` |
| 6 | [turtle_pid_follower.py](file:///home/lunog/apf_leader_pid_follower/src/my_package/my_package/turtle_pid_follower.py) | 42 | `'/turtle2.pose'` – dùng dấu chấm thay vì `/` | `'/turtle2/pose'` |
| 7 | [turtle_pid_follower.py](file:///home/lunog/apf_leader_pid_follower/src/my_package/my_package/turtle_pid_follower.py) | 58–59 | `prev_time` thiếu `self.` → `NameError` | `self.prev_time` |
| 8 | [turtle_pid_follower.py](file:///home/lunog/apf_leader_pid_follower/src/my_package/my_package/turtle_pid_follower.py) | 68 | `self.get_parameter('rho_follower')` – tham số không tồn tại | `self.get_parameter('rho_0')` |
| 9 | [turtle_pid_follower.py](file:///home/lunog/apf_leader_pid_follower/src/my_package/my_package/turtle_pid_follower.py) | 94 | `'angle_threhold'` – sai chính tả | `'angle_threshold'` |
| 10 | [turtle_pid_follower.py](file:///home/lunog/apf_leader_pid_follower/src/my_package/my_package/turtle_pid_follower.py) | 96 | `self.leafer_pose.x` – typo | `self.leader_pose.x` |

### 2.3 Lỗi logic

| # | File | Dòng | Mô tả |
|---|---|---|---|
| 11 | [turtle_apf_leader.py](file:///home/lunog/apf_leader_pid_follower/src/my_package/my_package/turtle_apf_leader.py) | 88 | Điều kiện đổi goal dùng `>` thay vì `<` — robot sẽ đổi goal khi **chưa** đến nơi thay vì khi **đã** đến nơi |
| 12 | [turtle_pid_follower.py](file:///home/lunog/apf_leader_pid_follower/src/my_package/my_package/turtle_pid_follower.py) | 105 | `math.atan2(fx, fy)` – tham số ngược, chuẩn là `math.atan2(fy, fx)` |
| 13 | [turtle_pid_follower.py](file:///home/lunog/apf_leader_pid_follower/src/my_package/my_package/turtle_pid_follower.py) | 112 | PID linear dùng `self.integral_angle` thay vì `self.integral_distance` – cross-contamination giữa 2 PID |
| 14 | [turtle_pid_follower.py](file:///home/lunog/apf_leader_pid_follower/src/my_package/my_package/turtle_pid_follower.py) | 127 | Khi `angle_error > threshold`, `cmd.angular.z = angle_threshold` (hằng số) thay vì dùng `angle_out` (PID output) |

---

## 3. Thiếu sót cấu hình (Configuration)

### 3.1 [setup.py](file:///home/lunog/apf_leader_pid_follower/src/my_package/setup.py) – Thiếu entry_points

```python
# Hiện tại:
entry_points={
    'console_scripts': [
    ],
},
```

Cần thêm:
```python
entry_points={
    'console_scripts': [
        'turtle_apf_leader = my_package.turtle_apf_leader:main',
        'turtle_pid_follower = my_package.turtle_pid_follower:main',
    ],
},
```

Và cần thêm `data_files` cho launch + config:
```python
data_files=[
    ...,
    (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
],
```

### 3.2 Launch file trống

File [apf_leader_pid_follower.launch.py](file:///home/lunog/apf_leader_pid_follower/src/my_package/launch/apf_leader_pid_follower.launch.py) chưa có nội dung. Cần tạo launch file để:
- Khởi động `turtlesim_node`
- Spawn các turtle (obstacle, follower)
- Chạy node `turtle_apf_leader` và `turtle_pid_follower`
- Load tham số từ file YAML

### 3.3 Config YAML trống

File [pid_and_apf.yaml](file:///home/lunog/apf_leader_pid_follower/src/my_package/config/pid_and_apf.yaml) chưa có nội dung. Tất cả tham số chỉ dùng giá trị mặc định khai báo trong code.

---

## 4. Vấn đề về code style & cấu trúc

| Vấn đề | Chi tiết |
|---|---|
| **Tên package** | `my_package` – quá chung chung, nên đổi thành tên có ý nghĩa (vd: `apf_leader_pid_follower`) |
| **Hàm trùng lặp** | [normalize_angle()](file:///home/lunog/apf_leader_pid_follower/src/my_package/my_package/turtle_apf_leader.py#7-9) được copy-paste ở cả 2 file → nên tách thành module riêng `utils.py` |
| **Thiếu docstring** | Không có docstring cho class và method nào |
| **Thiếu license** | [package.xml](file:///home/lunog/apf_leader_pid_follower/src/my_package/package.xml) và [setup.py](file:///home/lunog/apf_leader_pid_follower/src/my_package/setup.py) đều ghi `TODO: License declaration` |
| **Thiếu mô tả** | [setup.py](file:///home/lunog/apf_leader_pid_follower/src/my_package/setup.py) ghi `TODO: Package description` |
| **Topic naming** | `'turtle1/cmd_vel/'` (dòng 44, leader) có dấu `/` thừa ở cuối |
| **Anti-pattern** | Dùng `__setitem__` trực tiếp trong lambda callback thay vì viết method riêng |

---

## 5. Đánh giá tổng thể

| Tiêu chí | Đánh giá | Ghi chú |
|---|---|---|
| **Ý tưởng** | ✅ Tốt | APF cho leader + PID cho follower là approach hợp lý |
| **Kiến trúc ROS 2** | ⚠️ Cơ bản | Cấu trúc package đúng nhưng thiếu launch, config, entry_points |
| **Chất lượng code** | ❌ Chưa đạt | Quá nhiều lỗi cú pháp, typo, và lỗi logic → **không chạy được** |
| **Khả năng chạy** | ❌ Không | Cần sửa ít nhất **14 lỗi** trước khi có thể chạy thử |
| **Testing** | ⚠️ Chưa có | Chỉ có test mặc định (copyright, flake8, pep257) |

### Tóm tắt

> [!IMPORTANT]
> Project thể hiện hiểu biết cơ bản về APF và PID, nhưng **code chưa hoàn thiện và không chạy được**. Cần sửa toàn bộ lỗi cú pháp/logic, hoàn thành launch file + config YAML, và thêm entry_points vào [setup.py](file:///home/lunog/apf_leader_pid_follower/src/my_package/setup.py) trước khi có thể demo.

### Thứ tự ưu tiên sửa:
1. 🔴 Sửa 3 lỗi **syntax error** (mục 2.1)
2. 🔴 Sửa 7 lỗi **runtime error** (mục 2.2)
3. 🟡 Sửa 4 lỗi **logic** (mục 2.3)
4. 🟡 Hoàn thành [setup.py](file:///home/lunog/apf_leader_pid_follower/src/my_package/setup.py) (entry_points + data_files)
5. 🟡 Viết launch file + config YAML
6. 🟢 Cải thiện code style (mục 4)
