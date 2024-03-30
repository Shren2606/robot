1.	Gói chương trình tạo bản đồ hector slam
“sudo apt install ros-noetic-hector-mapping”
2.	Gói chương trình hỗ trợ lưu bản đồ 
“sudo apt install ros-noetic-map-server”
3.	Gói điều hướng robot Navigation
“sudo apt install ros-noetic-navigation”
4.	Gói lập kế hoạch chuyển động cục bộ DWA
“sudo apt install ros-noetic-dwa-local-planner”
5.	Cài đặt trình điều khiển robot bằng bàn phím
“sudo apt install ros-noetic-rqt-robot-steering”
Các bước thực hiện tạo bản đồ và tự động điều hướng
1 	các bước tạo bản đồ
•	Khởi động robot
Trước hết, chúng ta cần khởi động các chương trình cơ bản của robot như khởi động lidả, khởi động động cơ và các mô-đun giao tiếp
“roslaunch robot_bringup robot_bringup.launch”
•	Chạy lệnh SLAM
“roslaunch robot_slam robot_hector.launch”
Một cửa sổ mô phỏng trực quan (rviz) sẽ mở ra và bạn sẽ nhìn thấy phần bản đồ mà robot tạm thời tạo ra khi đang đứng yên.
•	Điều khiển robot để quét bản đồ
Chạy chương trình điều khiển
“rosrun rqt_robot_steering rqt_robot_steering”
Sử dụng các phím như trên cửa sổ dòng lệnh hướng dẫn. Ta cũng có thể điều chỉnh tăng/giảm tốc độ tịnh tiến, tốc độ quay của robot.
•	Lưu bản đồ
“rosrun map_server map_saver -f ~/map”
2 Các bước điều khiển robot tự động điều hướng và tránh vật cản
•	Trước tiên, chúng ta khởi động robot
“roslaunch robot_bringup robot_bringup.launch”
•	Tải bản đồ đã tạo lên công cụ hình ảnh trực quan Rviz
“roslaunch robot_navigation robot_navigation.launch           map_file:=$HOME/map.yaml”
