# Описание 
Тестовый проект по Point Cloud colorization/rasterization

# Запуск

Для запуска нужны топики /lidar_top/points, /CF , /tf_static.
`
ros2 bag play --clock --topics /lidar_top/points /CF /tf_static -p .../*.db3
`

`
ros2 run lidar_rgb_fusion CloudFusion
`

В rviz2 подписаться на топик CloudFusion_pub.

# Пример 
![me](./data/example.gif)
