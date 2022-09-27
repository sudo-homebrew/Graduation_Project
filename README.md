# **Konkuk University 2022 Graduation Project**

## **Project name : Self-Driving Mobile Robot throught DRL**

## **Members**

[남승협](https://github.com/sudo-homebrew) | [강신규](https://github.com/zox004) | [우창석](https://github.com/MOLOZISE)

## Introduction

심층 강화 학습을 통해 자율주행 모바일 로봇을 개발하고자 한다. 로봇 청소기 등 모바일 로봇이 향후 실버 세대를 위해 그 중요성이 확대될 것으로 기대한다. 현재 모바일 로봇은 **SLAM(Simultaneous Localisation and Mapping)**, 동시적 위치 추정 및 지도 작성 기법을 사용하여 현재 위치를 추정하면서 동시에 로봇 주변 환경을 지도를 작성한다. 기본적으로 글로벌 맵과 로컬  맵을 활용하기 때문에 동적 환경에서는 정확한 운영이 어렵다. 따라서 이러한 어려움을 극복하기 위해 현재 강화 학습, 정확히 말하면 **심층 강화 학습**이 제안되고 있다. 그래서 우리의 목표는 최적의 **Global Path Planning Algorithm**, **Local Path Planning Algorithm**(심층 강화 학습)과 SLAM 및 **ROS2**를 사용하여 Target Point가 주어지면 최단 경로로 Obstacle Avoidance를 수행하며 자율주행을 하는 모바일 로봇을 개발하는 것이다.

## Main Feature

- SLAM으로 Robot의 동시적 위치 추정과 지도 작성, Global Path Planning으로 Target Point까지 최단 경로 탐색, Local Path Planning으로 (Dynamic) Obstacle Avoidance를 수행한다.
- ROS2와 Gazebo를 이용하여 가상 시뮬레이션을 통한 강화학습을 진행 후 훈련된 모델을 Turtlebot3로 실제 환경에서의 성능을 평가한다.
- SLAM은 MATLAB으로, Path Planning Algorithm과 기타 setup 파일은 Python과 C++로 작성해 ROS2 상에서 DDS 통신을 통하여 topic을 주고 받는다.

## Algorithm

### Ant Colony Optimization(Global Path Planning)

<img src="https://user-images.githubusercontent.com/56228085/192175335-3a489d8e-986f-4b3a-b429-701dde231292.png" width=60% height=60%/>

계산문제를 푸는 확률적 해법 중 하나로 그래프에서 최적의 경로를 찾는 데 쓰인다. heuristic search로 접근하여 최적의 경로가 아니더라도 빠른 시간 내에 적절한 경로를 찾는 방법을 사용한다. 여러 비교 논문들을 분석 결과 global path planning algorithm 중에서 가장 좋은 성능을 보여 ACO algorithm을 사용하고자 한다.

### TD3(Local Path Planning)

TD3(Twin Delayed DDPG)의 경우, DDPG가 쓰이는 여러 도메인에서 DDPG에 비해 안정적인 수렴 성능을 보여준다고 알려져 있다. 이 알고리즘의 경우, DDPG에서 발생했던 여러 문제점 중 하나인 Q value의 overestimate 문제를 완화시킬 수 있는 여러 트릭을 사용함으로써 DDPG의 성능을 개선한 알고리즘으로 알려져 있다. DWA, TEB와 같은 traditional path planning algorithm보다 좋은 성능을 보여준 결과가 있어 TD3를 사용하고자 한다.

### SLAM

SLAM(Simultaneous Localisation and Mapping)은 미지의 환경에서 Map을 설계하고, Localization을 이용하여 현재의 위치를 파악하는 방법이다.

- SLAM 기술은 실시간으로 자신의 위치 추정과 (Localisation) 주변 지도 작성을 (Mapping) 할 수 있다.
- SLAM 프로그램이 시작한 시점에서부터 지속적으로 자신의 위치 추정을 할 수 있다.
- 최신 SLAM 기술들에 적용되는 Loop Closure라는 기능을 통해, 나의 현재 위치가 이전에 와본 적 있는 장소라는 것을 인지하고 자신의 위치에 대한 오차 수정을 할 수 있다.
- SLAM 프로그램이 종료되는 시점에서는 잘 그려진 지도와 나의 지난 위치들을 얻어낼 수 있다.

![Untitled 2](https://user-images.githubusercontent.com/56228085/192175349-db5e848c-a04f-442c-9fa2-835e8a48aae4.png)

## Architecture

<img src="https://user-images.githubusercontent.com/56228085/192175327-3c98092f-98de-4c39-a870-84c1a6d3ef84.png" width=85% height=85%/>

## System Build

- GPU Server Machine으로 Virtual environment상에서 Implementation 및 DRL Training한 후 실제 physical robot(Turtlebot3 Waffle Pi)에 Deployment할 예정이다.
- Python Code로 DRL algorithm 을 실행하고 병렬적으로 MATLAB code로 SLAM algorithm을 실행해 실시간으로 local DDS communication을 통해 Occupancy Grid Map 정보 및 Location 정보를 DRL Training module에 전달하는 방식으로 시스템을 구축할 계획이다.
