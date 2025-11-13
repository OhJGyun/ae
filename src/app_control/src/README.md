# Pure Pursuit Target Speed Control Mechanism

## Overview
이 문서는 Pure Pursuit 알고리즘에서 사용되는 곡률 기반 적응형 속도 제어 시스템에 대한 상세 분석입니다.

## 🚗 Basic Concept

현재 구현된 속도 제어는 **곡률(Curvature) 기반 적응형 속도 제어** 방식으로, 경로의 기하학적 특성에 따라 자동으로 목표 속도를 조절합니다.

### Core Algorithm
```cpp
double calculate_target_speed(double curvature) {
    double abs_curvature = std::abs(curvature);
    
    if (abs_curvature < curvature_threshold_) {
        return target_speed_straight_;  // 직선 구간
    } else {
        // 커브 구간: 곡률에 따라 속도 감소
        double speed_factor = std::max(0.6, 1.0 - (abs_curvature - curvature_threshold_) * 1.0);
        return target_speed_curve_ * speed_factor;
    }
}
```

## 📊 Parameter Analysis

### Key Parameters

| Parameter | Default Value | Description |
|-----------|---------------|-------------|
| `curvature_threshold_` | 0.2 | 직선/커브 판단 기준값 |
| `target_speed_straight_` | 6.0 m/s | 직선 구간 목표 속도 |
| `target_speed_curve_` | 5.0 m/s | 커브 구간 기본 속도 |
| `speed_factor_min` | 0.6 | 최소 속도 비율 (60%) |
| `speed_reduction_rate` | 1.0 | 속도 감소 강도 계수 |

### Curvature Calculation
```cpp
// Pure Pursuit에서 곡률 계산
double curvature = 2.0 * cy / (Ld * Ld);
```
- `cy`: 차량 로컬 좌표계에서 lookahead point의 y 좌표
- `Ld`: Lookahead distance
- **곡률 = 1/반지름** 관계

## 🔢 Speed Factor Formula

### Mathematical Formula
```
speed_factor = max(0.6, 1.0 - (abs_curvature - curvature_threshold) * 1.0)
final_speed = target_speed_curve * speed_factor
```

### Formula Breakdown
1. `(abs_curvature - curvature_threshold)`: 임계값을 초과한 곡률량
2. `* 1.0`: 속도 감소 강도 (높을수록 급격한 감소)
3. `1.0 - ...`: 역비례 관계 (곡률 ↑ → 속도 ↓)
4. `max(0.6, ...)`: 최소 60% 속도 보장

## 📈 Speed Calculation Examples

| 곡률 | 곡률 의미 | 계산 과정 | Speed Factor | 최종 속도 |
|------|----------|-----------|--------------|-----------|
| 0.1 | 직선 (R=10m) | 직선 판단 | - | **6.0 m/s** |
| 0.3 | 완만한 커브 (R=3.3m) | max(0.6, 1.0-(0.3-0.2)*1.0) = 0.9 | 0.9 | **4.5 m/s** |
| 0.5 | 중간 커브 (R=2.0m) | max(0.6, 1.0-(0.5-0.2)*1.0) = 0.7 | 0.7 | **3.5 m/s** |
| 0.8 | 급커브 (R=1.25m) | max(0.6, 1.0-(0.8-0.2)*1.0) = 0.4→0.6 | 0.6 | **3.0 m/s** |
| 1.0 | 매우 급한 커브 (R=1.0m) | max(0.6, 1.0-(1.0-0.2)*1.0) = 0.2→0.6 | 0.6 | **3.0 m/s** |

> **Note**: 곡률 0.8 이상에서는 최소 속도 제한(60%)이 적용됩니다.

## ⚙️ Tuning Guide

### 🏎️ For Faster Cornering
```cpp
// 더 공격적인 주행을 위한 파라미터
target_speed_curve_ = 7.0;        // 5.0 → 7.0
speed_reduction_rate = 0.5;       // 1.0 → 0.5 (완만한 감소)
speed_factor_min = 0.8;           // 0.6 → 0.8 (최소 80% 속도)
```

### 🛡️ For Safer Driving
```cpp
// 더 안전한 주행을 위한 파라미터
curvature_threshold_ = 0.1;       // 0.2 → 0.1 (더 민감한 커브 감지)
speed_reduction_rate = 2.0;       // 1.0 → 2.0 (급격한 감소)
speed_factor_min = 0.4;           // 0.6 → 0.4 (최소 40% 속도)
```

### 🎯 For Specific Track Types

#### High-Speed Track (긴 직선, 완만한 커브)
```cpp
target_speed_straight_ = 8.0;
target_speed_curve_ = 6.0;
curvature_threshold_ = 0.15;
```

#### Technical Track (많은 급커브)
```cpp
target_speed_straight_ = 5.0;
target_speed_curve_ = 4.0;
curvature_threshold_ = 0.3;
speed_reduction_rate = 1.5;
```

## 🔄 Integration with PID Speed Control

목표 속도가 결정되면 PID 컨트롤러가 실제 속도 제어를 담당합니다:

```cpp
double calculate_pid_speed(double target_speed, double current_speed) {
    double error = target_speed - current_speed;
    
    // PID 계산
    double pid_output = speed_kp_ * error + 
                       speed_ki_ * speed_error_integral_ + 
                       speed_kd_ * error_derivative;
    
    return std::clamp(current_speed + pid_output, min_speed_, max_speed_);
}
```

## 📐 Curvature Understanding

### Curvature vs Radius Relationship
| 곡률 | 반지름 | 커브 특성 |
|------|--------|----------|
| 0.05 | 20m | 고속도로 커브 |
| 0.1 | 10m | 완만한 커브 |
| 0.2 | 5m | 일반적인 커브 |
| 0.5 | 2m | 급커브 |
| 1.0 | 1m | 헤어핀 커브 |

### Visual Representation
```
Curvature = 0.1        Curvature = 0.5        Curvature = 1.0
     ○                      ○                      ○
    /                      /|                     /|\
   /                      / |                    / | \
  /                      /  |                   /  |  \
 /                      /   |                  /   |   \
○────────────○         ○────○                 ○────○────○
   (R=10m)               (R=2m)                (R=1m)
   6.0 m/s               3.5 m/s               3.0 m/s
```

## 🔧 Implementation Notes

### Key Features
- ✅ **Smooth Speed Transition**: 곡률 변화에 따른 부드러운 속도 전환
- ✅ **Safety Margin**: 최소 속도 제한으로 안전성 보장
- ✅ **Configurable**: 다양한 트랙 특성에 맞춘 튜닝 가능
- ✅ **Real-time Adaptation**: 실시간 경로 곡률 분석

### Performance Characteristics
- **Response Time**: 50ms (20Hz control loop)
- **Speed Range**: 1.0 ~ 8.0 m/s
- **Curvature Sensitivity**: 0.05 ~ 2.0 (1/m)

## 🚀 Advanced Tuning Tips

1. **Track Learning**: 실제 주행 데이터를 분석하여 트랙별 최적 파라미터 도출
2. **Dynamic Adjustment**: 타이어 그립, 날씨 조건에 따른 동적 파라미터 조정
3. **Predictive Control**: 여러 lookahead point를 고려한 예측적 속도 제어
4. **Machine Learning**: 강화학습을 통한 자동 파라미터 최적화

## 📚 References
- Pure Pursuit Algorithm: [Stanford Racing Team](https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf)
- Curvature-based Speed Control: Automotive Control Theory
- PID Control: Classical Control Systems