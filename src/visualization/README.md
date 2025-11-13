# Visualization Package

ROS2 패키지로 CSV 파일의 x, y 좌표를 읽어서 주기적으로 RViz에서 시각화할 수 있도록 토픽으로 발행합니다.

## 기능

- CSV 파일에서 x, y 좌표 데이터를 읽기
- 설정 파일을 통해 여러 CSV 파일 동시 처리
- MarkerArray를 사용하여 점과 선으로 시각화
- 각 데이터셋마다 색상 설정 가능
- 주기적으로 토픽 발행

## 설치

```bash
cd /home/ojg/ae
colcon build --packages-select visualization
source install/setup.bash
```

## 설정 파일

설정 파일은 [config/visualization.yaml](config/visualization.yaml)에 있습니다.

### 설정 예시

```yaml
csv_publisher:
  ros__parameters:
    # CSV 파일 설정
    csv_files:
      - name: "path"
        file_path: "/home/ojg/ae/csv/path/path.csv"
        topic_name: "/visualization/path"

      - name: "boundary"
        file_path: "/home/ojg/ae/csv/bound/boundary.csv"
        topic_name: "/visualization/boundary"

    # 발행 주기 (Hz)
    publish_rate: 1.0

    # 프레임 ID
    frame_id: "map"

    # 마커 크기
    marker_scale: 0.1

    # 색상 설정 (R, G, B, A)
    colors:
      path: [1.0, 0.0, 0.0, 1.0]      # 빨간색
      boundary: [0.0, 1.0, 0.0, 1.0]  # 초록색
```

### 파라미터 설명

- `csv_files`: CSV 파일 목록
  - `name`: 데이터셋 이름
  - `file_path`: CSV 파일 경로 (절대 경로)
  - `topic_name`: 발행할 토픽 이름
- `publish_rate`: 토픽 발행 주기 (Hz)
- `frame_id`: TF 프레임 ID
- `marker_scale`: 마커 크기
- `marker_alpha`: 마커 투명도
- `colors`: 각 데이터셋의 색상 (RGBA)

## CSV 파일 형식

CSV 파일은 다음과 같은 형식이어야 합니다:

```csv
x,y
0.0,0.0
1.0,0.5
2.0,1.0
```

- 첫 번째 열: x 좌표
- 두 번째 열: y 좌표
- 헤더 행은 선택 사항

## 사용 방법

### Launch 파일로 실행

```bash
ros2 launch visualization csv_visualization.launch.py
```

### 직접 실행

```bash
ros2 run visualization csv_publisher --ros-args --params-file src/visualization/config/visualization.yaml
```

## 발행되는 토픽

설정에 따라 다음과 같은 토픽이 발행됩니다:

- `/visualization/path` (visualization_msgs/msg/MarkerArray)
- `/visualization/boundary` (visualization_msgs/msg/MarkerArray)
- `/visualization/check_line` (visualization_msgs/msg/MarkerArray)

각 MarkerArray는 다음을 포함합니다:
- LINE_STRIP 마커: 점들을 선으로 연결
- SPHERE_LIST 마커: 각 점을 구로 표시

## RViz에서 확인

1. RViz 실행:
```bash
rviz2
```

2. Fixed Frame을 `map`으로 설정

3. MarkerArray Display 추가:
   - Add 버튼 클릭
   - By topic 탭에서 원하는 토픽 선택
   - 또는 By display type에서 MarkerArray 선택 후 토픽 설정

## 디렉토리 구조

```
visualization/
├── config/
│   └── visualization.yaml
├── launch/
│   └── csv_visualization.launch.py
├── resource/
│   └── visualization
├── visualization/
│   ├── __init__.py
│   └── csv_publisher.py
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

## 새로운 CSV 파일 추가

1. CSV 파일을 적절한 위치에 저장
2. `config/visualization.yaml` 파일에 새로운 항목 추가:

```yaml
- name: "my_data"
  file_path: "/path/to/my_data.csv"
  topic_name: "/visualization/my_data"
```

3. (선택사항) 색상 설정 추가:

```yaml
colors:
  my_data: [1.0, 1.0, 0.0, 1.0]  # 노란색
```

4. 노드 재시작

## 문제 해결

### CSV 파일을 찾을 수 없는 경우

- 파일 경로가 절대 경로인지 확인
- 파일 권한 확인

### 마커가 RViz에 표시되지 않는 경우

- Fixed Frame이 설정 파일의 `frame_id`와 일치하는지 확인
- 토픽이 정상적으로 발행되는지 확인: `ros2 topic list`
- 토픽 데이터 확인: `ros2 topic echo /visualization/path`

## 라이센스

MIT License
