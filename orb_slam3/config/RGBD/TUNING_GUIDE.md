# ORB-SLAM3 파라미터 튜닝 가이드

RealSense D405 RGB-D 카메라용 ORB-SLAM3 파라미터 설정 가이드입니다.

---

## 1. RGB-D / Depth 파라미터

### `Stereo.b` - 스테레오 베이스라인
```yaml
Stereo.b: 0.018  # 단위: 미터
```
- **의미**: 카메라의 스테레오 베이스라인 (IR 프로젝터와 센서 간 거리)
- **D405**: 약 18mm (0.018m)
- **수정 시**: `Camera.bf`도 함께 수정 필요 (`bf = b × fx`)

---

### `Stereo.ThDepth` - Close/Far 포인트 임계값
```yaml
Stereo.ThDepth: 50.0  # baseline 배수
```
- **의미**: `ThDepth × baseline` 거리 이내의 포인트를 "close point"로 분류
- **계산 예시**: 50.0 × 0.018m = **0.9m**
- **close point**: 단일 프레임에서 바로 맵포인트 생성
- **far point**: 두 프레임 이상에서 삼각측량 필요

| 환경 | 권장값 | 결과 거리 |
|------|--------|-----------|
| 근거리 작업 (D405) | 30~40 | 0.54~0.72m |
| 일반 실내 | 40~50 | 0.72~0.9m |
| 넓은 공간 | 50~60 | 0.9~1.08m |

**튜닝 팁**:
- 값을 **높이면**: 더 많은 포인트가 close로 처리 → 빠른 맵 생성, 하지만 노이즈 증가
- 값을 **낮추면**: 더 정확한 맵, 하지만 초기화 느림

---

### `RGBD.DepthMapFactor` - Depth 스케일 팩터
```yaml
RGBD.DepthMapFactor: 1000.0
```
- **의미**: depth 이미지 값을 미터로 변환하는 팩터
- **RealSense**: 밀리미터 단위 → 1000.0
- **다른 센서**: 센티미터면 100.0, 미터면 1.0

---

### `RGBD.MinDepth` - 최소 depth 필터 (커스텀)
```yaml
RGBD.MinDepth: 0.25  # 단위: 미터
```
- **의미**: 이 거리보다 가까운 특징점 무시
- **용도**: 로봇 그리퍼, 손 등 제외
- **비활성화**: 0.0으로 설정

| 그리퍼 거리 | 권장값 |
|-------------|--------|
| 10-15cm | 0.15~0.20 |
| 15-20cm | 0.20~0.25 |
| 20-30cm | 0.25~0.35 |

---

## 2. ORB 특징점 파라미터

### `ORBextractor.nFeatures` - 특징점 개수
```yaml
ORBextractor.nFeatures: 2000
```
- **의미**: 프레임당 추출할 최대 특징점 수

| 값 | 장점 | 단점 |
|----|------|------|
| 500~800 | 빠른 처리, 낮은 CPU | 트래킹 불안정 |
| 1000~1500 | 균형 | - |
| **2000~3000** | **안정적 트래킹** | 높은 CPU |

**튜닝 팁**:
- 텍스처 없는 환경 → **증가** (2000+)
- 빠른 움직임 → **증가**
- 실시간 성능 중요 → **감소** (1000)

---

### `ORBextractor.scaleFactor` - 스케일 팩터
```yaml
ORBextractor.scaleFactor: 1.2
```
- **의미**: 이미지 피라미드 레벨 간 스케일 비율
- **권장 범위**: 1.1 ~ 1.4

| 값 | 효과 |
|----|------|
| 1.1 | 세밀한 스케일 탐지, 느림 |
| **1.2** | **표준 (권장)** |
| 1.4 | 빠름, 스케일 변화에 약함 |

---

### `ORBextractor.nLevels` - 피라미드 레벨 수
```yaml
ORBextractor.nLevels: 10
```
- **의미**: 이미지 피라미드의 레벨 수
- **효과**: 다양한 거리에서 특징점 검출

| 값 | 효과 |
|----|------|
| 4~6 | 빠름, 근거리 위주 |
| **8** | **표준** |
| 10~12 | 다양한 거리 커버, 느림 |

---

### `ORBextractor.iniThFAST` / `minThFAST` - FAST 임계값
```yaml
ORBextractor.iniThFAST: 15  # 초기 임계값
ORBextractor.minThFAST: 5   # 최소 임계값
```
- **의미**: FAST 코너 검출기의 강도 임계값
- **iniThFAST**: 먼저 시도하는 임계값
- **minThFAST**: 특징점 부족 시 사용하는 낮은 임계값

| 환경 | iniThFAST | minThFAST |
|------|-----------|-----------|
| 고대비 (실외) | 20~25 | 7~10 |
| **일반 실내** | **15~20** | **5~7** |
| 저대비/어두움 | 10~15 | 3~5 |
| 텍스처 부족 | 8~12 | 3~5 |

**튜닝 팁**:
- 특징점이 너무 적으면 → 둘 다 **낮춤**
- 노이즈가 많으면 → 둘 다 **높임**

---

## 3. 환경별 권장 설정

### 로봇 그리퍼 + 근거리 작업 (현재 설정)
```yaml
Stereo.ThDepth: 50.0
RGBD.MinDepth: 0.25
ORBextractor.nFeatures: 2000
ORBextractor.nLevels: 10
ORBextractor.iniThFAST: 15
ORBextractor.minThFAST: 5
```

### 빠른 움직임 대응
```yaml
ORBextractor.nFeatures: 2500   # 증가
ORBextractor.iniThFAST: 12     # 낮춤
ORBextractor.minThFAST: 4      # 낮춤
```

### 저사양 시스템 (실시간 우선)
```yaml
ORBextractor.nFeatures: 800    # 감소
ORBextractor.nLevels: 6        # 감소
ORBextractor.iniThFAST: 20     # 높임
```

### 텍스처 없는 환경
```yaml
ORBextractor.nFeatures: 3000   # 크게 증가
ORBextractor.iniThFAST: 10     # 낮춤
ORBextractor.minThFAST: 3      # 낮춤
```

---

## 4. Viewer 파라미터

시각화 관련 파라미터로 SLAM 성능에는 영향 없음.

```yaml
Viewer.KeyFrameSize: 0.05      # 키프레임 표시 크기
Viewer.KeyFrameLineWidth: 1.0  # 키프레임 선 두께
Viewer.GraphLineWidth: 0.9     # 그래프 선 두께
Viewer.PointSize: 2.0          # 맵포인트 크기
Viewer.CameraSize: 0.08        # 카메라 표시 크기
Viewer.CameraLineWidth: 3.0    # 카메라 선 두께
Viewer.ViewpointX: 0.0         # 초기 시점 X
Viewer.ViewpointY: -0.7        # 초기 시점 Y
Viewer.ViewpointZ: -1.8        # 초기 시점 Z
Viewer.ViewpointF: 500.0       # 초기 시점 초점거리
```

---

## 5. 문제 해결 가이드

| 증상 | 원인 | 해결책 |
|------|------|--------|
| "Track lost" 자주 발생 | 특징점 부족 | `nFeatures` 증가, FAST 임계값 낮춤 |
| 초기화 안됨 | close point 부족 | `ThDepth` 증가 |
| 맵 드리프트 심함 | 노이즈 많은 특징점 | FAST 임계값 높임, `nFeatures` 감소 |
| CPU 사용률 높음 | 과도한 특징점 | `nFeatures` 감소, `nLevels` 감소 |
| 그리퍼가 맵에 포함됨 | MinDepth 부족 | `RGBD.MinDepth` 증가 |
| 가까운 물체 인식 안됨 | MinDepth 과다 | `RGBD.MinDepth` 감소 |

---

## 6. 카메라 파라미터 업데이트 방법

해상도 변경 시 `camera_info` 토픽에서 파라미터 확인:

```bash
ros2 topic echo --once /camera/camera/color/camera_info
```

출력된 `k` 매트릭스에서:
- `k[0]` → `Camera1.fx`
- `k[4]` → `Camera1.fy`
- `k[2]` → `Camera1.cx`
- `k[5]` → `Camera1.cy`

`Camera.bf` 재계산:
```
Camera.bf = Stereo.b × Camera1.fx
```
