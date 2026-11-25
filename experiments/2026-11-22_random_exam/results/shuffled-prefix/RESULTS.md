# 참고 실험 · 책상을 옮긴 네 배치

공식 시험 조건을 확인하기 전에 시작한 비교다. 책상까지 재배치한 네 번째 맵의 마지막 주행을 저장한 시점에 중단했다. 처음 계획했던 다섯 번째 맵과 첫 맵 반복에는 주행 표본이 없다. 공식 시험의 순위를 구할 때에는 원래 책상을 고정한 주행을 사용한다.

| 시드 | Balanced 접근 | Balanced Print 접근 | 종료 시간(초) · Balanced / Print | 범퍼 · Balanced / Print |
|---:|---:|---:|---:|---:|
| 3818821658 | 3/3 | 3/3 | 434.8 / 431.8 | 0 / 0 |
| 2164651793 | 3/3 | 3/3 | 197.6 / 276.5 | 0 / 0 |
| 1506996263 | 3/3 | 3/3 | 168.6 / 190.6 | 0 / 0 |
| 1565135385 | 2/3 | 3/3 | 600.0 / 127.3 | 0 / 0 |

시드 3818821658의 실제 카메라 영상에서 북쪽 벽의 75번이 401.624초에 오른쪽 가장자리의 12픽셀 단서로 판독됐다. Balanced Print가 약 34° 회전한 후 420.8초에 등록하고 431.3초에 근거리 접근을 입증했다. 같은 배치의 Balanced도 이 마커를 확인했다. [노출·회전·접근 기록](print75-camera.json) · [두 원본 프레임](print75-camera.png)

이 시드에서 후보판의 첫 기동은 Nav2 경로 계획기의 준비 상태 조회 실패로 출발 전 종료됐다. 영상·궤적 표본은 각각 0개였고 당시 79개 원본 파일을 보관했다. 동일한 정책과 장면으로 출발한 주행만 위 표에 포함했다. [기동 기록](nav2-before-start.json)

## 각 주행의 원본 카메라 영상

| 맵 | 정책 | 영상 |
|---:|---|---|
| 3818821658 | Balanced | [MP4 다운로드](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/v151-random-shuffled-balanced-print-1122__v151-survey-baseline-track-m3818821658-1021171615408980175.mp4) |
| 3818821658 | Balanced Print | [MP4 다운로드](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/v151-random-shuffled-balanced-print-1122__v151-survey-candidate-track-m3818821658-1021171615408980175.mp4) |
| 2164651793 | Balanced Print | [MP4 다운로드](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/v151-random-shuffled-balanced-print-1122__v151-survey-candidate-track-m2164651793-904460405419145261.mp4) |
| 2164651793 | Balanced | [MP4 다운로드](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/v151-random-shuffled-balanced-print-1122__v151-survey-baseline-track-m2164651793-904460405419145261.mp4) |
| 1506996263 | Balanced | [MP4 다운로드](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/v151-random-shuffled-balanced-print-1122__v151-survey-baseline-track-m1506996263-758615926508713792.mp4) |
| 1506996263 | Balanced Print | [MP4 다운로드](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/v151-random-shuffled-balanced-print-1122__v151-survey-candidate-track-m1506996263-758615926508713792.mp4) |
| 1565135385 | Balanced Print | [MP4 다운로드](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/v151-random-shuffled-balanced-print-1122__v151-survey-candidate-track-m1565135385-716985422643685146.mp4) |
| 1565135385 | Balanced | [MP4 다운로드](https://github.com/peter010905/simul/releases/download/simulation-videos-2026-11/v151-random-shuffled-balanced-print-1122__v151-survey-baseline-track-m1565135385-716985422643685146.mp4) |

[완료한 8개 영상의 지문](media.json) · [원래 계획과 중단 시점](receipt.json)
