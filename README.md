# Canny Edge Detection - ZedBoard FPGA

ZedBoard (Zynq-7020) FPGA 기반 Canny Edge Detection 하드웨어 가속기 프로젝트.
32x32x1 grayscale 이미지 100개를 대상으로 Canny Edge Detection 파이프라인을 RTL로 구현한다.

## Directory Structure

```
Canny-Edge-Detection/
├── dataset/                        # Python 데이터셋 생성 스크립트 및 생성된 이미지
│   ├── scripts/                    # 이미지 생성 Python 스크립트
│   └── images/                     # 생성된 32x32x1 이미지 저장
│       ├── input/                  # 원본 grayscale 입력 이미지
│       └── output/                 # Canny edge detection 결과 이미지
├── rtl/                            # Verilog RTL 소스
│   ├── canny_top.v                 # Top-level wrapper
│   ├── axil_ctrl.sv                # AXI4-Lite control/status registers (PS interface)
│   ├── ddr_interface.sv            # AXI master + buffer data-movement controller
│   ├── input_buffer.sv             # DDR-read FIFO + unpacker -> 8-bit pixel stream
│   ├── line_buffer.v               # Line buffer module
│   ├── gaussian_filter.v           # Gaussian filter module
│   ├── gradient_sobel.v            # Gradient computation (Sobel) module
│   ├── nms.v                       # Non-Maximum Suppression module
│   ├── hysteresis_threshold.v      # Hysteresis Thresholding module
│   └── output_buffer.sv            # Packer + DDR-write FIFO (hysteresis -> AXI)
├── bd/                             # Vivado Block Design (TCL export)
├── ip/                             # Custom IP 패키징 소스
├── tb/                             # Testbench files
├── sim/                            # Simulation scripts & waveforms
├── constraints/                    # XDC constraint files (ZedBoard)
├── sw/                             # Vitis Workspace (C/SDK) for Zynq PS
├── vivado_project/                 # Vivado 프로젝트 (gitignore 대상)
├── docs/                           # Documentation
├── .gitignore
└── README.md
```

## Module Pipeline

Canny Edge Detection 파이프라인은 다음 순서로 처리된다:

```
DDR Memory
    │
    ▼
┌─────────────────┐
│  ddr_interface   │  DDR controller로부터 이미지 데이터 읽기/쓰기
└────────┬────────┘
         ▼
┌─────────────────┐
│  input_buffer    │  입력 픽셀 데이터 버퍼링
└────────┬────────┘
         ▼
┌─────────────────┐
│  line_buffer     │  5x5 윈도우 연산을 위한 라인 버퍼
└────────┬────────┘
         ▼
┌─────────────────┐
│ gaussian_filter  │  5x5 Gaussian smoothing (노이즈 제거)
└────────┬────────┘
         ▼
┌─────────────────┐
│ gradient_sobel   │  3x3 Sobel operator (Gx, Gy → magnitude & direction)
└────────┬────────┘
         ▼
┌─────────────────┐
│      nms         │  Non-Maximum Suppression (에지 얇게 만들기)
└────────┬────────┘
         ▼
┌─────────────────┐
│ hysteresis_      │  Double threshold + edge tracking by hysteresis
│   threshold      │
└────────┬────────┘
         ▼
┌─────────────────┐
│ output_buffer    │  출력 픽셀 데이터 버퍼링
└────────┬────────┘
         ▼
    DDR Memory
```

## Getting Started

```bash
git clone https://github.com/kwhyun/Canny-Edge-Detection.git
cd Canny-Edge-Detection
```

## Target Platform

- **Board**: Avnet ZedBoard (Zynq-7020 SoC)
- **Image**: 32x32x1 grayscale, 100 images
- **Tools**: Vivado 2024.x, Vitis 2024.x
