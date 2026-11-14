# TouchGFX desktop preview

현재 저장소의 데스크톱 프리뷰는 `TouchGFX/simulator/gcc/Makefile`의 `preview` 타깃으로 빌드합니다. 이전 문서에서 안내하던 `tools/build_touchgfx_preview.ps1`은 현재 저장소에 없습니다.

## 요구 환경

- Windows용 TouchGFX Designer 4.26.1 및 호스트 GCC/MinGW 빌드 환경
- `make`, `g++`, `windres`와 필요한 셸 도구가 PATH에 설정된 환경
- Git에 포함된 `TouchGFX/generated` 소스
- 공백 없는 프로젝트 경로: 현재 Makefile은 경로의 공백을 허용하지 않습니다.

현재 `simulator/gcc/Makefile`은 호스트 라이브러리를 `C:\TouchGFX\4.26.1\touchgfx`에서 찾습니다. `config/gcc/app.mk`의 `touchgfx_env`는 `TouchGFX` 작업 폴더 기준 `../../../../TouchGFX/4.26.1/env`로 설정되어 있습니다. 설치 위치나 저장소 위치가 바뀌면 이 두 설정을 실제 경로와 대조해야 합니다. 저장소 내부 Middlewares는 주로 MCU 타깃용이며 호스트 라이브러리와 구분합니다.

## 빌드 및 실행

TouchGFX 호스트 빌드 도구가 준비된 셸에서 저장소 최상위를 기준으로 실행합니다.

```powershell
Set-Location .\TouchGFX
make -f simulator/gcc/Makefile preview
.\build\bin\simulator.exe
```

`preview`는 기존 생성 소스를 이용해 실행 파일을 빌드하는 타깃입니다. `.touchgfx`, 이미지, 폰트, 텍스트를 변경했다면 TouchGFX Designer에서 자산·코드를 의도적으로 재생성한 뒤 빌드합니다. 생성 소스가 수정될 수 있으므로 일반 프리뷰 확인과 구분합니다.

산출물 경로는 저장소 기준 `TouchGFX/build/bin/simulator.exe`입니다. 현재 저장소에 없는 `stm32f411_fw` 하위 경로를 붙이지 않습니다. 위 명령은 현재 Makefile 기준 안내이며, 이번 문서 개편에서 호스트 빌드를 실행해 검증한 결과는 아닙니다.

## 입력과 화면 캡처

`SIMULATOR` 빌드의 Model은 합성 입력을 생성합니다. 시뮬레이터의 숫자와 경고 표시는 레이아웃·상태 변화 확인용이며 실제 차량 CAN의 수신 결과가 아닙니다.

시뮬레이터 창에 포커스를 둔 상태에서 `F3`는 BMP 캡처 저장, `Shift+F3`는 복사에 사용합니다. 캡처 위치는 실행 작업 폴더에 따라 확인하며, `TouchGFX/build/` 아래 산출물은 커밋 대상 소스와 구분합니다.

- [프로젝트 소개](../README.md)
- [개편 전 프리뷰 안내 원본](PREVIEW.before-portfolio-2026-11-14.md)
