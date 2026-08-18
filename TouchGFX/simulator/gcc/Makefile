# Helper macros to convert spaces into question marks and back again
e := 
sp := $(e) $(e)
qs = $(subst ?,$(sp),$1)
sq = $(subst $(sp),?,$1)

# Get name of this Makefile (avoid getting word 0 and a starting space)
makefile_name := $(wordlist 1,1000,$(MAKEFILE_LIST))

# Get path of this Makefile
makefile_path := $(call qs,$(dir $(call sq,$(abspath $(call sq,$(makefile_name))))))

# Get path where the Application is
application_path := $(call qs,$(abspath $(call sq,$(makefile_path)../..)))

.PHONY: clean assets all preview

ifneq ($(words $(makefile_path))$(words $(MAKEFILE_LIST)),11)
all clean assets preview:
$(error Spaces not allowed in path)
else

ADDITIONAL_SOURCES :=
# This project carries only the Cortex-M4 TouchGFX library in-tree. The desktop
# simulator uses the matching TouchGFX Designer installation for host libraries
# and third-party headers.
touchgfx_install := C:\\TouchGFX\\4.26.1\\touchgfx
# Model.cpp 가 시뮬레이터 빌드에서도 표시 프로토콜의 valid mask 비트를 쓴다.
# 타깃 빌드는 CubeIDE 프로젝트 설정에 이 경로가 이미 들어 있다.
ADDITIONAL_INCLUDE_PATHS := ../../shared \
                            $(touchgfx_install)/3rdparty/libjpeg/include
# 이 저장소의 Middlewares 에는 타깃용 cortex_m4f 라이브러리만 들어 있다.
# 시뮬레이터가 쓰는 mingw32/SDL2/libjpeg 는 TouchGFX 설치본에서 가져온다.
# config/gcc/app.mk 가 touchgfx_env 를 절대경로로 박아 두는 것과 같은 방식이며,
# TouchGFX 버전을 올리면 여기도 같이 고쳐야 한다.
ADDITIONAL_LIBRARY_PATHS := $(touchgfx_install)/lib/win/mingw32 \
                            $(touchgfx_install)/lib/sdl2/win32 \
                            $(touchgfx_install)/3rdparty/libjpeg/lib/win32
ADDITIONAL_LIBRARIES :=
export ADDITIONAL_SOURCES ADDITIONAL_INCLUDE_PATHS ADDITIONAL_LIBRARY_PATHS ADDITIONAL_LIBRARIES

all: $(filter assets,$(MAKECMDGOALS))
all assets: $(filter clean,$(MAKECMDGOALS))
all clean assets:
	@$(MAKE) -r -f generated/simulator/gcc/Makefile -s $(MFLAGS) $@ -C "$(application_path)"

# Compile the simulator from the generated sources already tracked by Git.
# This keeps ordinary preview builds from rewriting generated source files.
preview:
	@$(MAKE) -r -f generated/simulator/gcc/Makefile -s $(MFLAGS) build_executable -C "$(application_path)"
endif
