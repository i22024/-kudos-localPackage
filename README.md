# -kudos-bh_vision
쿠도스 로컬라이제이션 레포지스토리입니다  

# 설치 방법
- qt5를 설치한다.
- catkin_ws/src에 필요에 따라 /mcl2만 넣거나 /soccer_simu폴더도 같이 넣는다.
- mcl2폴더를 설치하는 방법은 다음과 같다.
- 우선 cm을 한다.
- /mcl2의 내용물을 복사하다가 catkin_ws/build/mcl2폴더에 넣어준다.
- cmake  .
- make
- 위 명령어들을 해당 폴더에서 실행해주자
- 다시 catkin_make를 수행한다.
- ./mcl_localization
- 해당 폴더에서 ./명령어를 이용하여 프로그램을 실행시킬수 있다. 편의에 따라 bh_playground폴더의 bashrc.txt를 이용하여 명령어를 등록해두자
- 그 다음 노트북의 경우 시뮬레이터를 설치할 수 있다.
- 시뮬레이터를 작동시키기 위해서 python3가 필수이다. 이걸로 ros업데이트
- 시뮬레이터는 -kudos-vision_simulator를 이용하면 된다. 관리의 편의성을 위해 시뮬레이터는 이것만 건들 예정

- 만약 pyqt와 opencv가 충돌한다면 opencv에서 headless 버전으로 깔아줄 필요가 있다. pip3 install python-opencv-headless

    
