# CDSL_fire_monitor_control

- CnSim_Linux: (23일 18:24) 현재 방수총 컴퓨터에 올라와있는 최신 버전. PD Controller까지 완료\
- CnSim_Linux_original: 처음 방수총 받았을 때 원본 \
- 레퍼런스 변경시 ref_generator 내부의 alpha_calculator.py에서 alpha 계산하여, CnSim_linux 폴더에서 ReferenceGenerator.cpp 의 alpha[0]~alpha[5]에 넣으면 됩니다..

sudo mv /home/kiro/workspace/CnSim_linux/cdsl_data.csv /home/kiro/CDSL/CDSL_fire_monitor_control/csv_datas/trial5/cdsl_data_t5_2.csv
